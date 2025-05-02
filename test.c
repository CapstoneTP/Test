#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "Userdheader/Charge_Data.h"
#include "Userdheader/OCV_SOC_T.h"
#define LUT_SIZE 986 //OCV_SOC 길이

typedef struct {            // 셀 내부 상태 및 상수
    float SOC_Initial;      // 초기 SOC(%)
    float V1_Initial;       // 초기 전압 지연 (R1C1 캐패시터 전압)
    float coulomb_effi;     // 운용 효율 (1.0 = 100%)
    float ik_nominal;       // 기준 전류 (A, – 면 충전)
    float ik_noise;         // 노이즈 포함 전류 (A) – 현재는 ik_nominal 그대로
    float Cn_nominal;       // 정격 용량 (Ah)  —> 4.076 Ah
    float Cn;               // 1% SOC 당 전하량 (C)
    float R0, R1, C1;       // 회로 파라미터
    float V1;               // 최신 전압 지연 값
    float Vt;               // 최신 측정 터미널 전압
    float SOC;              // 실제 SoC (simulation state)
    float Temperature;       // 셀 온도
} Cell_Data_t;

typedef struct { // 한 타임스텝 실험 데이터
    float Vt;    // 측정 터미널 전압 (V)
    float V1;    // 모델이 예측한 전압 지연 (V)
    float ik_noise;// 사용 전류 (A)
} ExperData_t;

Cell_Data_t Init_Cell(float dt);
void fx(const float xk_before[2], float dt, const Cell_Data_t *cell, float x_pred[2]);
void Hk(float SOC_pred, float SOC_prev, float H_vec[2]);
float hx(const Cell_Data_t *cell, float SOC);
float OCV_from_SOC(float SOC);
void SOCEKF(const Cell_Data_t *cell, float dt, float *SOC_k, float *V1_k);
float SOC_from_OCV(float ocv);
float AdaptiveChargeCurrent(const Cell_Data_t *cell, float SOC_est);
void UpdateResistance(Cell_Data_t *cell);
float  CalcVirtualVt(Cell_Data_t *cell, float dt);
void   UpdateTemperature(Cell_Data_t *cell, float dt);

int main() {
    float dt = 1.0f;
    Cell_Data_t cell = Init_Cell(dt);
    //cell.Temperature = -10.0f; //온도 테스트 중

    //[SOC; V1]
    float xk[2] = { cell.SOC_Initial, cell.V1_Initial };
    for (int k = 0; ; ++k) {
        UpdateTemperature(&cell, dt); //온도 갱신 (냉각/히팅)
        UpdateResistance(&cell); //온도 기반 저항 변동
        cell.Vt       = CalcVirtualVt(&cell, dt);
        cell.ik_noise = cell.ik_nominal;
        // EKF 갱신
        float SOC_est, V1_est;
        SOCEKF(&cell, dt, &SOC_est, &V1_est);
        // 온도 기반 충전 전류 제한
        cell.ik_nominal = AdaptiveChargeCurrent(&cell, SOC_est);
        cell.ik_noise   = cell.ik_nominal;
        // SOC, V1 갱신
        xk[0] = SOC_est;
        xk[1] = V1_est;
        printf("Time %5d: 온도 = %5.2f°C, 추정 SOC = %6.3f%%, 추정 V1 = %8.5fV, 실측 터미널 전압 = %8.5f V\n",
               k, cell.Temperature, SOC_est, V1_est, cell.Vt);
        if ((k + 1) % 100 == 0) {
            printf("\n 시간 %d s — 계속 Enter, 종료 q 입력: ", k + 1);
            fflush(stdout);

            char buf[8] = {0};
            if (fgets(buf, sizeof(buf), stdin) && (buf[0] == 'q' || buf[0] == 'Q')) {
                break;
            }
        }
    }
    return 0;
}
/*
1. 내부 셀 초기화
2. 온도 저항 갱신
3. 가상 셀 전압 생성
4. SOCEKF로 SOC, V1 추정
5. 추정값 입력 -> AdaptiveChargeCurrent()로 충전 전류 재계산
6. 출력
*/

Cell_Data_t Init_Cell(float dt) {
    Cell_Data_t cell;
    cell.coulomb_effi = 1.0f; //쿨롱 효율 100%
    float V0 = Voltage_CHG[1005];
    cell.SOC_Initial = SOC_from_OCV(V0); //OCV -> 초기 SOC 변환
    cell.ik_nominal = -0.41f; //CC 전류 고정
    cell.ik_noise   = cell.ik_nominal;
    cell.Cn_nominal = 4.07611f; //정격 4.076Ah
    //Cn_nominal * 3600 = 14673.996C (전체 용량)
    //14673.996C / 100 = 1% SOC 변화당 141.74C -> 146.7초간 1A를 흘리면 SoC 1% 감소
    cell.Cn = (cell.Cn_nominal * 3600) / 100; //1% soc 전햐량 C
    //내부 저항 구조 참고 (ECM)
    cell.R0 = 0.00005884314f;
    cell.R1 = 0.01145801322f;
    cell.C1 = 4846.080679f;
    //초기 V1 (확산 전압 지연 -> ECM에 의한 지연)
    cell.V1_Initial = cell.ik_nominal * cell.R1 * (1- exp(-dt/(cell.R1 * cell.C1)));
    cell.V1 = cell.V1_Initial;
    cell.Temperature = 25.0f; //배터리 초기 온도
    cell.SOC = cell.SOC_Initial; //SOC 적용

    return cell;
}

//온도 변화 -> 저항 및 충전 전류 변화
void UpdateResistance(Cell_Data_t *cell) { //온도를 기준으로 저항 조절
    // 25도 기준 저항
    const float R0_ref = 0.00005884314f;
    const float R1_ref = 0.01145801322f;
    const float alpha  = 0.003f;   // R0 -> 1도 당 0.3 증가
    const float beta   = 0.003f;   // R1 -> ''
    float dT = cell->Temperature - 25.0f; // 현재 온도 - 기준 온도
    //선형 보정 공식 적용
    cell->R0 = R0_ref * (1.0f + alpha * dT);
    cell->R1 = R1_ref * (1.0f + beta  * dT);
}

//온도에 따라 전류 제한
float AdaptiveChargeCurrent(const Cell_Data_t *cell, float SOC_est)
{
    const float V_MAX_CHG = 4.200f;
    const float I_CC = -1.0f * cell->Cn_nominal; //–1 C
    const float I_MIN_CV = -0.05f * cell->Cn_nominal; // –0.05 C
    const float V_HYST = 0.010f;
    const float Kp_V = 50.0f; // 전압 P-gain
    float I_req = I_CC; // 기본 CC 전류 1C

    // SOC-기반 테이퍼 (선형 예시) 80% ~ 98%
    const float SOC_TAPER_START = 80.0f;
    const float SOC_TAPER_END   = 98.0f;
    if (SOC_est > SOC_TAPER_START) {
        float s = (SOC_est - SOC_TAPER_START) /
                    (SOC_TAPER_END - SOC_TAPER_START);
        if (s > 1.0f) s = 1.0f;
        I_req = I_CC * (1.0f - 0.8f * s); //80% ~ 100 %, 전류 최대 80% 감소
    }
    // CV 전환
    if (cell->Vt >= V_MAX_CHG - V_HYST) {
        I_req = I_CC + Kp_V * (cell->Vt - V_MAX_CHG);
        if (I_req < I_MIN_CV) I_req = I_MIN_CV;
        if (I_req > 0.0f) I_req = 0.0f;
    }
    //온도에 따라 충전량 제한
    if (cell->Temperature < 0.0f) I_req = 0.0f; //금지
    else if (cell->Temperature < 15.0f) I_req *= 0.5f; //0~15도 50% 제한

    return I_req;
}

//EKF 예측 함수
void fx(const float xk_before[2], float dt, const Cell_Data_t *cell, float x_pred[2]) {
    float dt_h = dt;
    float exp_term = expf(-dt / (cell->R1 * cell->C1));
    //SOC 예측
    x_pred[0] = xk_before[0] - cell->coulomb_effi * dt_h / cell->Cn * cell->ik_noise;
    // 1차 RC V1 예측
    x_pred[1] = exp_term * xk_before[1] + cell->R1 * (1.0f - exp_term) * cell->ik_noise;
    printf("[fx] 전류 적산 예측 SoC = %.6f, 예측 V1 = %.6f\n", x_pred[0], x_pred[1]);
}

//예측 상태 이용
void Hk(float SOC_pred, float SOC_prev, float H_vec[2]) {
    const float dSOC = 0.05f; // 테이블 간격 0.1의 절반 
    float s_hi = SOC_pred + dSOC;
    float s_lo = SOC_pred - dSOC;
    if (s_hi > 100.0f) s_hi = 100.0f;
    if (s_lo <   0.0f) s_lo = 0.0f;
    float ocv_hi = OCV_from_SOC(s_hi);
    float ocv_lo = OCV_from_SOC(s_lo);
    H_vec[0] = (ocv_hi - ocv_lo) / (s_hi - s_lo);  // ≈ dOCV/dSOC
    H_vec[1] = -1.0f; //aVt / aV1
    if (fabsf(H_vec[0]) < 1e-4f) { //0.0001
        H_vec[0] = (H_vec[0] >= 0.0f) ? 1e-4f : -1e-4f;
    }
    //자코비안 벡터
    printf("[Hk] dV/dSOC=%.6f, H_V1=%.6f\n", H_vec[0], H_vec[1]);
}

//EKF 측정
float hx(const Cell_Data_t *cell, float SOC) {
    if (SOC >= 100.0f) SOC = 100.0f;
    else if (SOC <= 0.0f) SOC = 0.0f;
    float ocv = OCV_from_SOC(SOC);
    //Vt 전압 예측
    float pred = ocv - cell->V1 - cell->R0 * cell->ik_noise;
    printf("[hx] EKF 예측 Vt = %.6f (OCV_T=%.6f, V1(전압 지연)=%.6f, 전압 강하=%.6f)\n", pred, ocv, cell->V1, cell->R0 * cell->ik_noise);
    return pred;
}

//선형 보간 함수
float OCV_from_SOC(float SOC) {
    if (SOC <= CHG_SOC[0]) return CHG_OCV[0];
    if (SOC >= CHG_SOC[LUT_SIZE-1]) return CHG_OCV[LUT_SIZE-1];
    int i = 0;
    while (i < LUT_SIZE - 1 && CHG_SOC[i+1] < SOC) i++;
    float soc_low = CHG_SOC[i];
    float soc_high = CHG_SOC[i+1];
    //입력한 SOC -> 상한 하한으로 정밀 SOC 계산 (선형 보간)
    float t = (SOC - soc_low) / (soc_high - soc_low);
    //OCV 리턴
    return (CHG_OCV[i] + t * (CHG_OCV[i+1] - CHG_OCV[i]));
}

//위 함수 반대
float SOC_from_OCV(float ocv) {
    if (ocv <= CHG_OCV[0]) return CHG_SOC[0];
    if (ocv >= CHG_OCV[LUT_SIZE-1]) return CHG_SOC[LUT_SIZE-1];
    int i = 0;
    while (i < LUT_SIZE - 1 && CHG_OCV[i+1] < ocv) i++;
    //선형 보간으로 테이블 값들보다 저 정확하게 사이 값을 계산해주는 함수
    float ocv_low  = CHG_OCV[i];
    float ocv_high = CHG_OCV[i+1];
    float soc_low  = CHG_SOC[i];
    float soc_high = CHG_SOC[i+1];
    float t = (ocv - ocv_low) / (ocv_high - ocv_low);
    return (soc_low + t * (soc_high - soc_low));
}

void SOCEKF(const Cell_Data_t *cell, float dt, float *SOC_k, float *V1_k) {
    static int init = 0;
    //F:상태 전이 행렬, Q:시스템 노이즈(예측 불확실성), P:추정 오차 공분산, Pp:예측 공분산
    static float F[2][2], Q[2][2], P[2][2], Pp[2][2];
    static float x_est[2], x_prev[2];
    //측정 노이즈 (터미널 전압 신뢰도)
    static float R;
    if (!init) {
        F[0][0] = 1.0f; F[0][1] = 0.0f;
        F[1][0] = 0.0f; F[1][1] = expf(-dt / (cell->R1 * cell->C1));
        Q[0][0] = 0.0000001f; Q[0][1] = 0.0f;
        Q[1][0] = 0.0f; Q[1][1] = 0.0000001f;
        R = 500.0f;
        P[0][0] = 3000.0f; P[0][1] = 0.0f;
        P[1][0] = 0.0f; P[1][1] = 3000.0f;
        x_prev[0] = cell->SOC_Initial;
        x_prev[1] = cell->V1_Initial;
        init = 1;
    }
    float x_pred[2];
    //상태 예측, 공분산 예측
    fx(x_prev, dt, cell, x_pred);
    // Pp = F * P * F^T + Q
    // compute F*P
    float FP[2][2];
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            FP[i][j] = F[i][0]*P[0][j] + F[i][1]*P[1][j];
        }
    }
    // compute Pp = FP*F^T + Q
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            Pp[i][j] = FP[i][0]*F[j][0] + FP[i][1]*F[j][1] + Q[i][j];
        }
    }
    float H_vec[2];
    //측정 전압
    Hk(x_pred[0], x_prev[0], H_vec);
    //H*Pp*H^T + R
    float HP[2];
    HP[0] = H_vec[0]*Pp[0][0] + H_vec[1]*Pp[1][0];
    HP[1] = H_vec[0]*Pp[0][1] + H_vec[1]*Pp[1][1];

    //H*Pp*H^T + R
    float denom = HP[0]*H_vec[0] + HP[1]*H_vec[1] + R;
    // 칼만 이득 K = Pp * H^T / denom
    float K[2];
    K[0] = HP[0] / denom;
    K[1] = HP[1] / denom;
    //residual
    float z_pred = hx(cell, x_pred[0]);
    float y = cell->Vt - z_pred;
    // 상태 추정 갱신
    x_est[0] = x_pred[0] + K[0]*y;
    x_est[1] = x_pred[1] + K[1]*y;
    //SOC 0 ~ 100
    if (x_est[0] < 0.0f) x_est[0] = 0.0f;
    else if (x_est[0] > 100.0f) x_est[0] = 100.0f;
    // 공분산 갱신 P = (I - K*H) * Pp
    float KH[2][2];
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            KH[i][j] = K[i] * H_vec[j];
        }
    }
    float I_KH[2][2] = {{1.0f - KH[0][0],    -KH[0][1]},
                       {   -KH[1][0], 1.0f - KH[1][1]}};
    float P_new[2][2];
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            P_new[i][j] = I_KH[i][0]*Pp[0][j] + I_KH[i][1]*Pp[1][j];
        }
    }
    x_prev[0] = x_est[0];
    x_prev[1] = x_est[1];
    memcpy(P, P_new, sizeof(P));

    *SOC_k = x_est[0];
    *V1_k  = x_est[1];
}

void UpdateTemperature(Cell_Data_t *cell, float dt) //발열/냉각 기능
{
    const float Cth = 200.0f; // 열용량
    const float Rth = 3.0f; // 열저항
    const float Tamb = 0.0f; // 외기 온도

    // 히터 및 쿨러 파라미터
    const float P_heater = 5.0f; // 히터 동작 시 열 공급량 -> 올리면 히터 기능 상승
    const float P_cool = 5.0f; // 쿨러 동작 시 열 제거량 -> 올리면 쿨링 기능 상승
    const float T_heater_on = 15.0f; // 히터 작동 임계 온도 (°C 이하)
    const float T_cool_on = 35.0f; // 쿨러 작동 임계 온도 (°C 이상)
    // 내부 발열
    float internal_heat = cell->R0 * cell->ik_nominal * cell->ik_nominal;
    // 히팅/냉각 제어
    float heater_power = (cell->Temperature < T_heater_on) ? P_heater : 0.0f;
    float cooling_power = (cell->Temperature > T_cool_on) ? P_cool : 0.0f;
    // 총 열 플럭스 (J/s)
    float total_heat = internal_heat + heater_power - cooling_power;
    // 온도 변화 계산
    float dT = dt / Cth * (total_heat - (cell->Temperature - Tamb) / Rth);
    cell->Temperature += dT;
}

float CalcVirtualVt(Cell_Data_t *cell, float dt) //바뀐 저항을 이용해 셀 전압 계산
{
    //SOC 갱신 0 ~ 100
    cell->SOC -= cell->coulomb_effi * dt / cell->Cn * cell->ik_nominal;
    if (cell->SOC < 0.f)   cell->SOC = 0.f;
    if (cell->SOC > 100.f) cell->SOC = 100.f;

    //V1 계산
    float tau = cell->R1 * cell->C1;
    float e   = expf(-dt / tau);
    cell->V1  = cell->V1 * e + cell->R1 * (1.f - e) * cell->ik_nominal;
    //선형 보간 OCV 출력
    float ocv = OCV_from_SOC(cell->SOC);
    //터미널 전압 리턴
    return ocv - cell->V1 - cell->R0 * cell->ik_nominal;
}