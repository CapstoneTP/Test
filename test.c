#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "Userdheader/Charge_Data.h"
#include "Userdheader/OCV_SOC_T.h"
#define OCV_SOC_T_SIZE 986 //OCV_SOC 길이

typedef struct {            // 셀 내부 상태 및 상수
    float SOC_Initial;      // 초기 SOC(%)
    float voltage_delay_Initial;       // 초기 전압 지연 (R1C1 캐패시터 전압)
    float coulombic_efficiency;     // 운용 효율 (1.0 = 100%)
    float ChargeCurrent;       // 기준 전류 (A, – 면 충전)
    float noiseincurrent;         // 노이즈 포함 전류 (A) – 현재는 ChargeCurrent 그대로
    float Capacity;       // 정격 용량 (Ah)  —> 4.076 Ah
    float capacity1c;               // 1% SOC 당 전하량 (C)
    float R0, R1, C1;       // 회로 파라미터
    float voltage_delay;               // 최신 전압 지연 값
    float voltage_terminal;               // 최신 측정 터미널 전압
    float SOC;              // 실제 SoC (simulation state)
    float Temperature;       // 셀 온도
} Cell_Data_t;

typedef struct { // 한 타임스텝 실험 데이터
    float voltage_terminal;    // 측정 터미널 전압 (V)
    float voltage_delay;    // 모델이 예측한 전압 지연 (V)
    float noiseincurrent;// 사용 전류 (A)
} ExperData_t;

Cell_Data_t Init_Cell(float delta_time);
void EKFpredict(const float soc_voltagedelay_before[2], float delta_time, const Cell_Data_t *cell, float soc_voltagedelay_after[2]);
void ComputeJacobianH(float SOC_predict, float SOC_previous, float Jacobian_vector[2]);
float EKFVoltage_terminal_predict(const Cell_Data_t *cell, float SOC);
float OCV_from_SOC(float SOC);
void SOCEKF(const Cell_Data_t *cell, float delta_time, float *estimate_soc_result, float *estimate_voltagedelay_result);
float SOC_from_OCV(float ocv);
float ChargeCurrentLimits(const Cell_Data_t *cell, float SOC_estimate);
void UpdateResistance(Cell_Data_t *cell);
float  SimulateTerminalVoltage(Cell_Data_t *cell, float delta_time);
void   UpdateTemperature(Cell_Data_t *cell, float delta_time);

int main() {
    float delta_time = 1.0f;
    Cell_Data_t cell = Init_Cell(delta_time);
    //[SOC; voltage_delay]
    float estimate_soc_voltagedelay[2] = { cell.SOC_Initial, cell.voltage_delay_Initial };
    for (int k = 0; ; ++k) {
        UpdateTemperature(&cell, delta_time); //온도 갱신 (냉각/히팅)
        UpdateResistance(&cell); //온도 기반 저항 변동
        cell.voltage_terminal = SimulateTerminalVoltage(&cell, delta_time);
        cell.noiseincurrent = cell.ChargeCurrent;
        // EKF 갱신
        float SOC_estimate, voltage_delay_est;
        SOCEKF(&cell, delta_time, &SOC_estimate, &voltage_delay_est);
        // 온도 기반 충전 전류 제한
        cell.ChargeCurrent = ChargeCurrentLimits(&cell, SOC_estimate);
        cell.noiseincurrent   = cell.ChargeCurrent;
        // SOC, voltage_delay 갱신
        estimate_soc_voltagedelay[0] = SOC_estimate;
        estimate_soc_voltagedelay[1] = voltage_delay_est;
        printf("Time %5d: 온도 = %5.2f°C, 추정 SOC = %6.3f%%, 추정 voltage_delay = %8.5fV, 실측 터미널 전압 = %8.5f V\n",
                k, cell.Temperature, SOC_estimate, voltage_delay_est, cell.voltage_terminal);
        if ((k + 1) % 100 == 0) {
            printf("\n 시간 %d s — 계속 Enter, 종료 q 입력: ", k + 1);
            fflush(stdout);
            char buf[8] = {0};
            if (fgets(buf, sizeof(buf), stdin) && (buf[0] == 'q' || buf[0] == 'Q')) break;
        }
    }
    return 0;
}
/*
1. 내부 셀 초기화
2. 온도 저항 갱신
3. 가상 셀 전압 생성
4. SOCEKF로 SOC, voltage_delay 추정
5. 추정값 입력 -> ChargeCurrentLimits()로 충전 전류 재계산
6. 출력
*/

Cell_Data_t Init_Cell(float delta_time) {
    Cell_Data_t cell;
    cell.coulombic_efficiency = 1.0f; //쿨롱 효율 100%
    float V0 = Voltage_CHG[1005];
    cell.SOC_Initial = SOC_from_OCV(V0); //OCV -> 초기 SOC 변환
    cell.ChargeCurrent = -0.41f; //CC 전류 고정
    cell.noiseincurrent   = cell.ChargeCurrent;
    cell.Capacity = 4.07611f; //정격 4.076Ah
    //Capacity * 3600 = 14673.996C (전체 용량)
    //14673.996C / 100 = 1% SOC 변화당 141.74C -> 146.7초간 1A를 흘리면 SoC 1% 감소
    cell.capacity1c = (cell.Capacity * 3600) / 100; //1% soc 전햐량 C
    //내부 저항 구조 참고 (ECM)
    cell.R0 = 0.00005884314f;
    cell.R1 = 0.01145801322f;
    cell.C1 = 4846.080679f;
    //초기 voltage_delay (확산 전압 지연 -> ECM에 의한 지연)
    cell.voltage_delay_Initial = cell.ChargeCurrent * cell.R1 * (1- exp(-delta_time/(cell.R1 * cell.C1)));
    cell.voltage_delay = cell.voltage_delay_Initial;
    cell.Temperature = 25.0f; //배터리 초기 온도
    cell.SOC = cell.SOC_Initial; //SOC 적용

    return cell;
}

//온도 변화 -> 저항 및 충전 전류 변화
void UpdateResistance(Cell_Data_t *cell) { //온도를 기준으로 저항 조절
    // 25도 기준 저항
    const float R0_reference = 0.00005884314f;
    const float R1_reference = 0.01145801322f;
    const float temperature_coeff_R0  = 0.003f;   // R0 -> 1도 당 0.3 증가
    const float temperature_coeff_R1   = 0.003f;   // R1 -> ''
    float delta_time = cell->Temperature - 25.0f; // 현재 온도 - 기준 온도
    //선형 보정 공식 적용
    cell->R0 = R0_reference * (1.0f + temperature_coeff_R0 * delta_time);
    cell->R1 = R1_reference * (1.0f + temperature_coeff_R1  * delta_time);
}

//온도에 따라 전류 제한
float ChargeCurrentLimits(const Cell_Data_t *cell, float SOC_estimate)
{
    const float voltage_max_charge = 4.200f;
    const float charge_current_cc = -1.0f * cell->Capacity; //–1 C
    const float charge_current_min_CV = -0.05f * cell->Capacity; // –0.05 C
    const float voltage_hysteresis = 0.010f;
    const float voltage_control = 50.0f; // 전압 P-gain
    float charge_current_limits = charge_current_cc; // 기본 CC 전류 1C

    // SOC-기반 테이퍼 (선형 예시) 80% ~ 98%
    const float SOC_TAPER_START = 80.0f;
    const float SOC_TAPER_END   = 98.0f;
    if (SOC_estimate > SOC_TAPER_START) {
        float SOC_TAPER_RATIO = (SOC_estimate - SOC_TAPER_START) /
                    (SOC_TAPER_END - SOC_TAPER_START);
        if (SOC_TAPER_RATIO > 1.0f) SOC_TAPER_RATIO = 1.0f;
        charge_current_limits = charge_current_cc * (1.0f - 0.8f * SOC_TAPER_RATIO); //80% ~ 100 %, 전류 최대 80% 감소
    }
    // CV 전환
    if (cell->voltage_terminal >= voltage_max_charge - voltage_hysteresis) {
        charge_current_limits = charge_current_cc + voltage_control * (cell->voltage_terminal - voltage_max_charge);
        if (charge_current_limits < charge_current_min_CV) charge_current_limits = charge_current_min_CV;
        if (charge_current_limits > 0.0f) charge_current_limits = 0.0f;
    }
    //온도에 따라 충전량 제한
    if (cell->Temperature < 0.0f) charge_current_limits = 0.0f; //금지
    else if (cell->Temperature < 15.0f) charge_current_limits *= 0.5f; //0~15도 50% 제한

    return charge_current_limits;
}

//EKF 예측 함수
void EKFpredict(const float soc_voltagedelay_before[2], float delta_time, const Cell_Data_t *cell, float soc_voltagedelay_after[2]) {
    float local_delta_time = delta_time;
    float exp_term = expf(-delta_time / (cell->R1 * cell->C1));
    //SOC 예측
    soc_voltagedelay_after[0] = soc_voltagedelay_before[0] - cell->coulombic_efficiency * local_delta_time / cell->capacity1c * cell->noiseincurrent;
    // 1차 RC voltage_delay 예측
    soc_voltagedelay_after[1] = exp_term * soc_voltagedelay_before[1] + cell->R1 * (1.0f - exp_term) * cell->noiseincurrent;
    printf("[EKFpredict] 전류 적산 예측 SoC = %.6f, 예측 voltage_delay = %.6f\n", soc_voltagedelay_after[0], soc_voltagedelay_after[1]);
}

//칼만 이득 빌드업 -> 예측-측정 값 통합
void ComputeJacobianH(float SOC_predict, float SOC_previous, float Jacobian_vector[2]) {
    const float dSOC = 0.05f; // 테이블 간격 0.1의 절반 
    float soc_hi = SOC_predict + dSOC;
    float soc_low = SOC_predict - dSOC;
    if (soc_hi > 100.0f) soc_hi = 100.0f;
    if (soc_low <   0.0f) soc_low = 0.0f;
    float ocv_hi = OCV_from_SOC(soc_hi);
    float ocv_lo = OCV_from_SOC(soc_low);
    Jacobian_vector[0] = (ocv_hi - ocv_lo) / (soc_hi - soc_low);  // ≈ dOCV/dSOC
    Jacobian_vector[1] = -1.0f; //avoltage_terminal / avoltage_delay
    if (fabsf(Jacobian_vector[0]) < 1e-4f) { //0.0001
        Jacobian_vector[0] = (Jacobian_vector[0] >= 0.0f) ? 1e-4f : -1e-4f;
    }
    //자코비안 벡터
    printf("[ComputeJacobianH] dV/dSOC=%.6f, H_voltage_delay=%.6f\n", Jacobian_vector[0], Jacobian_vector[1]);
}

//EKF 측정
float EKFVoltage_terminal_predict(const Cell_Data_t *cell, float SOC) {
    if (SOC >= 100.0f) SOC = 100.0f;
    else if (SOC <= 0.0f) SOC = 0.0f;
    float ocv = OCV_from_SOC(SOC);
    //voltage_terminal 전압 예측
    float voltage_terminal_predict = ocv - cell->voltage_delay - cell->R0 * cell->noiseincurrent;
    printf("[EKFVoltage_terminal_predict] EKF 예측 voltage_terminal = %.6f (OCV_T=%.6f, voltage_delay(전압 지연)=%.6f, 전압 강하=%.6f)\n", voltage_terminal_predict, ocv, cell->voltage_delay, cell->R0 * cell->noiseincurrent);
    return voltage_terminal_predict;
}

//선형 보간 함수
float OCV_from_SOC(float SOC) {
    if (SOC <= CHG_SOC[0]) return CHG_OCV[0];
    if (SOC >= CHG_SOC[OCV_SOC_T_SIZE-1]) return CHG_OCV[OCV_SOC_T_SIZE-1];
    int i = 0;
    while (i < OCV_SOC_T_SIZE - 1 && CHG_SOC[i+1] < SOC) i++;
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
    if (ocv >= CHG_OCV[OCV_SOC_T_SIZE-1]) return CHG_SOC[OCV_SOC_T_SIZE-1];
    int i = 0;
    while (i < OCV_SOC_T_SIZE - 1 && CHG_OCV[i+1] < ocv) i++;
    //선형 보간으로 테이블 값들보다 저 정확하게 사이 값을 계산해주는 함수
    float ocv_low  = CHG_OCV[i];
    float ocv_high = CHG_OCV[i+1];
    float soc_low  = CHG_SOC[i];
    float soc_high = CHG_SOC[i+1];
    float t = (ocv - ocv_low) / (ocv_high - ocv_low);
    return (soc_low + t * (soc_high - soc_low));
}

void SOCEKF(const Cell_Data_t *cell, float delta_time, float *estimate_soc_result, float *estimate_voltagedelay_result) {
    static int init = 0;
    //F:상태 전이 행렬, Q:시스템 노이즈(예측 불확실성), P:추정 오차 공분산, Pp:예측 공분산
    static float F[2][2], Q[2][2], P[2][2], Pp[2][2];
    static float estimate_SOC_Voltagedelay[2], previous_vector[2];
    //측정 노이즈 (터미널 전압 신뢰도)
    static float R;
    if (!init) {
        F[0][0] = 1.0f; F[0][1] = 0.0f;
        F[1][0] = 0.0f; F[1][1] = expf(-delta_time / (cell->R1 * cell->C1));
        Q[0][0] = 0.0000001f; Q[0][1] = 0.0f;
        Q[1][0] = 0.0f; Q[1][1] = 0.0000001f;
        R = 500.0f;
        P[0][0] = 3000.0f; P[0][1] = 0.0f;
        P[1][0] = 0.0f; P[1][1] = 3000.0f;
        previous_vector[0] = cell->SOC_Initial;
        previous_vector[1] = cell->voltage_delay_Initial;
        init = 1;
    }
    float soc_voltagedelay_after[2];
    //상태 예측, 공분산 예측
    EKFpredict(previous_vector, delta_time, cell, soc_voltagedelay_after);
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
    float Jacobian_vector[2];
    //측정 전압
    ComputeJacobianH(soc_voltagedelay_after[0], previous_vector[0], Jacobian_vector);
    //HP -> H*Pp
    float HP[2];
    HP[0] = Jacobian_vector[0]*Pp[0][0] + Jacobian_vector[1]*Pp[1][0];
    HP[1] = Jacobian_vector[0]*Pp[0][1] + Jacobian_vector[1]*Pp[1][1];

    //H*Pp* (H^T + R) <- 괄호 부분 구현
    float kalman_gain_denom = HP[0]*Jacobian_vector[0] + HP[1]*Jacobian_vector[1] + R;
    // 칼만 이득 K = Pp * H^T / kalman_gain_denom
    float kalman_gain[2];
    kalman_gain[0] = HP[0] / kalman_gain_denom;
    kalman_gain[1] = HP[1] / kalman_gain_denom;
    //residual
    float predict_voltage_terminal = EKFVoltage_terminal_predict(cell, soc_voltagedelay_after[0]);
    float y = cell->voltage_terminal - predict_voltage_terminal;
    // 상태 추정 갱신
    estimate_SOC_Voltagedelay[0] = soc_voltagedelay_after[0] + kalman_gain[0]*y;
    estimate_SOC_Voltagedelay[1] = soc_voltagedelay_after[1] + kalman_gain[1]*y;
    //SOC 0 ~ 100
    if (estimate_SOC_Voltagedelay[0] < 0.0f) estimate_SOC_Voltagedelay[0] = 0.0f;
    else if (estimate_SOC_Voltagedelay[0] > 100.0f) estimate_SOC_Voltagedelay[0] = 100.0f;
    // 공분산 갱신 P = (I - kalman_gain*H) * Pp
    float kalman_gain_h[2][2]; //K * H 행렬
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            kalman_gain_h[i][j] = kalman_gain[i] * Jacobian_vector[j];
        }
    }
    float I_KH[2][2] = {{1.0f - kalman_gain_h[0][0],    -kalman_gain_h[0][1]},
                        {-kalman_gain_h[1][0], 1.0f - kalman_gain_h[1][1]}};
    float update_error[2][2];
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            update_error[i][j] = I_KH[i][0]*Pp[0][j] + I_KH[i][1]*Pp[1][j];
        }
    }
    previous_vector[0] = estimate_SOC_Voltagedelay[0];
    previous_vector[1] = estimate_SOC_Voltagedelay[1];
    memcpy(P, update_error, sizeof(P));

    *estimate_soc_result = estimate_SOC_Voltagedelay[0];
    *estimate_voltagedelay_result  = estimate_SOC_Voltagedelay[1];
}

void UpdateTemperature(Cell_Data_t *cell, float delta_time) //발열/냉각 기능
{
    const float C_capacity = 200.0f; // 열용량
    const float C_resistance = 3.0f; // 열저항
    const float airtemp_C = 0.0f; // 외기 온도

    // 히터 및 쿨러 파라미터
    const float heater_power_w = 5.0f; // 히터 동작 시 열 공급량 -> 올리면 히터 기능 상승
    const float cooling_power_w = 5.0f; // 쿨러 동작 시 열 제거량 -> 올리면 쿨링 기능 상승
    const float heater_on_temperature = 15.0f; // 히터 작동 임계 온도 (°C 이하)
    const float cooling_on_temperature = 35.0f; // 쿨러 작동 임계 온도 (°C 이상)
    // 내부 발열
    float internal_heat = cell->R0 * cell->ChargeCurrent * cell->ChargeCurrent;
    // 히팅/냉각 제어
    float heater_power = (cell->Temperature < heater_on_temperature) ? heater_power_w : 0.0f;
    float cooling_power = (cell->Temperature > cooling_on_temperature) ? cooling_power_w : 0.0f;
    // 총 열 플럭스 (J/s)
    float total_heat = internal_heat + heater_power - cooling_power;
    // 온도 변화 계산
    float local_delta_time = delta_time / C_capacity * (total_heat - (cell->Temperature - airtemp_C) / C_resistance);
    cell->Temperature += local_delta_time;
}

float SimulateTerminalVoltage(Cell_Data_t *cell, float delta_time) //바뀐 저항을 이용해 셀 전압 계산
{
    //SOC 갱신 0 ~ 100
    cell->SOC -= cell->coulombic_efficiency * delta_time / cell->capacity1c * cell->ChargeCurrent;
    if (cell->SOC < 0.f)   cell->SOC = 0.f;
    if (cell->SOC > 100.f) cell->SOC = 100.f;

    //voltage_delay 계산
    float tau = cell->R1 * cell->C1;
    float e   = expf(-delta_time / tau);
    cell->voltage_delay  = cell->voltage_delay * e + cell->R1 * (1.f - e) * cell->ChargeCurrent;
    //선형 보간 OCV 출력
    float ocv = OCV_from_SOC(cell->SOC);
    //터미널 전압 리턴
    return ocv - cell->voltage_delay - cell->R0 * cell->ChargeCurrent;
}