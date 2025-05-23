#ifndef STRUCTPAGE_H
#define STRUCTPAGE_H

#include "all_headers.h"

typedef struct {
    double SOC;
    double V1;
    double Charge_Current;
    double Capacity;
    double R0, R1, C1;
    double Voltage_terminal;
    double Temperature;
} Battery_t;

typedef struct {
    double SOC;
    double V1;
    double Voltage_terminal;
} Estimate_t;

typedef struct {
    int init;
    double F[2][2];
    double Q[2][2];
    double P[2][2];
    double Pp[2][2];
    double R;
} State_t;

Battery_t battery[BATTERY_CELLS];
Estimate_t estimate[BATTERY_CELLS];
State_t battery_state[BATTERY_CELLS];

#endif