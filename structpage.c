#include "all_headers.h"

Battery_t default_battery = {
    .SOC = 0,
    .V1 = 0,
    .Charge_Current = 0,
    .Capacity = 0,
    .R0 = 0,
    .R1 = 0,
    .C1 = 0,
    .Voltage_terminal = 0,
    .Temperature = 0
};

Estimate_t exper = {
    .SOC = 0,
    .V1 = 0
};

State_t state = {
    .init = 0
};

Battery_t battery[BATTERY_CELLS];