#ifndef ALL_HEADERS_H
#define ALL_HEADERS_H

/*================================================================
all_headers.h
==================================================================*/

//standard headers
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

//MACROS for setting default values for dbc
#define BATTERY_CELLS 1
#define VOLTAGE_MIN 2.5
#define VOLTAGE_MAX 4.2
#define CELLS_IN_LINE 10
#define DELTA_TIME 1.0
#define COULOMIC_EFFICIENCY 1.0
#define HEAT_COOL_POWER 5
#define HEATER_ON_TEMP 15
#define COOLER_ON_TEMP 35
#define CAPACITY 4.07611
#define SOC_TAPER_START 80.0
#define SOC_TAPER_END 98.0

//userdefined headers
#include "OCV_SOC.h"
#include "structpage.h"

#endif // ALL_HEADERS_H