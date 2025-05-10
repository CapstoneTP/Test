#ifndef OCV_SOC_T_H
#define OCV_SOC_T_H

#define OCV_SOC_T_SIZE 986

extern double CHG_OCV[986];
extern double CHG_SOC[986];
double OCV_from_SOC(double SOC);
double SOC_from_OCV(double OCV);

#endif