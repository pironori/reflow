#ifndef FEEDBUCK_H_
#define FEEDBUCK_H_
#include <Arduino.h>
class Feedbuck
{
private:
    /* data */
    const float T_i; //step:80,limit_sense:50,limit_sence(second):100
    const float T_d;  //step:20,limit_sense:13.75,limit_sence(second):25
    const float K_p;  //default=0.11,step:0.00686,step(controll):0.01
    const float K_i;  //default=0.05
    const float K_d1; //default=0.11
    const float K_d2; //default=12.5
    const float TimeConstant;
    const float DeadTime;
    const float PwmCompareTop; //16Meg/(256[prescaler]*1[Hz])-1=62499
    const unsigned int DutySetup;
    const unsigned int TempTh1;
    const unsigned int TempTh2;
    float pretemperature;
    float offset;
    float GainTemp1;
    float GainTemp2;
    float Gain1;
    float Gain2;
    uint8_t count;
    bool flag;
    #define ProfileTemperatureFirstStep 800 //default=150
    #define ProfileTemperatureSecondStep 205  //default=230->205
    #define ProfileTemperatureThirdStep 200 //default=225
    #define ProfileTemperatureFourthStep 0  //default=0
public:
    Feedbuck();
    Feedbuck(float Kp,float Ti,float Td,float TimeConstant,float DeadTime);
    ~Feedbuck();
    unsigned int Feedbuck_cal_TwoDegreesOfFreedomControl(float temperature);
    unsigned int Feedbuck_cal_PIDControl(float temperature,unsigned int duty_target_value);
};


#endif