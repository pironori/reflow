#include "Feedbuck.h"
extern float temperatureMax; // target Temprature

Feedbuck::Feedbuck()
    :T_i(100.0)
    ,T_d(25.0)
    ,K_p(0.01)
    ,K_i(K_p/T_i)
    ,K_d1(K_p*T_d)
    ,K_d2(25*K_d1)
    ,TimeConstant(100.0)
    ,DeadTime(17.33)
    ,PwmCompareTop(62499.0)
    ,DutySetup(62498)
    ,TempTh1(100)
    ,TempTh2(50)
    ,pretemperature(0.0)
    ,offset(0.0)
    ,count(0)
    ,flag(false)
    ,GainTemp1(0.0)
    ,GainTemp2(0.0)
    ,Gain1(0.0)
    ,Gain2(0.0)
{
}

Feedbuck::Feedbuck(float Kp,float Ti,float Td,float Time_Constant,float Dead_Time)
    :T_i(Ti) //step:80,limit_sense:50,limit_sence(second):100
    ,T_d(Td) //step:20,limit_sense:13.75,limit_sence(second):25
    ,K_p(Kp) //default=0.11,step:0.00686
    ,K_i(K_p/T_i)  //default=0.05
    ,K_d1(K_p*T_d) //default=0.11
    ,K_d2(25*K_d1) //default=12.5
    ,TimeConstant(Time_Constant)    //TimeConstant=100.0(measurement)
    ,DeadTime(Dead_Time)    //DeadTime=17.33(measurement)
    ,PwmCompareTop(62499.0) //16Meg/(256[prescaler]*1[Hz])-1=62499
    ,DutySetup(62498)
    ,TempTh1(100)
    ,TempTh2(50)
    ,pretemperature(0.0)
    ,offset(0.0)
    ,count(0)
    ,flag(false)
    ,GainTemp1(0.0)
    ,GainTemp2(0.0)
    ,Gain1(0.0)
    ,Gain2(0.0)
{
}

Feedbuck::~Feedbuck(){
}

/*
    @brief  PID controller for reflow

    @return duty value caliculated PID control
*/
unsigned int Feedbuck::Feedbuck_cal_PIDControl(float temperature,unsigned int duty_target_value){
  //float target_error=temperature_control_data[tableCounter][1]-temperature;
  float target_error=temperatureMax-temperature;
  int temp_ref=0;
  switch ((unsigned int)(temperatureMax)){
  case ProfileTemperatureFirstStep:
    temp_ref=TempTh1;
    break;
  case ProfileTemperatureSecondStep:
  case ProfileTemperatureThirdStep:
    if(!flag){
      duty_target_value=DutySetup;
      flag=true;
    }
    temp_ref=TempTh2;
    break;
  default:
    break;
  }

  /*propotional control*/
    if(target_error>=0){
      if(target_error<(float)temp_ref){
        if(temperature>=pretemperature){
          //duty down
          duty_target_value=duty_target_value>(PwmCompareTop-1)*(K_p*target_error/temperatureMax)?(unsigned int)duty_target_value-(PwmCompareTop-1)*(K_p*target_error/temperatureMax):0;
          /*
          duty_target_value-=(PWM_COMPARE_TOP-1)*(K_p*target_error/temperature_control_data[tableCounter][1]);
          duty_target_value=duty_target_value>=0?duty_target_value:0;
          */
        }else{
          duty_target_value=duty_target_value+(unsigned int)(PwmCompareTop-1)*(K_p*target_error/temperatureMax)<PwmCompareTop-1?(unsigned int)duty_target_value+(PwmCompareTop-1)*(K_p*target_error/temperatureMax):PwmCompareTop;
        }
      }else{
        duty_target_value=DutySetup;
      }
    }else{
      //duty=0
      duty_target_value=0;
    }

    /*integtal control*/
    
    if(target_error>=0){
      if(target_error<(float)temp_ref){
        if(temperature>=pretemperature){
          offset+=16000000/(256*(PwmCompareTop+1));   //add every timer interrupt
          duty_target_value=duty_target_value>(PwmCompareTop-1)*(K_i*target_error*offset/temperatureMax)?duty_target_value-(unsigned int)(PwmCompareTop-1)*(K_i*target_error*offset/temperatureMax):0;
          //duty_target_value-=(PWM_COMPARE_TOP-1)*(K_i*target_error*offset/temperature_control_data[tableCounter][2]);
          //duty_target_value=duty_target_value>=0?duty_target_value:0;
        }else{
          duty_target_value=duty_target_value+(unsigned int)(PwmCompareTop-1)*(K_i*target_error*offset/temperatureMax)<PwmCompareTop?duty_target_value+(unsigned int)(PwmCompareTop-1)*(K_i*target_error*offset/temperatureMax):PwmCompareTop;
        }
      }else{
        offset=0;
      }
    }else{
      duty_target_value=0;
      offset=0;
    }
    

    /*derivative control*/
    
    if(target_error>=0){
      if(temperature>=pretemperature){
        if(target_error<(float)temp_ref){
          duty_target_value=duty_target_value>(PwmCompareTop-1)*(K_d1*((temperature-pretemperature)/(16000000/(256*(PwmCompareTop+1))))/temperatureMax)?duty_target_value-(unsigned int)(PwmCompareTop-1)*(K_d1*((temperature-pretemperature)/(16000000/(256*(PwmCompareTop+1))))/temperatureMax):0;
        }
      }else{
        duty_target_value=duty_target_value+(unsigned int)(PwmCompareTop-1)*(K_d2*((pretemperature-temperature)/(16000000/(256*(PwmCompareTop+1))))/temperatureMax)>=PwmCompareTop?PwmCompareTop:duty_target_value+(unsigned int)(PwmCompareTop-1)*(K_d2*((pretemperature-temperature)/(16000000/(256*(PwmCompareTop+1))))/temperatureMax);
      }
    }else{
      duty_target_value=0;
    }
  count++;
  if(count==3){
    pretemperature=temperature; //renew pretemperature
    count=0;
  }

  return duty_target_value;
}



unsigned int Feedbuck::Feedbuck_cal_TwoDegreesOfFreedomControl(float temperature){
    float target_value=0.0;
    unsigned int duty=0;

    offset+=16000000/(256*(PwmCompareTop+1));   //add every timer interrupt
    GainTemp1=K_p*temperature+K_i*temperature*offset+K_d1*temperature/(16000000/(256*(PwmCompareTop+1)));
    Gain1=K_p*(T_d+offset+T_i*offset*offset);
    Gain2=(TimeConstant/temperatureMax)*(1+DeadTime/(16000000/(256*(PwmCompareTop+1))))+
    (1/temperatureMax)*(DeadTime+offset+DeadTime*DeadTime/(2*(16000000/(256*(PwmCompareTop+1)))));
    GainTemp2=(Gain1+Gain2)*temperatureMax;
    target_value=GainTemp2-GainTemp1;
    target_value=target_value>=0?target_value:0;
    duty=(unsigned int)target_value/GainTemp2*PwmCompareTop;
    
    return(duty);
}