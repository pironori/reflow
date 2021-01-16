#include <Arduino.h>
//#include <ESP8266.h>
#include <SoftwareSerial.h>
//#include <BlynkSimpleStream.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "Adafruit_MAX31855.h"
//#include <avr/interrupt.h>
//#include <MsTimer2.h>

#include "Feedbuck.h"

#define allOFF 0
#define upON   1
#define downON 2
#define allON  3
#define ControlDataLen 3
#define ProfileTemperatureFirstStep 150 //default=150
#define ProfileTemperatureSecondStep 205  //default=230->205
#define ProfileTemperatureThirdStep 200 //default=225
#define ProfileTemperatureFourthStep 0  //default=0
int temperature_control_data[ControlDataLen + 1][3] = {
  {downON, ProfileTemperatureFirstStep, 60},  // 2ON   , temperature130, keep15sec->60s
  {allON , ProfileTemperatureSecondStep,  0},  // 1&2ON , temperature230, keep0sec
  {downON, ProfileTemperatureThirdStep,  20},  // 2ON   , temperature225, keep100sec->20s
  {allOFF, ProfileTemperatureFourthStep,  0}   // 1&2OFF, temperature0  , keep0sec
};

// LCD(D2-D7)
#define LCDrsPin 2
#define LCDenablePin 3
#define LCDd4Pin 4
#define LCDd5Pin 5
#define LCDd6Pin 6
#define LCDd7Pin 7
// button(D8)
#define StartButton 8
// pwm(D9)
#define PWM 9
// Beep(D11)
#define TonePin 11
// Tempratier(D10,D12-D13)+((D0)
#define TemperatureSlavePin1 10
#define TemperatureMisoPin 14
#define TemperatureSckPin 15
#define TemperatureSlavePin2 16
//delay setting
#define delayWait 250 //max_AD_converter transimit time=225ms
#define oneSec (1000 / delayWait)
//PID parameter setting
#define T_i 100.0 //step:80,limit_sense:50,limit_sence(second):100
#define T_d 25.0  //step:20,limit_sense:13.75,limit_sence(second):25
#define K_p 0.01  //default=0.11,step:0.00686
#define K_i K_p/T_i  //default=0.05
#define K_d1 K_p*T_d //default=0.11
#define K_d2 25*K_d1 //default=12.5
#define TIME_CONSTANT 100.0
#define DEAD_TIME 17.3
#define PWM_COMPARE_TOP 62499.0 //16Meg/(256[prescaler]*1[Hz])-1=62499
#define DUTY_SETUP 62498 //duty=100%
#define TEMP_TH1 100
#define TEMP_TH2 50
#define COMP 5.0  //thermocouple2-5.0=thermocouple1

Adafruit_MAX31855 thermocouple1(TemperatureSckPin, TemperatureSlavePin1, TemperatureMisoPin);
Adafruit_MAX31855 thermocouple2(TemperatureSckPin,TemperatureSlavePin2,TemperatureMisoPin);
LiquidCrystal lcd(LCDrsPin,LCDenablePin,LCDd4Pin,LCDd5Pin,LCDd6Pin,LCDd7Pin);
Feedbuck reflow(K_p,T_i,T_d,TIME_CONSTANT,DEAD_TIME);
//SoftwareSerial mySerial(0, 1); // RX, TX

float tempratureRead(uint8_t number);
void setTempratureData(void);
void heatControl(float temperature);
void lcdDisplay(float temperature1,float temperature2);
/*pid_feedbuck*/

byte state;          // main program mode
byte heatMode;       // UpDown heater mode
byte heatState;      // UpDown heater status
byte tableCounter;   // data table counter
boolean blinkFlag;   // blink ON/OFF flag
uint8_t count=0;
int temperatureWait;  // temprature keep time(SEC)
int blinkTimer;      // blink timer
unsigned int duty=DUTY_SETUP;
unsigned int duty_target_value=DUTY_SETUP;
uint8_t cs1=TemperatureSlavePin1;
uint8_t cs2=TemperatureSlavePin2;
uint8_t first_flag=0;
float temperature1=0.0;
float temperature2=0.0;
float max_temp=0.0;
float temperatureMax; // target Temprature
float offset=0.0;
float pretemperature=0.0;
double per=0.00;

void setup() {
  // degug Initialize(SerialMonitor)
  Serial.begin(9600);
  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc
  // LCD initialize
  lcd.begin(20, 4);
  // button initialize
  pinMode(StartButton, INPUT_PULLUP);
  // Temprature initialize
  Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);
  Serial.print("Initializing sensor...");
  if (!thermocouple1.begin()&&!thermocouple2.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }
  Serial.println("DONE.");
  // memory initialize
  state = 0;

  //interrupt_stop
  cli();
  /*pwm setup*/
  pinMode(PWM,OUTPUT);
  /*resistor_initialize*/
  TCCR1A=0; 
  TCCR1B=0;
  TIMSK1=0;
  /*resisitor_setup*/
  TCCR1A|=(1<<COM1A1)|(1<<WGM11);
  TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS12);
  ICR1=PWM_COMPARE_TOP;
  OCR1A=DUTY_SETUP;
  TIMSK1|=(1<<ICIE1);
  sei();
}

void loop() {
  do{
    temperature1=tempratureRead(cs1);
  }while(temperature1<5.00);
  do{
    temperature2=tempratureRead(cs2)-COMP;
  }while(temperature2<5.00);
  switch (state) {
    case 0: // initialize
      lcd.clear();
      heatMode = 0;
      temperatureMax = ProfileTemperatureFirstStep;
      tableCounter = 0;
      state++;
      break;
    case 1: // start switch wait
      if (digitalRead(StartButton) == LOW) {
        tone(TonePin,600,800);  // StartSound
        lcd.clear();
        setTempratureData();
        state++;
      }
      break;
    case 2: // target Temperature
      if (temperatureMax <= temperature1) {
        state++;
      }
      break;
    case 3: // keep time
      if (--temperatureWait <= 0) {
        state++;
      }
      break;
    case 4: // Loop or Finish?
      tableCounter++;
      setTempratureData();
      if (tableCounter < ControlDataLen) {
        state = 2;
      } else {
        tone(TonePin,600,1500);  // FinishSound
        state++;
      }
      break;
    case 5: // finish switch wait
      if (digitalRead(StartButton) == LOW) {
        state = 0;
      }
      break;
  }

  max_temp=temperature1>max_temp?temperature1:max_temp;

  heatControl(temperature1);
  lcdDisplay(temperature1,temperature2);
  Serial.println(int(temperature1),DEC);
  Serial.println(int(temperature2),DEC);
  Serial.println(duty_target_value,DEC);
  Serial.flush();

  // max temperature caliculate
  //lcdDisplay(max_temp);
  //Serial.print(temperature1,DEC);
  //Serial.print("\t");
  //Serial.println(temperature2,DEC);  

  delay(delayWait);
}

void setTempratureData() {
  heatMode = temperature_control_data[tableCounter][0];
  temperatureMax = temperature_control_data[tableCounter][1];
  temperatureWait = temperature_control_data[tableCounter][2] * oneSec;
  heatState = heatMode;
}

float tempratureRead(uint8_t number) {
  unsigned int thermocouple;
  //unsigned int internal;
  unsigned int error=0;
  //float disp;
  //float temp;
  // read tem
  if(number==TemperatureSlavePin1){
    thermocouple=thermocouple1.readCelsius();
    error=thermocouple1.readError();
  }else if(number==TemperatureSlavePin2){
    thermocouple=thermocouple2.readCelsius();
    error=thermocouple2.readError();
  }else{
    thermocouple=0.0;
  }
  if ((error & 0x0001) != 0) {
    Serial.print("ERROR: ");
    if ((error & 0x0004) !=0) {
      Serial.print("Short to Vcc, ");
    }
    if ((error & 0x0002) !=0) {
      Serial.print("Short to GND, ");
    }
    if ((error & 0x0001) !=0) {
      Serial.print("Open Circuit, ");
    }    
    Serial.println();
  }
  return(thermocouple);
}

void heatControl(float temperature) {
  if (temperature > temperatureMax) {
    heatState = 0;
  } else if (temperature < (temperatureMax - 0.5)) {
    heatState = heatMode;
  }
}

void lcdDisplay(float temperature1,float temperature2) {
  lcd.setCursor(3, 0);
  lcd.print("STATUS:");

  per=duty_target_value/PWM_COMPARE_TOP;
  per*=100.0;

  switch (state) {
    case 0: // initialize
    case 1: // start switch wait
      lcd.print("-------");
      lcd.setCursor(1, 1);
      if (blinkFlag == true) {
        lcd.print("press START button");
      } else {
        lcd.print("                  ");
      }
      lcd.setCursor(3, 3);
      lcd.print("IGARASHI Labo");//SWITCH SCIENCE
      break;
    case 2: // target Temperature
    case 3: // keep time
    case 4: // Loop or Finish?
    case 5: // finish switch wait
      if (state != 5) {
        if (blinkFlag == true) {
          lcd.print("RUNNING");
        } else {
          lcd.print("       ");
        }
      } else {
        lcd.print("FINISH!");
      }
      lcd.setCursor(0, 1);
      if ((heatState & 1) == 0) {
        //lcd.print("HEAT1:OFF  ");
        lcd.print(" PWM:OFF           ");
      } else {
        //lcd.print("HEAT1:ON   ");
        lcd.print(" PWM:ON duty=");
        lcd.print(per,DEC);
        lcd.setCursor(18,1);
        lcd.print("%   ");
      }
      lcd.setCursor(0, 1);
      if ((heatState & 2) == 0) {
        //lcd.print("HEAT2:OFF");
        lcd.print(" PWM:OFF           ");
      } else {
        //lcd.print("HEAT2:ON ");
        lcd.print(" PWM:ON duty=");
        lcd.print(per,DEC);
        lcd.setCursor(18,1);
        lcd.print("%   ");
      }
      lcd.setCursor(2, 3);
      lcd.print("WAIT:");
      if (state == 3) {
        lcd.print(temperatureWait / oneSec);
        lcd.print(".");
        lcd.print(temperatureWait % oneSec * 10 / oneSec);
        lcd.print("sec");
      } else {
        lcd.print("---.-  ");
      }
      lcd.print("  ");
      break;
  }
  lcd.setCursor(0, 2);
  if (temperature1 < 100.0) lcd.print(" ");
  if (temperature1 < 10.0) lcd.print(" ");
  lcd.print(temperature1);
  lcd.print("/");
  lcd.print(temperature2);
  lcd.print("/");
  lcd.print(temperatureMax);
  lcd.print("");
  // blink control
  if (++blinkTimer >= oneSec) {
    blinkTimer = 0;
    if (blinkFlag == false) {
      blinkFlag = true;
    } else {
      blinkFlag = false;
    }
  }
}



//loop every 1sec
ISR(TIMER1_CAPT_vect){
  /*code_timer interrupt*/
  duty_target_value=reflow.Feedbuck_cal_TwoDegreesOfFreedomControl(temperature1);
  OCR1A=duty_target_value;  //max=62499
  /*
  Serial.print(duty_target_value);
  Serial.print("\t");
  Serial.println(OCR1A);
  */
  /*
  lcd.setCursor(0,1);
  lcd.print(OCR1A);
  */
}