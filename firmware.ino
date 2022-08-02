#include <Ticker.h>
#include <PID_v1.h>
#include <thermistor.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

const uint8_t d4 = 10;
const uint8_t d5 = 11;
const uint8_t d6 = 16;
const uint8_t d7 = 17;
const uint8_t rs = A3;
const uint8_t e = A2;
const uint8_t buttons = 25;

#define EPPROM_ADDR 0

const uint8_t temp_sensor = 31;
const uint8_t heater = 13;
const uint8_t stepper_step = 22;//12
const uint8_t stepper_dir = 23;
const uint8_t stepper_enable = 14;
double Kp=21, Ki=1.25, Kd=86;
//double Kp=2, Ki=5, Kd=1;
double Setpoint, Input, Output;
double filter_k = 0.5;
uint8_t targetTemp = 200;
uint8_t motorSpeed = 1;
uint8_t filamentLen = 0;
int8_t menuIndex = 0;
bool motorEnable = false;
bool heaterEnable = true;

char line1[21] = "                    ", line2[21] = "                    ", line3[21] = "                    ", line4[21] = "                    "; 
char menuSelect = '<';

struct Settings {
  uint8_t motorSpeed;
  uint8_t targetTemp;
};

void updateLcd();
void updateBtn();
void handlePID();
void ok();

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
thermistor therm1(temp_sensor, 5);
LiquidCrystal lcd ( rs , e , d4 , d5 , d6 , d7 );
Ticker lcdUpdate(updateLcd, 1000, 0, MILLIS);
Ticker buttonsUpdate(updateBtn, 200, 0, MILLIS);
Ticker pidUpdate(handlePID, 100, 0, MILLIS);
Ticker okTimer(ok, 1500, 0, MILLIS);

double expRunningAverage(float newVal) {
  static double filVal = 0;
  filVal += (newVal - filVal) * filter_k;
  return filVal;
}

void ok(){
  menuSelect = '<';
  okTimer.stop();
}

void stepper(){
  tone(stepper_step, motorSpeed * 1000);
}
  

void updateBtn(){
  int adc = analogRead(buttons);
  if(adc <= 100){//back
    switch(menuIndex){
        case 0:
          targetTemp--;
          Setpoint = targetTemp;
          //Serial.println(Setpoint);
        break;
        case 1:
          motorSpeed--;
          if (motorEnable) stepper();
        break;
        }
    }else if(adc >= 160 && adc <= 190){//down
      menuIndex++;
      //Serial.println("menuIndex++");
      if (menuIndex > 3) menuIndex = 3;
      if (menuIndex == 2) menuIndex = 3;
    }else if(adc >= 305 && adc <= 345){//menu
      if (menuIndex == 3){//settings
        Settings settings = {
          motorSpeed,
          targetTemp
          };
          EEPROM.put(EPPROM_ADDR, settings);
          menuSelect = '+';
          okTimer.start();
        }else if(menuIndex == 1){//motor
          motorEnable = !motorEnable;
          if (motorEnable){
            digitalWrite(stepper_enable, LOW);
            stepper();
            }else{
              noTone(stepper_step);
              digitalWrite(stepper_enable, HIGH);
              }
          }else if(menuIndex == 0){//heater
            heaterEnable = !heaterEnable;
              if (!heaterEnable){
                analogWrite(heater, 0);
              }
            }
    }else if(adc >= 489 && adc <= 529){//enter
      switch(menuIndex){
        case 0:
          targetTemp++;
          Setpoint = targetTemp;
        break;
        case 1:
          motorSpeed++;
          if (motorEnable) stepper();
         
        break;
        }
    }else if(adc >= 680 && adc <= 720){//up
      menuIndex--;
      //Serial.println("menuIndex--");
      if (menuIndex < 0) menuIndex = 0;
      if (menuIndex == 2) menuIndex = 1;
    }
}

void updateLcd(){
  char tempStr[5];
  char speedStr[3];
  //char filament[3];
  dtostrf(Input, 5, 1, tempStr);
  sprintf(line1, "Temp: %s/%d     ", tempStr, targetTemp); // %6s right pads the strin
  //dtostrf(filamentLen,3,0,filament);
  //sprintf(line3, "Filament len: %s   ", filament);
  sprintf(line3, "--------------------");
  dtostrf(motorSpeed,3,0,speedStr);
  sprintf(line2, "Speed: %s          ", speedStr);
  sprintf(line4, "Save settings       ");
  switch (menuIndex) {
  case 0:
    line1[19] = menuSelect;
    break;
  case 1:
     line2[19] = menuSelect;
    break;
    case 2:
     line3[19] = menuSelect;
    break;
    case 3:
     line4[19] = menuSelect;
    break;
}
  if(motorEnable){
    line2[16] = 'O';
    line2[17] = 'N';
    }
  if(heaterEnable){
    line1[16] = 'O';
    line1[17] = 'N';
  }
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
  lcd.setCursor(0, 2);
  lcd.print(line3);
  lcd.setCursor(0, 3);
  lcd.print(line4);
}

void handlePID(){
 Input = expRunningAverage(therm1.analog2temp());
 if(heaterEnable){
  myPID.Compute();
  analogWrite(heater, Output);
 }
  }

void setup() {
  lcd.begin( 20 , 4 );
  lcd.setCursor(0, 0);
  lcd.print("init");
  Settings settings;
  EEPROM.get( EPPROM_ADDR, settings );
  motorSpeed = settings.motorSpeed;
  targetTemp = settings.targetTemp;
  pinMode(heater, OUTPUT);
  pinMode(stepper_step, OUTPUT);
  pinMode(stepper_dir, OUTPUT);
  pinMode(stepper_enable, OUTPUT);
  pinMode(buttons, INPUT);
  pinMode(temp_sensor, INPUT);
   //Serial.begin(9600);
   //Serial.println("start");
   Input = therm1.analog2temp();
   Setpoint = targetTemp;
   myPID.SetMode(AUTOMATIC);
   digitalWrite(stepper_enable, HIGH);
   lcdUpdate.start();
   buttonsUpdate.start();
   pidUpdate.start();
}

void loop() {
  pidUpdate.update();
  buttonsUpdate.update();
  lcdUpdate.update();
  okTimer.update();
}
