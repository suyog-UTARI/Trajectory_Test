#include "Arduino.h"
#include "TFT_eSPI.h"/* Please use the TFT library provided in the library. */
#include <Wire.h>
#include <SPI.h>
#include "NxpDemuxsw.h"     // MC33996 SPI 
#include "AnalogADG731.h"   //Analog 16 in 1 out selector
#include "TCA9548A.h"       //I2C multiplex
#include <Adafruit_PWMServoDriver.h>  // PCA9685 PWM Servo Driver Library
#include "esp_adc_cal.h"    // For ESP32 accurate ADC reading calibration
#include "AbpSensor.h"      // SPI Pressure sensor
#include "NotoSansBold15.h" // Font for LCD display
#include "NotoSansMonoSCB20.h" // Font for LCD display
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
// TCA9548A I2CMux_J1(0x73);
// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;

#define MC33996_CS1   16
#define MC33996_CS2   14 
#define MC33996_CS3   10
#define ADG731_CS1    11
#define ADG731_CS2    4 
#define ADG731_CS3    3
#define PRESSURE_SEN1 13
#define PRESSURE_SEN2 2
#define PRESSURE_SEN3 1

byte  Readcommand[10]; 
int Therapy_time = 0, Therapy_cycle = 0;
float Glove_Set_Pressure = 1, Wrist_Set_Pressure = 1, Elbow_Set_Pressure = 1 ;
float Degree = 20; // Initial degree: 20, will be replaced by 40 in the loop.
float Pressure[10];
int Flex_angel[5] = {10,20,30,40,110};
unsigned long start_time;
int count = 0;

TFT_eSPI lcd = TFT_eSPI(); // LCD setting
TFT_eSprite sprite = TFT_eSprite(&lcd);
 
#define gray   0x6B6D // Color setting for LCD
#define blue   0x0967
#define orange 0xC260
#define purple 0x604D
#define green  0x1AE9
#define yellow 0xF7E0

TCA9548A I2CMux_J1(0x73);

NxpDemuxsw MC33996_J1  ( MC33996_CS1 ); //MC33996 at daughter board1
NxpDemuxsw MC33996_J2  ( MC33996_CS2 ); //MC33996 at daughter board2
// NxpDemuxsw MC33996_J3  ( MC33996_CS3 ); //MC33996 at daughter board3

AnalogADG731 ADG731_J1 ( ADG731_CS1  ); //ADG731 at daughter board1
AnalogADG731 ADG731_J2 ( ADG731_CS2  ); //ADG731 at daughter board2
// AnalogADG731 ADG731_J3 ( ADG731_CS3  ); //ADG731 at daughter board3

Adafruit_PWMServoDriver Pro_pwm_J1 = Adafruit_PWMServoDriver( 0x41 ); // Jumper setting on the board1
//Adafruit_PWMServoDriver Pro_pwm_J2 = Adafruit_PWMServoDriver( 0x42 ); // Jumper setting on the board2
double secp;
double secv;
int controlDuration;
char commandChar;
int noOfCycles;
int numberOfTrials; 
int timeBetweenTrials; // Time between trials in milliseconds


void handleIncomingData(String data);
void Command_Detection(char commandChar);
void parseControlStrings(String data);
void Glove_Flex();
void Glove_Ext();
void Glove_Cycle();
void Wrist_Cycle();
void Elbow_Cycle();
void Glove_Wrist();
void Wrist_Elbow();
void Vac_Def_ON();
void Glove_Wrist_Elbow();
void Pre_Sensor_Read();
uint32_t readADC_Cal(int ADC_Raw);
void Command_Detection();
void Post_Sensors();
void Air_Valve_Test();
void Flex_Sensor_Test();
void Pre_Pump_ON();
void Pre_Pump_OFF();
void Pre_Def_OFF();
void Pre_Def_ON();
void Vac_Pump_ON();
void Vac_Pump_OFF();
void Pro_Valve_Test();
void All_Pro_OFF();
void All_OFF();
void Glove_Flex_Read_Left();
void Glove_Flex_Read_Right();
float Single_Pre_Read_J1(int sensor);
void LCD_Display();
void Deflation();
void tcaSelect(uint8_t i);

void setup()
{
    Serial.begin(115200);
    Serial.print("Serial begin");

    Wire.begin( 17, 18); // Diff with Veysel's design, cannot copy
    SPI.begin(21, -1, 12, -1); //SCLK, MISO, MOSI, SS

    pinMode(15, OUTPUT); // to boot with DEV board
    digitalWrite(15, 1);  // and/or power from 5v rail instead of USB

    Pro_pwm_J1.begin();
    Pro_pwm_J1.setPWMFreq(1600);  // Set PWM Freq

    Pro_pwm_J1.setPWM(0, 0, 4095);//ON
    Pro_pwm_J1.setPWM(1, 0, 4095);

    lcd.init(); // LCD setting
    lcd.fillScreen(TFT_BLACK);
    lcd.setRotation(3);
    
    sprite.createSprite(320, 170);
    sprite.setTextDatum(3);
    sprite.setSwapBytes(true);
    }
  

void loop() {
    Post_Sensors();
    LCD_Display();
    pressure_Control();
}

void pressure_Control(){
  int ready_cell = 0;
    //All Fingers
    MC33996_J1.turn_pin_on(6); //pressure pump on
    for (int i = 0; i < 2; i++)
    {
        MC33996_J1.turn_pin_on(i);
        MC33996_J1.turn_pin_off(i+7);
        Pro_pwm_J1.setPWM(i, 0, 4095);
    }

    while (ready_cell!=2) {

        for(int i = 0; i < 2; i++) {

            if(Single_Pre_Read_J1(i+6)>5) {
                MC33996_J1.turn_pin_off(i);
               MC33996_J1.turn_pin_on(0);
                Pro_pwm_J1.setPWM(i, 4095, 0);
                ready_cell++;
            }
        }
    }
   MC33996_J1.turn_pin_off(6);
}

void vacuum_Control(){
  int ready_cell1 = 0;
    //All Fingers
    Vac_Pump_ON();  
    for (int i = 0; i < 5; i++)
    {
        MC33996_J1.turn_pin_on(i+7);
        MC33996_J1.turn_pin_off(i);
        Pro_pwm_J1.setPWM(i, 0, 4095);
    }

    while (ready_cell1!=5) {

        for(int i = 0; i < 5; i++) {

            if(Single_Pre_Read_J1(i+6)<0.1) {
                MC33996_J1.turn_pin_off(i+7);
                MC33996_J1.turn_pin_on(11);
                Pro_pwm_J1.setPWM(i, 4095, 0);
                Pro_pwm_J1.setPWM(4, 0, 4095);
                ready_cell1++;
            }
        }
    }
    delay(2000);
   Vac_Pump_OFF(); 
}


void Post_Sensors(){
    Pre_Sensor_Read();
}


void Pre_Pump_ON(){
    MC33996_J1.turn_pin_on(6);
}

void Pre_Pump_OFF(){
    MC33996_J1.turn_pin_off(6);
}

void Vac_Def_ON(){
    MC33996_J1.turn_pin_on(12);
}

void Vac_Def_OFF(){
    MC33996_J1.turn_pin_off(12);
}

void Vac_Pump_ON(){
    MC33996_J1.turn_pin_on(13);
}

void Vac_Pump_OFF(){
    MC33996_J1.turn_pin_off(13);
}

void Pre_Sensor_Read(){
    Serial.print(millis());
    Serial.print(',');
    for (int i = 10; i > 0; i--)
    {   
        Pressure[ i - 1 ] = Single_Pre_Read_J1( i ); // 10-1 save to 9-0 in array
        Serial.print(Pressure[ i - 1 ]); // Convert to PSI
        Serial.print(',');
    }
    Serial.println("zz");
}

float Single_Pre_Read_J1(int sensor){
    float Sensor_read = 0;
    float Volt = 0;
    if ( sensor > 5 ){ // 6 - 10 pressure sensors
        ADG731_J1.select( sensor ); //Array starting from 0
        int ADC_Reading =  analogRead( PRESSURE_SEN1 );
        int Cal_read = readADC_Cal(ADC_Reading);
        Volt = Cal_read * 0.0012207 * 1.506; // 0.0012207 = 5/4096, 1.53 is compensation so sensor readout matches MCU reading
        // ana_read = 0.02439 * ( analogRead( PRESSURE_SEN1 ) - sensor_bias[ sensor - 6] );
        Sensor_read = ( Volt - 0.5 ) * 14.5 * 1.142;//ABPBANN004, 4Bar=58PSI
        if ( Sensor_read < 0 ) { Sensor_read = 0; }
    }
    else{ // 1 - 5 Vac sensors
        ADG731_J1.select( sensor ); //Array starting from 0
        // ana_read = 0.005882 * ( Vac_bias[ sensor - 1] - analogRead( PRESSURE_SEN1 )  );
        int ADC_Reading = analogRead( PRESSURE_SEN1 );
        int Cal_read = readADC_Cal(ADC_Reading);
        Volt = Cal_read * 0.0012207 * 1.6752; // 0.0012207 = 5/4096, 1.53 is compensation so sensor readout matches MCU reading
        Sensor_read = 18.9543 * ( Volt * 0.2 - 0.92 ) - 0.2; // Vac sensor read, mmHg tp PSI
        if ( Sensor_read > 0 ) { Sensor_read = 0; }
    }
    return abs( Sensor_read );
}
  

uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}  

void LCD_Display(){
    int fromTop = 15;
    int fromLeft = 15;
    int w = 320; // LCD: 320x170 resolution
    int h = 170;

    sprite.fillSprite( TFT_BLACK ); //Create each small colored area
    sprite.loadFont(NotoSansBold15);
    sprite.drawString("Upper-body Exoskeleton Controller V1.0", 10, 15);

    for (int i = 0; i < 5; i++) { // Show 5 finger degree
        sprite.drawString(String( i + 1), 35 + 60 * i, 38);
        sprite.fillSmoothRoundRect(fromLeft + (i * 60), fromTop + 30, 50, 50, 6, gray, TFT_BLACK);
        sprite.drawSmoothArc(fromLeft + (i * 60) + 25, fromTop  + 60, 20, 22, 45, 315, 0x21C9, gray);
        sprite.drawSmoothArc(fromLeft + (i * 60) + 25, fromTop  + 60, 20, 22, 45, map(Flex_angel[i], 0, 120, 46, 315), yellow, gray);
        sprite.drawString(String(Flex_angel[i]), fromLeft + (i * 60) + 15, fromTop + 70);

    }

    for ( int i = 0; i < 5; i++ ) { // Show 5 finger press, pre and vac
        sprite.fillSmoothRoundRect(fromLeft + (i * 60), fromTop + 60 + 30, 50, 50, 6, gray, TFT_BLACK);
        
        if( Pressure[ 9 - i ] - Pressure[ 4 - i ] > 0.1 ){
            sprite.fillSmoothRoundRect(fromLeft + (i * 60), fromTop + 60 + 30, 50, 50, 6, orange, TFT_BLACK);
            sprite.drawSmoothArc(fromLeft + (i * 60) + 25, fromTop  + 60 + 60, 20, 22, 45, 315, 0x21C9, gray);
            sprite.drawSmoothArc(fromLeft + (i * 60) + 25, fromTop  + 60 + 60, 20, 22, 45, map(Pressure[9 - i], 0, 100, 46, 315), yellow, gray);
            sprite.drawString(String(Pressure[9 - i]), fromLeft + (i * 60) + 10, fromTop + 120);
        }
        if( Pressure[ 9 - i ] - Pressure[ 4 - i ] < -0.1 ){
            sprite.fillSmoothRoundRect(fromLeft + (i * 60), fromTop + 60 + 30, 50, 50, 6, purple, TFT_BLACK);
            sprite.drawSmoothArc(fromLeft + (i * 60) + 25, fromTop  + 60 + 60, 20, 22, 45, 315, 0x21C9, gray);
            sprite.drawSmoothArc(fromLeft + (i * 60) + 25, fromTop  + 60 + 60, 20, 22, 45, map(Pressure[4 - i], 0, 100, 46, 315), yellow, gray);
            sprite.drawString(String(Pressure[4 - i]), fromLeft + (i * 60) + 10, fromTop + 120);
        }
    }
    sprite.pushSprite(0,0);
}

