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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


float value1 = 0, value2 =0;
int done_1 = 0, done_2 =0;
int state = 1;
int Actuator1_P, Actuator2_P;   // Variable to determine if the actuator need to be pressurized or deflated 0 is to be deflation and 1 is inflation.
byte  Readcommand[10]; 
int Therapy_time = 0, Therapy_cycle = 0;
float Glove_Set_Pressure = 1, Wrist_Set_Pressure = 1, Elbow_Set_Pressure = 1 ;
float Degree = 20; // Initial degree: 20, will be replaced by 40 in the loop.
float Pressure[10];
int Flex_angel[5] = {10,20,30,40,110};
unsigned long start_time;
int count = 0;


#define NUM_SENSORS 10
#define FILTER_SIZE 5

float pressure_buffer[NUM_SENSORS][FILTER_SIZE] = {0};
int buffer_index[NUM_SENSORS] = {0};


#define MC33996_CS1   16
#define MC33996_CS2   14 
#define MC33996_CS3   10
#define ADG731_CS1    11
#define ADG731_CS2    4 
#define ADG731_CS3    3
#define PRESSURE_SEN1 13
#define PRESSURE_SEN2 2
#define PRESSURE_SEN3 1


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
AnalogADG731 ADG731_J1 ( ADG731_CS1  ); //ADG731 at daughter board1

Adafruit_PWMServoDriver Pro_pwm_J1 = Adafruit_PWMServoDriver( 0x41 ); // Jumper setting on the board1


void TaskReadSensors(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    float sensor10 = Single_Pre_Read_J1(10);
    float sensor9  = Single_Pre_Read_J1(9);
    Serial.print(millis());
    Serial.print(',');
    Serial.print("Sensor10: ");
    Serial.print(sensor10, 2);
    Serial.print("  |  Sensor9: ");
    Serial.println(sensor9, 2);

    // also update global Pressure[] if needed
    Pressure[9] = sensor10;
    Pressure[8] = sensor9;

    vTaskDelay(pdMS_TO_TICKS(200)); // read every 200 ms
  }
}

void TaskControl(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    if (state == 1) {
      done_1 = 0;
      done_2 = 0;
      MC33996_J1.turn_pin_off(6);
      MC33996_J1.turn_pin_off(13);
      MC33996_J1.turn_pin_off(7);
      MC33996_J1.turn_pin_off(8);
      MC33996_J1.turn_pin_off(0);
      MC33996_J1.turn_pin_off(1);

      Command_Detection();

      if (Pressure[9] <= value1) Actuator1_P = 1;
      else Actuator1_P = 0;

      if (Pressure[8] <= value2) Actuator2_P = 1;
      else Actuator2_P = 0;

      Post_Sensors();
      LCD_Display();

    } else if (state == 0) {
      Post_Sensors();
      LCD_Display();
      pressure_Control();
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // control loop every 50 ms
  }
}

void pressure_Control() {

    float sensor10 = Pressure[9];
    float sensor9  = Pressure[8];

    if ((sensor10 >= 20) || (sensor9 >= 20)) {
        state = 1;   // Emergency stop
    }

    if (Actuator1_P) {
        MC33996_J1.turn_pin_on(6); // pressure pump on
        if (sensor10 <= value1) {
            MC33996_J1.turn_pin_on(7);
        }
        if (sensor10 > value1) {
            MC33996_J1.turn_pin_off(7);
            done_1 = done_1 + 1;
        }
    }

    if (!Actuator1_P) {
        MC33996_J1.turn_pin_on(13); // Vaccum pump on
        if (sensor10 > value1) {
            MC33996_J1.turn_pin_on(0);
        }
        if (sensor10 <= value1) {
            MC33996_J1.turn_pin_off(0);
            done_1 = done_1 + 1;
        }
    }

    if (Actuator2_P) {
        MC33996_J1.turn_pin_on(6); // pressure pump on
        if (sensor9 <= value2) {
            MC33996_J1.turn_pin_on(8);
        }
        if (sensor9 > value2) {
            MC33996_J1.turn_pin_off(8);
            done_2 = done_2 + 1;
        }
    }

    if (!Actuator2_P) {
        MC33996_J1.turn_pin_on(13); // Vaccum pump on
        if (sensor9 > value2) {
            MC33996_J1.turn_pin_on(1);
        }
        if (sensor9 <= value2) {
            MC33996_J1.turn_pin_off(1);
            done_2 = done_2 + 1;
        }
    }

    if ((done_1 >= 5) && (done_2 >= 5)) {
        state = 1;
    }
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
    for (int i = 10; i > 8; i--)
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
        ADG731_J1.select( sensor ); //Array starting from 0
        int ADC_Reading =  analogRead( PRESSURE_SEN1 );
        int Cal_read = readADC_Cal(ADC_Reading);
        Volt = Cal_read * 0.0012207 * 1.506; // 0.0012207 = 5/4096, 1.53 is compensation so sensor readout matches MCU reading
        // ana_read = 0.02439 * ( analogRead( PRESSURE_SEN1 ) - sensor_bias[ sensor - 6] );
        //Sensor_read = ( Volt - 0.5 ) * 14.5 * 1.142;//ABPBANN004, 4Bar=58PSI
        Sensor_read = (Volt - 0.48)*15.25;
        //Sensor_read = ((Volt - 0.58)*16.9) + 0.3;
         if ( Sensor_read < 0 ) { Sensor_read = 0; }
    int idx = sensor - 1; // sensors are 1-indexed
    pressure_buffer[idx][buffer_index[idx]] = Sensor_read;
    buffer_index[idx] = (buffer_index[idx] + 1) % FILTER_SIZE;

    // Calculate average
    float sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += pressure_buffer[idx][i];
    }
    return sum / FILTER_SIZE;
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

void Command_Detection(){
    String serialReceived;
    if (Serial.available() > 0)
    {
        serialReceived = Serial.readString();
    }
    serialReceived.trim();

    if (serialReceived.length() >= 1) {
        int space1 = serialReceived.indexOf(' ');
        value1 = serialReceived.substring(0, space1).toFloat();
        value2 = serialReceived.substring(space1 + 1).toFloat();
   state = 0;
}
}

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
    Pro_pwm_J1.setPWM(2, 0, 4095);

    lcd.init(); // LCD setting
    lcd.fillScreen(TFT_BLACK);
    lcd.setRotation(3);
    
    sprite.createSprite(320, 170);
    sprite.setTextDatum(3);
    sprite.setSwapBytes(true);

    // FreeRTOS tasks
    xTaskCreatePinnedToCore(TaskReadSensors, "ReadSensors", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(TaskControl, "Control", 8192, NULL, 2, NULL, 1);

    }
  

void loop() {
    
}
