#include "Arduino.h"
#include "TFT_eSPI.h"
#include <Wire.h>
#include <SPI.h>
#include "NxpDemuxsw.h"
#include "AnalogADG731.h"
#include "TCA9548A.h"
#include <Adafruit_PWMServoDriver.h>
#include "esp_adc_cal.h"
#include "AbpSensor.h"
#include "NotoSansBold15.h"
#include "NotoSansMonoSCB20.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

float value1 = 0, value2 = 0;
int done_1 = 0, done_2 = 0;
int state   = 1;
int Actuator1_P, Actuator2_P;
byte Readcommand[10];
float Pressure[10];
int i = 0, j = 0;                 // current pair
bool upwardFlag = true;           // current direction

int Actuator1_Off = 0, Actuator2_Off = 0;
bool currentUpward = true;

#define NUM_SENSORS 10
#define FILTER_SIZE 5
float pressure_buffer[NUM_SENSORS][FILTER_SIZE] = {0};
int   buffer_index[NUM_SENSORS] = {0};

#define MC33996_CS1   16
#define ADG731_CS1    11
#define PRESSURE_SEN1 13

TFT_eSPI lcd = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&lcd);

TCA9548A I2CMux_J1(0x73);
NxpDemuxsw MC33996_J1  ( MC33996_CS1 );
AnalogADG731 ADG731_J1 ( ADG731_CS1  );
Adafruit_PWMServoDriver Pro_pwm_J1 = Adafruit_PWMServoDriver(0x41);

// ===============================================================
// NEW: 20×20 pair generator
// ===============================================================
bool nextPair20x20(int &row, int &col, bool &upward)
{
    static bool firstCall = true;
    if (firstCall) {
        row = 0; col = 0; upward = true;
        firstCall = false;
        return true;
    }

    if (upward) {                       // upward sweep
        if (col < 19) { col++; return true; }
        upward = false;                 // reached top, start downward
        return true;
    } else {                            // downward sweep
        if (col > 0) { col--; return true; }
        // reached bottom of column
        pressure_Release();             // <<< RELEASE when column done
        if (row < 19) {
            row++; col = 0; upward = true;
            return true;
        }
        return false;                   // finished final column
    }
}

// ===============================================================
void pressure_Release() {
    MC33996_J1.turn_pin_on(7);
    MC33996_J1.turn_pin_on(8);
    MC33996_J1.turn_pin_on(12);
    delay(6000);
    MC33996_J1.turn_pin_off(7);
    MC33996_J1.turn_pin_off(8);
    MC33996_J1.turn_pin_off(12);
    state = 1;
}
// ===============================================================

void TaskReadSensors(void *pvParameters) {
  for (;;) {
    float sensor10 = Single_Pre_Read_J1(10);
    float sensor9  = Single_Pre_Read_J1(8);
    Serial.printf("%lu, Sensor10: %.2f | Sensor9: %.2f | (%0.f ,%0.f) %d%d\n",
                  millis(), sensor10, sensor9, value1, value2, done_1, done_2);
    Pressure[9] = sensor10;
    Pressure[8] = sensor9;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void TaskControl(void *pvParameters) {
  for (;;) {
    if (state == 1) {
      done_1 = done_2 = 0;
      MC33996_J1.turn_pin_off(6); MC33996_J1.turn_pin_off(13);
      MC33996_J1.turn_pin_off(12);MC33996_J1.turn_pin_off(7);
      MC33996_J1.turn_pin_off(8); MC33996_J1.turn_pin_off(0);
      MC33996_J1.turn_pin_off(1);

      Command_Detection();

      Actuator1_P = (Pressure[9] <= value1);
      Actuator2_P = (Pressure[8] <= value2);
    }
    else if (state == 0) {
      pressure_Control();
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void Command_Detection(){
    delay(500);
    if (state == 1) {
        if (!nextPair20x20(i, j, upwardFlag)) {
            Serial.println("Finished 20x20 pattern.");
            return;
        }
        value1 = i;
        value2 = j;
        currentUpward = upwardFlag;   // for pressure_Control()
        state = 0;
    }
}

// ---------------- pressure_Control() unchanged -----------------
void pressure_Control()
{
    float sensor10 = Pressure[9];
    float sensor9  = Pressure[8];

    if ((value1 >= 19) && (value2 >= 19)) {
        MC33996_J1.turn_pin_on(7);
        MC33996_J1.turn_pin_on(8);
        MC33996_J1.turn_pin_on(12);
        for(;;){}  // emergency stop
    }

    if (!Actuator1_Off) {
        if (currentUpward && sensor10 >  value1) done_1 = 5;
        if (!currentUpward && sensor10 <= value1) done_1 = 5;
    }
    if (!Actuator2_Off) {
        if (currentUpward && sensor9  >  value2) done_2 = 5;
        if (!currentUpward && sensor9  <= value2) done_2 = 5;
    }

    if (!Actuator1_Off) {
        if (currentUpward) {
            MC33996_J1.turn_pin_on(6);
            if (sensor10 <= value1) MC33996_J1.turn_pin_on(7);
            else { MC33996_J1.turn_pin_off(7); done_1++; }
        } else {
            MC33996_J1.turn_pin_on(13);
            if (sensor10 > value1) MC33996_J1.turn_pin_on(0);
            else { MC33996_J1.turn_pin_off(0); done_1++; }
        }
    }

    if (!Actuator2_Off) {
        if (currentUpward) {
            MC33996_J1.turn_pin_on(6);
            if (sensor9 <= value2) MC33996_J1.turn_pin_on(8);
            else { MC33996_J1.turn_pin_off(8); done_2++; }
        } else {
            MC33996_J1.turn_pin_on(13);
            if (sensor9 > value2) MC33996_J1.turn_pin_on(1);
            else { MC33996_J1.turn_pin_off(1); done_2++; }
        }
    }

    if ((done_1 >= 5) && (done_2 >= 5)) state = 1;
}

// ---------------- helper functions (unchanged) -----------------
float Single_Pre_Read_J1(int sensor){
    float Volt = 0;
    ADG731_J1.select(sensor);
    int ADC_Reading = analogRead(PRESSURE_SEN1);
    int Cal_read = readADC_Cal(ADC_Reading);
    Volt = Cal_read * 0.0012207 * 1.506;
    float Sensor_read = (Volt - 0.5) * 15.25;
    if (Sensor_read < 0) Sensor_read = 0;
    int idx = sensor - 1;
    pressure_buffer[idx][buffer_index[idx]] = Sensor_read;
    buffer_index[idx] = (buffer_index[idx] + 1) % FILTER_SIZE;
    float sum = 0; for (int k=0;k<FILTER_SIZE;k++) sum += pressure_buffer[idx][k];
    return sum / FILTER_SIZE;
}

uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11,
                           ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars);
}

// -------------------- setup & loop -----------------------------
void setup() {
    Serial.begin(115200);
    Wire.begin(17,18);
    SPI.begin(21, -1, 12, -1);
    pinMode(15, OUTPUT); digitalWrite(15, 1);

    Pro_pwm_J1.begin();
    Pro_pwm_J1.setPWMFreq(1600);
    Pro_pwm_J1.setPWM(0, 0, 4095);
    Pro_pwm_J1.setPWM(1, 0, 4095);
    Pro_pwm_J1.setPWM(2, 0, 4095);

    lcd.init(); lcd.fillScreen(TFT_BLACK); lcd.setRotation(3);
    sprite.createSprite(320, 170);
    sprite.setTextDatum(3);
    sprite.setSwapBytes(true);

    xTaskCreatePinnedToCore(TaskReadSensors,"ReadSensors",4096,NULL,1,NULL,1);
    xTaskCreatePinnedToCore(TaskControl,"Control",8192,NULL,2,NULL,1);
}

void loop() { }
