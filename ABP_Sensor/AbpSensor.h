// Class for accessing Honeywell ABP series pressure sensors with SPI
// SPI library is a dependency
#include <SPI.h> 
class AbpSensor
{
  private:
    uint8_t _readpin = 0; // pin number of chip select on arduino or pin number (0-15) on the NXP 33996
    int _inByte_1, _inByte_2, _inByte_3;  // incoming bytes in each packet 
    int _OUTPUT_MIN = 1638;       // 1638 counts (10% of 2^14 counts or 0x3999)
    int _OUTPUT_MAX = 14745;      // 14745 counts (90% of 2^14 counts or 0x3999)
    int _PRESSURE_MIN = 0;        // min is 0 for sensors that give absolute values
    //float  _PRESSURE_MAX = 60.0;  // 60 kpa (I want results in kpa)
    float  _PRESSURE_MAX = 10.0;  // 60 kpa (I want results in psi)
    boolean _mux_def = false;     // flag for indicating whether the chipselect is an output of NXP 33996
    uint32_t _SPIrate = 800000;   // SPI clock speed
    NxpDemuxsw* _muxpointer;
  public:
    float pressure = -100.0;    // default values which indicate SPI malfunction
    float temperature = -100.0; // default values which indicate SPI malfunction

//    Initializes the object with given Chip select pin connected to one of the digital outputs of Arduino
    AbpSensor (uint8_t selectPin)
    {
      pinMode(selectPin,OUTPUT);
      _readpin = selectPin;
    }
    
//    Initializes the object with given pointer to NxpDemuxsw object and corresponding pin number(0-15)
    AbpSensor (NxpDemuxsw* muxpointer,uint8_t selectPin)
    {
      _muxpointer = muxpointer;
      _readpin = selectPin;
      _mux_def = true;
    }

//    Reads and returns float pressure in kPa
    float read_pressurekPa()
    {
      if(_mux_def){
        _muxpointer->switch_all_off();
        _muxpointer->toggle_pin(_readpin);
      }
      else
      {
        digitalWrite(_readpin, LOW);       //pull Chipselect Pin to Low
      } 
      SPI.begin();    // Initializes the SPI bus by setting SCK, MOSI, and SS to outputs, pulling SCK and MOSI low, and SS high
      SPI.beginTransaction(SPISettings(_SPIrate, MSBFIRST, SPI_MODE0));   //begin SPI transaction 
      
      _inByte_1 = SPI.transfer(0x00);  // Read first Byte of Pressure
      _inByte_2 = SPI.transfer(0x00);  // Read second Byte of Pressure
      // if(_mux_def){
      //   _muxpointer->switch_all_off();
      //   Serial.println("YEs");
      // }
      // else
      // {
      digitalWrite(_readpin, HIGH);      //pull Chipselect Pin to High
      
      SPI.endTransaction();               //end SPI Transaction
      int16_t pressure_dig = _inByte_1 << 8 | _inByte_2; // assemble the pressure data
      //convert to kPa
      float pressure=1.0 * ((float)pressure_dig - _OUTPUT_MIN) * (_PRESSURE_MAX - _PRESSURE_MIN) / (_OUTPUT_MAX - _OUTPUT_MIN) + _PRESSURE_MIN;
      return pressure; //return Pressure Value in kPa
    }

//    Reads and returns float temperature in C
    float read_tempC()
    {
      SPI.begin();
      SPI.beginTransaction(SPISettings(_SPIrate, MSBFIRST, SPI_MODE1)); 
      digitalWrite(_readpin, LOW);       //pull Chipselect Pin to Low
      SPI.transfer(0x00);  // Read first Byte of Pressure
      SPI.transfer(0x00);  // Read second Byte of Pressure
      _inByte_3 = SPI.transfer(0x00);  // Read first Byte of Temperature
      SPI.transfer(0x00);  // Read second Byte of Temperature

      digitalWrite(_readpin, HIGH);      //pull Chipselect Pin to High
      SPI.endTransaction();               //end SPI Transaction

      _inByte_3 = _inByte_3 << 3; //Shift first Temperature byte 3 left

      float temperature = ((float)_inByte_3 * 200 / 2047) - 50; //Convert Digital value to °C
      return temperature; //return digital Pressure Value
    }

//  Reads and returns an array of pressure in kPa and temperature in C to the given pointer location
    float* read_pressurekPa_tempC(float (& pressure_temp_data) [2])
    {
      SPI.begin();
      SPI.beginTransaction(SPISettings(_SPIrate, MSBFIRST, SPI_MODE0)); 
      digitalWrite(_readpin, LOW);       //pull Chipselect Pin to Low
      _inByte_1 = SPI.transfer(0x00);  // Read first Byte of Pressure
      _inByte_2 = SPI.transfer(0x00);  // Read second Byte of Pressure
      _inByte_3 = SPI.transfer(0x00);  // Read first Byte of Temperature
      SPI.transfer(0x00);  // Read second Byte of Temperature
      digitalWrite(_readpin, HIGH);      //pull Chipselect Pin to High
      SPI.endTransaction();               //end SPI Transaction
      int16_t pressure_dig = _inByte_1 << 8 | _inByte_2; // assemble the pressure data
      _inByte_3 = _inByte_3 << 3; //Shift first Temperature byte 3 left
      //convert to kPa
      pressure_temp_data[0]=1.0 * ((float)pressure_dig- _OUTPUT_MIN) * (_PRESSURE_MAX - _PRESSURE_MIN) / (_OUTPUT_MAX - _OUTPUT_MIN) + _PRESSURE_MIN;
      pressure_temp_data[1]= ((float)_inByte_3 * 200 / 2047) - 50; //Convert Digital value to °C
      pressure = pressure_temp_data[0];
      temperature = pressure_temp_data[1];
      return (float*) &pressure_temp_data; //return array of pressure and temperature
    }
};
