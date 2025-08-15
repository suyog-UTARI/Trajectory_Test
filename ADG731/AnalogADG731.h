// Class for handling ADG731 32-Channel Switch with SPIControl
// SPI library is a dependency


#include <SPI.h>

 
class AnalogADG731
{
    uint8_t _readpin = 0;            // pin number of chip select
    //uint8_t dout = 43;
    uint8_t _mask8 = 1;             //8 bit mask for bit operations
    uint8_t _sw8_byte;             // pointer to the encoded byte
    uint32_t _SPIrate = 6000000;   //25000000;   // SPI clock speed
  
    public:
    // Initializes the object with given Chip select pin
    AnalogADG731 (uint8_t selectPin)
    {
        _readpin = selectPin;       // Store chip select pin
        pinMode(selectPin, OUTPUT);  // Set CS pin as OUTPUT
   
    }
    
   // SPI slave expects 1 byte input conveying a commanded switch position. 
   // 1 byte Packet structure:   MSB  -> | EN | CS | X | A4 | A3 | A2 | A1 | A0 |  <-  LSB
   // Refer to the Analog devices ADG731 documentation for further details


   // --------------- Turns all the switches off (none of the inputs are connected to the output)
   void switch_off()
   {
      digitalWrite(_readpin, LOW);          // select slave
      SPI.begin();  // Initializes the SPI bus by setting SCK, MOSI, and SS to outputs, pulling SCK and MOSI low, and SS high
      SPI.beginTransaction(SPISettings(_SPIrate, MSBFIRST, SPI_MODE1));   // begin SPI transaction 
      _sw8_byte = _mask8<<7;                // CSA - 1 
      float  receivedVal;
      SPI.transfer(_sw8_byte);              //command    
      digitalWrite(_readpin, HIGH);         //de-select slave
      SPI.endTransaction();                 //end SPI Transaction
   }


   // --------------- Retain previous switch condition
   void previous()
   {
      digitalWrite(_readpin, LOW);          // select slave
      SPI.begin();  // Initializes the SPI bus by setting SCK, MOSI, and SS to outputs, pulling SCK and MOSI low, and SS high
      SPI.beginTransaction(SPISettings(_SPIrate, MSBFIRST, SPI_MODE1));   // begin SPI transaction 
      _sw8_byte = _mask8<<6;                // EN - 1 
      SPI.transfer(_sw8_byte);              // command
      digitalWrite(_readpin, HIGH);         // de-select slave
      SPI.endTransaction();                 // end SPI Transaction
    }

    
  // --------------- Switch selected input pin (connect selected input to output) (sel_pin -> (1-32))
  void select(uint8_t sel_pin)
  {
    //  Serial.println(sel_pin-1);
    //  Serial.println('\n');
    digitalWrite(_readpin, LOW);      // select slave
    SPI.beginTransaction(SPISettings(_SPIrate, MSBFIRST, SPI_MODE1));   // begin SPI transaction 
    _sw8_byte = sel_pin-1;
    SPI.transfer(_sw8_byte);         // SPI.transfer(_sw8_byte); // command
    digitalWrite(_readpin, HIGH);    // de-select slave
    SPI.endTransaction();            // end SPI Transaction
  }
  
};
