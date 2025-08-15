// Class for handling NXP 33996 16 Output Switch with SPIControl
// SPI library is a dependency
#include <SPI.h> 
class NxpDemuxsw
{
    uint8_t _readpin = 0; // pin number of chip select
    uint16_t _mask16 = 1; //16 bit mask for bit operations
    uint8_t _byte1 = 0;   // first byte containing command mode information 
    uint16_t _sw16_2byte = 0; // bytes 2 and 3 of the packet which contain switch state information to be given
    byte * _sigpointer = (byte *) &_sw16_2byte;   // pointer to the encoded 2 bytes
    uint32_t _SPIrate = 6000000;   // SPI clock speed
  public:
    boolean states[16]; //states of individual pins
  
  //    Initializes the object with given Chip select pin
  NxpDemuxsw (uint8_t selectPin)
  {
    _readpin = selectPin; // Store chip select pin
    pinMode(selectPin,OUTPUT);  // Set CS pin as OUTPUT
    for (int i=0;i<16;i++)
      states[i]=false;
  }

//  Turns all the outputs off (stay high as default)
  void switch_all_off()
  {
    digitalWrite(_readpin, LOW);          //select slave
    SPI.begin();  // Initializes the SPI bus by setting SCK, MOSI, and SS to outputs, pulling SCK and MOSI low, and SS high
    SPI.beginTransaction(SPISettings(_SPIrate, MSBFIRST, SPI_MODE1));   //begin SPI transaction 
    SPI.transfer(_byte1);                 //command
    SPI.transfer(0x00);                   //first 8 outputs OFF
    SPI.transfer(0x00);                   //last 8 outputs OFF
    digitalWrite(_readpin, HIGH);         //de-select slave
    SPI.endTransaction();                 //end SPI Transaction
    for(int i=0; i<16; i++)               //Store the value of all states in public field
      states[i]=false;
  }

//  Turns all outputs on (pulls them low)
  void switch_all_on()
  {
    digitalWrite(_readpin, LOW);          //select slave
    SPI.beginTransaction(SPISettings(_SPIrate, MSBFIRST, SPI_MODE1));   //begin SPI transaction 
    SPI.transfer(_byte1);                 //command
    SPI.transfer(0xFF);                   //first 8 outputs ON
    SPI.transfer(0xFF);                   //last 8 outputs ON
    digitalWrite(_readpin, HIGH);         //de-select slave
    SPI.endTransaction();                 //end SPI Transaction
    for(int i=0; i<16; i++)               //Store the value of all states in public field
      states[i]=true;
  }

//  Turns all switches on or off according to given boolean array
  void switch_all_given(boolean (& sw_data)[16])
  {
    _sw16_2byte = 0;
    for (int i=0; i<8; i++)
    {
      _mask16 = 1;
      if (sw_data[i])
          _mask16 = _mask16<<(i+8);
      else
          _mask16 = 0;
      _sw16_2byte = _sw16_2byte|_mask16;
    }
    for (int i=8; i<16; i++)
    {
      _mask16 = 1;
      if (sw_data[i])
          _mask16 = _mask16<<(i-8);
      else
          _mask16 = 0;
      _sw16_2byte = _sw16_2byte|_mask16;
    }
    digitalWrite(_readpin, LOW);    //select slave
    SPI.beginTransaction(SPISettings(_SPIrate, MSBFIRST, SPI_MODE1));   //begin SPI transaction 
    SPI.transfer(_byte1);           //command
    SPI.transfer(_sigpointer,2);    //switch all to given
    digitalWrite(_readpin, HIGH);   //de-select slave
    SPI.endTransaction();           //end SPI Transaction
    for(int i=0; i<16; i++)         //Store the value of all states in public field
      states[i]=sw_data[i];
  }

// Toggle given pin (0-15)
  void toggle_pin(uint8_t pin){
    if (pin<16)
    {
      states[pin] = !states[pin];
      this->switch_all_given(states);
    }
  }

// Turn on given pin (0-15)
   void turn_pin_on(uint8_t pin){
    if (pin<16)
    {
      states[pin] = true;
      this->switch_all_given(states);
    }
  }

// Turn off given pin (0-15)
    void turn_pin_off(uint8_t pin){
    if (pin<16)
    {
      states[pin] = false;
      this->switch_all_given(states);
    }
  }
  
};
