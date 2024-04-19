#ifndef MS5611_h
#define MS5611_h

#include "Arduino.h"
#ifdef __AVR_ATtiny85__
#include <TinyWireM.h> 
#include <DigiCDC.h>
#define Serial SerialUSB
#define Wire TinyWireM
#else
#include <Wire.h>
#endif

// typical I2C-Address of chip
#define I2C_MS5611 0x77

// I2C commands of chip
#define MS5611_CMD_RESET	0x1E    // perform reset
#define MS5611_CMD_ADC_READ 0x00    // initiate read sequence
#define MS5611_CMD_ADC_CONV 0x40    // start conversion
#define MS5611_CMD_ADC_D1   0x00    // read ADC 1
#define MS5611_CMD_ADC_D2   0x10    // read ADC 2
#define MS5611_CMD_ADC_256  0x00    // set ADC oversampling ratio to 256
#define MS5611_CMD_ADC_512  0x02    // set ADC oversampling ratio to 512
#define MS5611_CMD_ADC_1024 0x04    // set ADC oversampling ratio to 1024
#define MS5611_CMD_ADC_2048 0x06    // set ADC oversampling ratio to 2048
#define MS5611_CMD_ADC_4096 0x08    // set ADC oversampling ratio to 4096
#define MS5611_CMD_PROM_RD  0xA0    // initiate readout of PROM registers

const char MS5611_MESSAGE_ERROR_I2C[] = "Error connecting MS5611...";
const char MS5611_MESSAGE_ERROR_PROM[]= "Bad read PROM";

enum error_id{
  NO_ERROR,
  ERROR_I2C,
	ERROR_PROM
};

enum state_id{
  READY_FOR_CONVERSION,
  CONVERSION1_IN_PROGRESS,
  CONVERSION1_FINISHED,
  CONVERSION2_IN_PROGRESS,
  CONVERSION2_FINISHED
};


#ifdef __AVR_ATtiny85__
#define C1 50215*pow(2,15)
#define C2 46885*pow(2,16)
#define C3 30837/pow(2,8)
#define C4 26973/pow(2,7)
#define C5 32101*pow(2,8)
#define C6 27932/pow(2,23)
#endif

class MS5611
{
  protected:
    uint16_t C[8];
    double temp;
    double pressure;
    uint8_t i2caddr;
    uint8_t lastError = NO_ERROR;
    uint32_t endTime;
    uint8_t state = READY_FOR_CONVERSION; //queue for reading adc values
    uint32_t adc1; //adc pressure conversion
    uint32_t adc2; //adc temperature conversion

    uint8_t sendCmd(uint8_t aCMD);
    void readADC(bool async = false, uint8_t aCMD = 0);
    bool readPROM();
    bool CRC4(uint16_t prom[]);
	
  public:
    MS5611(uint8_t aAddr=I2C_MS5611);
    bool begin(int8_t sda=-1, int8_t scl=-1);
    void readOut(bool async = false, bool _debug=false);
    bool isReady();

    void setI2Caddr(uint8_t aAddr);
    uint8_t getI2Caddr();
    uint16_t getPROM(uint8_t i);
    uint8_t foundI2C;
    uint8_t i2cAdresses[127];
    bool scanI2C();
    double getTemp();
    double getPres();
    uint8_t getLastError();
    bool CRCTest();
    void debug();
};

#endif
