#include "Arduino.h"
#include "MS5611.h"

MS5611::MS5611(uint8_t aAddr){
    i2caddr=aAddr;
}

uint8_t MS5611::sendCmd(uint8_t aCMD)
{
  Wire.beginTransmission(i2caddr);
  Wire.write(aCMD);
  return Wire.endTransmission();
}

bool MS5611::begin(int8_t sda, int8_t scl) {
  #ifdef ESP32
    if (sda!=-1 && scl!=-1) Wire.begin(sda,scl);
    else Wire.begin();
  #else
	Wire.begin();
  #endif
	Wire.beginTransmission(i2caddr);
	uint8_t ret = Wire.endTransmission();
#ifndef __AVR_ATtiny85__
  if ( ret==0 ){
    bool rprom = readPROM();
    if( rprom ){
      return true;
    }else{
      lastError = ERROR_PROM;
      return false;
    }
  }
  else {
    lastError = ERROR_I2C;
    return false;
  }
#else
  return ret;
#endif
}

void MS5611::readADC(bool async, uint8_t aCMD){
#ifndef __AVR_ATtiny85__
  if (!async){
    sendCmd(MS5611_CMD_ADC_CONV + aCMD); // start DAQ and conversion of ADC data
    uint8_t tWait=10;
    switch (aCMD & 0x0f) {
      case MS5611_CMD_ADC_256 : tWait=1;//delayMicroseconds(900);
      break;
      case MS5611_CMD_ADC_512 : tWait=3;//delay(3);
      break;
      case MS5611_CMD_ADC_1024: tWait=4;//delay(4);
      break;
      case MS5611_CMD_ADC_2048: tWait=6;//delay(6);
      break;
      case MS5611_CMD_ADC_4096: tWait=10;//delay(10);
      break;
    }
    delay(tWait);
  }
  else if (state == READY_FOR_CONVERSION){
    aCMD = MS5611_CMD_ADC_D1 + MS5611_CMD_ADC_4096;
    sendCmd(MS5611_CMD_ADC_CONV + aCMD); // start DAQ and conversion of ADC data
    state = CONVERSION1_IN_PROGRESS;
    endTime = millis() + 10;
    return;
  }
  else if (state == CONVERSION1_FINISHED){
    aCMD = MS5611_CMD_ADC_D2 + MS5611_CMD_ADC_4096;
    sendCmd(MS5611_CMD_ADC_CONV + aCMD); // start DAQ and conversion of ADC data
    state = CONVERSION2_IN_PROGRESS;
    endTime = millis() + 10;
    return;
  }
  else if ( millis() > endTime ){
    if (state == CONVERSION1_IN_PROGRESS) state = CONVERSION1_FINISHED;
    else state = CONVERSION2_FINISHED;
  }
  
#else
  sendCmd(MS5611_CMD_ADC_CONV + aCMD); // start DAQ and conversion of ADC data
  Serial.delay(10);
#endif
  if (!async || state==CONVERSION1_FINISHED || state==CONVERSION2_FINISHED) {
    sendCmd(MS5611_CMD_ADC_READ); // read out values
    Wire.requestFrom(i2caddr, (uint8_t)3);
    uint32_t adc = (uint32_t)Wire.read()<<16;
    adc += ((uint16_t)Wire.read()<<8);
    adc += Wire.read();

    if ((aCMD & 0xF0) == MS5611_CMD_ADC_D2 || state==CONVERSION2_FINISHED) {
      adc2 = adc;
    }else{
      adc1 = adc;
    }
  }
}

void MS5611::readOut(bool async, bool _debug) {
#ifndef __AVR_ATtiny85__
  if (!_debug){
    if (!async){
      readADC(false, MS5611_CMD_ADC_D1 + MS5611_CMD_ADC_4096);
      readADC(false, MS5611_CMD_ADC_D2 + MS5611_CMD_ADC_4096);
    }
    else if (state != CONVERSION2_FINISHED){
      readADC(true);
      return;
    }
  }else{
    adc1 = 8480966; adc2 = 8658780;
    C[1]=44035;C[2]=43783;C[3]=28125;C[4]=25133;C[5]=33947;C[6]=29133;C[7]=3;
  } 
	// calculate 1st order pressure and temperature (MS5611 1st order algorithm)
  double dt = adc2-(C[5]*pow(2,8));
	double off = (C[2]*pow(2,16)) + dt*C[4]/pow(2,7);
	double sens = (C[1]*pow(2,15)) + dt*C[3]/pow(2,8);
	temp = 2000+(dt*C[6]/pow(2,23));
#else
  readADC(false, MS5611_CMD_ADC_D1 + MS5611_CMD_ADC_4096);
  readADC(false, MS5611_CMD_ADC_D2 + MS5611_CMD_ADC_4096);
	double dt = adc2-C5;
	double off = C2 + dt*C4;
	double sens = C1 + dt*C3;
	temp = 2000+(dt*C6);
#endif 
	// perform higher order corrections
	if(temp<2000) {
	  double t2 = dt*dt/pow(2,31);
	  double off2 = 5*(temp-2000)*(temp-2000)/pow(2,1);
	  double sens2 = 5*(temp-2000)*(temp-2000)/pow(2,2);
	  if(temp<-1500) {
	    off2 += 7*(temp+1500)*(temp+1500);
	    sens2 += 11*(temp+1500)*(temp+1500)/pow(2,1);
	  }
    temp -= t2;
    off -= off2;
    sens -= sens2;
  }
	pressure = (((adc1*sens)/pow(2,21)-off)/pow(2,15));
  state = READY_FOR_CONVERSION;
}

bool MS5611::readPROM() {
	sendCmd(MS5611_CMD_RESET);
	delay(3);
	for(uint8_t i=0;i<8;i++) 
	{
	    sendCmd(MS5611_CMD_PROM_RD+2*i);
	    Wire.requestFrom(i2caddr, 2);
	    C[i] = (uint16_t)Wire.read() << 8;
	    C[i] += Wire.read();
	}
  return CRC4(C);
}

double MS5611::getTemp() {
	return temp/100.;
}

double MS5611::getPres() {
	return pressure;
}

void MS5611::setI2Caddr(uint8_t aAddr) {
	i2caddr=aAddr;
}

uint8_t MS5611::getI2Caddr() {
	return i2caddr;
}

uint16_t MS5611::getPROM(uint8_t i){
  if (i >= sizeof(C) ) return 0;
  return C[i];
}

uint8_t MS5611::getLastError(){
  return lastError;
}

bool MS5611::isReady(){
  return (state = READY_FOR_CONVERSION);
}


bool MS5611::scanI2C(){
  uint8_t count = 0;
  for(uint8_t address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0)
    {
      i2cAdresses[count] = address;
      count++;
    }
  }
  foundI2C = count;
  return (count != 0);
}

bool MS5611::CRCTest(){
  uint16_t prom_test[8] = {0,44462,46297,28578,26888,33690,29306,4};
  return CRC4(prom_test);
}

bool MS5611::CRC4(uint16_t prom[])
{
    uint16_t n_rem = 0;  // CRC remainder
    uint16_t crc_backup = prom[7]; // save read CRC
    prom[7] = 0xFF00 & prom[7];   // CRC byte is replaced by 0
    for ( uint8_t cnt = 0; cnt < 16; cnt++ ) {   // operation is performed on bytes
      if ( cnt % 2 == 1 ) // choose LSB or MSB
        n_rem ^= (prom[cnt >> 1] & 0x00FF);
      else
        n_rem ^= (prom[cnt >> 1] >> 8);
      for ( uint8_t n_bit = 8; n_bit > 0; n_bit-- ) {
        if ( n_rem & 0x8000 )
          n_rem = (n_rem << 1) ^ 0x3000;
        else
            n_rem <<= 1 ;
      }
    }
    n_rem = (n_rem >> 12) & 0x000F; // final 4-bit remainder is CRC code
    prom[7] = crc_backup;
    return (n_rem == (prom[7] & 0x000F));
}

void MS5611::debug(){
  if (lastError==ERROR_I2C){
    Serial.println(MS5611_MESSAGE_ERROR_I2C);
    if (scanI2C()){
      Serial.print("found I2C devices :");
      for (uint8_t i=0; i<foundI2C;i++){
          Serial.print("0x");
          Serial.print(i2cAdresses[i],HEX);
          Serial.print(",");
      }
      Serial.println();
    }else{
      Serial.println("found no I2C devices");
    }
  }
  else if(lastError==ERROR_PROM){
    Serial.println(MS5611_MESSAGE_ERROR_PROM);
  } 

  // printing PROM
  Serial.print("PROM(C0->C8):");
  for (uint8_t i=0;i<7;i++){
    Serial.print(C[i]);Serial.print(",");
  }
  Serial.println(C[7]);

}