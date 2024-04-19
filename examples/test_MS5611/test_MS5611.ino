#include <MS5611.h>

MS5611 sensor;

void setup() {
  Serial.begin(115200);
  if (!sensor.begin()){
    Serial.print(sensor.getLastError());
    Serial.flush();
    sensor.scanI2C();
    for(uint8_t i = 0; sizeof(sensor.i2cAdresses); i++ ) {
      if (sensor.i2cAdresses[i]==0) break;
      Serial.print(sensor.i2cAdresses[i]);
    }
  }
}

void loop() {
  sensor.readOut();
  Serial.print(sensor.getTemp());
  Serial.print(sensor.getPres());
  delay(2000);   
}