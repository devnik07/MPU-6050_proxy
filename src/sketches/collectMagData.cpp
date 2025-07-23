#include <Arduino_LSM9DS1.h>

float mx, my, mz;

void setup() {
  Serial.begin(9600);
  while (!Serial); //wait for connection

  if (!IMU.begin())
  {
    Serial.println(F("LSM9DS1 not detected"));
    while (1);
  }
}

void loop() {
  if ( IMU.magneticFieldAvailable() )
  {
    IMU.readMagneticField(mx, my, mz);

    Serial.print(mx);  Serial.print(",");
    Serial.print(my);  Serial.print(",");
    Serial.print(mz);  Serial.println("");
  }
  delay(100);
}
