/*
  Library for the Bend Labs Flex Sensors (Angular Displacement Sensor - ADS)
  By: Nathan Seidle @ SparkFun Electronics
  Date: March 2nd, 2019
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14813

  This example reads the flex values of the single or dual axis bend sensor
*/

#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor

ADS myFlexSensor; //Create object of the ADS class

byte resetPin = 7;
const byte dataReadyPin = 2;

long lastTime;

unsigned long samples = 0;
byte deviceType; //Keeps track of if this sensor is a one axis of two axis sensor

void setup() {
  pinMode(dataReadyPin, INPUT);

  Serial.begin(115200);
  Serial.println(F("SparkFun Two Axis Displacement Sensor Example"));

  Wire.begin();
  Wire.setClock(400000);

  if (myFlexSensor.begin(0x13) == false) //Address, Wire port, reset pin
  {
    Serial.println(F("No sensor detected. Check wiring. Freezing..."));
    while (1);
  }

  deviceType = myFlexSensor.getDeviceType();
  if (deviceType == ADS_ONE_AXIS)
    Serial.println(F("One axis displacement sensor detected"));
  else if (deviceType == ADS_TWO_AXIS)
    Serial.println(F("Two axis displacement sensor detected"));

  int firmwareVersion = myFlexSensor.getFirmwareVersion();
  Serial.print(F("Firmware version: "));
  Serial.println(firmwareVersion);

  //reset(); //Causes sensor to do a soft reset
  //delay(50); //Wait for sensor to come back online

  //shutdown(); //Power down sensor to 50nA
  //setResetPin(7); //Useful for low power operation. Call during setup.
  //wake(); //Wake sensor, requires reset pin
  //setSampleRate(ADS_100_HZ);
  //run(); //Begin outputting data in free run mode
  //setAddress(0x50); //Change I2C address to 0x50
  myFlexSensor.enableInterrupt();
  //disableInterrupt();

  myFlexSensor.run(); //Begin sensor outputting readings
}

void loop() {

  if (digitalRead(dataReadyPin) == LOW)
    //if (millis() - lastTime > 10)
  {
    //lastTime = millis();
    if (myFlexSensor.available() == true)
    {
      samples++;
      Serial.print(samples / (millis() / 1000.0), 2);
      Serial.print("Hz");
      Serial.print(",");
      Serial.print(myFlexSensor.getX());
      if (deviceType == ADS_TWO_AXIS)
      {
        Serial.print(",");
        Serial.println(myFlexSensor.getY());
      }
    }
  }

  if (Serial.available())
  {
    byte incoming = Serial.read();

    if (incoming == 'c')
      calibrate();
    else if (incoming == 's')
      myFlexSensor.stop();
    else if (incoming == 'r')
      myFlexSensor.run();
    else if (incoming == '1')
      myFlexSensor.setSampleRate(ADS_1_HZ);
    else if (incoming == '2')
      myFlexSensor.setSampleRate(ADS_10_HZ);
    else if (incoming == '3')
      myFlexSensor.setSampleRate(ADS_100_HZ);
    else if (incoming == '4')
      myFlexSensor.setSampleRate(ADS_500_HZ);
  }
}

void calibrate()
{
  Serial.println(F("Calibration routine"));

  while (Serial.available() > 0) Serial.read(); //Flush all characters
  Serial.println(F("Press a key when the sensor is flat and straight on a table"));
  while (Serial.available() == 0) delay(10); //Wait for user to press character

  myFlexSensor.calibrateZero(); //Call when sensor is straight on both axis

  while (Serial.available() > 0) Serial.read(); //Flush all characters
  Serial.println(F("Good. Now press a key when the sensor is flat on table but bent at 90 degrees (along X axis)."));
  while (Serial.available() == 0) delay(10); //Wait for user to press character

  myFlexSensor.calibrateX(); //Call when sensor is straight on Y axis and 90 degrees on X axis

  while (Serial.available() > 0) Serial.read(); //Flush all characters
  Serial.println(F("Good. Now press a key when the sensor is straight from base but 90 degrees up from table (along Y axis)."));
  while (Serial.available() == 0) delay(10); //Wait for user to press character

  myFlexSensor.calibrateY(); //Call when sensor is straight on Y axis and 90 degrees on X axis

  Serial.println(F("Calibration complete."));

  myFlexSensor.saveCalibration(); //Commit these values to NVM. They will be loaded on next POR.
}
