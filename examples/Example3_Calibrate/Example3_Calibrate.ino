/*
  Reading the one and two axis flex sensors from Bend Labs
  By: Nathan Seidle @ SparkFun Electronics
  Date: March 2nd, 2019
  License: This code is public domain but you buy me a beer if you use this
  and we meet someday (Beerware license).

  This example shows how to calibrate a sensor. Open a terminal and press 'c'
  to calibrate.

  Note, the orientation on the
  2-axis sensor can be confusing. With the sensor flat on a table, the x axis
  moves across the surface of the table, the y axis moves up from the table.

  SparkFun labored with love to create this code. Feel like supporting open
  source? Buy a sensor from SparkFun!
  https://www.sparkfun.com/products/15245 (2-axis sensor)
  https://www.sparkfun.com/products/15244 (1-axis sensor)

  Hardware Connections:
  Use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  to connect to the RedBoard Qwiic and the following pins on the ADS:
  SCL: Yellow wire on Qwiic cable
  SDA: Blue
  VCC: Red
  GND: Black

  Single axis pinout: https://cdn.sparkfun.com/assets/9/f/8/2/d/Bendlabs_Single_Axis_Flex_Sensor_Pinout.png
  Dual axis pintout: https://cdn.sparkfun.com/assets/f/f/9/e/6/Bendlabs_Dual_Axis_Flex_Sensor_Pinout.png

  Open the serial monitor at 9600 baud to see the output
*/

#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor

ADS myFlexSensor; //Create object of the ADS class

byte deviceType; //Keeps track of if this sensor is a one axis of two axis sensor

void setup() {
  Serial.begin(9600);
  Serial.println(F("SparkFun Displacement Sensor Example"));

  Wire.begin();

  if (myFlexSensor.begin() == false)
  {
    Serial.println(F("No sensor detected. Check wiring. Freezing..."));
    while (1);
  }

  deviceType = myFlexSensor.getDeviceType();
  if (deviceType == ADS_ONE_AXIS)
    Serial.println(F("One axis displacement sensor detected"));
  else if (deviceType == ADS_TWO_AXIS)
    Serial.println(F("Two axis displacement sensor detected"));
}

void loop() {

  if (myFlexSensor.available() == true)
  {
    Serial.print(myFlexSensor.getX());
    if (deviceType == ADS_TWO_AXIS)
    {
      Serial.print(",");
      Serial.println(myFlexSensor.getY());
    }
  }

  if (Serial.available())
  {
    byte incoming = Serial.read();

    if (incoming == 'c')
      calibrate();
  }

  delay(10);
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
}
