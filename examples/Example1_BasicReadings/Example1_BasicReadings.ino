/*
  Reading the one and two axis flex sensors from Bend Labs
  By: Nathan Seidle @ SparkFun Electronics
  Date: March 2nd, 2019
  License: This code is public domain but you buy me a beer if you use this 
  and we meet someday (Beerware license).

  SparkFun labored with love to create this code. Feel like supporting open
  source? Buy a board from SparkFun!
  https://www.sparkfun.com/products/15242

  This example reads the flex values of the single or 
  dual axis angular bend sensor (ADS)

  TODO - example showing multiple sensors at same time
*/

#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor

ADS myFlexSensor; //Create instance of the ADS class

byte deviceType; //Keeps track of if this sensor is a one axis of two axis sensor

void setup() {
  Serial.begin(9600);
  Serial.println("SparkFun Displacement Sensor Example");

  Wire.begin();

  if (myFlexSensor.begin() == false)
  {
    Serial.println(F("No sensor detected. Check wiring. Freezing..."));
    while (1);
  }

  //Identify if this is a one axis or two axis sensor
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
}
