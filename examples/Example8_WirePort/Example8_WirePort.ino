/*
  Reading the one and two axis flex sensors from Bend Labs
  By: Nathan Seidle @ SparkFun Electronics
  Date: March 2nd, 2019
  License: This code is public domain but you buy me a beer if you use this
  and we meet someday (Beerware license).

  This example shows how pass a different Wire port into the library.
  For example, some platforms have Wire1, Wire2, etc. You can
  pass this into the library.

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

  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor

ADS myFlexSensor; //Create object of the ADS class

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println(F("SparkFun Displacement Sensor Example"));

  Wire1.begin(); //Compilation will fail here if your platform (ie, the Arduino Uno)
  //does not support multiple Wire ports. Use a Teensy or other platform instead.

  Wire1.setClock(400000); //Increase I2C clock rate to 400kHz

  //Begin communication with sensor on Wire1 port
  if (myFlexSensor.begin(19, Wire1) == false)
  {
    Serial.println(F("Sensor not detected. Check wiring. Freezing..."));
    while (1)
      ;
  }
}

void loop()
{
  if (myFlexSensor.available() == true)
  {
    Serial.print("1,");
    Serial.print(myFlexSensor.getX());
    Serial.print(",");
    Serial.print(myFlexSensor.getY());
    Serial.println();
  }
}
