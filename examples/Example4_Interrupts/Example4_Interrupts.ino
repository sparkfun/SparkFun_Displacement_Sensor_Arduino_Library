/*
  Reading the one and two axis flex sensors from Bend Labs
  By: Nathan Seidle @ SparkFun Electronics
  Date: March 2nd, 2019
  License: This code is public domain but you buy me a beer if you use this
  and we meet someday (Beerware license).

  This example shows how to read from the device when the dataready
  pin goes low. Can be used as an interrupt vector as well.

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
  You'll also need an extra wire running from the 'nDRDY' pin on sensor to pin 2 on Uno

  Single axis pinout: https://cdn.sparkfun.com/assets/9/f/8/2/d/Bendlabs_Single_Axis_Flex_Sensor_Pinout.png
  Dual axis pintout: https://cdn.sparkfun.com/assets/f/f/9/e/6/Bendlabs_Dual_Axis_Flex_Sensor_Pinout.png

  Open the serial monitor at 9600 baud to see the output*/

#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor

ADS myFlexSensor; //Create object of the ADS class

const byte dataReadyPin = 4;  //This can be any pin, but avoid pin 2. See: https://learn.sparkfun.com/tutorials/sparkfun-pro-nrf52840-mini-hookup-guide 
// "Because pin 2's state at reset is sampled by the bootloader, be careful using it with any component that may pull the pin low on startup."

long lastTime;

unsigned long samples = 0; //Allows us to calculate the actual read rate in Hz
byte deviceType; //Keeps track of if this sensor is a one axis of two axis sensor

void setup() {
  pinMode(dataReadyPin, INPUT);

  Serial.begin(9600);
  while(!Serial);
  Serial.println(F("SparkFun Displacement Sensor Example"));

  Wire.begin();
  Wire.setClock(400000); //Note this sensor supports 400kHz I2C

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

  myFlexSensor.enableInterrupt(); //Tell sensor to pull DRDY pin low when new data is available
  //myFlexSensor.disableInterrupt(); //We can also disable this if needed

  myFlexSensor.run(); //Begin sensor outputting readings
}

void loop() {

  if (digitalRead(dataReadyPin) == LOW)
  {
    if (myFlexSensor.available() == true) //We still need to call .available because it loads the X and Y variables
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
}
