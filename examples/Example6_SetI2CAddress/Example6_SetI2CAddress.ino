/*
  Reading the one and two axis flex sensors from Bend Labs
  By: Nathan Seidle @ SparkFun Electronics
  Date: March 2nd, 2019
  License: This code is public domain but you buy me a beer if you use this
  and we meet someday (Beerware license).

  This example shows how to change the I2C address of the sensor.

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

void setup()
{
  Serial.begin(9600);
  Serial.println(F("SparkFun Displacement Sensor Example"));

  Wire.begin();

  //Scan bus looking for a sensor
  byte currentAddress;
  for (currentAddress = 1 ; currentAddress < 127 ; currentAddress++)
  {
    currentAddress = findI2CDevice(currentAddress); //Start scanning at last address
    if(currentAddress == 0) break; //No device found!
    if (myFlexSensor.begin(currentAddress) == true) break; //Device found!
  }

  if (currentAddress == 0 || currentAddress == 127)
  {
    Serial.println("No Flex Sensors found on the I2C bus. Freezing...");
    while (1);
  }

  //Begin communication with sensorr at current address
  if (myFlexSensor.begin(currentAddress) == true)
  {
    Serial.print("Flex Sensor found at address 0x");
    Serial.print(currentAddress, HEX);
    Serial.print(" / ");
    Serial.print(currentAddress); //Print decimal
    Serial.println("(decimal)");

    byte newAddress = 0;
    while (1)
    {
      while (Serial.available()) Serial.read(); //Trash any incoming chars
      Serial.println("Enter the address you'd like to change to in decimal. Valid is 8 to 119.");
      while (Serial.available() == false) ; //Wait for user to send character

      newAddress = Serial.parseInt(); //Get decimal address from user
      if (newAddress >= 8 && newAddress <= 119) break; //Address is valid
      Serial.println("Invalid address. Please try again.");
    }

    myFlexSensor.setAddress(newAddress); //Change I2C address of this device
    //Valid addresses are 0x08 to 0x77 - 111 possible addresses!
    //Device's I2C address is stored to memory and loaded on each power-on

    delay(100); //Time required for device to record address to EEPROM and re-init its I2C

    if (myFlexSensor.begin(newAddress) == true)
    {
      Serial.print("Address successfully changed to 0x");
      Serial.print(newAddress, HEX);
      Serial.print(" / ");
      Serial.print(newAddress); //Print decimal
      Serial.println("(decimal)");
      Serial.print("Now load another example sketch using .begin(0x");
      Serial.print(newAddress, HEX);
      Serial.println(") to use this Flex Sensor");
      Serial.println("Freezing...");
      while (1);
    }
  }

  //Something went wrong, begin scanning I2C bus for valid addresses
  Serial.println("Address change failed. Beginning an I2C scan.");
}

void loop()
{
  Serial.println("Scanning...");

  byte found = 0;
  for (byte address = 1 ; address < 127 ; address++)
  {
    address = findI2CDevice(address); //Scans bus starting from given address. Returns address of discovered device.

    if (address > 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 0x0F) Serial.print("0"); //Pretty print
      Serial.print(address, HEX);
      Serial.print(" / ");
      Serial.print(address); //Print decimal
      Serial.println("(decimal)");

      found++;
    }
    else
    {
      if (found == 0) Serial.println("No I2C devices found\n");
      break; //Done searching
    }
  }

  delay(5000);
}

//Scans the ICC bus looking for devices
//Start scanning from a given address
byte findI2CDevice(byte startingAddress)
{
  if (startingAddress == 0) startingAddress = 1; //Error check

  for (byte address = startingAddress; address < 127; address++)
  {
    Wire.beginTransmission(address);
    byte response = Wire.endTransmission();

    if (response == 0) //A device acknowledged us at this address!
      return (address);
  }

  return (0); //No device found
}
