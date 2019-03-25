/*
  This is an Arduino library written for the Bend Labs One Axis
  and Two Axis Angular Displacement Sensor (ADS).
  By Nathan Seidle @ SparkFun Electronics, March 2nd, 2019

  The command set and a smattering of functions were taken from the HAL written by BendLabs.
  https://github.com/bendlabs
  If you are using a different platform than Arduino or want a more advanced
  interface checkout the Hardware Abstraction Layer they provide.

  The Bend Labs Sensors are very precise, very flexible, flex sensors
  providing their angle of flex with two decimals of accuracy and
  an update rate up to 500Hz.

  https://github.com/sparkfun/SparkFun_ADS_Arduino_Library

  SparkFun labored with love to create this code. Feel like supporting open
  source? Buy a product from SparkFun!
  https://www.sparkfun.com/products/14690

  Note: While it is possible to disable the X axis or the Y axis on the
  dual-axis sensor, this library does not support it.
*/

#ifndef SparkFun_Displacement_Sensor_Arduino_Library_h
#define SparkFun_Displacement_Sensor_Arduino_Library_h

#include "Arduino.h"
#include <Wire.h>

//Command set for ADS
typedef enum
{
  ADS_RUN = 0,
  ADS_SPS,
  ADS_RESET,
  ADS_DFU,
  ADS_SET_ADDRESS,
  ADS_INTERRUPT_ENABLE,
  ADS_GET_FW_VER,
  ADS_CALIBRATE,
  ADS_AXES_ENABLED,
  ADS_SHUTDOWN,
  ADS_GET_DEV_ID
} ADS_COMMAND_T;

//Identifier for packet received from ADS
typedef enum
{
  ADS_SAMPLE = 0,
  ADS_FW_VER,
  ADS_DEV_ID
} ADS_PACKET_T;

//Calibration registers
typedef enum
{
  ADS_CALIBRATE_FIRST = 0,
  ADS_CALIBRATE_FLAT,
  ADS_CALIBRATE_PERP,
  ADS_CALIBRATE_CLEAR
} ADS_CALIBRATION_STEP_T;

//There are two types of ADS - single axis and two axis
typedef enum
{
  ADS_ONE_AXIS = 1,
  ADS_TWO_AXIS = 2,
} ADS_DEV_IDS_T;

//Available output rates
typedef enum
{
  ADS_1_HZ = 16384,
  ADS_10_HZ = 1638,
  ADS_20_HZ = 819,
  ADS_50_HZ = 327,
  ADS_100_HZ = 163,
  ADS_200_HZ = 81,
  ADS_333_HZ = 49,
  ADS_500_HZ = 32,
} ADS_SPS_T;

class ADS
{
public:
  ADS();                                                              //Default constructor
  bool begin(uint8_t deviceAddress = 0x13, TwoWire &wirePort = Wire); //Check communication and initialize sensor
  bool isConnected();                                                 //Returns true if device acks at the I2C address

  uint16_t getFirmwareVersion(); //Get the firmware version of this device

  bool available(); //Checks to see if new data is available. Called regularly to update getX and getY functions
  float getX();     //Return a reading from the sensor. Check .available() before calling this
  float getY();     //Return a reading from the sensor. Check .available() before calling this

  bool calibrateZero();   //Call when sensor is straight on both axis
  bool calibrateX();      //Call when sensor is straight on Y axis and 90 degrees on X axis. The X axis is moveable when the sensor is lying on a table.
  bool calibrateY();      //Call when sensor is straight on Y axis and 90 degrees on X axis. Y axis is moved when you pull sensor up from table.
  bool saveCalibration(); //Commit the current valibration values to non-volatile memory (NVM). They will be loaded on next power up.

  uint8_t getDeviceType(void);         //Returns the number of axis of the sensor attached to (one or two)
  bool reset();                        //Send command to initiate soft reset
  bool setSampleRate(uint16_t sps);    //Set samples per second
  bool setAddress(uint8_t newAddress); //Change I2C address of device

  bool run();                         //Set sensor to constantly output readings
  bool stop();                        //Tell device to stop reading
  bool shutdown(void);                //Shutdown ADS. Requires reset to wake up from Shutdown. ~50nA in shutdwon
  bool wake(void);                    //Wakes up ADS from shutdown using hardware reset pin. Takes 100ms.
  void setResetPin(uint8_t pinToUse); //Assign the pin number to use for wake()

  void hardwareReset(void); //Reset the Angular Displacement Sensor using the pin defined by setResetPin()

  bool enableInterrupt();  //Enables the data ready interrupt
  bool disableInterrupt(); //Disables the data ready pin

private:
  TwoWire *_i2cPort;               //This stores the requested i2c port
  uint8_t _deviceAddress = 0x13;   //Unshifted 7-bit default address of the ADS is 0x13
  volatile float currentSample[2]; //This is where the calculated angular values are stored pre and post filtering

  uint8_t _adsResetPin = 0; //Optional pin connections to sensor
  bool inFreeRun = false;
  uint8_t axisAmount; //Tracks one or two axis sensor type. Set in .begin().

  bool readBuffer(uint8_t *buffer, uint8_t len);  //Read a number of bytes (5) from ADS
  bool writeBuffer(uint8_t *buffer, uint8_t len); //Write a number of bytes (5) to ADS

  uint8_t readDeviceType(void);       //Reads the axis of the device attached to
  void parseSamples(uint8_t *buffer); //Convert bytes floats
  void processNewData();              //Takes the data from the latest sample and loads it into the filters
  void signalFilter(float *sample);   // Low pass IIR filter
  void deadzoneFilter(float *sample); // Deadzone filter

  bool beginReadingData(bool run);                               //Places ADS in free run or sleep mode
  bool setDataReadyInterrupt(bool enable);                       //Enables the ADS data ready interrupt line
  bool calibrate(uint8_t ads_calibration_step, uint8_t degrees); //Set the four calibration points: zero, right on x, right on y, save to NVM

  inline int16_t ads_int16_decode(const uint8_t *p_encoded_data); //Convert two bytes of buffer[] to int16
};
#endif
