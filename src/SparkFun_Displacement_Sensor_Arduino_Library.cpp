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

#include "SparkFun_Displacement_Sensor_Arduino_Library.h"

#define ADS_TRANSFER_SIZE 5 //All communication with sensor is done in 5 byte frames

//Constructor
ADS::ADS()
{
}

bool ADS::begin(uint8_t deviceAddress, TwoWire i2cPort)
{
  //Get user's options
  _i2cPort = i2cPort;
  _deviceAddress = deviceAddress;

  if (isConnected() == false)
    return (false);

  reset();   //Issue command to do a software reset
  delay(50); //Wait for device to come back online.

  axisAmount = readDeviceType(); //Set global sensor axis number

  setSampleRate(ADS_100_HZ);

  run(); //Set sensor to output data continuously

  return (true); //All done!
}

//Returns true if device acknowledges its address
bool ADS::isConnected()
{
  _i2cPort->beginTransmission(_deviceAddress);
  if (_i2cPort->endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);    //All good
}

//Returns the type of sensor (either one or two axis)
uint8_t ADS::getDeviceType(void)
{
  return (axisAmount);
}

/**
   @brief Returns the device type. ADS should not be in free run
          when this function is called.

   @return  ADS_ONE_AXIS, ADS_TWO_AXIS or zero if no device is detected
*/
uint8_t ADS::readDeviceType(void)
{
  if (inFreeRun == true)
    stop(); //Don't check device ID while in free run mode

  uint8_t buffer[ADS_TRANSFER_SIZE];

  //The sensor may respond with sample packets before it responds to GET_DEV_ID request
  for (uint8_t counter = 0; counter < 100; counter++)
  {
    buffer[0] = ADS_GET_DEV_ID;

    writeBuffer(buffer, ADS_TRANSFER_SIZE);
    delay(2);
    readBuffer(buffer, ADS_TRANSFER_SIZE);

    //Check that packet is a device id packet
    if (buffer[0] == ADS_DEV_ID)
    {
      return buffer[1];
    }
  }
  return false;
}

//Send command to initiate soft reset
bool ADS::reset()
{
  uint8_t buffer[ADS_TRANSFER_SIZE];

  buffer[0] = ADS_RESET;

  return writeBuffer(buffer, ADS_TRANSFER_SIZE);
}

/**
   @brief Sets the sample rate of the ADS in free run mode

   @param  sample rate
   @return  True if successful, false if failed
*/
bool ADS::setSampleRate(uint16_t sps)
{
  uint8_t buffer[ADS_TRANSFER_SIZE];

  buffer[0] = ADS_SPS;

  //SPS is loaded little endian
  buffer[1] = (uint8_t)((sps & 0x00FF) >> 0);
  buffer[2] = (uint8_t)((sps & 0xFF00) >> 8);

  return writeBuffer(buffer, ADS_TRANSFER_SIZE);
}

/*
   @brief Updates the I2C address of the selected ADS. The default address
        is 0x13. Use this function to program an ADS to allow multiple
        devices on the same I2C bus.

   @param device  device number of the device that is being updated
   @param address new address of the ADS
   @return  True if successful false if failed
*/
bool ADS::setAddress(uint8_t newAddress)
{
  if (newAddress < 0x08 || newAddress > 0x77)
    return (false); //Address is out of bounds

  uint8_t buffer[ADS_TRANSFER_SIZE];

  buffer[0] = ADS_SET_ADDRESS;
  buffer[1] = newAddress;

  if (writeBuffer(buffer, ADS_TRANSFER_SIZE) == false)
    return (false);

  _deviceAddress = newAddress; //Update address only after success

  return (true);
}

//Set sensor to constantly output readings
bool ADS::run()
{
  inFreeRun = true;
  return (beginReadingData(true));
}

//Tell device to stop reading
bool ADS::stop()
{
  inFreeRun = false;
  return (beginReadingData(false));
}

/*
   @brief Places ADS in free run or sleep mode

   @param  run true if activating ADS, false is putting in suspend mode
   @return  ADS_OK if successful ADS_ERR_IO if failed
*/
bool ADS::beginReadingData(bool run)
{
  uint8_t buffer[ADS_TRANSFER_SIZE];

  buffer[0] = ADS_RUN;
  buffer[1] = run;

  return writeBuffer(buffer, ADS_TRANSFER_SIZE);
}

//Checks to see if new data is available
//This must be called regularly to update getX and getY functions
//Returns true if a new sample is received
//The sample is then parsed and run through the filters
bool ADS::available()
{
  uint8_t buffer[ADS_TRANSFER_SIZE];

  if (readBuffer(buffer, ADS_TRANSFER_SIZE) == true)
  {
    if (buffer[0] == ADS_SAMPLE) //Verify this is a data sample report
    {
      parseSamples(buffer); //Pull bytes from buffer and turn them into floats

      processNewData(); //Run new samples through the deadband and signal filters
      //This assigns good, filtered readings to currentSamples 0 and 1

      return (true);
    }
  }
  return (false);
}

//Return a reading from the sensor
//Check .available() before calling this
//The currentSample is calculated when available() is called and receives new data
float ADS::getX()
{
  return (currentSample[0]);
}

//Return a reading from the sensor
//Check .available() before calling this
//The currentSample is calculated when available() is called and receives new data
float ADS::getY()
{
  return (currentSample[1]);
}

/**
   @brief Parses sample buffer from two axis ADS. Scales to degrees and
          executes callback registered in ads_two_axis_init.
*/
void ADS::parseSamples(uint8_t *buffer)
{

  int16_t temp = ads_int16_decode(&buffer[1]);
  currentSample[0] = (float)temp / 32.0f;

  temp = ads_int16_decode(&buffer[3]);
  currentSample[1] = (float)temp / 32.0f;
}

/**@brief Function for decoding a int16 value.

   @param[in]   p_encoded_data   Buffer where the encoded data is stored.
   @return      Decoded value.
*/
inline int16_t ADS::ads_int16_decode(const uint8_t *p_encoded_data)
{
  return ((((uint16_t)(p_encoded_data)[0])) |
          (((int16_t)(p_encoded_data)[1]) << 8));
}

/**
   @brief Shutdown ADS. Requires reset to wake up from Shutdown. ~50nA in shutdwon

   @return  True if successful false if failed
*/
bool ADS::shutdown(void)
{
  uint8_t buffer[ADS_TRANSFER_SIZE];

  buffer[0] = ADS_SHUTDOWN;

  return writeBuffer(buffer, ADS_TRANSFER_SIZE);
}

/**
   @brief Wakes up ADS from shutdown. Delay is necessary for ADS to reinitialize

   @return  ADS_OK if successful ADS_ERR_IO if failed
*/
bool ADS::wake(void)
{
  // Reset ADS to wake from shutdown
  hardwareReset();

  // Allow time for ADS to reinitialize
  delay(100); //100 ms

  return true;
}

//Assign the private var to the user's choice
void ADS::setResetPin(uint8_t pinToUse)
{
  _adsResetPin = pinToUse;
}

/**
   @brief Reset the Angular Displacement Sensor
*/
void ADS::hardwareReset(void)
{
  if (_adsResetPin == 0)
    return;

  // Configure reset line as an output
  pinMode(_adsResetPin, OUTPUT);

  digitalWrite(_adsResetPin, LOW);
  delay(10);
  digitalWrite(_adsResetPin, HIGH);

  pinMode(_adsResetPin, INPUT_PULLUP);
}

//Enables the data ready interrupt
bool ADS::enableInterrupt()
{
  return (setDataReadyInterrupt(true));
}

//Disables the data ready pin
bool ADS::disableInterrupt()
{
  return (setDataReadyInterrupt(false));
}

/**
   @brief Enables the ADS data ready interrupt line

   @param  run true if activating ADS, false is putting in suspend mode
   @return  True if successful false if failed
*/
bool ADS::setDataReadyInterrupt(bool enable)
{
  uint8_t buffer[ADS_TRANSFER_SIZE];

  buffer[0] = ADS_INTERRUPT_ENABLE;
  buffer[1] = enable;

  return writeBuffer(buffer, ADS_TRANSFER_SIZE);
}

//Call when sensor is straight on both axis
bool ADS::calibrateZero()
{
  calibrate(ADS_CALIBRATE_FIRST, 0);
}

//Call when sensor is straight on Y axis and 90 degrees on X axis
//The X axis is moveable when the sensor is lying on a table. Y axis is moved when you pull sensor up from table.
bool ADS::calibrateX()
{
  calibrate(ADS_CALIBRATE_FLAT, 90);
}

//Call when sensor is straight on Y axis and 90 degrees on X axis
//Y axis is moved when you pull sensor up from table. The X axis is moveable when the sensor is lying on a table.
bool ADS::calibrateY()
{
  calibrate(ADS_CALIBRATE_PERP, 90);
}

//Commit the current valibration values to non-volatile memory (NVM). They will be loaded on next power up.
bool ADS::saveCalibration()
{
  calibrate(ADS_CALIBRATE_CLEAR, 0);
}

/**
   @brief Calibrates two axis ADS. ADS_CALIBRATE_FIRST must be at 0 degrees on both AXES.
          ADS_CALIBRATE_FLAT can be at 45 - 255 degrees, recommended 90 degrees.
          When calibrating the flat axis the perpendicular axis should be at 0 degrees.
          ADS_CALIBRATE_PERP can be at 45 - 255 degrees, recommended 90 degrees.
          When calibrating the perpendicular axis the flat axis should be at 0 degrees

          Note: The flat axis is sample[0] (axis 0) perp axis is sample[1] (axis 1)
          from ads_data_callback

   @param ads_calibration_step  ADS_CALIBRATE_STEP_T to perform
   @param degrees uint8_t angle at which sensor is bent when performing
          ADS_CALIBRATE_FLAT, and ADS_CALIBRATE_PERP
   @return  ADS_OK if successful ADS_ERR_IO or ADS_BAD_PARAM if failed
*/
bool ADS::calibrate(uint8_t ads_calibration_step, uint8_t degrees)
{
  uint8_t buffer[ADS_TRANSFER_SIZE];

  buffer[0] = ADS_CALIBRATE;
  buffer[1] = ads_calibration_step;
  buffer[2] = degrees;

  return writeBuffer(buffer, ADS_TRANSFER_SIZE);
}

//Get the firmware version of this device
uint16_t ADS::getFirmwareVersion()
{
  uint8_t buffer[ADS_TRANSFER_SIZE];

  buffer[0] = ADS_GET_FW_VER;

  writeBuffer(buffer, ADS_TRANSFER_SIZE);
  delay(2);
  readBuffer(buffer, ADS_TRANSFER_SIZE);

  if (buffer[0] == ADS_FW_VER) //Check that this is a response to our query
  {
    //Response is little endian
    uint16_t firmwareVersion = buffer[1];
    firmwareVersion |= (uint16_t)buffer[2] << 8;
    return (firmwareVersion);
  }
  return (0);
}

/**
   @brief Read buffer of data from the Angular Displacement Sensor

   @param buffer[out]  Read buffer
   @param len     Length of buffer.
   @return  True if successful, false if failed
*/
bool ADS::readBuffer(uint8_t *buffer, uint8_t len)
{
  _i2cPort->requestFrom(_deviceAddress, len);

  uint8_t i = 0;

  while (_i2cPort->available())
  {
    if (i < len)
      buffer[i++] = _i2cPort->read();
  }

  if (i == len)
    return true;
  return false; //Error
}

/**
   @brief Write buffer of data to the Angular Displacement Sensor

   @param buffer[in]  Write buffer
   @param len     Length of buffer.
   @return  True if successful false if failed
*/
bool ADS::writeBuffer(uint8_t *buffer, uint8_t len)
{
  _i2cPort->beginTransmission(_deviceAddress);
  uint8_t bytesWritten = _i2cPort->write(buffer, len);
  _i2cPort->endTransmission();

  if (bytesWritten == len)
    return true;
  return false;
}

// Low pass IIR filter
void ADS::signalFilter(float *sample)
{
  static float filter_samples[2][6];

  for (uint8_t i = 0; i < axisAmount; i++)
  {
    filter_samples[i][5] = filter_samples[i][4];
    filter_samples[i][4] = filter_samples[i][3];
    filter_samples[i][3] = (float)sample[i];
    filter_samples[i][2] = filter_samples[i][1];
    filter_samples[i][1] = filter_samples[i][0];

    // 20 Hz cutoff frequency @ 100 Hz Sample Rate
    filter_samples[i][0] = filter_samples[i][1] * (0.36952737735124147f) - 0.19581571265583314f * filter_samples[i][2] +
                           0.20657208382614792f * (filter_samples[i][3] + 2 * filter_samples[i][4] + filter_samples[i][5]);

    sample[i] = filter_samples[i][0];
  }
}

// Deadzone filter
void ADS::deadzoneFilter(float *sample)
{
  static float prev_sample[2];
  float dead_zone = 0.5f;

  for (uint8_t i = 0; i < axisAmount; i++)
  {
    if (fabs(sample[i] - prev_sample[i]) > dead_zone)
      prev_sample[i] = sample[i];
    else
      sample[i] = prev_sample[i];
  }
}

//Takes the data from the latest sample and loads it into the filters
void ADS::processNewData()
{
  // Low pass IIR filter
  signalFilter(currentSample);

  // Deadzone filter
  deadzoneFilter(currentSample);

  //currentSample 0 and 1 are now ready to be read
}
