// RubbermaidGuidanceSystem.ino
// https://github.com/gregkrsak/RubbermaidGuidanceSystem
//
// PURPOSE:
// ========
// Displays Adafruit StemmaQT LSM6DSOX 6dof IMU data across three SparkFun Qwiic Alphanumeric Displays.
// Also displays SparkFun Qwiic LIDAR Lite v4 data on a fourth SparkFun Qwiic Alphanumeric Display.
//
// HARDWARE REQUIRED:
// ==================
// (1) Arduino, with Qwiic support
// (1) SparkFun Qwiic LIDAR Lite v4
// (1) Adafruit StemmaQT LSM6DSOX 6dof IMU
// (4) SparkFun Qwiic Alphanumeric Display
//
// HARDWARE RECOMMENDED:
// =====================
// (1) *additional* Arduino, with Qwiic support (to reduce LIDAR polling lag)
// (1) SparkFun Qwiic MultiPort
// (8) SparkFun Qwiic cable - 100mm
//
// NOTES:
// ======
// - Other miscellaneous hardware may be required.
// - You may need to use the Arduino IDE library manager to add the appropriate SparkFun and Adafruit libraries.
// - Are you new to programming or electronics? Check out the SparkFun or Adafruit websites for tutorials!
//
// I2C CONFIGURATION:
// ==================
// REQUIRES: One of the display boards have only the A0 (I2C Address) jumper closed.
//           One of the display boards have only the A1 (I2C Address) jumper closed.
//           One of the display boards have both A0 and A1 (I2C Address) jumpers closed.
// AS ORIGINALLY WRITTEN: pitch:A0, roll:A1, yaw:A0+A1, lidar:none 
// RECOMMENDED: Two of the display boards have a single I2C pull-up resistor trace opened.
//
// GITHUB REPOSITORIES USED:
// =========================
// https://github.com/gregkrsak/ArduinoProtoThread
// https://github.com/sparkfun/SparkFun_LIDARLitev4_Arduino_Library
// https://github.com/sparkfun/SparkFun_Alphanumeric_Display_Arduino_Library
// https://github.com/adafruit/Adafruit_LSM6DS
//
// MIT LICENSED:
// =============
// Copyright 2022 Greg M. Krsak <greg.krsak@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or
// substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//


// ArduinoProtoThread class
#include "ArduinoProtoThread.hpp"
#include "ArduinoProtoThreadStateMachine.hpp"


// Feature flippers
#define FEATURE_DUAL_CONTROLLER false


//////////////////////////////////////////////////////////////////////////////////////////////////
// LIDAR
//////////////////////////////////////////////////////////////////////////////////////////////////
#if FEATURE_DUAL_CONTROLLER == false
#include <LIDARLite_v4LED.h>
#include <SparkFun_Alphanumeric_Display.h>

class LidarController : public ArduinoProtoThreadEventHandler
{
  public:
    LidarController()
    {
      // Constructor code here
    }
    ~LidarController() { }

    void onStart()
    {
      lidar.begin();
      display.begin();
    }
    void onRunning()
    {
      // Read LIDAR range in centimeters
      lidarRange = lidar.getDistance();
      lidarRangeString = String(lidarRange);
      lidarRangeDigitsToLeftOfDecimal = lidarRangeString.indexOf('.');
      friendlyLidarRange = lidarRangeString.substring(0, lidarRangeDigitsToLeftOfDecimal);
      // Display LIDAR range on alphanumeric display
      display.print(friendlyLidarRange);
    }
    void onKill()
    {
      return;
    }

  protected:
    // (Model) SparkFun Qwiic LIDAR Lite v4
    LIDARLite_v4LED lidar;
    float lidarRange;
    int lidarRangeDigitsToLeftOfDecimal;
    String lidarRangeString;
    String friendlyLidarRange;
    // (View) SparkFun Qwiic Alphanumeric Display
    HT16K33 display;
};

ArduinoProtoThread *lidarThread;
LidarController *lidarThreadDelegate;
#endif


//////////////////////////////////////////////////////////////////////////////////////////////////
// IMU
//////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_LSM6DSOX.h>
#include <SparkFun_Alphanumeric_Display.h>

// Math-related defines
#define RAD_TO_DEG_PER_SEC_CONVERSION_SCALAR 57.2958
// I2C-related defines
#define PITCH_DISPLAY_I2C_ADDRESS 0x71
#define ROLL_DISPLAY_I2C_ADDRESS 0x72
#define YAW_DISPLAY_I2C_ADDRESS 0x73

class ImuController : public ArduinoProtoThreadEventHandler
{
  public:
    ImuController()
    {
      // Constructor code here
    }
    ~ImuController() { }

    void onStart()
    {
      // Initialize Inertial Measurement Unit
      imu.begin_I2C();
      imu.setAccelDataRate(LSM6DS_RATE_SHUTDOWN);
      imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
      imu.setGyroDataRate(LSM6DS_RATE_26_HZ);
      // Initialize alphanumeric displays for pitch/roll/yaw
      pitchDisplay.begin(PITCH_DISPLAY_I2C_ADDRESS);
      rollDisplay.begin(ROLL_DISPLAY_I2C_ADDRESS);
      yawDisplay.begin(YAW_DISPLAY_I2C_ADDRESS);
    }
    void onRunning()
    {  
      // Get a new normalized sensor event
      sensors_event_t translation;        // Accelerometer (not used)
      sensors_event_t rotation;           // Gyroscope
      sensors_event_t heat;               // Thermometer (not used)
      imu.getEvent(&translation, &rotation, &heat);

      // Read gyro rates in rad/s
      pitchRadiansPerSecond = rotation.gyro.x;
      rollRadiansPerSecond = rotation.gyro.y;
      yawRadiansPerSecond = rotation.gyro.z;
      
      // Convert to deg/s
      pitchDegreesPerSecond = pitchRadiansPerSecond * RAD_TO_DEG_PER_SEC_CONVERSION_SCALAR;
      rollDegreesPerSecond = rollRadiansPerSecond * RAD_TO_DEG_PER_SEC_CONVERSION_SCALAR;
      yawDegreesPerSecond = yawRadiansPerSecond * RAD_TO_DEG_PER_SEC_CONVERSION_SCALAR;
      pitchAbsoluteDegreesPerSecond = (int)abs(pitchDegreesPerSecond);
      rollAbsoluteDegreesPerSecond = (int)abs(rollDegreesPerSecond);
      yawAbsoluteDegreesPerSecond = (int)abs(yawDegreesPerSecond);
      
      // Display gyro rates on the alphanumeric displays
      pitchDisplay.print(pitchAbsoluteDegreesPerSecond);
      rollDisplay.print(rollAbsoluteDegreesPerSecond);
      yawDisplay.print(yawAbsoluteDegreesPerSecond);
    }
    void onKill()
    {
      return;
    }

  protected:
    // (Model) Adafruit StemmaQT (Qwiic) LSM6DSOX 6dof IMU
    Adafruit_LSM6DSOX imu;
    float pitchRadiansPerSecond, rollRadiansPerSecond, yawRadiansPerSecond;
    float pitchDegreesPerSecond, rollDegreesPerSecond, yawDegreesPerSecond;
    int pitchAbsoluteDegreesPerSecond, rollAbsoluteDegreesPerSecond, yawAbsoluteDegreesPerSecond;
    // (View) SparkFun Qwiic Alphanumeric Displays
    HT16K33 pitchDisplay, rollDisplay, yawDisplay;
};

ArduinoProtoThread *imuThread;
ImuController *imuThreadDelegate;


//////////////////////////////////////////////////////////////////////////////////////////////////
// RUN ONCE
//////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
#if FEATURE_DUAL_CONTROLLER == false
  // Initialize LIDAR delegate
  lidarThreadDelegate = new LidarController();
  // Initialize LIDAR protothread
  lidarThread = new ArduinoProtoThread();
  lidarThread->setEventHandlerTo(lidarThreadDelegate);
  lidarThread->setExecutionIntervalTo(500);
  lidarThread->changeStateTo(Start);
#endif
  // Initialize IMU delegate
  imuThreadDelegate = new ImuController();
  // Initialize IMU protothread
  imuThread = new ArduinoProtoThread();
  imuThread->setEventHandlerTo(imuThreadDelegate);
  imuThread->setExecutionIntervalTo(50);
  imuThread->changeStateTo(Start);

  // Initialize I2C bus
  Wire.begin();
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// INFINITE LOOP
//////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  lidarThread->timeSlice();
  imuThread->timeSlice();
}

// End of RubbermaidGuidanceSystem.ino
