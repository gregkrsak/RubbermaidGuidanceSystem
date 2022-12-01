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


//////////////////////////////////////////////////////////////////////////////////////////////////
// FEATURE FLIPPERS
//////////////////////////////////////////////////////////////////////////////////////////////////
#define FEATURE_DUAL_CONTROLLER true  // True if LIDAR runs on a separate Arduino.
                                      // It's safer to leave this set to FALSE.


//////////////////////////////////////////////////////////////////////////////////////////////////
// BUILD TARGET
//////////////////////////////////////////////////////////////////////////////////////////////////
#define __IMU   1
#define __LIDAR 2

#define BUILD_TARGET __IMU // <- CAN BE IGNORED IF NOT IN DUAL CONTROLLER MODE


//////////////////////////////////////////////////////////////////////////////////////////////////
// LIDAR
//////////////////////////////////////////////////////////////////////////////////////////////////
#if (FEATURE_DUAL_CONTROLLER == false) || (FEATURE_DUAL_CONTROLLER == true && BUILD_TARGET == __LIDAR)

  #include "Lidar.hpp"

  ArduinoProtoThread *lidarThread;
  LidarController *lidarThreadDelegate;

#endif


//////////////////////////////////////////////////////////////////////////////////////////////////
// IMU
//////////////////////////////////////////////////////////////////////////////////////////////////
#if (FEATURE_DUAL_CONTROLLER == false) || (FEATURE_DUAL_CONTROLLER == true && BUILD_TARGET == __IMU)

  #include "Imu.hpp"
  
  ArduinoProtoThread *imuThread;
  ImuController *imuThreadDelegate;

#endif


//////////////////////////////////////////////////////////////////////////////////////////////////
// RUN ONCE
//////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  
#if (FEATURE_DUAL_CONTROLLER == false) || (FEATURE_DUAL_CONTROLLER == true && BUILD_TARGET == __LIDAR)
  // Initialize LIDAR delegate
  lidarThreadDelegate = new LidarController();
  // Initialize LIDAR protothread
  lidarThread = new ArduinoProtoThread();
  lidarThread->setEventHandlerTo(lidarThreadDelegate);
  lidarThread->setExecutionIntervalTo(500);
  lidarThread->changeStateTo(Start);
#endif

#if (FEATURE_DUAL_CONTROLLER == false) || (FEATURE_DUAL_CONTROLLER == true && BUILD_TARGET == __IMU)
  // Initialize IMU delegate
  imuThreadDelegate = new ImuController();
  // Initialize IMU protothread
  imuThread = new ArduinoProtoThread();
  imuThread->setEventHandlerTo(imuThreadDelegate);
  imuThread->setExecutionIntervalTo(50);
  imuThread->changeStateTo(Start);
#endif

  // Initialize I2C bus
  Wire.begin();
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// INFINITE LOOP
//////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  
#if (FEATURE_DUAL_CONTROLLER == false) || (FEATURE_DUAL_CONTROLLER == true && BUILD_TARGET == __LIDAR)
  lidarThread->timeSlice();
#endif

#if (FEATURE_DUAL_CONTROLLER == false) || (FEATURE_DUAL_CONTROLLER == true && BUILD_TARGET == __IMU)
  imuThread->timeSlice();
#endif

}

// End of RubbermaidGuidanceSystem.ino
