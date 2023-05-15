// Imu.hpp
// Part of the /RubbermainGuidanceSystem project on GitHub
// Greg M. Krsak (@gregkrsak)(greg.krsak@gmail.com)


// ArduinoProtoThread class
#include "ArduinoProtoThread.hpp"
#include "ArduinoProtoThreadStateMachine.hpp"

// IMU
#include <Adafruit_LSM6DSOX.h>
// Displays
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
