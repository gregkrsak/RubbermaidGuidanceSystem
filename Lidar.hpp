// Lidar.hpp
// Part of the /RubbermainGuidanceSystem project on GitHub
// Greg M. Krsak (@gregkrsak)(greg.krsak@gmail.com)


// ArduinoProtoThread class
#include "ArduinoProtoThread.hpp"
#include "ArduinoProtoThreadStateMachine.hpp"

// Lidar
#include <LIDARLite_v4LED.h>
// Display
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
