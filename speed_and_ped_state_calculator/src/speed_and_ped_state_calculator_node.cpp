#include "speed_and_ped_state_calculator/speed_and_ped_state_calculator.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "speed_and_ped_state_calculator");
  SpeedAndPedCalculator speedandpedcalculator;
  speedandpedcalculator.process();

  return 0;
}