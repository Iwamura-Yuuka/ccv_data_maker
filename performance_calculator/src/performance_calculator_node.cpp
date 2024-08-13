#include "performance_calculator/performance_calculator.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "performance_calculator");
  PerformanceCalculator performancecalculator;
  performancecalculator.process();

  return 0;
}