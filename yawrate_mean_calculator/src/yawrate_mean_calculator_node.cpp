#include "yawrate_mean_calculator/yawrate_mean_calculator.h"

// メイン関数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "yawrate_mean_calculator");
    YawrateMeanCalculator yawratemeancalculator;
    yawratemeancalculator.process();
    return 0;
}