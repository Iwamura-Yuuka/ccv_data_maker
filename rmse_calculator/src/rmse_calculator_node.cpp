#include "rmse_calculator/rmse_calculator.h"

// メイン関数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "rmse_calculator");
    RmseCalculator rmsecalculator;
    rmsecalculator.process();
    return 0;
}