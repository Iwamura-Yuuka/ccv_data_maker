#include "xy_prediction_rmse_calculator/xy_prediction_rmse_calculator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "xy_prediction_rmse_calculator");
    XYPredictionRmseCalculator xypredictionrmsecalculator;
    xypredictionrmsecalculator.process();

    return 0;
}