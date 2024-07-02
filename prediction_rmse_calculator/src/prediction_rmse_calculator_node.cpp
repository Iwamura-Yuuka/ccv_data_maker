#include "prediction_rmse_calculator/prediction_rmse_calculator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "prediction_rmse_calculator");
    PredictionRmseCalculator predictionrmsecalculator;
    predictionrmsecalculator.process();

    return 0;
}