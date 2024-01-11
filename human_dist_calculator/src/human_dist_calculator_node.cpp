#include "human_dist_calculator/human_dist_calculator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "human_dist_calculator");
    HumanDistCalculator humandistcalculator;
    humandistcalculator.process();

    return 0;
}