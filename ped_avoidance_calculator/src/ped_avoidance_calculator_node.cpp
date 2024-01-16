#include "ped_avoidance_calculator/ped_avoidance_calculator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ped_avoidance_calculator");
    PedAvoidanceCaluculator pedavoidancecaluculator;
    pedavoidancecaluculator.process();

    return 0;
}