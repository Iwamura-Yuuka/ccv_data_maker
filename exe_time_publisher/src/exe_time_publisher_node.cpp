#include "exe_time_publisher/exe_time_publisher.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "exe_time_publisher");
    ExeTimePublisher exetimepublisher;
    exetimepublisher.process();

    return 0;
}