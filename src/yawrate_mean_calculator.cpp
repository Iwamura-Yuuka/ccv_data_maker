#include <yawrate_mean_calculator/yawrate_mean_calculator.h>

YawrateMeanCalculator::YawrateMeanCalculator():private_nh_("~")
{
    private_nh_.param("hz", hz_, {30});
    private_nh_.param("init_x", init_x_, {0.0});
    private_nh_.param("cource_length", cource_length_, {10.0});
    private_nh_.param("resolution", resolution_, {0.05});
    private_nh_.param("current_x", current_x_, {0.0});
    private_nh_.param("yawrate", yawrate_, {0.0});
    private_nh_.param("yawrate_sum", yawrate_sum_, {0.0});
    private_nh_.param("data_number", data_number_, {0});
    private_nh_.param("yawrate_mean_", yawrate_mean_, {0.0});

    //Subscriber
    sub_odom_ = nh_.subscribe("/sq2_ccv/diff_drive_steering_controller/odom", 1, &YawrateMeanCalculator::odom_callback, this);
    sub_cmd_vel_ = nh_.subscribe("/sq2_ccv/diff_drive_steering_controller/cmd_vel", 1, &YawrateMeanCalculator::cmd_vel_callback, this);
}

// odomのコールバック関数
void YawrateMeanCalculator::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_ = *msg;

    current_x_ = odom_.pose.pose.position.x;
    // ROS_INFO_STREAM("current_x is " << current_x_);  // デバック用

    flag_odom_ = true;
}

// cmd_velのコールバック関数
void YawrateMeanCalculator::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel_ = *msg;
    yawrate_ = cmd_vel_.angular.z;
    flag_cmd_vel_ = true;
}

// pathの終端に着くまでtrueを返す
bool YawrateMeanCalculator::is_goal()
{
    double goal_x = init_x_ + cource_length_;

    if(current_x_ < goal_x + resolution_)
        return true;
    else
        return false;
}

// メイン文で実行する関数
void YawrateMeanCalculator::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(flag_odom_ == true)
        {
            if(is_goal())
            {
                // odomのx座標がpathの始点に近づいたところから探索開始
                if(current_x_ > init_x_)
                {
                    yawrate_sum_ += fabs(yawrate_);
                    data_number_++;
                }
            }
            else
            {
                yawrate_mean_ = yawrate_sum_ / data_number_;
                ROS_INFO_STREAM("=============================== mean is " << yawrate_mean_);
                // ROS_INFO_STREAM("x is " << current_x_);  // デバック用
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

// メイン関数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "yawrate_mean_calculator");
    YawrateMeanCalculator yawratemeancalculator;
    yawratemeancalculator.process();
    return 0;
}