#include "exe_time_publisher/exe_time_publisher.h"

ExeTimePublisher::ExeTimePublisher():private_nh_("~")
{
    // param
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("start_x", start_x_, {1.0});
    private_nh_.param("goal_x", goal_x_, {18.0});

    // subscriber
    sub_robot_odom_ = nh_.subscribe("/sq2_ccv/diff_drive_steering_controller/odom", 1, &ExeTimePublisher::robot_odom_callback, this, ros::TransportHints().reliable().tcpNoDelay());
}

// ロボットのodomのコールバック関数
void ExeTimePublisher::robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_odom_ = *msg;
    flag_robot_odom_ = true;
}

// 実行時間を計算
void ExeTimePublisher::calc_exe_time()
{
    if((flag_clock_start_ == false) && (robot_odom_.pose.pose.position.x > start_x_))
    {
        // 実行時のスタート時刻を設定
        begin_ = ros::Time::now();
        flag_clock_start_ = true;

        ROS_INFO_STREAM("start!");  // デバック用
    }
    else if((flag_clock_goal_ == false) && (robot_odom_.pose.pose.position.x > goal_x_))
    {
        // 実行時のゴール時刻を設定
        end_ = ros::Time::now();
        flag_clock_goal_ = true;

        ROS_INFO_STREAM("goal!");  // デバック用
    }
    else if(flag_clock_goal_ == true)
    {
        // 実行時間を表示
        show_exe_time();
    }

}

// 実行時間を表示
void ExeTimePublisher::show_exe_time()
{
    ROS_INFO_STREAM("Duration : " << end_.toSec() - begin_.toSec() << "s");
}

//メイン文で実行する関数
void ExeTimePublisher::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(flag_robot_odom_ == true)
        {
            calc_exe_time();
        }

        // msgの受け取り判定用flagをfalseに戻す
        flag_robot_odom_ = false;

        ros::spinOnce();
        loop_rate.sleep();
    }
}