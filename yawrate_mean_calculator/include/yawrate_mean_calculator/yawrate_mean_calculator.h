#ifndef YAWRATE_MEAN_CALCULATOR_H
#define YAWRATE_MEAN_CALCULATOR_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class YawrateMeanCalculator
{
public:
    YawrateMeanCalculator();
    void process();

private:
    // コールバック関数
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg);

    // 引数なし関数
    bool is_goal();  // pathの終端に着くまでtrueを返す

    // yamlファイルで設定可能な変数
    int hz_;                // ループ周波数 [Hz]
    double init_x_;         // スタート地点のx座標
    double cource_length_;  // 目標軌道の長さ
    double resolution_;     // 軌道生成時の刻み幅

    // その他の変数
    double current_x_;     // odom_x
    double yawrate_;       // yawrate
    double yawrate_sum_;   // yawrateの絶対値の合計
    int data_number_;      // データ個数カウント用
    double yawrate_mean_;  // yawrateの絶対値の平均値

    // msgの受け取り判定用
    bool flag_odom_ = false;
    bool flag_cmd_vel_ = false;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    //Subscriber
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_cmd_vel_;

    nav_msgs::Odometry odom_;       // odom
    geometry_msgs::Twist cmd_vel_;  // 制御入力

};


#endif // YAWRATE_MEAN_CALCULATOR_H