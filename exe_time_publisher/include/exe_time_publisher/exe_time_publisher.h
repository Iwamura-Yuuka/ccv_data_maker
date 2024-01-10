#ifndef EXE_TIME_PUBLISHER_H
#define EXE_TIME_PUBLISHER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

class ExeTimePublisher
{
public:
    ExeTimePublisher();
    void process();

private:
    // コールバック関数
    void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    
    // 引数なし関数
    void calc_exe_time();  // 実行時間を計算
    void show_exe_time();  // 実行時間を表示

    // yamlファイルで設定可能な変数
    int hz_;          // ループ周波数 [Hz]
    double start_x_;  // 計測を開始するx座標 [m]
    double goal_x_;   // 計測を終了するx座標 [m]

    // 実行時間表示用
    ros::Time begin_;  // 計測開始時刻
    ros::Time end_;    // 計測終了時刻

    // msgの受け取り判定用
    bool flag_robot_odom_ = false;

    // 実行時間関連
    bool flag_clock_start_ = false;  // 計測開始の確認用
    bool flag_clock_goal_ = false;   // 計測終了の確認用

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_robot_odom_;

    // 各種オブジェクト
    nav_msgs::Odometry robot_odom_;  // ロボットの位置情報
};


#endif // EXE_TIME_PUBLISHER_H