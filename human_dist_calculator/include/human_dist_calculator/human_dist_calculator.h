#ifndef HUMAN_DIST_CALCULATOR_H
#define HUMAN_DIST_CALCULATOR_H

#include <ros/ros.h>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <cstdlib>
#include <fstream>

// original_msgs
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>

class HumanDistCalculator
{
public:
    HumanDistCalculator();
    void process();

private:
    // コールバック関数
    void pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents);
    void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

    // 引数あり関数
    double calc_distance(const double robot_x, const double robot_y, const double person_x, const double person_y);  // ロボットと歩行者の間の距離を計算
    void collision_counter(const int id);                                                                          // 衝突回数をカウント

    // 引数なし関数
    void calc_min_dist();  // ロボットと歩行者の距離の最小値を計算
    void display_data();   // 計算した値を表示
    void output_csv();     // 結果をcsvファイルに出力

    // yamlファイルで設定可能な変数
    std::string output_file_;  // 出力ファイルパス
    int hz_;                   // ループ周波数 [Hz]
    double start_x_;           // 計測を開始するx座標 [m]
    double goal_x_;            // 計測を終了するx座標 [m]
    double collision_dist_;    // 衝突した判定にするロボットと歩行者の距離

    // 最小距離計算用
    double all_min_dist_;   // 1回のシミュレーション内でのロボットと歩行者の距離の最小値
    double min_dist_;       // 1回のループ内でのロボットと歩行者の距離の最小値
    double sum_min_dist_;   // 各ループにおけるロボットと歩行者の距離の最小値の合計
    double min_dist_mean_;  // 各ループにおけるロボットと歩行者の距離の最小値の平均
    int loop_counter_;      // 計算回数カウント用

    // 衝突判定用
    std::vector<int> collision_id_;  // 衝突した歩行者のid格納用

    // 1ステップ前のロボットの位置情報格納用
    double tmp_robot_x_;
    double tmp_robot_y_;
    
    // msgの受け取り判定用
    bool flag_ped_states_ = false;
    bool flag_robot_odom_ = false;

    // ファイル出力の判定用
    bool flag_output_file_ = false;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_ped_states_;
    ros::Subscriber sub_robot_odom_;

    // 各種オブジェクト
    std::queue<pedsim_msgs::AgentStatesConstPtr> ped_states_;  // 歩行者情報
    nav_msgs::Odometry robot_odom_;                            // ロボットの位置情報
};

#endif  // HUMAN_DIST_CALCULATOR_H