#ifndef SPEED_AND_PED_STATE_CALCULATOR_H
#define SPEED_AND_PED_STATE_CALCULATOR_H

#include <ros/ros.h>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <cstdlib>
#include <fstream>

// original_msgs
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>

class SpeedAndPedCalculator
{
public:
  SpeedAndPedCalculator();
  void process();

private:
  // コールバック関数
  void pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents);
  void robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void local_goal_callback(const geometry_msgs::PointStampedConstPtr& msg);

  // 引数あり関数
  double calc_distance(const double robot_x, const double robot_y, const double person_x, const double person_y);
  double normalize_angle(double theta);

  // 引数なし関数
  void calc_ped_dist();
  void calc_goal_theta();
  void display_data();

  // yamlファイルで設定可能な変数
  std::string output_file_;  // 出力ファイルパス
  int hz_;                   // ループ周波数 [Hz]
  double start_x_;           // 計測を開始するx座標 [m]
  double goal_x_;            // 計測を終了するx座標 [m]
  double count_dist_;        // 歩行者人数をカウントするロボットと歩行者の距離

  // 歩行者人数カウント用
  int ped_counter_ = 0;  // 歩行者人数

  // 最小距離計算用
  double all_min_dist_ = 100.0;  // 1回のシミュレーション内でのロボットと歩行者の距離の最小値
  double min_dist_ = 100.0;      // 1回のループ内でのロボットと歩行者の距離の最小値
  double sum_min_dist_ = 0.0;    // 各ループにおけるロボットと歩行者の距離の最小値の合計
  double min_dist_mean_ = 0.0;   // 各ループにおけるロボットと歩行者の距離の最小値の平均
  int loop_counter_ = 0;         // 計算回数カウント用

  // ロボットの速度計算用
  double robot_speed_ = 0.0;  // ロボットの速度 [m/s]
  double tmp_robot_x_ = 0.0;  // 1ステップ前のロボットのx座標
  double tmp_robot_y_ = 0.0;  // 1ステップ前のロボットのy座標

  // 方位計算用
  double goal_theta_ = 0.0;  // ゴールの方位 [rad]
  double ped_theta_ = 0.0;   // 最も近い歩行者の方位 [rad]

  // 平均速度計算用
  double mileage_ = 0.0;        // ロボットの走行距離 [m]
  double travel_time_ = 0.0;    // ロボットの走行時間 [s]
  double average_speed_ = 0.0;  // ロボットの平均速度 [m/s]

  // msgの受け取り判定用
  bool flag_ped_states_ = false;
  bool flag_robot_odom_ = false;
  bool flag_local_goal_ = false;

  // ファイル出力の判定用
  bool flag_output_file_ = false;

  // NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Subscriber
  ros::Subscriber sub_ped_states_;
  ros::Subscriber sub_robot_odom_;
  ros::Subscriber sub_local_goal_;

  // 各種オブジェクト
  std::queue<pedsim_msgs::AgentStatesConstPtr> ped_states_;  // 歩行者情報
  nav_msgs::Odometry robot_odom_;                            // ロボットの位置情報
  geometry_msgs::PointStamped local_goal_;                   // local_goal
};

#endif  // SPEED_AND_PED_STATE_CALCULATOR_H