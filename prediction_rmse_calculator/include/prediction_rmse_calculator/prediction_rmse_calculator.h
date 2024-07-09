#include "prediction_rmse_calculator/prediction_rmse_calculator.h"

#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <cstdlib>
#include <fstream>

// ===== 構造体 =====
struct State
{
    double x;    // [m]
    double y;    // [m]
    double vel;  // [m/s]
    double yaw;  // [rad]
};

// ===== クラス =====
class PredictionRmseCalculator
{
public:
  PredictionRmseCalculator();
  void process();

private:
  // コールバック関数
  void obs_data_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);

  // 引数なし関数
  void store_predicted_data();     // 予測データを格納
  void calc_dist_squared_error();  // 位置に関する二乗誤差を計算
  void calc_vel_squared_error();   // 速度に関する二乗誤差を計算
  void calc_yaw_squared_error();   // 方位に関する二乗誤差を計算
  void calc_rmse();                // RMSEを計算

  // 引数あり関数
  void output_csv(const std::string output_file, const std::vector<double> rmse);  // 結果をcsvファイルに出力する

  // yamlファイルで設定可能な変数
  std::string dist_rmse_output_file_;  // 位置に関する評価値の出力ファイルパス
  std::string vel_rmse_output_file_;   // 速度に関する評価値の出力ファイルパス
  std::string yaw_rmse_output_file_;   // 方位に関する評価値の出力ファイルパス
  int hz_;                             // ループ周波数 [Hz]
  int id_;                             // 評価する障害物のid
  double horizon_;                     // 予測ホライズン [s]
  double dt_;                          // 微小時間 [s]

  // 予測データ格納用
  std::vector< std::vector<State> > predicted_states_;

  // 評価用変数
  int data_num_;                   // データ数
  double dist_squared_error_sum_;  // 位置に関する二乗誤差の合計
  std::vector<double> dist_rmse_;  // 位置に関する二乗平均平方根誤差（RMSE）
  double vel_squared_error_sum_;   // 速度に関する二乗誤差の合計
  std::vector<double> vel_rmse_;   // 速度に関する二乗平均平方根誤差（RMSE）
  double yaw_squared_error_sum_;   // 方位に関する二乗誤差の合計
  std::vector<double> yaw_rmse_;   // 方位に関する二乗平均平方根誤差（RMSE）

  // msgの受け取り判定用
  bool flag_obs_data_ = false;

  // ファイル出力の判定用
  bool flag_output_file_ = false;

  // NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Subscriber
  ros::Subscriber sub_obs_data_;

  // 各種オブジェクト
  std_msgs::Float32MultiArray obs_data_;
};

#endif  // PREDICTION_RMSE_CALCULATOR_H_