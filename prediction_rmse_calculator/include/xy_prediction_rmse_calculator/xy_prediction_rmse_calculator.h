#ifndef XY_PREDICTION_RMSE_CALCULATOR_H
#define XY_PREDICTION_RMSE_CALCULATOR_H

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
class XYPredictionRmseCalculator
{
public:
  XYPredictionRmseCalculator();
  void process();

private:
  // コールバック関数
  void obs_data_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);

  // 引数なし関数
  void store_predicted_data();     // 予測データを格納
  void calc_vel_squared_error();   // 速度に関する二乗誤差を計算
  void calc_yaw_squared_error();   // 方位に関する二乗誤差を計算
  void calc_rmse();                // RMSEを計算

  // 引数あり関数
  void calc_dist_squared_error(const double observed_x, const double observed_y, const double predict_x, const double predict_y, const double start_x, const double start_y);  // 位置に関する二乗誤差を計算
  void calc_vel_squared_error(const double observed_vel, const double predict_vel);                                                                                            // 速度に関する二乗誤差を計算
  void calc_yaw_squared_error(const double observed_yaw, const double predict_yaw);                                                                                            // 方位に関する二乗誤差を計算
  double normalize_angle(double theta);                                                                                                                                        // 適切な角度(-M_PI ~ M_PI)を返す
  void output_csv(const std::string output_file, const std::vector<double> rmse);                                                                                              // 結果をcsvファイルに出力する

  // yamlファイルで設定可能な変数
  std::string h_dist_rmse_output_file_;  // 進行方向に対する位置に関する評価値の出力ファイルパス
  std::string v_dist_rmse_output_file_;  // 進行方向の垂直方向に対する位置に関する評価値の出力ファイルパス
  std::string vel_rmse_output_file_;     // 速度に関する評価値の出力ファイルパス
  std::string yaw_rmse_output_file_;     // 方位に関する評価値の出力ファイルパス
  int hz_;                               // ループ周波数 [Hz]
  int id_;                               // 評価する障害物のid
  double horizon_;                       // 予測ホライズン [s]
  double dt_;                            // 微小時間 [s]
  int start_step_;                       // 開始ステップ
  int calc_step_;                        // RMSEを計算するステップ数

  // 予測データ格納用
  std::vector< std::vector<State> > predicted_states_;

  // 評価用変数
  double h_dist_squared_error_sum_;  // 進行方向に対する位置に関する二乗誤差の合計
  std::vector<double> h_dist_rmse_;  // 進行方向に対する位置に関する二乗平均平方根誤差（RMSE）
  double v_dist_squared_error_sum_;  // 進行方向の垂直方向に対する位置に関する二乗誤差の合計
  std::vector<double> v_dist_rmse_;  // 進行方向の垂直方向に対する位置に関する二乗平均平方根誤差（RMSE）
  double vel_squared_error_sum_;     // 速度に関する二乗誤差の合計
  std::vector<double> vel_rmse_;     // 速度に関する二乗平均平方根誤差（RMSE）
  double yaw_squared_error_sum_;     // 方位に関する二乗誤差の合計
  std::vector<double> yaw_rmse_;     // 方位に関する二乗平均平方根誤差（RMSE）

  // その他の変数
  int data_counter_;                 // 障害物情報を何回subしたかのカウント用

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

#endif  // XY_PREDICTION_RMSE_CALCULATOR_H