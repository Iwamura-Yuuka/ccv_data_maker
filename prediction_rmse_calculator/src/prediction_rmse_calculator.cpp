#include "prediction_rmse_calculator/prediction_rmse_calculator.h"

PredictionRmseCalculator::PredictionRmseCalculator():private_nh_("~")
{
  // param
  private_nh_.param("dist_rmse_output_file", dist_rmse_output_file_, {"/data/dist_rmse_output.csv"});
  private_nh_.param("vel_rmse_output_file", vel_rmse_output_file_, {"/data/vel_rmse_output.csv"});
  private_nh_.param("yaw_rmse_output_file", yaw_rmse_output_file_, {"/data/yaw_rmse_output.csv"});
  private_nh_.param("hz", hz_, {10});
  private_nh_.param("id", id_, {0});
  private_nh_.param("horizon", horizon_, {1.0});
  private_nh_.param("dt", dt_, {0.1});
  private_nh_.param("start_step", start_step_, {10});
  private_nh_.param("calc_step", calc_step_, {100});
  private_nh_.param("dist_squared_error_sum", dist_squared_error_sum_, {0.0});
  private_nh_.param("vel_squared_error_sum", vel_squared_error_sum_, {0.0});
  private_nh_.param("yaw_squared_error_sum", yaw_squared_error_sum_, {0.0});
  private_nh_.param("data_counter", data_counter_, {0});

  // subscriber
  sub_obs_data_ = nh_.subscribe("/predicted_state", 1, &PredictionRmseCalculator::obs_data_callback, this, ros::TransportHints().reliable().tcpNoDelay());
}

// 障害物情報のコールバック関数
void PredictionRmseCalculator::obs_data_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  obs_data_ = *msg;
  flag_obs_data_ = true;
}

// 予測データを格納
void PredictionRmseCalculator::store_predicted_data()
{
  std::vector<State> current_predicted_states;
  
  // ROS_INFO_STREAM("obs_data_.data.size() = " << obs_data_.data.size());  // デバック用

  // 評価するidに対応する障害物情報の最初と最後のインデックスを計算
  int stored_step_num = (horizon_ / dt_) + 1;  //保管されてる情報のステップ数（現在情報＋予測情報）
  double start_index = id_ * stored_step_num * 4;   // 最初のインデックス
  double end_index = (id_ + 1) * stored_step_num * 4;  // 最後のインデックス

  for(int i=start_index; i<end_index; i+=4)
  {
    State state;
    state.x = obs_data_.data[i];
    state.y = obs_data_.data[i+1];
    state.vel = obs_data_.data[i+2];
    state.yaw = obs_data_.data[i+3];
    current_predicted_states.push_back(state);
  }

  predicted_states_.push_back(current_predicted_states);  // 予測データを格納
}

// 位置に関する二乗誤差を計算
void PredictionRmseCalculator::calc_dist_squared_error(const double observed_x, const double observed_y, const double predict_x, const double predict_y)
{
  const double dist_squared_error = (observed_x - predict_x) * (observed_x - predict_x) + (observed_y - predict_y) * (observed_y - predict_y);
  dist_squared_error_sum_ += dist_squared_error;
}

// 速度に関する二乗誤差を計算
void PredictionRmseCalculator::calc_vel_squared_error(const double observed_vel, const double predict_vel)
{
  const double vel_squared_error = (observed_vel - predict_vel) * (observed_vel - predict_vel);
  vel_squared_error_sum_ += vel_squared_error;
}

// 方位に関する二乗誤差を計算
void PredictionRmseCalculator::calc_yaw_squared_error(const double observed_yaw, const double predict_yaw)
{
  const double yaw_squared_error = normalize_angle(observed_yaw - predict_yaw) * normalize_angle(observed_yaw - predict_yaw);
  yaw_squared_error_sum_ += yaw_squared_error;
}

// 適切な角度(-M_PI ~ M_PI)を返す
double PredictionRmseCalculator::normalize_angle(double theta)
{
  if(theta > M_PI)
    theta -= 2.0 * M_PI;
  if(theta < -M_PI)
    theta += 2.0 * M_PI;

  return theta;
}

// RMSEを計算
void PredictionRmseCalculator::calc_rmse()
{
  // 終了ステップを計算
  int end_step = start_step_ + calc_step_ + (horizon_ / dt_);

  if((data_counter_ > 10) && (data_counter_ <= end_step))
  {
    // 予測データを格納
    store_predicted_data();
  }
  else if((data_counter_ > end_step) && (flag_output_file_ == false))
  {
    int predict_step = horizon_ / dt_;  // 予測ステップ数 
    // RMSEを計算
    for(int i=1; i<=predict_step; i++)
    {
      // 初期化
      dist_squared_error_sum_ = 0.0;
      vel_squared_error_sum_ = 0.0;
      yaw_squared_error_sum_ = 0.0;

      for(int j=i; j<i+calc_step_; j++)
      {
        // 位置に関する二乗誤差を計算
        calc_dist_squared_error(predicted_states_[j][0].x, predicted_states_[j][0].y, predicted_states_[j-i][i].x, predicted_states_[j-i][i].y);

        // 速度に関する二乗誤差を計算
        calc_vel_squared_error(predicted_states_[j][0].vel, predicted_states_[j-i][i].vel);

        // 方位に関する二乗誤差を計算
        calc_yaw_squared_error(predicted_states_[j][0].yaw, predicted_states_[j-i][i].yaw);
      }

      // 位置に関する二乗平均平方根誤差（RMSE）を計算
      double dist_rmse = sqrt(dist_squared_error_sum_ / calc_step_);
      dist_rmse_.push_back(dist_rmse);

      // 速度に関する二乗平均平方根誤差（RMSE）を計算
      double vel_rmse = sqrt(vel_squared_error_sum_ / calc_step_);
      vel_rmse_.push_back(vel_rmse);

      // 方位に関する二乗平均平方根誤差（RMSE）を計算
      double yaw_rmse = sqrt(yaw_squared_error_sum_ / calc_step_);
      yaw_rmse_.push_back(yaw_rmse);
    }

    // csvファイルに出力
    output_csv(dist_rmse_output_file_, dist_rmse_);
    output_csv(vel_rmse_output_file_, vel_rmse_);
    output_csv(yaw_rmse_output_file_, yaw_rmse_);

    flag_output_file_ = true;
    ROS_INFO_STREAM("id : " << id_);
    ROS_INFO_STREAM("===== output csv file! =====");
  }

  data_counter_++;
}

// 結果をcsvファイルに出力する
void PredictionRmseCalculator::output_csv(const std::string output_file, const std::vector<double> rmse)
{
  // getenv("HOME")でホームディレクトリのパスを取得
  // std::string型で定義されたものが1つでも含まれていれば+で文字列をくっつけられる
  std::string file_path = getenv("HOME") + output_file;

  // csvファイルを開く(std::ofstreamのコンストラクタで開く)
  // std::ios::appで既存のファイルに追記
  std::ofstream ofs(file_path, std::ios::app);

  if(!ofs.is_open())
    ROS_WARN_STREAM("can't open file!");

  // データをファイルに書き込み
  ofs << id_ << ",";
  for(int i=0; i<rmse.size(); i++)
    ofs << rmse[i] << ",";
  ofs << std::endl;
}

// メイン文で実行する関数
void PredictionRmseCalculator::process()
{
  ros::Rate loop_rate(hz_);

  while(ros::ok())
  {
    if(flag_obs_data_)
    {
      calc_rmse(); 
    }

    // msgの受け取り判定用flagをfalseに戻す
    flag_obs_data_ = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
}