#include "prediction_rmse_calculator/prediction_rmse_calculator.h"

PredictionRmseCalculator::PredictionRmseCalculator():private_nh("~")
{
  // param
  private_nh_.param("dist_rmse_output_file", dist_rmse_output_file_, {"/data/dist_rmse_output.csv"});
  private_nh_.param("vel_rmse_output_file", vel_rmse_output_file_, {"/data/vel_rmse_output.csv"});
  private_nh_.param("yaw_rmse_output_file", yaw_rmse_output_file_, {"/data/yaw_rmse_output.csv"});
  private_nh_.param("hz", hz_, {10});
  private_nh_.param("id", id_, {0});
  private_nh_.param("horizon", horizon_, {1.0});
  private_nh_.param("dt", dt_, {0.1});
  private_nh_.param("data_num", data_num_, {0});
  private_nh_.param("dist_squared_error_sum", dist_squared_error_sum_, {0.0});
  private_nh_.param("vel_squared_error_sum", vel_squared_error_sum_, {0.0});
  private_nh_.param("yaw_squared_error_sum", yaw_squared_error_sum_, {0.0});

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
  
  ROS_INFO_STREAM("obs_data_.data.size() = " << obs_data_.data.size());  // デバック用

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
  data_num_++;                                            // データ数をカウント
}

// 位置に関する二乗誤差を計算
void PredictionRmseCalculator::calc_dist_squared_error()
{
 
}

// 速度に関する二乗誤差を計算
void PredictionRmseCalculator::calc_vel_squared_error()
{
  
}

// 方位に関する二乗誤差を計算
void PredictionRmseCalculator::calc_yaw_squared_error()
{
  
}

// RMSEを計算
void PredictionRmseCalculator::calc_rmse()
{
  // 予測データを格納
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
  

  outputfile.close();
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