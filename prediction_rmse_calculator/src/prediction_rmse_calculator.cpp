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

  // 評価するidに対応する障害物情報の最初と最後のインデックスを計算
  int stored_step_num = (horizon_ / dt_) + 1;  //保管されてる情報のステップ数（現在情報＋予測情報）
  double start_index = id_ * horizon_ / dt_;   // 最初のインデックス
  double end_index = start_index + (stored_step_num * 4);  // 最後のインデックス

  for(int i=0; i<obs_data_.data.size(); i+=4)
  {
    State state;
    state.x = obs_data_.data[i];
    state.y = obs_data_.data[i+1];
    state.vel = obs_data_.data[i+2];
    state.yaw = obs_data_.data[i+3];
    predicted_states.push_back(state);
  }
  predicted_states_.push_back(predicted_states);
}

// 結果をcsvファイルに出力する
void PredictionRmseCalculator::output_csv()
{
  // getenv("HOME")でホームディレクトリのパスを取得
  // std::string型で定義されたものが1つでも含まれていれば+で文字列をくっつけられる
  std::string file_path = getenv("HOME") + output_file_;

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
      // 処理
    }

    // msgの受け取り判定用flagをfalseに戻す
    flag_obs_data_ = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
}