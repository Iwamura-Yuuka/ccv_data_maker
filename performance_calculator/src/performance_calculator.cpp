#include "performance_calculator/performance_calculator.h"

PerformanceCalculator::PerformanceCalculator():private_nh_("~")
{
  // param
  private_nh_.param("output_file", output_file_, {"/data/output.csv"});
  private_nh_.param("hz", hz_, {10});
  private_nh_.param("start_x", start_x_, {1.0});
  private_nh_.param("goal_x", goal_x_, {18.0});
  private_nh_.param("collision_dist", collision_dist_, {0.2});
  private_nh_.param("all_min_dist", all_min_dist_, {100.0});
  private_nh_.param("min_dist", min_dist_, {100.0});
  private_nh_.param("sum_min_dist", sum_min_dist_, {0.0});
  private_nh_.param("min_dist_mean", min_dist_mean_, {100.0});
  private_nh_.param("loop_counter", loop_counter_, {0});
  private_nh_.param("tmp_robot_x", tmp_robot_x_, {0.0});
  private_nh_.param("tmp_robot_y", tmp_robot_y_, {0.0});
  private_nh_.param("mileage", mileage_, {0.0});
  private_nh_.param("travel_time", travel_time_, {0.0});
  private_nh_.param("average_speed", average_speed_, {0.0});

  // subscriber
  sub_ped_states_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1, &PerformanceCalculator::pedestrian_data_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_robot_odom_ = nh_.subscribe("/sq2_ccv/diff_drive_steering_controller/odom", 1, &PerformanceCalculator::robot_odom_callback, this, ros::TransportHints().reliable().tcpNoDelay());
}

// 歩行者データのコールバック関数
void PerformanceCalculator::pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents)
{
   while(ped_states_.size() > 0)
  {
    // people_states_の配列のうち取得済みのデータ（配列の先頭の要素）を削除
    // これをしないと，front() でデータを取得する際，同じデータしか取得できない
    ped_states_.pop();
  }

  ped_states_.emplace(agents);
  flag_ped_states_ = true;
}

// ロボットのodomのコールバック関数
void PerformanceCalculator::robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_odom_ = *msg;
  flag_robot_odom_ = true;
}

// ロボットと歩行者の間の距離を計算
double PerformanceCalculator::calc_distance(const double robot_x, const double robot_y, const double person_x, const double person_y)
{
  const double dx = person_x - robot_x;
  const double dy = person_y - robot_y;

  return hypot(dx, dy);
}

// ロボットと歩行者の距離の最小値を計算
void PerformanceCalculator::calc_min_dist()
{
  //初期化
  min_dist_ = 100.0;

  // ped_states_の配列のうち，1回のpublish分のデータ（配列の先頭の要素）のみ取得
  const auto people_states = ped_states_.front();

  // msg型を pedsim_msgs/AgentStates から pedestrian_msgs/PeopleStates に変更
  for(const auto& person : people_states->agent_states)
  {
    const double dist = calc_distance(robot_odom_.pose.pose.position.x, robot_odom_.pose.pose.position.y, person.pose.position.x, person.pose.position.y);
    
    // ロボットと歩行者の距離の最小値を更新
    if(dist < min_dist_)
      min_dist_ = dist;
    
    // 衝突チェック
    if(dist < collision_dist_)
      collision_counter(person.id);
  }

  sum_min_dist_ += min_dist_;
}

// 衝突回数をカウント
void PerformanceCalculator::collision_counter(const int id)
{
  const int size = collision_id_.size();
  bool flag_match = false;

  // ロボットが動いているかを計算
  double robot_dist = calc_distance(tmp_robot_x_, tmp_robot_y_, robot_odom_.pose.pose.position.x, robot_odom_.pose.pose.position.y);
  
  // ロボットが動いている場合，衝突した歩行者のidを格納
  if(robot_dist > 0.01)
  {
    if(collision_id_.size() == 0)
    {
      collision_id_.push_back(id);
      ROS_ERROR_STREAM("id : " << id);  // 衝突した歩行者id
    }
    else
    {
      for(int i; i<size; i++)
      {
        if(collision_id_[i] == id)
        {
          flag_match = true;
          break;
        }
      }

      if(flag_match == false)
      {
        collision_id_.push_back(id);
        ROS_ERROR_STREAM("id : " << id);  // 衝突した歩行者id
      }
    }
  }
}

// 計算した値を表示
void PerformanceCalculator::display_data()
{
  if((robot_odom_.pose.pose.position.x > start_x_) && (robot_odom_.pose.pose.position.x <= goal_x_))
  {
    calc_min_dist();

    // ロボットと歩行者の距離の最小値を更新
    if(min_dist_ < all_min_dist_)
      all_min_dist_ = min_dist_;

    loop_counter_++;

    // ロボットの走行距離を計算
    double dist = calc_distance(tmp_robot_x_, tmp_robot_y_, robot_odom_.pose.pose.position.x, robot_odom_.pose.pose.position.y);
    mileage_ += dist;

    // ロボットの走行時間を計算
    travel_time_ += 1.0 / (double)hz_;

    // 60秒経過してもゴールに到達しない場合，走行失敗とする
    if((robot_odom_.pose.pose.position.x > 1.0) && (travel_time_ > (60.0 * (goal_x_ - start_x_) / 20.0)) && (flag_output_file_ == false))
    {
      flag_goal_ = false;
      ROS_ERROR_STREAM("goal : " << flag_goal_);
      output_csv();
      flag_output_file_ = true;
    }

    // 1ステップ前のロボットの位置情報を格納
    tmp_robot_x_ = robot_odom_.pose.pose.position.x;
    tmp_robot_y_ = robot_odom_.pose.pose.position.y;
  }
  else if(robot_odom_.pose.pose.position.x > goal_x_)
  {
    // 距離の最小値の平均を計算
    min_dist_mean_ = sum_min_dist_ / loop_counter_;

    // 走行時の平均速度を計算
    average_speed_ = mileage_ / travel_time_;

    // データをcsvファイルに出力
    if(flag_output_file_ == false)
    {
      // 完走できたかの判定
      // 衝突回数が0回の場合，完走とする
      if(collision_id_.size() > 0)
        flag_goal_ = false;
      else
        flag_goal_ = true;

      // データをcsvファイルに出力
      output_csv();
      flag_output_file_ = true;
    }

    // データを表示
    ROS_INFO_STREAM_THROTTLE(1.0, "mean of min dist : " << min_dist_mean_ << "m");      // 最小値の平均
    ROS_INFO_STREAM_THROTTLE(1.0, "min dist : " << all_min_dist_ << "m");               // 最小値
    ROS_ERROR_STREAM_THROTTLE(1.0, "collision : " << collision_id_.size() << "times");  // 衝突回数
    ROS_INFO_STREAM_THROTTLE(1.0, "mileage : " << mileage_ << "m");                     // 走行距離
    ROS_INFO_STREAM_THROTTLE(1.0, "travel time : " << travel_time_ << "s");             // 走行時間
    ROS_INFO_STREAM_THROTTLE(1.0, "average speed : " << average_speed_ << "m/s");       // 平均速度
    ROS_INFO_STREAM_THROTTLE(1.0, "goal : " << flag_goal_);                             // 完走できたか

    // ROS_INFO_STREAM("mean of min dist : " << min_dist_mean_ << "m");      // 最小値の平均
    // ROS_INFO_STREAM("min dist : " << all_min_dist_ << "m");               // 最小値
    // ROS_ERROR_STREAM("collision : " << collision_id_.size() << "times");  // 衝突回数
    // ROS_INFO_STREAM("mileage : " << mileage_ << "m");                     // 走行距離
    // ROS_INFO_STREAM("travel time : " << travel_time_ << "s");             // 走行時間
    // ROS_INFO_STREAM("average speed : " << average_speed_ << "m/s");       // 平均速度
    // ROS_INFO_STREAM("goal : " << flag_goal_);                             // 完走できたか
  }
}

// 結果をcsvファイルに出力
void PerformanceCalculator::output_csv()
{
  // getenv("HOME")でホームディレクトリのパスを取得
  // std::string型で定義されたものが1つでも含まれていれば+で文字列をくっつけられる
  std::string file_path = getenv("HOME") + output_file_;

  // csvファイルを開く(std::ofstreamのコンストラクタで開く)
  // std::ios::appで既存のファイルに追記
  std::ofstream ofs(file_path, std::ios::app);

  // mileage_とtravel_time_を修正
  mileage_ = mileage_ * 20.0 / (goal_x_ - start_x_);
  travel_time_ = travel_time_ * 20.0 / (goal_x_ - start_x_);

  if(!ofs.is_open())
    ROS_WARN_STREAM("can't open file!");

  // データをファイルに書き込み
  ofs << min_dist_mean_ << ',' << all_min_dist_ << ',' << collision_id_.size() << ',' << mileage_ << ',' << travel_time_ << ',' << average_speed_ << ',' << flag_goal_ << ',' << std::endl;
}

//メイン文で実行する関数
void PerformanceCalculator::process()
{
  ros::Rate loop_rate(hz_);

  while(ros::ok())
  {
    if((flag_ped_states_ == true) && (flag_robot_odom_ == true))
    {
      display_data();
    }

    // msgの受け取り判定用flagをfalseに戻す
    flag_ped_states_ = false;
    flag_robot_odom_ = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
}