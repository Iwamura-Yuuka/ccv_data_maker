#include "speed_and_ped_state_calculator/speed_and_ped_state_calculator.h"

SpeedAndPedCalculator::SpeedAndPedCalculator():private_nh_("~")
{
  // param
  private_nh_.param("output_file", output_file_, {"/data/output.csv"});
  private_nh_.param("hz", hz_, {10});
  private_nh_.param("start_x", start_x_, {1.0});
  private_nh_.param("goal_x", goal_x_, {18.0});
  private_nh_.param("count_dist", count_dist_, {1.5});

  // subscriber
  sub_ped_states_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1, &SpeedAndPedCalculator::pedestrian_data_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_robot_odom_ = nh_.subscribe("/sq2_ccv/diff_drive_steering_controller/odom", 1, &SpeedAndPedCalculator::robot_odom_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  sub_local_goal_ = nh_.subscribe("/local_goal", 1, &SpeedAndPedCalculator::local_goal_callback, this, ros::TransportHints().reliable().tcpNoDelay());
}

// 歩行者データのコールバック関数
void SpeedAndPedCalculator::pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents)
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
void SpeedAndPedCalculator::robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_odom_ = *msg;
  flag_robot_odom_ = true;
}

// local_goalコールバック関数
void SpeedAndPedCalculator::local_goal_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
  local_goal_ = *msg;
  flag_local_goal_ = true;
}

// ロボットと歩行者の間の距離を計算
double SpeedAndPedCalculator::calc_distance(const double robot_x, const double robot_y, const double person_x, const double person_y)
{
  const double dx = person_x - robot_x;
  const double dy = person_y - robot_y;

  return hypot(dx, dy);
}

// 適切な角度(-M_PI ~ M_PI)を返す
double SpeedAndPedCalculator::normalize_angle(double theta)
{
  if(theta > M_PI)
    theta -= 2.0 * M_PI;
  if(theta < -M_PI)
    theta += 2.0 * M_PI;

  return theta;
}

// ロボットと歩行者の距離を計算
void SpeedAndPedCalculator::calc_ped_dist()
{
  //初期化
  min_dist_ = 100.0;

  // ped_states_の配列のうち，1回のpublish分のデータ（配列の先頭の要素）のみ取得
  const auto people_states = ped_states_.front();

  // msg型を pedsim_msgs/AgentStates から pedestrian_msgs/PeopleStates に変更
  for(const auto& person : people_states->agent_states)
  {
    const double dist = calc_distance(robot_odom_.pose.pose.position.x, robot_odom_.pose.pose.position.y, person.pose.position.x, person.pose.position.y);

    // ロボットと歩行者の距離が一定以下の場合，カウント
    if(dist < count_dist_)
      ped_counter_++;
    
    // ロボットと歩行者の距離の最小値を更新
    if(dist < min_dist_)
    {
      min_dist_ = dist;

      // 最も近い歩行者の方位を計算
      ped_theta_ = atan2(person.pose.position.y - robot_odom_.pose.pose.position.y, person.pose.position.x - robot_odom_.pose.pose.position.x);
      ped_theta_ = normalize_angle(ped_theta_);
    }
  }

  sum_min_dist_ += min_dist_;
}

// local_goalの方位を計算
void SpeedAndPedCalculator::calc_goal_theta()
{
  goal_theta_ = atan2(local_goal_.point.y - robot_odom_.pose.pose.position.y, local_goal_.point.x - robot_odom_.pose.pose.position.x);
  goal_theta_ = normalize_angle(goal_theta_);
}

// 計算した値を表示
void SpeedAndPedCalculator::display_data()
{
  if((robot_odom_.pose.pose.position.x > start_x_) && (robot_odom_.pose.pose.position.x <= goal_x_))
  {
    // 初期化
    ped_counter_ = 0;
    goal_theta_ = 0.0;
    ped_theta_ = 0.0;

    calc_ped_dist();

    // ロボットと歩行者の距離の最小値を更新
    if(min_dist_ < all_min_dist_)
      all_min_dist_ = min_dist_;

    loop_counter_++;

    // ゴールの方位を計算
    calc_goal_theta();

    // ロボットの走行距離を計算
    double dist = calc_distance(tmp_robot_x_, tmp_robot_y_, robot_odom_.pose.pose.position.x, robot_odom_.pose.pose.position.y);
    mileage_ += dist;

    // ロボットの速度を計算
    robot_speed_ = dist * (double)hz_;

    // ロボットの走行時間を計算
    travel_time_ += 1.0 / (double)hz_;

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

    // データを表示
    ROS_INFO_STREAM("average speed : " << average_speed_ << "m/s");       // 平均速度
    ROS_INFO_STREAM("mean of min dist : " << min_dist_mean_ << "m");      // 最小値の平均
    ROS_INFO_STREAM("min dist : " << all_min_dist_ << "m");               // 最小値
  }
}

//メイン文で実行する関数
void SpeedAndPedCalculator::process()
{
  ros::Rate loop_rate(hz_);

  // getenv("HOME")でホームディレクトリのパスを取得
  // std::string型で定義されたものが1つでも含まれていれば+で文字列をくっつけられる
  std::string file_path = getenv("HOME") + output_file_;

  // csvファイルを開く(std::ofstreamのコンストラクタで開く)
  // std::ios::appで既存のファイルに追記
  std::ofstream ofs(file_path, std::ios::app);

  if(!ofs.is_open())
    ROS_WARN_STREAM("can't open file!");

  ofs << 'Loop_counter' << ',' << 'Robot_speed[m/s]' << ',' << 'Ped_num' << ',' << 'Min_dist[m]' << ',' << 'Min_ped theta[rad]' << ',' << 'Local_goal_theta[rad]' << ',' << std::endl;

  while(ros::ok())
  {
    if((flag_ped_states_ == true) && (flag_robot_odom_ == true) && (flag_local_goal_ == true))
    {
      display_data();

      if((robot_odom_.pose.pose.position.x > start_x_) && (robot_odom_.pose.pose.position.x <= goal_x_))
      {
        // データをcsvファイルに出力
        ofs << loop_counter_ << ',' << robot_speed_ << ',' << ped_counter_ << ',' << min_dist_ << ',' << ped_theta_ << ',' << goal_theta_ << ',' << std::endl;
      }
      else if(robot_odom_.pose.pose.position.x > goal_x_)
      {
        // データをcsvファイルに出力
        if(flag_output_file_ == false)
        {
          // mileage_とtravel_time_を修正
          mileage_ = mileage_ * 20.0 / (goal_x_ - start_x_);
          travel_time_ = travel_time_ * 20.0 / (goal_x_ - start_x_);

          ofs << "Avg_speed[m/s]" << ',' << 'Mean_of_min_dist[m]' << ',' << 'Min_dist[m]' << ',' << std::endl;
          ofs << average_speed_ << ',' << min_dist_mean_ << ',' << all_min_dist_ << ',' << std::endl;
          flag_output_file_ = true;
        }
      }
    }

    // msgの受け取り判定用flagをfalseに戻す
    flag_ped_states_ = false;
    flag_robot_odom_ = false;
    flag_local_goal_ = false;

    ros::spinOnce();
    loop_rate.sleep();
  }
}