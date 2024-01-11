#include "human_dist_calculator/human_dist_calculator.h"

HumanDistCalculator::HumanDistCalculator():private_nh_("~")
{
    // param
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("start_x", start_x_, {1.0});
    private_nh_.param("goal_x", goal_x_, {18.0});
    private_nh_.param("collision_dist", collision_dist_, {0.2});
    private_nh_.param("all_min_dist", all_min_dist_, {100.0});
    private_nh_.param("min_dist", min_dist_, {100.0});
    private_nh_.param("sum_min_dist", sum_min_dist_, {0.0});
    private_nh_.param("min_dist_mean", min_dist_mean_, {100.0});
    private_nh_.param("loop_counter", loop_counter_, {0});

    // subscriber
    sub_ped_states_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1, &HumanDistCalculator::pedestrian_data_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_robot_odom_ = nh_.subscribe("/sq2_ccv/diff_drive_steering_controller/odom", 1, &HumanDistCalculator::robot_odom_callback, this, ros::TransportHints().reliable().tcpNoDelay());
}

// 歩行者データのコールバック関数
void HumanDistCalculator::pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents)
{
    ped_states_.emplace(agents);
    flag_ped_states_ = true;
}

// ロボットのodomのコールバック関数
void HumanDistCalculator::robot_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_odom_ = *msg;
    flag_robot_odom_ = true;
}

// ロボットと歩行者の間の距離を計算
double HumanDistCalculator::calc_distance(const double robot_x, const double robot_y, const double person_x, const double person_y)
{
    const double dx = person_x - robot_x;
    const double dy = person_y - robot_y;

    return hypot(dx, dy);
}

// ロボットと歩行者の距離の最小値を計算
void HumanDistCalculator::calc_min_dist()
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
void HumanDistCalculator::collision_counter(const int id)
{
    // 2個前までidを参照し，一致しなかったら衝突した人のidを格納
    if(collision_id_.size() == 0)  // セグフォ防止
    {
        collision_id_.push_back(id);
    }
    else if(collision_id_.size() == 1)  // セグフォ防止
    {
        if(collision_id_.back() != id)
            collision_id_.push_back(id);
    }
    else
    {
        const int size = collision_id_.size();

        if((collision_id_.back() != id) && (collision_id_[size-2] != id))
            collision_id_.push_back(id);
    }
}

// 計算した値を表示
void HumanDistCalculator::display_data()
{
    if((robot_odom_.pose.pose.position.x > start_x_) && (robot_odom_.pose.pose.position.x <= goal_x_))
    {
        calc_min_dist();

        ROS_INFO_STREAM("x : " << robot_odom_.pose.pose.position.x);  // 最小値の平均

        // ロボットと歩行者の距離の最小値を更新
        if(min_dist_ < all_min_dist_)
            all_min_dist_ = min_dist_;

        loop_counter_++;
    }
    else if(robot_odom_.pose.pose.position.x > goal_x_)
    {
        // 距離の最小値の平均を計算
        min_dist_mean_ = sum_min_dist_ / loop_counter_;

        // データを表示
        ROS_INFO_STREAM("mean of min dist : " << min_dist_mean_ << "m");  // 最小値の平均
        ROS_INFO_STREAM("min dist : " << all_min_dist_ << "m");           // 最小値
        ROS_INFO_STREAM("collision : " << collision_id_.size() << "times");  // 衝突回数
    }
}

//メイン文で実行する関数
void HumanDistCalculator::process()
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