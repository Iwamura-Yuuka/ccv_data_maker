#include "ped_avoidance_calculator/ped_avoidance_calculator.h"

PedAvoidanceCaluculator::PedAvoidanceCaluculator():private_nh_("~")
{
    // param
    private_nh_.param("output_file", output_file_, {"/data/output.csv"});
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("start_x", start_x_, {18.0});
    private_nh_.param("goal_x", goal_x_, {1.0});
    private_nh_.param("area", area_, {0.0});
    private_nh_.param("tmp_x", tmp_x_, {0.0});
    private_nh_.param("tmp_y", tmp_y_, {0.0});

    // subscriber
    sub_ped_states_ = nh_.subscribe("/pedsim_simulator/simulated_agents", 1, &PedAvoidanceCaluculator::pedestrian_data_callback, this, ros::TransportHints().reliable().tcpNoDelay());
}

// 歩行者データのコールバック関数
void PedAvoidanceCaluculator::pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents)
{
    while(ped_states_.size() > 0)
    {
        // ped_states_の配列のうち取得済みのデータ（配列の先頭の要素）を削除
        // これをしないと，front() でデータを取得する際，同じデータしか取得できない
        ped_states_.pop();
    }

    ped_states_.emplace(agents);
    flag_ped_states_ = true;
}

// 回避量を計算
void PedAvoidanceCaluculator::calc_avoidance()
{
    // ped_states_の配列のうち，1回のpublish分のデータ（配列の先頭の要素）のみ取得
    const auto people_states = ped_states_.front();

    //
    for(const auto& person : people_states->agent_states)
    {   
        if((flag_start_ == false) && (person.pose.position.x <= start_x_))
        {
            flag_start_ = true;

            tmp_x_ = person.pose.position.x;
            tmp_y_ = person.pose.position.y;
        }
        else if((flag_start_ == true) && (person.pose.position.x > goal_x_))
        {
            const double ped_x = person.pose.position.x;
            const double ped_y = person.pose.position.y;

            // 面積を計算
            const double dx = tmp_x_ - ped_x;
            const double dy = std::abs(ped_y);
            area_ += dx * dy;

            tmp_x_ = person.pose.position.x;
            tmp_y_ = person.pose.position.y;
        }
        else if((flag_start_ == true) && (person.pose.position.x <= goal_x_) && (flag_goal_ == false))
        {
            flag_goal_ = true;
        }
        else if(flag_goal_ == true)
        {
            ROS_INFO_STREAM("area : " << area_ << "m^2");

            // データをcsvファイルに出力
            if(flag_output_file_ == false)
            {
                output_csv();
                flag_output_file_ = true;
            }
        }
    }
}

// 結果をcsvファイルに出力
void PedAvoidanceCaluculator::output_csv()
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
    ofs << area_ << std::endl;
}


//メイン文で実行する関数
void PedAvoidanceCaluculator::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(flag_ped_states_ == true)
        {
            calc_avoidance();
            // ROS_INFO_STREAM("area : " << area_ << "m^2");
        }

        // msgの受け取り判定用flagをfalseに戻す
        flag_ped_states_ = false;

        ros::spinOnce();
        loop_rate.sleep();
    }
}