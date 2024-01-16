#ifndef PED_AVOIDANCE_CALUCULATOR_H
#define PED_AVOIDANCE_CALUCULATOR_H

#include <ros/ros.h>
#include <queue>

// original_msgs
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>

class PedAvoidanceCaluculator
{
public:
    PedAvoidanceCaluculator();
    void process();

private:
    // コールバック関数
    void pedestrian_data_callback(const pedsim_msgs::AgentStatesConstPtr& agents);

    // 引数なし関数
    void calc_avoidance();  // 回避量を計算

    // yamlファイルで設定可能な変数
    int hz_;          // ループ周波数 [Hz]
    double start_x_;  // 計算を開始するx座標 [m]
    double goal_x_;   // 計算を終了するx座標 [m]

    // その他の変数
    double area_;   // 面積
    double tmp_x_;  // 1ループ前の歩行者のx座標格納用
    double tmp_y_;  // 1ループ前の歩行者のy座標格納用
    
    // msgの受け取り判定用
    bool flag_ped_states_ = false;

    // 計算開始・終了の判定用
    bool flag_start_ = false;
    bool flag_goal_ = false;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_ped_states_;

    // 各種オブジェクト
    std::queue<pedsim_msgs::AgentStatesConstPtr> ped_states_;  // 歩行者情報
};

#endif  // PED_AVOIDANCE_CALUCULATOR_H