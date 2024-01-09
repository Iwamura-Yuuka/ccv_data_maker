#include "rmse_calculator/rmse_calculator.h"

RmseCalculator::RmseCalculator():private_nh_("~")
{
    private_nh_.param("hz", hz_, {30});
    private_nh_.param("init_x", init_x_, {0.0});
    private_nh_.param("init_y", init_y_, {0.0});
    private_nh_.param("init_theta", init_theta_, {0.0});
    private_nh_.param("cource_length", cource_length_, {10.0});
    private_nh_.param("resolution", resolution_, {0.05});
    private_nh_.param("current_x", current_x_, {0.0});
    private_nh_.param("current_y", current_y_, {0.0});
    private_nh_.param("target_x", target_x_, {0.0});
    private_nh_.param("target_y", target_y_, {0.0});
    private_nh_.param("path_index", path_index_, {0});
    private_nh_.param("data_number", data_number_, {0});
    private_nh_.param("diff_x", diff_x_, {0.0});
    private_nh_.param("squared_error_sum", squared_error_sum_, {0.0});
    private_nh_.param("rmse", rmse_, {0.0});

    //Subscriber
    sub_path_ = nh_.subscribe("/local_path", 1, &RmseCalculator::path_callback, this);
    sub_odom_ = nh_.subscribe("/sq2_ccv/diff_drive_steering_controller/odom", 1, &RmseCalculator::odom_callback, this);
}

// pathのコールバック関数
void RmseCalculator::path_callback(const nav_msgs::Path::ConstPtr &msg)
{
    path_ = *msg;
    flag_path_ = true;
}

// odomのコールバック関数
void RmseCalculator::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_ = *msg;

    current_x_ = odom_.pose.pose.position.x;
    // ROS_INFO_STREAM("current_x is " << current_x_);  // デバック用
    current_y_ = odom_.pose.pose.position.y;

    flag_odom_ = true;
}

// pathの終端に着くまでtrueを返す
bool RmseCalculator::is_goal()
{
    double goal_x = init_x_ + cource_length_;

    if(current_x_ < goal_x + resolution_)
        return true;
    else
        return false;
}

// target_xとtarget_yを更新
void RmseCalculator::update_target()
{
    target_x_ = path_.poses[path_index_].pose.position.x;
    target_y_ = path_.poses[path_index_].pose.position.y;

    path_index_++;
    flag_path_update_ = false;
}

// 目標軌道のx座標に最も近いx座標を探索
int RmseCalculator::search_x()
{
    // 1ステップ前のx座標の目標値との差を格納（今回ループと比較用）
    double tmp_diff_x = diff_x_;

    diff_x_ = fabs(target_x_ - current_x_);

    // current_xがtarget_xに最も近いx座標だった場合は1を返す
    // tmp_xがtarget_xに最も近いx座標だった場合は2を返す
    // target_xに最も近いx座標がcurrent_xでもtmp_xでもなかった場合は0を返す
    if(current_x_ > target_x_)
    {
        if(tmp_diff_x > diff_x_)
            return 1;
        else
            return 2;
    }
    else
    {
        return 0;
    }
}

// 二乗誤差を計算
void RmseCalculator::calc_squared_error(const double y)
{
    squared_error_sum_ += pow(target_y_ - y, 2.0);

    // データの個数をカウント
    data_number_++;
    
    // 目標軌道の座標を更新できるようにflagの値を変更
    flag_path_update_ = true;
}

// RMSEを計算
void RmseCalculator::calc_rmse(const double tmp_x, const double tmp_y)
{
    // 目標軌道の座標を更新
    if(flag_path_update_)
    {
        update_target();
        diff_x_ = resolution_;
    }

    // 目標軌道との誤差を計算
    int calc_checker = search_x();
    if(calc_checker == 1)
    {
        if(target_x_ > resolution_)
        {
            calc_squared_error(current_y_);
            // ROS_INFO_STREAM("target_x is " << target_x_);                    // デバック用
            // ROS_INFO_STREAM("x is " << current_x_);                          // デバック用
            // ROS_INFO_STREAM("squared_error_sum is " << squared_error_sum_);  // デバック用
        }
    }
    else if(calc_checker == 2)
    {
        if(target_x_ > resolution_)
        {
            calc_squared_error(tmp_y);
            calc_squared_error(current_y_);
            // ROS_INFO_STREAM("target_x is " << target_x_);                    // デバック用
            // ROS_INFO_STREAM("x is " << tmp_x << " (tmp)");                   // デバック用
            // ROS_INFO_STREAM("squared_error_sum is " << squared_error_sum_);  // デバック用
        }
    }
}

// メイン文で実行する関数
void RmseCalculator::process()
{
    ros::Rate loop_rate(hz_);

    double tmp_x = 0.0;
    double tmp_y = 0.0;

    while(ros::ok())
    {
        if((flag_path_ && flag_odom_) == true)
        {
            if(is_goal())
            {
                // odomのx座標がpathの始点に近づいたところから探索開始
                if(current_x_ > init_x_ - resolution_)
                {
                    calc_rmse(tmp_x, tmp_y);
                    rmse_ = sqrt(squared_error_sum_ / data_number_);
                }
            }
            else
            {
                ROS_INFO_STREAM("=============================== RMSE is " << rmse_);
                // ROS_INFO_STREAM("x is " << current_x_);  // デバック用
            }
            
            // 座標探索用に1ステップ前の座標を保存
            tmp_x = current_x_;
            tmp_y = current_y_;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

// // メイン関数
// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "rmse_calculator");
//     RmseCalculator rmsecalculator;
//     rmsecalculator.process();
//     return 0;
// }