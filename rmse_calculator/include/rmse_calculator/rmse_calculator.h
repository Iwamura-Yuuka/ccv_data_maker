#ifndef RMSE_CALCULATOR_H
#define RMSE_CALCULATOR_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

class RmseCalculator
{
public:
    RmseCalculator();
    void process();

private:
    // コールバック関数
    void path_callback(const nav_msgs::Path::ConstPtr &msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);

    // 引数あり関数
    void calc_squared_error(const double y);                 // 二乗誤差を計算
    void calc_rmse(const double tmp_x, const double tmp_y);  // RMSEを計算

    // 引数なし関数
    bool is_goal();                                          // pathの終端に着くまでtrueを返す
    void update_target();                                    // target_xとtarget_yを更新
    int search_x();                                          // 目標軌道のx座標に最も近いx座標を探索

    // yamlファイルで設定可能な変数
    int hz_;                // ループ周波数 [Hz]
    double init_x_;         // スタート地点のx座標
    double init_y_;         // スタート地点のy座標
    double init_theta_;     // 生成する軌道の向き
    double cource_length_;  // 目標軌道の長さ
    double resolution_;     // 軌道生成時の刻み幅

    // その他の変数
    double current_x_;              // odom_x
    double current_y_;              // odom_y
    double target_x_;               // path_x
    double target_y_;               // path_y
    int path_index_;                // path_の配列番号
    int data_number_;               // 誤差の計算回数
    double diff_x_;                 // x座標の目標値との差
    double squared_error_sum_;      // 二乗誤差の合計
    bool flag_path_update_ = true;  // 目標軌道の座標更新用
    double rmse_;                   // 二乗平均平方根誤差（RMSE）

    // msgの受け取り判定用
    bool flag_path_ = false;
    bool flag_odom_ = false;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    //Subscriber
    ros::Subscriber sub_path_;
    ros::Subscriber sub_odom_;

    nav_msgs::Path path_;      // 目標軌道
    nav_msgs::Odometry odom_;  // 処理前のodomデータ

};


#endif // RMSE_CALCULATOR_H