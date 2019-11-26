#ifndef _LATERAL_CONTROLLER_HPP_
#define _LATERAL_CONTROLLER_HPP_
/*
*作者：lee-shun
*本程序是基于L1控制器的改动版本，输入是飞机的当前的位置，当前的地速
*(向量），空速，目标位置点，输出的是期望的偏航角以及期望的滚转角
*估计应该会写几个种类的横向控制器，看最后用那个吧。。。。。。。。
*
*
*/
#include <iostream>
#include <math.h>
#include "mathlib.h"
#include "vector.hpp"

using namespace std;

class LATERAL_CONTROLLER
{
private:
    float acc_lateral{0};
    float roll_sp{0};

public:
    float get_lateral_roll_sp()
    {
        return roll_sp;
    }

    float get_lateral_acc_lateral()
    {
        return acc_lateral;
    }

    void lateral_L1_modified();

    void lateral_yaw_rate();

    void lateral_yaw();

    void lateral_sliding_mode();
};

void LATERAL_CONTROLLER::lateral_yaw()
{
    float delat_a_n = control_lateral_params.kp * error_follwer1.n_diatance + control_lateral_params.kd * error_follwer1.ned_vel_x;
    float delat_a_e = control_lateral_params.kp * error_follwer1.e_distance + control_lateral_params.kd * error_follwer1.ned_vel_y;

    follower_setpoint.ned_acc_x = delat_a_n + leader_status.ned_acc_x; //测试时领机的加速度为0
    follower_setpoint.ned_acc_y = delat_a_e + leader_status.ned_acc_y;

    //将ned下的加速度期望值转换到体轴系下
    follower_setpoint.body_acc_x = cos(follower_status.yaw_angle) * follower_setpoint.ned_acc_x +
                                   sin(follower_status.yaw_angle) * follower_setpoint.ned_acc_y;

    follower_setpoint.body_acc_y = -sin(follower_status.yaw_angle) * follower_setpoint.ned_acc_x +
                                   cos(follower_status.yaw_angle) * follower_setpoint.ned_acc_y;

    //去掉x方向的加速度，利用协调转弯，计算滚转角期望值

    follower_setpoint.roll_angle = constrain(atan(follower_setpoint.body_acc_y / CONSTANTS_ONE_G), -1, 1); //atan返回的是弧度制下的-90到90
}

void LATERAL_CONTROLLER::lateral_yaw_rate()
{
    float last_angle_error = 0.0;

    float DT = current_time - last_time_lateral;

    int angle_zone_flag = 0;

    float angle_error;

    //将ned下的位置误差转换到体轴系下,得到的是机体系下的目标点的位置坐标
    error_follwer1.Xb_distance = cos(follower_status.yaw_angle) * error_follwer1.n_diatance +
                                 sin(follower_status.yaw_angle) * error_follwer1.e_distance;

    error_follwer1.Yb_distance = -sin(follower_status.yaw_angle) * error_follwer1.n_diatance +
                                 cos(follower_status.yaw_angle) * error_follwer1.e_distance;

    cout << "error_follwer1.Xb_distance" << error_follwer1.Xb_distance << endl;

    cout << "error_follwer1.Yb_distance" << error_follwer1.Yb_distance << endl;

    float angle_error_raw = atan(error_follwer1.Yb_distance / (error_follwer1.Xb_distance + 0.01)); //注意机体系的坐标

    if (error_follwer1.Xb_distance >= 0 && error_follwer1.Yb_distance > 0)
    {
        angle_zone_flag = 1;
        angle_error = angle_error_raw;
    }
    else if (error_follwer1.Xb_distance >= 0 && error_follwer1.Yb_distance < 0)
    {
        angle_zone_flag = 2;
        angle_error = deg_2_rad(360) + angle_error_raw; //本身是个负的
    }
    else if (error_follwer1.Xb_distance < 0 && error_follwer1.Yb_distance <= 0)
    {
        angle_zone_flag = 3;
        angle_error = angle_error_raw + deg_2_rad(180); //////
    }
    else
    {
        angle_zone_flag = 4;
        angle_error = angle_error_raw + deg_2_rad(180);
    }
    cout << "angle_zone_flag====" << angle_zone_flag << endl;
    cout << "angle_error====" << angle_error << endl;

    //1和4象限为正，2和3象限为负，和期望产生的加速度的符号相同
    float angle_error_dot = (angle_error - last_angle_error) / DT;
    float gama_dot = control_lateral_params.kp * angle_error + control_lateral_params.kd * angle_error_dot;

    follower_setpoint.roll_angle = constrain(atan(gama_dot * CONSTANTS_ONE_G / follower_status.air_speed), -1, 1); //atan返回的是弧度制下的-90到90

    last_angle_error = angle_error;

    last_time_lateral = current_time;
}

void LATERAL_CONTROLLER::lateral_L1_modified()
{
}

void LATERAL_CONTROLLER::lateral_sliding_mode()
{
}
#endif