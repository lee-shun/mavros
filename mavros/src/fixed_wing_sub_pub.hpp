#ifndef _FIXED_WING_SUB_PUB_HPP_
#define _FIXED_WING_SUB_PUB_HPP_

// ros程序必备头文件
#include <ros/ros.h>
//mavros相关头文件
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h> //提示local——position在这里

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h> //GPS Fix.
#include <std_msgs/Float64.h>
#include <sensor_msgs/BatteryState.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h> //UTM coords
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h> //Velocity fused by FCU
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

//setpoint_raw:在这个底下，全都有，SET_ATTITUDE_TARGET++SET_POSITION_TARGET_LOCAL_NED
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include "fixed_wing_mathlib.hpp"

#define the_space_between_lines 1  //为了打印中间空格
#define the_space_between_blocks 3 //为了打印中间空格

class _FIXED_WING_SUB_PUB
{

private:
public:
    _FIXED_WING_MATHLIB mathlib; //数学类

    //订阅的数据暂时容器
    mavros_msgs::State current_state; //无人机当前状态[包含上锁状态 模式] (从飞控中读取)

    sensor_msgs::Imu imu;

    sensor_msgs::NavSatFix global_position_form_px4;

    std_msgs::Float64 global_rel_alt_from_px4;

    nav_msgs::Odometry umt_position_from_px4;

    geometry_msgs::TwistStamped velocity_global_fused_from_px4;

    geometry_msgs::TwistStamped velocity_ned_fused_from_px4;

    geometry_msgs::PoseStamped local_position_from_px4;

    geometry_msgs::AccelWithCovarianceStamped acc_ned_from_px4;

    geometry_msgs::TwistWithCovarianceStamped wind_estimate_from_px4;

    sensor_msgs::BatteryState battrey_state_from_px4;

    mavros_msgs::SetMode mode_cmd;

    //发布的数据暂时容器
    mavros_msgs::PositionTarget local_pos_sp;

    mavros_msgs::GlobalPositionTarget global_pos_sp;

    mavros_msgs::AttitudeTarget att_sp;

    float PIX_Euler_target[3]; //无人机 期望欧拉角(从飞控中读取)
    float att_angle_Euler[3];  //无人机当前欧拉角(从飞控中读取)

    void state_cb(const mavros_msgs::State::ConstPtr &msg)
    {
        current_state = *msg;
    }

    void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
    {
        imu = *msg;
        float q[4];
        q[0] = msg->orientation.w;
        q[1] = msg->orientation.x;
        q[2] = msg->orientation.y;
        q[3] = msg->orientation.z;

        mathlib.quaternion_2_euler(q, att_angle_Euler);
    }

    void global_position_form_px4_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        global_position_form_px4 = *msg;
    }

    void fixed_wing_global_rel_alt_from_px4_cb(const std_msgs::Float64::ConstPtr &msg)
    {
        global_rel_alt_from_px4 = *msg;
    }

    void umt_position_from_px4_cb(const nav_msgs::Odometry::ConstPtr &msg)
    {
        umt_position_from_px4 = *msg;
    }

    void velocity_global_fused_from_px4_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        velocity_global_fused_from_px4 = *msg;
    }

    void velocity_ned_fused_from_px4_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        velocity_ned_fused_from_px4 = *msg;
    }

    void local_position_from_px4_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        local_position_from_px4 = *msg;
    }

    void acc_ned_from_px4_cb(const geometry_msgs::AccelWithCovarianceStamped::ConstPtr &msg)
    {
        acc_ned_from_px4 = *msg;
    }

    void wind_estimate_from_px4_cb(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg)
    {
        wind_estimate_from_px4 = *msg;
    }

    void battrey_state_from_px4_cb(const sensor_msgs::BatteryState::ConstPtr &msg)
    {
        battrey_state_from_px4 = *msg;
    }
};

#endif