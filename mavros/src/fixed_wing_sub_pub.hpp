// ros程序必备头文件
#include <ros/ros.h>
//mavros相关头文件
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

#include "fixed_wing_mathlib.hpp"

class _FIXED_WING_SUB_PUB
{

private:
public:
    mavros_msgs::State current_state; //无人机当前状态[包含上锁状态 模式] (从飞控中读取)

    mavros_msgs::PositionTarget pos_setpoint; //即将发给无人机的控制指令

    mavros_msgs::PositionTarget pos_target; //从无人机回传的 vehicle_local_position_setpoint

    mavros_msgs::AttitudeTarget att_target; //从无人机回传的 vehicle_attitude_setpoint [四元数形式]

    sensor_msgs::Imu imu;

    float PIX_Euler_target[3]; //无人机 期望欧拉角(从飞控中读取)
    float att_angle_Euler[3];        //无人机当前欧拉角(从飞控中读取)
    float Thrust_target;       //期望推力

    _FIXED_WING_MATHLIB mathlib; //数学类

    //*******************ros_call_back*************************//
    void state_cb(const mavros_msgs::State::ConstPtr &msg)
    {
        current_state = *msg;
    }

    void pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr &msg)
    {
        pos_target = *msg;
    }

    void att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
    {
        att_target = *msg;

        float q[4];
        q[0] = msg->orientation.w;
        q[1] = msg->orientation.x;
        q[2] = msg->orientation.y;
        q[3] = msg->orientation.z;

        mathlib.quaternion_2_euler(q, PIX_Euler_target);

        Thrust_target = msg->thrust;
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

    //*******************ros_call_back*************************//
};