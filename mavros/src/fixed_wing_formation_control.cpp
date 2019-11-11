/* this software is developed by the CGNC_LAB BIT, all rights reserved.*/
/* 作者：CGNC_LAB,BIT
 * 时间：2019.11.09
 */
#include "fixed_wing_formation_control.hpp" //记得修改一下路径

float _FIXED_WING_FORMATION_CONTROL::get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void _FIXED_WING_FORMATION_CONTROL::handle_status_from_receiver()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::handle_message_from_px4()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::send_the_command_to_px4()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::send_message_to_sender()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::ros_sub_and_pub()
{

    //
}

void _FIXED_WING_FORMATION_CONTROL::run()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::test(int argc, char **argv)
{
}

bool _FIXED_WING_FORMATION_CONTROL::show_fixed_wing_status()
{

    ros::NodeHandle nh;
    ros::Rate rate(5.0);

    _FIXED_WING_SUB_PUB fixed_wing_sub_pub; //定义订阅发布对象，所有的回调函数的结果在这里面。

    ros::Subscriber // 【订阅】无人机当前状态
        fixed_wing_states_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &_FIXED_WING_SUB_PUB::state_cb, &fixed_wing_sub_pub);

    ros::Subscriber // 【订阅】无人机imu信息
        fixed_wing_imu_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, &_FIXED_WING_SUB_PUB::imu_cb, &fixed_wing_sub_pub);

    ros::Subscriber // 【订阅】无人机位置期望值[飞控中读取]
        fixed_wing_pos_setpoints_from_px4_sub = nh.subscribe<mavros_msgs::PositionTarget>("mavros/setpoint_raw/target_local", 10, &_FIXED_WING_SUB_PUB::pos_target_cb, &fixed_wing_sub_pub);

    ros::Subscriber // 【订阅】无人机姿态期望值[飞控中读取]
        fixed_wing_att_setpoints_from_px4_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/target_attitude", 10, &_FIXED_WING_SUB_PUB::att_target_cb, &fixed_wing_sub_pub);

    ros::Time begin_time = ros::Time::now(); // 记录启控时间

    while (ros::ok())
    {
        float current_time = get_ros_time(begin_time);

        cout << "***************fixed_wing_status_from_px4(show_fixed_wing_status)******************" << endl;
        cout << "Time: " << current_time << " [s] " << endl;
        cout << "Mode : [ " << fixed_wing_sub_pub.current_state.mode << " ]" << endl;

        cout << "ned下的位置【XYZ】" << fixed_wing_sub_pub.pos_target.position.x << " [m] "
             << fixed_wing_sub_pub.pos_target.position.y << " [m] "
             << fixed_wing_sub_pub.pos_target.position.z << " [m] " << endl;

        cout << "飞机当前姿态【pitch，roll，yaw】" << fixed_wing_sub_pub.att_angle_Euler[0] << " [deg] "
             << fixed_wing_sub_pub.att_angle_Euler[1] << " [deg] "
             << fixed_wing_sub_pub.att_angle_Euler[2] << " [deg] " << endl;
        cout << "***************##################################################******************" << endl;

        ros::spinOnce();
        //挂起一段时间(rate为 5HZ)
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fixed_wing_formation_control");

    _FIXED_WING_FORMATION_CONTROL formation_control;

    if (true)
    {
        formation_control.show_fixed_wing_status();
    }
}
