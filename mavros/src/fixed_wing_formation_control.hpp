/* this software is developed by the CGNC_LAB BIT, all rights reserved.*/
/* 作者：CGNC_LAB,BIT
 * 时间：2019.11.09
 */

// ros程序必备头文件
#ifndef _FIXED_WING_FORMATION_CONTROL_HPP_
#define _FIXED_WING_FORMATION_CONTROL_HPP_

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include "fixed_wing_sub_pub.hpp"

using namespace std;

class _FIXED_WING_FORMATION_CONTROL
{

private:

	float current_time;

	struct _s_fixed_wing_status
	{
		string mode;

		float ground_speed_ned_param1{0};

		float ground_speed_ned_param2{0};

		float global_vel_x;

		float global_vel_y;

		float global_vel_z;

		float air_speed;

		float relative_hight;

		float latitude;

		float altitude;

		float longtitude;

		float ned_pos_x;

		float ned_pos_y;

		float ned_pos_z;

		float ned_vel_x;

		float ned_vel_y;

		float ned_vel_z;

		float pitch_angle;

		float yaw_angle;

		float roll_angle;

	} leader_status, follower_status;


	struct _s_follower_setpiont
	{
		string mode;

		float ground_speed_ned_param1{0};

		float ground_speed_ned_param2{0};

		float global_vel_x;

		float global_vel_y;

		float global_vel_z;

		float air_speed;

		float relative_hight;

		float latitude;

		float altitude;

		float longtitude;

		float ned_pos_x;

		float ned_pos_y;

		float ned_pos_z;

		float ned_vel_x;

		float ned_vel_y;

		float ned_vel_z;

		float pitch_angle;

		float yaw_angle;

		float roll_angle;

	} follower_setpoint;



public:

	void write_to_files(string file_path_name, float time_stamp, float data);

	float get_ros_time(ros::Time begin); //获取ros当前时间

	void run(int argc, char **argv);

	bool update_follwer_status(_FIXED_WING_SUB_PUB *fixed_wing_sub_pub_pointer);

	bool update_leader_status();

	void show_fixed_wing_status(int PlaneID);

	void test(int argc, char **argv);

	void ros_sub_and_pub();

	void handle_status_from_receiver();

	void handle_message_from_px4();

	void send_the_command_to_px4();

	void send_message_to_sender();

	void control_formation();

    void send_setpoint_to_px4();

    void send_setpoint_to_ground_station();
};

#endif