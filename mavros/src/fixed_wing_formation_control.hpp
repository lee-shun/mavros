/* this software is developed by the CGNC_LAB BIT, all rights reserved.*/
/* 作者：CGNC_LAB,BIT
 * 时间：2019.11.09
 */

// ros程序必备头文件
#include <ros/ros.h>
#include "fixed_wing_sub_pub.hpp"

using namespace std;

class _FIXED_WING_FORMATION_CONTROL
{

private:
	struct
	{

		float leader_ground_speed_ned_param1{0};

		float leader_ground_speed_ned_param2{0};

		float leader_air_speed;

		float leader_attitude;

		float leader_relative_hight;

		float leader_altitude;

		float leader_longtitude;

		float leader_yaw_angle;

		float leader_roll_angle;

	} _s_leader_status;

public:
	// _FIXED_WING_FORMATION_CONTROL();

	// ~_FIXED_WING_FORMATION_CONTROL();

	void run();

	void test(int argc, char **argv);

	void ros_sub_and_pub();

	void handle_status_from_receiver();

	void handle_message_from_px4();

	void send_the_command_to_px4();

	void send_message_to_sender();

	float get_ros_time(ros::Time begin); //获取ros当前时间

};
