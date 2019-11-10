/* this software is developed by the CGNC_LAB BIT, all rights reserved.*/
/* 作者：CGNC_LAB,BIT
 * 时间：2019.11.09
 */

// ros程序必备头文件
#include <ros/ros.h>

//mavros相关头文件
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

using namespace std;


class _FIXED_WING_FORMATION_CONTROL {

	private:
		mavros_msgs::State current_state;                       //无人机当前状态[包含上锁状态 模式] (从飞控中读取)

		mavros_msgs::PositionTarget pos_setpoint;               //即将发给无人机的控制指令

		mavros_msgs::PositionTarget pos_target;                 //从无人机回传的 vehicle_local_position_setpoint
		
		mavros_msgs::AttitudeTarget att_target;                 //从无人机回传的 vehicle_attitude_setpoint [四元数形式]
		
		sensor_msgs::Imu imu;
		
		float PIX_Euler_target[3];                              //无人机 期望欧拉角(从飞控中读取)
		float PIX_Euler[3];                                         //无人机当前欧拉角(从飞控中读取)
		float Thrust_target;                                    //期望推力
		

		struct {
		
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
		FIXED_WING_FORMATION_CONTROL();
			
		~FIXED_WING_FORMATION_CONTROL();

		void run();

		void test();

		void ros_sub_and_pub();

		void handle_status_from_receiver();

		void handle_message_from_px4();

		void send_the_command_to_px4();

		void send_message_to_sender();
		
		float get_ros_time(ros::Time begin);                                                 //获取ros当前时间
		
		void quaternion_2_euler(float quat[4], float angle[3]);                              //四元数转欧拉角

		//*******************ros_call_back*************************//
				void state_cb(const mavros_msgs::State::ConstPtr& msg)
			{
				current_state = *msg;
			}

			void pos_target_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
			{
				pos_target = *msg;
			}

			void att_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
			{
				att_target = *msg;

				float q[4];
				q[0] = msg->orientation.w;
				q[1] = msg->orientation.x;
				q[2] = msg->orientation.y;
				q[3] = msg->orientation.z;
				quaternion_2_euler(q, PIX_Euler_target);

				Thrust_target = msg->thrust;
			}

			void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
			{
				imu = *msg;
				float q[4];
				q[0] = msg->orientation.w;
				q[1] = msg->orientation.x;
				q[2] = msg->orientation.y;
				q[3] = msg->orientation.z;

				quaternion_2_euler(q, PIX_Euler);
			}


		//*******************ros_call_back*************************//



}

