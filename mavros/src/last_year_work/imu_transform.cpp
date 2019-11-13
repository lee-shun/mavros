#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include<iomanip>
using namespace std;
//topic
#include <mavros_msgs/Command.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>


sensor_msgs::Imu imu_laser;
ros::Publisher imu_raw_pub;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void quaternion_2_euler(float quat[4], float angle[3]);                              //四元数转欧拉角[与PX4相同]
void euler_2_quaternion(float angle[3], float quat[4]);                              //欧拉角转四元数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void imu_raw_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu imu_laser;

    //欧拉角
    float euler_laser[3];
    float q[4];

    q[0] = msg->orientation.w;
    q[1] = msg->orientation.x;
    q[2] = msg->orientation.y;
    q[3] = msg->orientation.z;
    quaternion_2_euler(q, euler_laser);

    //NED系 到 laser系
    euler_laser[0] = euler_laser[0];
    euler_laser[1] = - euler_laser[1];
    euler_laser[2] = - euler_laser[2];

    //转换会四元数形式
    euler_2_quaternion(euler_laser, q);
    imu_laser.header = msg->header;
    imu_laser.orientation.w = 0;
    imu_laser.orientation.x = 0;
    imu_laser.orientation.y = 0;
    imu_laser.orientation.z = 0;
    imu_laser.orientation_covariance = msg->orientation_covariance;

    imu_laser.angular_velocity.x = msg->angular_velocity.x;
    imu_laser.angular_velocity.y = - msg->angular_velocity.y;
    imu_laser.angular_velocity.z = - msg->angular_velocity.z;
    imu_laser.angular_velocity_covariance = msg->angular_velocity_covariance;

    imu_laser.linear_acceleration.x = msg->linear_acceleration.x;
    imu_laser.linear_acceleration.y = - msg->linear_acceleration.y;
    imu_laser.linear_acceleration.z = - msg->linear_acceleration.z;
    imu_laser.linear_acceleration_covariance = msg->linear_acceleration_covariance;

    imu_raw_pub.publish(imu_laser);

}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_transform");
    ros::NodeHandle nh;

    // 【订阅】imu数据 针对需要融合imu数据的情况
    ros::Subscriber imu_raw_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw", 100, imu_raw_cb);

    // 【发布】imu数据 针对需要融合imu数据的情况
    imu_raw_pub = nh.advertise<sensor_msgs::Imu>("/imu", 100);

    // 频率 [100Hz]
    ros::Rate rate(100.0);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {

        //回调一次 更新传感器状态
        ros::spinOnce();

        cout << "IMU transforming..." <<endl;

        rate.sleep();

    }

    return 0;

}



// 四元数转Euler
// q0 q1 q2 q3
// w x y z
void quaternion_2_euler(float quat[4], float angle[3])
{
    angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
    //angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
}


// Euler转四元数
// q0 q1 q2 q3
// w x y z
void euler_2_quaternion(float angle[3], float quat[4])
{
    double cosPhi_2 = cos(double(angle[0]) / 2.0);

    double sinPhi_2 = sin(double(angle[0]) / 2.0);

    double cosTheta_2 = cos(double(angle[1] ) / 2.0);

    double sinTheta_2 = sin(double(angle[1] ) / 2.0);

    double cosPsi_2 = cos(double(angle[2]) / 2.0);

    double sinPsi_2 = sin(double(angle[2]) / 2.0);


    quat[0] = float(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);

    quat[1] = float(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);

    quat[2] = float(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);

    quat[3] = float(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);

}

