/******************************************************
 * mavros_example_2.cpp                                *
 *                                                     *
 * Author: Qyp                                         *
 *                                                     *
 * Time: 2018.3.25                                     *
 *                                                     *
 * 说明: mavros示例程序1                                 *
 *      1. 上锁 订阅IMU数据                              *
 *      2. 纯做演示用,无实际用途                          *
 *                                                     *
 ******************************************************/

// ros程序必备头文件
#include <ros/ros.h>

//mavros相关头文件

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>

using namespace std;

sensor_msgs::Imu imu;
sensor_msgs::Imu imu_raw;
sensor_msgs::MagneticField mag;
sensor_msgs::Temperature tem;
sensor_msgs::FluidPressure atm;

float PIX_Euler[3];                                         //无人机当前欧拉角(从飞控中读取)

void quaternion_2_euler(float quat[4], float angle[3]);                              //四元数转欧拉角
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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
void imu_raw_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_raw = *msg;
}
void mag_cb(const sensor_msgs::MagneticField::ConstPtr& msg)
{
    mag = *msg;
}
void tem_cb(const sensor_msgs::Temperature::ConstPtr& msg)
{
    tem = *msg;
}
void atm_cb(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
    atm = *msg;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_example_2");
    ros::NodeHandle nh;

    // 频率 [50Hz]
    ros::Rate rate(50.0);

    // topic订阅 [消息流向: 飞控 通过mavlink发送到 mavros包, mavros包发布, 我们订阅使用]
    // topic发布 [消息流向: 本程序发布特定的topic,mavros接收指定的topic,封装成mavlink协议的消息发送给飞控, 飞控通过mavlink模块解码接收,存为本地UORB消息并发布给其他模块使用 ]

    // imu相关信息定义在 /home/nuc/mavros/src/mavros/mavros/src/plugins/imu_pub.cpp
    // 作用:接收并发布飞控传输过来的IMU信息

    //以下消息分别对应的是 IMU信息,原始IMU信息,磁力计信息,温度信息,气压信息.
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, imu_cb);

    ros::Subscriber imu_raw_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data_raw", 10, imu_raw_cb);

    ros::Subscriber mag_sub = nh.subscribe<sensor_msgs::MagneticField>("mavros/imu/mag", 10, mag_cb);

    ros::Subscriber temperature_sub = nh.subscribe<sensor_msgs::Temperature>("mavros/imu/temperature", 10, tem_cb);

    ros::Subscriber atm_pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>("mavros/imu/atm_pressure", 10, atm_cb);

    //一般在PX4中,姿态以四元数传递;但是显示的时候,我们一般开欧拉角比较直接,这里提供给大家四元数转欧拉角的函数



    while(ros::ok())
    {
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "imu: " <<endl;
        cout << "orientation: " << " " << imu.orientation.x << " "<< imu.orientation.y<<"  "<< imu.orientation.z<<" "<< imu.orientation.w << " " <<endl;
        cout << "Attitude: [roll pitch yaw] " << " " << PIX_Euler[0]*180/3.1415 << " ° "<< PIX_Euler[1]*180/3.1415 <<" ° "<< PIX_Euler[2]*180/3.1415 << " ° " <<endl;
        cout << "angular_velocity: " << " " << imu.angular_velocity.x << " "<< imu.angular_velocity.y<<"  "<< imu.angular_velocity.z<< " " <<endl;
        cout << "linear_acceleration: " << " " << imu.linear_acceleration.x << " "<< imu.linear_acceleration.y<<"  "<< imu.linear_acceleration.z<< " " <<endl;


        //其余存储下来的信息就不cout了..

        //回调
        ros::spinOnce();
        //挂起一段时间(rate为 50HZ)
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
}
