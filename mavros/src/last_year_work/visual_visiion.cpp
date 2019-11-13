/***************************************************************************************************************************
 * catch_mission.cpp
 *
 * Author:
 *
 * Update Time: 2018.11.1
 *
 * 说明: mavros 完成目标追踪和抓取程序
 *      1. 订阅目标位置(来自视觉的ros节点)
 *      2. 追踪算法及追踪策略
 *      3. 抓取和搬运
 *      3. 发布上层控制指令
***************************************************************************************************************************/
//ros头文件
#include <ros/ros.h>

//topic 头文件
#include <mavros_msgs/Command.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Point.h>

//其他头文件
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

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
geometry_msgs::Pose pos_drone;              //无人机当前位置

geometry_msgs::Pose vision_delta_position;

std_msgs::Int32 Num_obj_vision;

int num_catch_obj;

float obj_1_x, obj_1_y;
float obj_2_x, obj_2_y;
float obj_3_x, obj_3_y;

float vision_width;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void quaternion_2_euler(float quat[4], float angle[3]);                              //四元数转欧拉角
void euler_2_quaternion(float angle[3], float quat[4]);                              //欧拉角转四元数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void Num_obj_cb(const std_msgs::Int32::ConstPtr &msg)
{
    Num_obj_vision = *msg;
    num_catch_obj = Num_obj_vision.data;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone = msg->pose;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 主 函 数 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_visiion");
    ros::NodeHandle nh("~");

    // 【订阅】无人机当前位置 坐标系:NED系
    //  本话题来自飞控(通过/plugins/local_position.cpp发送), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);

    // 【订阅】接收 搜索的物体编号
    ros::Subscriber Num_Obj_pub = nh.subscribe<std_msgs::Int32>("/vision/ObjNum", 10, Num_obj_cb);

    // 【发布】视觉消息
    //  方向定义： 目标位置[机体系下：前方x为正，右方y为正，下方z为正]
    //  标志位：   orientation.w 用作标志位 1代表识别到目标 0代表丢失目标
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::Pose>("/vision/target", 10);

    // 频率 [20Hz]
    // 这个频率取决于视觉程序输出的频率，一般不能低于10Hz，不然追踪效果不好
    ros::Rate rate(20.0);

    nh.param<float>("obj_1_x", obj_1_x, 0.5);
    nh.param<float>("obj_1_y", obj_1_y, 0.5);

    nh.param<float>("obj_2_x", obj_2_x, 0.5);
    nh.param<float>("obj_2_y", obj_2_y, 0.5);

    nh.param<float>("obj_3_x", obj_3_x, 0.5);
    nh.param<float>("obj_3_y", obj_3_y, 0.5);

    nh.param<float>("vision_width", vision_width, 1.5);

    vision_delta_position.position.x = 0;
    vision_delta_position.position.y = 0;
    vision_delta_position.position.z = 0;
    vision_delta_position.orientation.w = 0;

      while (ros::ok())
    {
        //回调
        ros::spinOnce();

        switch (num_catch_obj)
        {


        case 1:

            if ( abs(obj_1_x - pos_drone.position.x) < vision_width)\
            {
                vision_delta_position.position.x = obj_1_x - pos_drone.position.x;
                vision_delta_position.position.y = obj_1_y - pos_drone.position.y;
                //vision_delta_position.position.z = 0 - pos_drone.position.z;
                vision_delta_position.orientation.w = 1;
                vision_delta_position.position.z = 0;
            }
            else
            {
                //vision_delta_position.position.z = 0 - pos_drone.position.z;
                vision_delta_position.orientation.w = 0;  //out of vision
                vision_delta_position.position.z = 0;
            }

            vision_pub.publish(vision_delta_position);

            cout << endl;
            cout <<"--------"<<endl;
            cout<< "Num: " << num_catch_obj<<endl;
            cout << "vision x: " << vision_delta_position.position.x <<endl;
            cout << "vision y: " << vision_delta_position.position.y <<endl;
            cout << "vision flag: " << vision_delta_position.orientation.w <<endl;

            break;  //case 1.

        case 2:

            if ( abs(obj_2_x - pos_drone.position.x) < vision_width)\
            {
                vision_delta_position.position.x = obj_2_x - pos_drone.position.x;
                vision_delta_position.position.y = obj_2_y - pos_drone.position.y;
                //vision_delta_position.position.z = 0 - pos_drone.position.z;
                vision_delta_position.orientation.w = 1;
                vision_delta_position.position.z = 0;
            }
            else
            {
                //vision_delta_position.position.z = 0 - pos_drone.position.z;
                vision_delta_position.orientation.w = 0;  //out of vision
                vision_delta_position.position.z = 0;
            }

            vision_pub.publish(vision_delta_position);

            cout << endl;
            cout <<"--------"<<endl;
            cout<< "Num: " << num_catch_obj<<endl;
            cout << "vision x: " << vision_delta_position.position.x <<endl;
            cout << "vision y: " << vision_delta_position.position.y <<endl;
            cout << "vision flag: " << vision_delta_position.orientation.w <<endl;

            break;  // case 2.

           case 3:

            if ( abs(obj_3_x - pos_drone.position.x) < vision_width)\
            {
                vision_delta_position.position.x = obj_3_x - pos_drone.position.x;
                vision_delta_position.position.y = obj_3_y - pos_drone.position.y;
                //vision_delta_position.position.z = 0 - pos_drone.position.z;
                vision_delta_position.orientation.w = 1;
                vision_delta_position.position.z = 0;
            }
            else
            {
                //vision_delta_position.position.z = 0 - pos_drone.position.z;
                vision_delta_position.orientation.w = 0;  //out of vision
                vision_delta_position.position.z = 0;
            }

            vision_pub.publish(vision_delta_position);

            cout << endl;
            cout <<"--------"<<endl;
            cout<< "Num: " << num_catch_obj<<endl;
            cout << "vision x: " << vision_delta_position.position.x <<endl;
            cout << "vision y: " << vision_delta_position.position.y <<endl;
            cout << "vision flag: " << vision_delta_position.orientation.w <<endl;

            break; // case 2. fly to end_search_point

        default :
            vision_delta_position.position.x = 0;
            vision_delta_position.position.y = 0;
            vision_delta_position.position.z = 0;

            vision_delta_position.orientation.w = 0;

            vision_pub.publish(vision_delta_position);

            break;
        }

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
   // angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
    angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
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

//饱和函数
float satfunc(float data, float Max, float Thres)
{
    if (abs(data)<Thres)
    {
        return 0;
    }
    else if(abs(data)>Max)
    {
        return ( data > 0 ) ? Max : -Max;
    }
    else
    {
        return data;
    }
}
