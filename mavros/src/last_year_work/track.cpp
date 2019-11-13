/***************************************************************************************************************************
 * track.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2018.8.17
 *
 * 说明: mavros目标追踪示例程序
 *      1. 订阅目标位置(来自视觉的ros节点)
 *      2. 追踪算法及追踪策略
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
#define PI 3.1415926535898
enum Command
{
    Standby = 0,
    Takeoff,
    Hold,
    Land,
    Move,
    Disarm,
    Moving_Body
};
//---------------------------------------Vision---------------------------------------------
geometry_msgs::Pose pos_target;                                 //目标位置[机体系下：前方x为正，右方y为正，下方z为正]

int flag_detected = 0;                                          // 是否检测到目标标志
//---------------------------------------Track---------------------------------------------
int Num_StateMachine = 0;                                       // 状态机编号
int Num_StateMachine_Last = 0;                                  // 状态机编号last

float delta_x;
float distance_thres;
float kpx_track;                                                //追踪比例系数
float kpy_track;                                                //追踪比例系数
float kpz_track;                                                //追踪比例系数

int flag_x;

float track_max_vel_x;                                          //追踪最大速度
float track_max_vel_y;                                          //追踪最大速度
float track_max_vel_z;                                          //追踪最大速度

float track_thres_vel_x;                                          //追踪速度死区
float track_thres_vel_y;                                          //追踪速度死区
float track_thres_vel_z;                                          //追踪速度死区

int num_count_vision_lost = 0;                                                      //视觉丢失计数器
int count_vision_lost = 0;                                                          //视觉丢失计数器阈值
//---------------------------------------Output---------------------------------------------
mavros_msgs::Command Command_now;                               //发送给position_control.cpp的命令

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void quaternion_2_euler(float quat[4], float angle[3]);                              //四元数转欧拉角
void euler_2_quaternion(float angle[3], float quat[4]);                              //欧拉角转四元数
void printf_track();                                                                 //打印函数
void printf_param();                                                                 //打印各项参数以供检查
float satfunc(float data, float Max, float Thres);                                   //限幅函数
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void vision_cb(const geometry_msgs::Pose::ConstPtr &msg)
{
    pos_target = *msg;

    if(pos_target.orientation.w == 0)
    {
        num_count_vision_lost++;
    }else if(pos_target.orientation.w == 1)
    {
        flag_detected = 1;
        num_count_vision_lost = 0;
    }

    if(num_count_vision_lost > count_vision_lost)
    {
        flag_detected = 0;
    }

}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "track");
    ros::NodeHandle nh("~");

    // 【订阅】视觉消息 来自视觉节点
    //  方向定义： 目标位置[机体系下：前方x为正，右方y为正，下方z为正]
    //  标志位：   orientation.w 用作标志位 1代表识别到目标 0代表丢失目标
    ros::Subscriber vision_sub = nh.subscribe<geometry_msgs::Pose>("/vision/ellipse", 10, vision_cb);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<mavros_msgs::Command>("/mavros/Command", 10);

    // 频率 [20Hz]
    // 这个频率取决于视觉程序输出的频率，一般不能低于10Hz，不然追踪效果不好
    ros::Rate rate(20.0);

    int comid = 0;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    //追踪算法比例参数
    nh.param<float>("kpx_track", kpx_track, 1.0);
    nh.param<float>("kpy_track", kpy_track, 1.0);
    nh.param<float>("kpz_track", kpz_track, 1.0);

    //追踪算法最大追踪速度
    //取决与使用场景，室内或者第一次实验时，建议选小一点
    nh.param<float>("track_max_vel_x", track_max_vel_x, 0.5);
    nh.param<float>("track_max_vel_y", track_max_vel_y, 0.5);
    nh.param<float>("track_max_vel_z", track_max_vel_z, 0.5);

    //追踪算法速度死区
    nh.param<float>("track_thres_vel_x", track_thres_vel_x, 0.02);
    nh.param<float>("track_thres_vel_y", track_thres_vel_y, 0.02);
    nh.param<float>("track_thres_vel_z", track_thres_vel_z, 0.02);

    //前后方向是否追踪标志位 1 for track x, 0 for not track x
    //设计这个标志位是为了刚开始测试的时候不作前后的位移，较为安全
    nh.param<int>("flag_x", flag_x, 0);

    //视觉丢失次数阈值
    //处理视觉丢失时的情况
    nh.param<int>("count_vision_lost", count_vision_lost, 20);

    //追踪的前后间隔
    nh.param<float>("delta_x", delta_x, 1.5);

    //追踪距离阈值
    nh.param<float>("distance_thres", distance_thres, 0.2);


    //打印现实检查参数
    printf_param();

    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    int flag_command;                                                  //机体系FLAG
    float setpoint_t[4];                                            //cin的目标位置点

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        switch (Num_StateMachine)
        {
            // input
            case 0:
                Num_StateMachine_Last = Num_StateMachine;
                printf_track();
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;

                cout << "Please input command [0 for move[ned],1 for move[body],777 for track, 999 for land]: "<< endl;
                cin >> flag_command;

                //777 track
                if (flag_command == 777)
                {
                    Num_StateMachine = 4;
                    break;
                }
                //999  land
                else if (flag_command == 999)
                {
                    Num_StateMachine = 3;
                    break;
                }

                cout << "Please input next setpoint [x y z yaw]: "<< endl;

                cout << "setpoint_t[0] --- x [m] : "<< endl;
                cin >> setpoint_t[0];
                cout << "setpoint_t[1] --- y [m] : "<< endl;
                cin >> setpoint_t[1];
                cout << "setpoint_t[2] --- z [m] : "<< endl;
                cin >> setpoint_t[2];
                cout << "setpoint_t[3] --- yaw [du] : "<< endl;
                cout << "500 for input again"<< endl;
                cin >> setpoint_t[3];

                //500  重新输入各数值
                if (setpoint_t[3] == 500)
                {
                    Num_StateMachine = 0;
                }//如果是机体系移动
                else if(flag_command == 1)
                {
                    Num_StateMachine = 2;
                }//惯性系移动
                else if(flag_command == 0)
                {
                    Num_StateMachine = 1;
                }else
                {
                    Num_StateMachine = 0;
                }

            break;

            // 惯性系移动
            case 1:
                Command_now.command = Move;
                Command_now.sub_mode = 0;
                Command_now.comid = comid;
                Command_now.pos_sp[0] = setpoint_t[0];
                Command_now.pos_sp[1] = setpoint_t[1];
                Command_now.pos_sp[2] = setpoint_t[2];
                Command_now.yaw_sp = setpoint_t[4];
                comid++;

                Num_StateMachine_Last = Num_StateMachine;
                command_pub.publish(Command_now);
                printf_track();
                Num_StateMachine = 0;
            break;

            // 机体系移动
            case 2:
                Command_now.command = Moving_Body;
                Command_now.sub_mode = 0;
                Command_now.comid = comid;
                Command_now.pos_sp[0] = setpoint_t[0];
                Command_now.pos_sp[1] = setpoint_t[1];
                Command_now.pos_sp[2] = setpoint_t[2];
                Command_now.yaw_sp = setpoint_t[4];
                comid++;

                Num_StateMachine_Last = Num_StateMachine;
                command_pub.publish(Command_now);
                printf_track();
                Num_StateMachine = 0;
            break;

            //Land
            case 3:
                Command_now.command = Land;

                Num_StateMachine_Last = Num_StateMachine;
                command_pub.publish(Command_now);
                printf_track();
            break;

            //Track
            case 4:

                //首先计算距离期望位置距离
                float distance;

                if (flag_x == 0)
                {
                    distance = sqrt( pos_target.position.y * pos_target.position.y);
                }else
                {
                    distance = sqrt((pos_target.position.x-delta_x) * (pos_target.position.x-delta_x) + pos_target.position.y * pos_target.position.y);
                }

                cout << "distance : "<< distance << endl;

                //如果 视觉丢失目标 或者 当前与目标距离小于距离阈值
                //发送悬停指令
                if(flag_detected == 0 || (distance < distance_thres))
                {
                    Command_now.command = Hold;
                }
                //如果捕捉到目标
                else
                {
                    //追踪是在机体系下完成
                    Command_now.command = Moving_Body;
                    Command_now.sub_mode = 2;   //xy velocity z position
                    Command_now.comid = comid;
                    comid++;

                    if (flag_x == 0)
                    {
                        Command_now.vel_sp[0] =  0;
                    }else
                    {
                        Command_now.vel_sp[0] =  kpx_track * (pos_target.position.x - delta_x);
                    }

                    Command_now.vel_sp[1] =  kpy_track * pos_target.position.y;
                    //Command_now.vel_sp[2] =  kpz_track * pos_target.position.z;
                    Command_now.pos_sp[2] =  0;

                    //目前航向角锁定
                    Command_now.yaw_sp = 0;

                    //速度限幅
                    Command_now.vel_sp[0] = satfunc(Command_now.vel_sp[0], track_max_vel_x, track_thres_vel_x);
                    Command_now.vel_sp[1] = satfunc(Command_now.vel_sp[1], track_max_vel_y, track_thres_vel_y);
                   // Command_now.vel_sp[2] = satfunc(Command_now.vel_sp[2], track_max_vel_z, track_thres_vel_z);

                    //如果期望速度为0,则直接执行悬停指令
                    if(Command_now.vel_sp[0]==0 && Command_now.vel_sp[1] == 0)
                    {
                        Command_now.command = Hold;
                    }

                }
                Num_StateMachine_Last = Num_StateMachine;
                command_pub.publish(Command_now);
                printf_track();

            break;
         }
        rate.sleep();
    }

    return 0;

}

void printf_track()
{
    cout.setf(ios::fixed);
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Num_StateMachine : " << Num_StateMachine <<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "flag_detected: " <<  flag_detected <<endl;
    cout << "num_count_vision_lost: " <<  num_count_vision_lost <<endl;

    cout << "pos_target: [X Y Z] : " << " " << pos_target.position.x  << " [m] "<< pos_target.position.y  <<" [m] "<< pos_target.position.z <<" [m] "<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Command: " << Command_now.vel_sp[0] << " [m/s] "<< Command_now.vel_sp[1] << " [m/s] "<< Command_now.vel_sp[2] << " [m/s] "<<endl;
}

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "distance_thres : "<< distance_thres << endl;
    cout << "delta_x : "<< delta_x << endl;

    cout << "kpx_track : "<< kpx_track << endl;
    cout << "kpy_track : "<< kpy_track << endl;
    cout << "kpz_track : "<< kpz_track << endl;

    cout << "track_max_vel_x : "<< track_max_vel_x << endl;
    cout << "track_max_vel_y : "<< track_max_vel_y << endl;
    cout << "track_max_vel_z : "<< track_max_vel_z << endl;


    cout << "track_thres_vel_x : "<< track_thres_vel_x << endl;
    cout << "track_thres_vel_y : "<< track_thres_vel_y << endl;
    cout << "track_thres_vel_z : "<< track_thres_vel_z << endl;
    cout << "flag_x : "<< flag_x << endl;
    cout << "count_vision_lost : "<< count_vision_lost << endl;



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


