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
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
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

geometry_msgs::Pose pos_drone;              //无人机当前位置
geometry_msgs::Pose vel_drone;              //无人机当前速度

//------------------------------Search---------------
float search_start_x, search_start_y;       //搜索区开始点
float search_end_x, search_end_y;           //搜索区结束点
float search_z, search_yaw;                 //搜索高度，yaw

float search_vel_x;     //搜索速度x
float search_vel_y;     //搜索速度y

geometry_msgs::Pose Search_Back_Position;   //搜索返回点

//----------------------------- Vision---------------------------------------------
geometry_msgs::Pose pos_target;             // 目标位置[机体系下：前方x为正，右方y为正，下方z为正]
int flag_detected = 0;                      // 是否检测到目标 标志

//---------------------------------------Track---------------------------------------------
int Num_StateMachine = 0;                   // 状态机编号
int Num_StateMachine_Last = 0;              // 状态机编号last

float kpx_track;                                                //追踪比例系数
float kpy_track;                                                //追踪比例系数
float kpz_track;                                                //追踪比例系数

float track_max_vel_x;                                          //追踪最大速度x
float track_max_vel_y;                                          //追踪最大速度y
float track_max_vel_z;                                          //追踪最大速度z

float track_thres_vel_x;                                        //追踪速度死区x
float track_thres_vel_y;                                        //追踪速度死区y
float track_thres_vel_z;                                        //追踪速度死区z

float delta_x = 0.0;                                            //x方向偏差

float delta_disance;                                            //距离误差
float distance_thres = 0.2;          //追踪 距离 阈值
int num_distance_count = 0;          //追踪 距离 计数器
int count_distance = 0;              //追踪 距离 计数器阈值

int num_count_vision_lost = 0;       //视觉丢失计数器
int count_vision_lost = 0;           //视觉丢失计数器阈值

int num_catch_obj;                   //待抓取的物体编号
int catch_obj_total;                       //需要抓取的物体块总个数

//--------------------------- Others -------------------------------------------
char modeflag;  //选择飞行模式 t m p

float catch_max_vel;                        //抓取最大速度
float catch_max_z;                          //抓取最大高度

float Land_place_x, Land_place_y;           //着陆/起飞点

//---------------------------------------Output---------------------------------------------
mavros_msgs::Command Command_now;                               //发送给position_control.cpp的命令
std_msgs::Int32 Num_obj_vision;
std_msgs::UInt16 Flag_RoboticArm;
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

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_drone.position.x = msg->twist.linear.x;
    vel_drone.position.y = msg->twist.linear.y;
    vel_drone.position.z = msg->twist.linear.z;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone = msg->pose;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 主 函 数 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "track");
    ros::NodeHandle nh("~");

    // 【订阅】视觉消息 来自视觉节点
    //  方向定义： 目标位置[机体系下：前方x为正，右方y为正，下方z为正]
    //  标志位：   orientation.w 用作标志位 1代表识别到目标 0代表丢失目标
    ros::Subscriber vision_sub = nh.subscribe<geometry_msgs::Pose>("/vision/target", 10, vision_cb);

    // 【订阅】无人机当前位置 坐标系:NED系
    //  本话题来自飞控(通过/plugins/local_position.cpp发送), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);

    // 【订阅】无人机当前速度 坐标系:NED系
    //  本话题来自飞控(通过/plugins/local_position.cpp发送), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 100, vel_cb);


    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<mavros_msgs::Command>("/mavros/Command", 10);

    // 【发布】发送给图像的命信息 搜索的物体编号
    ros::Publisher Num_Obj_pub = nh.advertise<std_msgs::Int32>("/vision/ObjNum", 10);

    // 【发布】发送给arduion的命令
    ros::Publisher RoboticArm_pub = nh.advertise<std_msgs::UInt16>("/flag_close", 10);  // 1关闭； 0开启

    // 频率 [20Hz]
    // 这个频率取决于视觉程序输出的频率，一般不能低于10Hz，不然追踪效果不好
    ros::Rate rate(20.0);

    int comid = 0;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    //追踪算法 比例参数
    nh.param<float>("kpx_track", kpx_track, 1.0);
    nh.param<float>("kpy_track", kpy_track, 1.0);
    nh.param<float>("kpz_track", kpz_track, 1.0);

    //追踪算法 最大追踪速度
    //取决与使用场景，室内或者第一次实验时，建议选小一点
    nh.param<float>("track_max_vel_x", track_max_vel_x, 0.5);
    nh.param<float>("track_max_vel_y", track_max_vel_y, 0.5);
    nh.param<float>("track_max_vel_z", track_max_vel_z, 0.5);

    //追踪算法 速度死区
    nh.param<float>("track_thres_vel_x", track_thres_vel_x, 0.02);
    nh.param<float>("track_thres_vel_y", track_thres_vel_y, 0.02);
    nh.param<float>("track_thres_vel_z", track_thres_vel_z, 0.02);

    //追踪的前后间隔偏差
    nh.param<float>("delta_x", delta_x, 0.0);

    // 追踪距离阈值
    nh.param<float>("distance_thres", distance_thres, 0.2);
    // 追踪距离计数阈值
    nh.param<int>("count_distance", count_distance, 50);

    //视觉丢失次数阈值
    //处理视觉丢失时的情况
    nh.param<int>("count_vision_lost", count_vision_lost, 20);

    // 搜索相关定义
     // 搜索 起始点
      nh.param<float>("search_start_x", search_start_x, 0.0);
      nh.param<float>("search_start_y", search_start_y, 0.0);
     // 搜索 结束点
      nh.param<float>("search_end_x", search_end_x, 0.0);
      nh.param<float>("search_end_y", search_end_y, 0.0);
     //
      nh.param<float>("search_vel_x", search_vel_x, 0.0);
      nh.param<float>("search_vel_y", search_vel_y, 0.0);
     //搜索 高度 yaw
      nh.param<float>("search_z", search_z, 0.0);
      nh.param<float>("search_yaw", search_yaw, 0.0);

      //抓取相关参数
      nh.param<int>("catch_obj_total", catch_obj_total, 3);  //总共需要抓取的物体数

      nh.param<float>("catch_max_z", catch_max_z, -0.5);     //机械臂抓取高度
      nh.param<float>("catch_max_vel", catch_max_vel, 0.3);  //抓取最大速度


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

    int flag_command;                                               //机体系FLAG
    float setpoint_t[4];                                            //cin的目标位置点

    ros::spinOnce();
    rate.sleep();

    Land_place_x = pos_drone.position.x;
    Land_place_y = pos_drone.position.y;

    Flag_RoboticArm.data = 0;
    RoboticArm_pub.publish(Flag_RoboticArm);  //机械臂张开

    //选择模式：任务模式 或 打印模式
    cout << "Please choose mode(m for mission mode, p for only printf mode): "<<endl;
    cin >> modeflag;

    // 打印模式 -- 无实际用途 用于打印相关信息测试
    if (modeflag == 'p')
    {
        while (ros::ok())
        {
            ros::spinOnce();

            printf_track();
            rate.sleep();
         }
    }

 //任务模式
  if (modeflag == 'm')
   {
      Num_StateMachine_Last = 0;
      Num_StateMachine = 1;  //状态机 1

      num_catch_obj = 1;  //待抓取的物体编号
      num_distance_count = 0; //距离计数器 = 0

      while (ros::ok())
    {
        //回调
        ros::spinOnce();

        Num_obj_vision.data = num_catch_obj;
        Num_Obj_pub.publish(Num_obj_vision);  //发布目标数给视觉

        switch (Num_StateMachine)
        {

           case 0:
             if (Num_StateMachine == 0)
             {

             }
           break;  //case 0 takeoff

          //fly to start_search point 飞到搜索区起点
           case 1:

            Num_StateMachine_Last = Num_StateMachine;

            Command_now.command = Move;
            Command_now.sub_mode = 0;  //sub_mode: # 0 for xy/z position control; 3 for xy/z velocity control
            Command_now.comid = comid;
            Command_now.pos_sp[0] = search_start_x;
            Command_now.pos_sp[1] = search_start_y;
            Command_now.pos_sp[2] = search_z;
            Command_now.yaw_sp = search_yaw;
            comid++;

            //判断是否飞到了
             delta_disance = (pos_drone.position.x - Command_now.pos_sp[0]) * (pos_drone.position.x - Command_now.pos_sp[0]);
             delta_disance = (pos_drone.position.y - Command_now.pos_sp[1]) * (pos_drone.position.y - Command_now.pos_sp[1]) + delta_disance;
             //delta_disance = (pos_drone.position.z - Command_now.pos_sp[2]) * (pos_drone.position.z - Command_now.pos_sp[2]) + delta_disance;
             delta_disance = sqrt(delta_disance);

             if ( delta_disance < distance_thres)
             {
                 num_distance_count++;

                 if (num_distance_count > count_distance)

                 {   num_distance_count = 0;
                     Num_StateMachine = 2;    //进入2
                    // Command_now.command = Hold;   //发布悬停指令
                 }

             }
             else
             {
                 num_distance_count = 0;
             }

            command_pub.publish(Command_now);

            cout << endl << endl;
            cout << "---  1. Fly to Search_Start point  ---" <<endl;

            //printf_track();

           break;  // case 1. fly to start_search_point 飞到搜索区起点

          //fly to end_search_point  在搜索区域内 定速 搜索
            case 2:

             Command_now.command = Moving_Body;
             Command_now.sub_mode = 2;  //xy velocity z position
             Command_now.comid = comid;
             Command_now.vel_sp[0] = search_vel_x;
             Command_now.vel_sp[1] = search_vel_y;
             Command_now.pos_sp[2] = 0;
             Command_now.yaw_sp = search_yaw;
             comid++;

             Num_StateMachine_Last = Num_StateMachine;

             //计算和搜索终点 距离
             delta_disance = (pos_drone.position.x - search_end_x) * (pos_drone.position.x - search_end_x);
             delta_disance = (pos_drone.position.y - search_end_y) * (pos_drone.position.y - search_end_y) + delta_disance;
             delta_disance = sqrt(delta_disance);

             if ( flag_detected == 1) //发现了目标
               {  Num_StateMachine = 3;  //进入 追踪算法3
                  //记录下当前時刻位置
                 Search_Back_Position.position.x = pos_drone.position.x;
                 Search_Back_Position.position.y = pos_drone.position.y;
                 Command_now.command = Hold;
               }  //如果到达搜索终点， 则返航26 降落27
             else if ( delta_disance < distance_thres )
             { Num_StateMachine = 26;
               Command_now.command = Hold;
             }
             /*
              else if ( abs(pos_drone.position.x - search_start_x) > ( abs(search_end_x - search_start_x )+0.3 ) )
             {  Num_StateMachine = 26;
                Command_now.command = Hold;
             }
             else if ( abs(pos_drone.position.y - search_start_y) > ( abs(search_end_y - search_start_y ) +0.3) )
              {  Num_StateMachine = 26;
                Command_now.command = Hold;
              }*/

             command_pub.publish(Command_now);

             cout << endl << endl;
             cout << "---  2.  Search  ---" <<endl;

             //printf_track();

            break; // case 2. fly to end_search_point

          //Track  飞到目标正上方
             case 3:

               Num_StateMachine_Last = Num_StateMachine;

               //计算 和 目标的距离
               delta_disance = (pos_target.position.y) * (pos_target.position.y) + (pos_target.position.x) * (pos_target.position.x);
               delta_disance = sqrt(delta_disance);

               cout << "delta_disance : "<< delta_disance << endl;

               //如果 视觉丢失目标
               if ( flag_detected == 0)
               {   num_distance_count = 0;
                   Command_now.command = Hold; //发送悬停指令
                   Num_StateMachine = 21;     //同时无人机需要 返回中途搜索点21
               }
               else //如果捕捉到目标
               {
                //追踪在机体系下完成
                   Command_now.command = Moving_Body;
                   Command_now.sub_mode = 2;   //xy velocity z position
                   Command_now.comid = comid;
                   comid++;

                   Command_now.vel_sp[0] =  kpx_track * (pos_target.position.x - delta_x);
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

                   //如果当前与目标距离小于距离阈值
                   if ( delta_disance < distance_thres )
                   {
                       num_distance_count++;
                       if (num_distance_count > count_distance)
                       {
                           num_distance_count = 0;
                           Command_now.command = Hold; //发送悬停指令
                           Num_StateMachine = 7;       //无人机需要 降低高度4,并 抓取5  此处暂时先跳过抓取等任务
                       }
                   }
                   else //当前与目标距离大于距离阈值
                   {
                       num_distance_count = 0;  //距离计数器清零
                   }

               }

               command_pub.publish(Command_now);

               cout << endl << endl;

               cout << "---  3. Track  ---" <<endl;

               //printf_track();

             break;  //case 3. Track

           //  下降高度
            case 4:
              Num_StateMachine_Last = Num_StateMachine;

              Command_now.command = Moving_Body;
              Command_now.sub_mode = 3;   //xy velocity z velocity
              Command_now.comid = comid;
              comid++;

              //如果 视觉丢失目标
              if(flag_detected == 0)
              {   num_distance_count = 0;
                  Command_now.command = Hold; //发送悬停指令
                  Num_StateMachine = 21;       //同时无人机需要 返回中途搜索点21
              }
              else // 未丢失目标
              {
                  Command_now.vel_sp[0] =  kpx_track * (pos_target.position.x - delta_x);
                  Command_now.vel_sp[1] =  kpy_track * pos_target.position.y;
                  Command_now.vel_sp[2] =  kpz_track * (pos_target.position.z + catch_max_z);
                  Command_now.yaw_sp = 0;

                  //速度限幅
                  Command_now.vel_sp[0] = satfunc(Command_now.vel_sp[0], track_max_vel_x, track_thres_vel_x);
                  Command_now.vel_sp[1] = satfunc(Command_now.vel_sp[1], track_max_vel_y, track_thres_vel_y);
                  Command_now.vel_sp[2] = satfunc(Command_now.vel_sp[2], track_max_vel_z, track_thres_vel_z);

                  if (abs(pos_drone.position.z - catch_max_z) < 0.1)  //无人机达到抓取高度
                  { // 需要添加判断次数吗？
                      num_distance_count = 0;
                      Command_now.command = Hold; //发送悬停指令
                      Num_StateMachine = 5;       //进入抓取5
                  }
              }

              command_pub.publish(Command_now);
              printf_track();

              cout << "---  4. Down  ---" <<endl;

            break;   //case 4.

           //  抓取
            case 5:




            Flag_RoboticArm.data = 1;
            RoboticArm_pub.publish(Flag_RoboticArm);  //机械臂闭合

            Num_StateMachine = 6;
            cout << "---  5. Catch  ---" <<endl;


            break;   //case 5.

            //  抬升高度
             case 6:
                Num_StateMachine_Last = Num_StateMachine;

                Command_now.command = Moving_Body;
                Command_now.sub_mode = 1;  //xy pos z vel
                Command_now.comid = comid;
                Command_now.pos_sp[0] = 0;
                Command_now.pos_sp[1] = 0;
                Command_now.vel_sp[2] = -0.3;
                Command_now.yaw_sp = 0;
                comid++;

                Num_StateMachine_Last = Num_StateMachine;

                if( pos_drone.position.z < search_z) //爬升上限为设定搜索高度
                {
                    Num_StateMachine = 7;
                    Command_now.command = Hold;
                }

                command_pub.publish(Command_now);
                cout << "---  6. Climb  ---" <<endl;

             break;   //case 6.

            //  飞到放置物体地点
             case 7:
               Num_StateMachine_Last = Num_StateMachine;

               Num_StateMachine = 8;

             break;   //case 7.

            //  降低高度
             case 8:
               Num_StateMachine_Last = Num_StateMachine;


               Num_StateMachine = 9;

             break;   //case 8.

            //  放置物体
             case 9:
               Num_StateMachine_Last = Num_StateMachine;



               Flag_RoboticArm.data = 0;
               RoboticArm_pub.publish(Flag_RoboticArm);  //机械臂张开


               Num_StateMachine = 10;

             break;   //case 9.


            //  抬升高度
             case 10:
            //待完成
               Num_StateMachine_Last = Num_StateMachine;
               num_catch_obj++;

               //爬升完高度后, 根据目标数判断是 重回搜索起点1 还是 返航降落26
                 if (num_catch_obj > catch_obj_total)
                 {
                     Num_StateMachine = 26;
                     Command_now.command = Hold;
                 }
                  else
                 {

                     Num_StateMachine = 1;
                     Command_now.command = Hold;
                 }

                 command_pub.publish(Command_now);
                 printf_track();

             break;   //case 10.

          // 视觉目标丢失后， 返回中途搜索点
             case 21:

               Num_StateMachine_Last = Num_StateMachine;

               Command_now.command = Move;
               Command_now.sub_mode = 0;  //sub_mode: # 0 for xy/z position control; 3 for xy/z velocity control
               Command_now.comid = comid;
               Command_now.pos_sp[0] = Search_Back_Position.position.x;
               Command_now.pos_sp[1] = Search_Back_Position.position.y;
               Command_now.pos_sp[2] = search_z;
               Command_now.yaw_sp = search_yaw;
               comid++;

               //判断是否飞到了
               delta_disance = (pos_drone.position.x - Command_now.pos_sp[0]) * (pos_drone.position.x - Command_now.pos_sp[0]);
               delta_disance = (pos_drone.position.y - Command_now.pos_sp[1]) * (pos_drone.position.y - Command_now.pos_sp[1]) + delta_disance;
               //delta_disance = (pos_drone.position.z - Command_now.pos_sp[2]) * (pos_drone.position.z - Command_now.pos_sp[2]) + delta_disance;
               delta_disance = sqrt(delta_disance);

               if ( delta_disance < distance_thres)
               {   num_distance_count++;
                   if (num_distance_count > count_distance)
                   {
                       num_distance_count = 0;
                       Num_StateMachine = 2;
                       Command_now.command = Hold;  //进入2 继续搜索 同时发布悬停指令
                   }
               }
               else
               {
                   num_distance_count = 0;
               }

               command_pub.publish(Command_now);
               printf_track();

             break;   //case 21.  返回中途搜索点

            //Back to Land/Takeoff Place
            case 26:  //未写
                 Num_StateMachine_Last = Num_StateMachine;

                 Command_now.command = Move;
                 Command_now.sub_mode = 0;  //sub_mode: # 0 for xy/z position control; 3 for xy/z velocity control
                 Command_now.comid = comid;
                 Command_now.pos_sp[0] = Land_place_x;
                 Command_now.pos_sp[1] = Land_place_y;
                 Command_now.pos_sp[2] = search_z;
                 Command_now.yaw_sp = search_yaw;
                 comid++;

                 //判断是否飞到了
                  delta_disance = (pos_drone.position.x - Command_now.pos_sp[0]) * (pos_drone.position.x - Command_now.pos_sp[0]);
                  delta_disance = (pos_drone.position.y - Command_now.pos_sp[1]) * (pos_drone.position.y - Command_now.pos_sp[1]) + delta_disance;
                  //delta_disance = (pos_drone.position.z - Command_now.pos_sp[2]) * (pos_drone.position.z - Command_now.pos_sp[2]) + delta_disance;
                  delta_disance = sqrt(delta_disance);

                  if ( delta_disance < distance_thres)
                  {
                      num_distance_count++;

                      if (num_distance_count > count_distance)

                      {   num_distance_count = 0;
                          Num_StateMachine = 27;    //进入27
                         // Command_now.command = Hold;   //发布悬停指令
                      }

                  }
                  else
                  {
                      num_distance_count = 0;
                  }

                 command_pub.publish(Command_now);

                 cout << endl << endl;
                 cout << "---  1. Back to Land/Takeoff Place  ---" <<endl;

                 //printf_track();

            break;    //case 26. Back to Takeoff Place

           //Land
            case 27:
                 Command_now.command = Land;

                 Num_StateMachine_Last = Num_StateMachine;
                 command_pub.publish(Command_now);
                 printf_track();
            break;    //case 27. Land
        }

        rate.sleep();
    }


   } //对应 'm'
    return 0;

}

void printf_track()
{
    cout.setf(ios::fixed);
    cout << endl;
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
    cout << "count_distance : "<< count_distance << endl;

    cout << "kpx_track : "<< kpx_track << endl;
    cout << "kpy_track : "<< kpy_track << endl;
    cout << "kpz_track : "<< kpz_track << endl;

    cout << "track_max_vel_x : "<< track_max_vel_x << endl;
    cout << "track_max_vel_y : "<< track_max_vel_y << endl;
    cout << "track_max_vel_z : "<< track_max_vel_z << endl;

    cout << "track_thres_vel_x : "<< track_thres_vel_x << endl;
    cout << "track_thres_vel_y : "<< track_thres_vel_y << endl;
    cout << "track_thres_vel_z : "<< track_thres_vel_z << endl;

    cout << "count_vision_lost : "<< count_vision_lost << endl;

    cout << "search_start_x : "<< search_start_x << endl;
    cout << "search_start_y : "<< search_start_y << endl;
    cout << "search_end_x : "<< search_end_x << endl;
    cout << "search_end_y : "<< search_end_y << endl;

    cout << "search_vel_x : "<< search_vel_x << endl;
    cout << "search_vel_y : "<< search_vel_y << endl;
    cout << "search_z : "<< search_z << endl;
    cout << "search_yaw : "<< search_yaw << endl;

    cout << "catch_obj_total : "<< catch_obj_total << endl;
    cout << "catch_max_z : "<< catch_max_z << endl;
    cout << "catch_max_vel : "<< catch_max_vel << endl;

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
