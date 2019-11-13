/***************************************************************************************************************************
 * track_land.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2018.8.17
 *
 * 说明: 基于单目摄像头的自主降落程序
 *      1. 订阅目标位置(来自视觉的ros节点)
 *      2. 追踪算法及降落策略
 *      3. 发布上层控制指令
***************************************************************************************************************************/
//ROS 头文件
#include <ros/ros.h>

//msg 头文件
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/Command.h>
#include <sensor_msgs/Imu.h>

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
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全局变量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//-----------------------------------------无人机状态量相关----------------------------------------------------
geometry_msgs::Pose pos_drone;                                  //无人机当前位置
geometry_msgs::Pose vel_drone;                                  //无人机当前位置
float PIX_Euler[3];                                             //无人机当前欧拉角 单位:rad
int Num_StateMachine = 0;                                       //状态机编号
int Num_StateMachine_Last = 0;                                  //上一时刻 状态机编号

mavros_msgs::Command Command_now;                               //发送给position_control.cpp的topic
int comid = 1;                                                  //Command_now的id号

geometry_msgs::Pose pos_vehicle;                                  //无人机当前位置
float vehicle_Euler[3];                                             //无人机当前欧拉角 单位:rad
//-----------------------------------------视觉相关----------------------------------------------------
float flag_vision = 0;                                          //视觉FLAG 是否识别到目标 1代表能识别到目标，0代表不能
geometry_msgs::Point relative_position;                         //机体固连坐标系下 降落板的位置
float relative_yaw;                                             //降落板与无人机的相对偏航角 单位:rad
float relative_yaw_last;
float height_on_pad;                                            //降落板高度

geometry_msgs::Pose vel_pad;                                    //降落板移动速度
//-----------------------------------------降落控制相关----------------------------------------------------
float kpx_land,kpy_land,kpz_land;                                                 //控制参数 - 比例参数
float Max_velx_land, Max_vely_land, Max_velz_land;                                //控制参数 - 最大值限幅
float Thres_velx_land, Thres_vely_land, Thres_velz_land;                          //控制参数 - 死区限幅

float distance_pad;                                                               //无人机与降落板中心的距离
float Thres_distance_land,Thres_count_land;                                       //控制参数 - 阈值（追踪程序）
int flag_track_yaw;
int num_count = 0;
float fly_min_z;
int Flag_reach_pad_center = 0;
int Flag_z_below_30cm = 0;
float land_max_z;

int num_count_lost = 0;
float Thres_vision_lost = 30;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float satfunc(float data, float Max, float Thres);                           //限幅函数
void track_land();
void printf_land();
float get_dt(ros::Time last);
void save_flight_data(std::ofstream& out_file, float timenow);
void quaternion_2_euler(float quat[4], float angle[3]);
void euler_2_quaternion(float angle[3], float quat[4]);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone = msg->pose;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_drone.position.x = msg->twist.linear.x;
    vel_drone.position.y = msg->twist.linear.y;
    vel_drone.position.z = msg->twist.linear.z;
}

void euler_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    float q[4];
    q[0] = msg->orientation.w;
    q[1] = msg->orientation.x;
    q[2] = msg->orientation.y;
    q[3] = msg->orientation.z;
    quaternion_2_euler(q, PIX_Euler);
}

void relative_position_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
    relative_position = msg->position;
}

void relative_yaw_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
    relative_yaw = msg->orientation.w;
}

void vision_flag(const geometry_msgs::Point::ConstPtr& msg)
{
    flag_vision = msg->x;
}

void vehicle_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //位置 -- optitrack系 到 NED系
    pos_vehicle.position.x = msg->pose.position.y;
    pos_vehicle.position.y = msg->pose.position.x;
    pos_vehicle.position.z = - msg->pose.position.z;


    //欧拉角
    float euler_optitrack[3];
    float q[4];
    q[0] = msg->pose.orientation.w;
    q[1] = msg->pose.orientation.x;
    q[2] = msg->pose.orientation.y;
    q[3] = msg->pose.orientation.z;
    quaternion_2_euler(q, euler_optitrack);

    //optitrack系 到 NED系
    vehicle_Euler[0] = euler_optitrack[0];
    vehicle_Euler[1] = - euler_optitrack[1];
    vehicle_Euler[2] = - euler_optitrack[2];

}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_land");
    ros::NodeHandle nh("~");

    //节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate rate(20.0);

    // 【订阅】无人机当前位置 坐标系:NED系
    //  本话题来自飞控(通过/plugins/local_position.cpp发送), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);

    // 【订阅】无人机当前速度 坐标系:NED系
    //  本话题来自飞控(通过/plugins/local_position.cpp发送), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 100, vel_cb);


    //【订阅】降落板与无人机的相对位置 单位：米
    // 来自视觉节点 方向定义：[机体系下：前方x为正，右方y为正，下方z为正]
    ros::Subscriber relative_pos_sub = nh.subscribe<geometry_msgs::Pose>("/vision/relative_position", 10, relative_position_cb);

    //【订阅】降落板与无人机的相对偏航角 单位：弧度
    // 利用orientation.w 传递
    ros::Subscriber relative_yaw_sub = nh.subscribe<geometry_msgs::Pose>("/vison/relative_yaw", 10, relative_yaw_cb);

    //【订阅】视觉flag 来自视觉节点
    // orientation.w ： 0 for目标丢失，1 for 正常识别
    ros::Subscriber vision_flag_sub = nh.subscribe<geometry_msgs::Point>("/vision/vision_flag", 10, vision_flag);

    // 【订阅】小车位置 来自VICON系统 可直接用于控制 也可用于数据记录
    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/vehicle/pose", 30, vehicle_cb);

    // 【发布】发送给position_control.cpp的命令
    ros::Publisher command_pub = nh.advertise<mavros_msgs::Command>("/mavros/Command", 10);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //降落追踪控制算法 的比例参数
    nh.param<float>("kpx_land", kpx_land, 0.3);
    nh.param<float>("kpy_land", kpy_land, 0.3);
    nh.param<float>("kpz_land", kpz_land, 0.3);

    //降落追踪控制算法的最大速度
    nh.param<float>("Max_velx_land", Max_velx_land, 1.0);
    nh.param<float>("Max_vely_land", Max_vely_land, 1.0);
    nh.param<float>("Max_velz_land", Max_velz_land, 1.0);

    //降落追踪控制算法的速度死区
    nh.param<float>("Thres_velx_land", Thres_velx_land, 0.02);
    nh.param<float>("Thres_vely_land", Thres_vely_land, 0.02);
    nh.param<float>("Thres_velz_land", Thres_velz_land, 0.02);

    //允许降落最大距离阈值
    nh.param<float>("Thres_distance_land", Thres_distance_land, 0.2);

    //允许降落计数阈值
    nh.param<float>("Thres_count_land", Thres_count_land, 30);

    //允许降落最大高度阈值
    nh.param<float>("land_max_z", land_max_z, 0.3);

    //允许飞行最低高度[这个高度是指降落板上方的相对高度]
    nh.param<float>("fly_min_z", fly_min_z, 0.3);

    //视觉丢失计数阈值
    nh.param<float>("Thres_vision_lost", Thres_vision_lost, 30);

    //降落板高度
    nh.param<float>("height_on_pad", height_on_pad, -0.4);

    //是否追踪偏航角flag 0 for not track yaw，1 for track
    nh.param<int>("flag_track_yaw", flag_track_yaw, 0);

    float vel_pad_x;
    //降落板速度
    nh.param<float>("vel_pad_x", vel_pad_x, 0.0);

    vel_pad.position.x = vel_pad_x;
    vel_pad.position.y = 0;
    vel_pad.position.z = 0;


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>储存数据<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    ros::Time begin_time = ros::Time::now();

    time_t tt = time(NULL);
    tm* t = localtime(&tt);
    char iden_path[256];
    sprintf(iden_path, "/home/nvidia/mavros/data/track-land-%d-%02d-%02d_%02d-%02d.txt", t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min);
    std::ofstream out_data_file(iden_path);

    if (!out_data_file)
    {
        std::cout << "Error: Could not write data!" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "save data!!"<<std::endl;
        out_data_file <<" Time "<< " pos_drone.x " << " pos_drone.y " << " pos_drone.z " \
                                << " vel_drone.x " << " vel_drone.y " << " vel_drone.z " \
                                << " pos_drone.yaw " \
                                << " relative.x " << " relative.y " << " relative.z " \
                                << " relative.yaw " \
                                << " vel_sp.x " << " vel_sp.y " << " vel_sp.z " \
                                << " pos_vehicle.x " << " pos_vehicle.y " << " pos_vehicle.z " \
                                << " pos_vehicle.yaw " \
                                <<std::endl;
    }


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>模式选择<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //打印现实检查参数
    //printf_param();

    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }


    Num_StateMachine = 0;

    int flag_command;                                                  //机体系FLAG
    float setpoint_t[4];                                            //cin的目标位置点

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        float cur_time = get_dt(begin_time);

        switch (Num_StateMachine)
        {
            // input
            case 0:
                Num_StateMachine_Last = Num_StateMachine;
                printf_land();
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;

                cout << "Please input command [0 for move[ned],1 for move[body],777 for track_land, 999 for land]: "<< endl;
                cin >> flag_command;

                //777 track_land
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
                Command_now.yaw_sp = setpoint_t[3];
                comid++;

                Num_StateMachine_Last = Num_StateMachine;
                command_pub.publish(Command_now);
                printf_land();
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
                Command_now.yaw_sp = setpoint_t[3];
                comid++;

                Num_StateMachine_Last = Num_StateMachine;
                command_pub.publish(Command_now);
                printf_land();
                Num_StateMachine = 0;
                break;

            //Land
            case 3:
                Command_now.command = Land;

                Num_StateMachine_Last = Num_StateMachine;
                command_pub.publish(Command_now);
                printf_land();
                break;

            //追踪降落
            case 4:

                //自主降落追踪算法
                track_land();

                //计算与降落板之间的距离
                distance_pad = sqrt(relative_position.x * relative_position.x + relative_position.y * relative_position.y);

                cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>Land State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
                //降落的三个条件：
                //1：水平距离小于阈值，即无人机抵达降落板中心位置附近
                //2：相对高度小于阈值，

                //如果 相对距离 小于 阈值，计数增加
                if(distance_pad < Thres_distance_land && flag_vision == 1)
                {
                    num_count++;
                    cout<< "Distance_pad: " << distance_pad <<endl;
                    cout<< "Distance_land_count: " << num_count <<endl;
                }else
                {
                    num_count = 0;
                    cout<< "Distance_pad: " << distance_pad <<endl;
                    cout<< "Distance_land_count: " << num_count <<endl;
                }

                //如果计数增加超过阈值，则满足降落的第一个条件（水平距离）
                if(distance_pad < Thres_distance_land && num_count > Thres_count_land)
                {
                    cout<< "Flag_reach_pad_center: " << "true" <<endl;
                    Flag_reach_pad_center = 1;
                }else
                {
                    cout<< "Flag_reach_pad_center: " << "flase" <<endl;
                    Flag_reach_pad_center = 0;
                }

                //如果 相对高度 小于 阈值，则满足降落的第二个条件（高度条件）
                if(relative_position.z <=  land_max_z)
                {
                    cout<< "Flag_z_below_30cm: " << "true" <<endl;
                    Flag_z_below_30cm = 1;
                }else
                {
                    cout<< "Flag_z_below_30cm: " << "flase" <<endl;
                    Flag_z_below_30cm = 0;
                }

                //如果降落的两个条件都满足，则切换状态机（上锁）
                if(Flag_reach_pad_center == 1 && Flag_z_below_30cm == 1)
                {
                    // arm!
                    Num_StateMachine = 5;
                }else
                {
                    // keep track
                    Num_StateMachine = 4;
                }

                //如果视觉丢失了降落板目标，计数增加
                if(flag_vision == 0)
                {
                    num_count_lost++;
                    cout<< "vision_lost_num: " << num_count_lost <<endl;
                }else
                {
                    num_count_lost = 0;
                    cout<< "vision_lost_num: " << num_count_lost <<endl;
                }

                //如果丢失计数超过阈值，则切换状态机（爬升）
                if(num_count_lost > Thres_vision_lost)
                {
                    num_count_lost = 0;
                    Num_StateMachine = 6;
                }

                //发布控制量
                command_pub.publish(Command_now);
                Num_StateMachine_Last = Num_StateMachine;
                //打印相关信息
                printf_land();
                break;

        case 5:
            Command_now.command = Disarm;
            command_pub.publish(Command_now);
            Num_StateMachine_Last = Num_StateMachine;
            //打印相关信息
            printf_land();

            break;

        case 6:

            Command_now.command = Moving_Body;
            Command_now.sub_mode = 1;  //xy pos z vel
            Command_now.comid = comid;
            Command_now.pos_sp[0] = 0;
            Command_now.pos_sp[1] = 0;
            Command_now.vel_sp[2] = -0.3;
            Command_now.yaw_sp = 0;
            comid++;

            Num_StateMachine_Last = Num_StateMachine;
            command_pub.publish(Command_now);

            cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>Search State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
            //重新获得视觉信息，计数
            if(flag_vision == 1)
            {
                num_count_lost++;
                cout<< "vision_regain_num: " << num_count_lost <<endl;
            }else
            {
                num_count_lost = 0;
                cout<< "vision_regain_num: " << num_count_lost <<endl;
            }

            //如果重新获得视觉计数超过阈值，则切换状态机（追踪降落）
            if(num_count_lost > Thres_vision_lost)
            {
                num_count_lost = 0;
                Num_StateMachine = 4;
            }

            //爬升上限为2.5米
            if(pos_drone.position.z < -2.5)
            {
                Command_now.command = Moving_Body;
                Command_now.sub_mode = 0;  //xy pos z vel
                Command_now.comid = comid;
                Command_now.pos_sp[0] = 0;
                Command_now.pos_sp[1] = 0;
                Command_now.pos_sp[2] = 0;
                Command_now.yaw_sp = 0;
                comid++;
                command_pub.publish(Command_now);
                Num_StateMachine = 0;
            }

            break;

        }

        save_flight_data(out_data_file,cur_time);

        rate.sleep();

    }

    return 0;

}


//自主降落追踪算法
void track_land()
{
    Command_now.command = Moving_Body;
    Command_now.comid = comid;
    comid++;

    //小技巧：
    //虽然视觉系统也能获得无人机和降落板的相对高度，但毕竟不是特别准确
    //因为降落板的高度已知，且在平面内运动，可通过无人机自身高度去结算与降落板的相对高度
    relative_position.z = (height_on_pad  - pos_drone.position.z);

    //xyz速度控制模式
    Command_now.sub_mode = 3; // xy velocity z velocity
    //如果要去追踪一个动态的降落板，则需要反馈其速度
    Command_now.vel_sp[0] =  kpx_land * relative_position.x + vel_pad.position.x;              // vel_pad.position.x为降落板移动速度
    Command_now.vel_sp[1] =  kpy_land * relative_position.y + vel_pad.position.y;
    //z轴的期望位置是 降落板上方的某一个高度
    //之所以这么做是因为飞机飞得太低，视觉系统无法捕捉降落板
    Command_now.vel_sp[2] =  kpz_land * (relative_position.z - fly_min_z);

    //不追踪偏航角，则锁死偏航角为0
    if(flag_track_yaw == 0)
    {
        Command_now.yaw_sp =0;
    }else
    {
        //死区
        if(abs(relative_yaw/3.1415926 * 180) < 3)
        {
            relative_yaw = 0;
        }

        Command_now.yaw_sp =( PIX_Euler[2] + relative_yaw) /3.1415926 * 180;
    }

    //饱和函数
    satfunc(Command_now.vel_sp[0], Max_velx_land, Thres_velx_land);
    satfunc(Command_now.vel_sp[1], Max_vely_land, Thres_vely_land);
    satfunc(Command_now.vel_sp[2], Max_velz_land, Thres_velz_land);
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

void printf_land()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Num_StateMachine : " << Num_StateMachine <<endl;
    cout << "Position: [X Y Z] : " << " " << pos_drone.position.x << " [m] "<< pos_drone.position.y<<" [m] "<<pos_drone.position.z<<" [m] "<<endl;
    cout << "Velocity: [X Y Z] : " << " " << vel_drone.position.x << " [m/s] "<< vel_drone.position.y<<" [m/s] "<<vel_drone.position.z<<" [m/s] "<<endl;
    cout << "euler_drone: : " << " " << PIX_Euler[2]/3.1415926*180<<" [du] "<<endl;
    cout << "Position_car: [X Y Z] : " << " " << pos_vehicle.position.x << " [m] "<< pos_vehicle.position.y<<" [m] "<<pos_vehicle.position.z<<" [m] "<<endl;
    cout << "euler_Car: : " << " " << vehicle_Euler[2]/3.1415926*180<<" [du] "<<endl;
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "flag_vision: " << flag_vision <<endl;
    cout << "relative position: " << relative_position.x << " [m] "<< relative_position.y << " [m] "<< relative_position.z << " [m] "<<endl;
    cout << "relative_yaw: " << relative_yaw/3.1415926 *180 << " [du] "<<endl;


    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Land Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "command: " << Command_now.vel_sp[0] << " [m/s] "<< Command_now.vel_sp[1] << " [m/s] "<< Command_now.vel_sp[2] << " [m/s] "<<endl;
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

//获取当前时间 单位：秒
float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}
void save_flight_data(std::ofstream& out_file, float timenow)
{
    out_file << timenow <<"  "<< pos_drone.position.x <<"  "<< pos_drone.position.y <<"  "<< pos_drone.position.z <<"  "\
                              << vel_drone.position.x <<"  "<< vel_drone.position.y <<"  "<< vel_drone.position.z <<"  "\
                            << PIX_Euler[2] <<"  "\
                            << relative_position.x <<"  "<< relative_position.y <<"  "<< relative_position.z <<"  "\
                             << relative_yaw  <<"  "\
                            << Command_now.vel_sp[0] <<"  "<< Command_now.vel_sp[1] <<"  "<< Command_now.vel_sp[2] <<"  "\
                            << pos_vehicle.position.x <<"  "<< pos_vehicle.position.y <<"  "<< pos_vehicle.position.z <<"  "\
                            << vehicle_Euler[2] <<"  "\
                              << std::endl;
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
