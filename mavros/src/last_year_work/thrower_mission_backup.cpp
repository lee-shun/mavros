/***************************************************************************************************************************
无人蜂群 2019
@file:/home/ubuntu/ws_dji/src/Onboard-SDK-ROS/dji_sdk_demo/src/mission_thrower.cpp
@author: Zzy
@date:2019.04.07
@date:2019.04.19
***************************************************************************************************************************/
//ROS 头文件
#include <ros/ros.h>

//msg 头文件
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/Joy.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/NavSatFix.h>

//DJI SDK 头文件
//#include <dji_sdk/dji_sdk.h>
//#include <dji_sdk/DroneTaskControl.h>
//#include <dji_sdk/SDKControlAuthority.h>
//#include <dji_sdk/QueryDroneVersion.h>
//#include <dji_sdk/SetLocalPosRef.h>

#include <mavros/Data_Throw_target.h>
#include <mavros/Data_Drone_State.h>

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
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全局变量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define PI 3.1415926535898

const uint8_t CONFIG_ID = 0x03;           //通信模块编号 | 与飞机编号相同   0x01&0x02_侦察机    0x03-0x08_投掷机    0x0a_地面站
//-----------------------------------------无人机状态量相关----------------------------------------------------
sensor_msgs::NavSatFix current_gps;                        //无人机当前GPS坐标
geometry_msgs::Point current_local_pos;                    //无人机当前XYZ位置
geometry_msgs::Point current_local_vel;                    //无人机当前XYZ速度
geometry_msgs::Quaternion current_attitude;                //无人机当前姿态   四元数形式
float euler_drone[3];                                      //无人机当前欧拉角 欧拉角形式 单位：rad
uint8_t flight_status = 255;                               //M100状态量
int Drone_Number;                                          //M100无人编号
//-----------------------------------------飞行控制相关----------------------------------------------------
geometry_msgs::Pose pos_sp;                                 //无人机期望的位置 控制函数中的期望值 惯性系
float kpx,kpy,kpz,kpyaw;                                    //控制参数 - 比例参数
float kdx,kdy,kdz;                                          //控制参数 - 微分参数
float Max_velx, Max_vely, Max_velz, Max_velyaw;             //控制参数 - 最大值限幅
float Max_velx_return, Max_vely_return, Max_velz_return, Max_velyaw_return;             //控制参数 - 最大值限幅
float Thres_velx, Thres_vely, Thres_velz, Thres_velyaw;     //控制参数 - 死区限幅
float Thres_distance,Thres_count;                           //控制参数 - 阈值

float xCmd, yCmd, zCmd, yawCmd;                             //控制函数输出 - 期望速度控制量
//-----------------------------------------GPS相关----------------------------------------------------
double Lon1, Lat1;                                          //1 起飞点-经度、纬度
double Lon2, Lat2;                                          //2 投掷目标点 经度、纬度
double LonEN,LatEN,LonWS,LatWS;                             //搜索区域-东北角西南角-GPS点 经纬度
double dZ, dX;                                              //用于计算出ENU系下和起飞点的位置--dz指向正东 dx指向正北
//-----------------------------------------任务执行相关----------------------------------------------------
char modeflag;                                             // 定义飞行模式 t 代表测试模式  m 代表任务模式
int Num_StateMachine = 0;                                  // 状态机编号
int Num_StateMachine_Last = 0;                             // 状态机编号_last
int num_count = 0;                                         // 计数器
//-----------------------------------------投掷相关----------------------------------------------------
std_msgs::UInt16 drop;              // 投掷信息 0-未投    1-开     0->1投掷  1->0装载 // 允许投掷flag
float Throw_height;                 //投掷高度

//-----------------------------------------电台通讯相关----------------------------------------------------
mavros::Data_Throw_target received_mission;
mavros::Data_Drone_State wait_Send_Data;

sensor_msgs::NavSatFix current_state;

int target_ID;//投掷目标圆 编号
int target_type;//投掷目标圆 类型
int target_Num;//投掷目标圆 已投掷次数
double target_Lat, target_Lon;                              //投掷目标圆GPS 纬度&经度

//-----------------------------------------起飞返航降落相关----------------------------------------------------
geometry_msgs::Pose climb_point;                                                    // 起飞爬升点
geometry_msgs::Pose rerun_point;                                                    // 返回点
geometry_msgs::Pose land_point;                                                     // 着陆降落点
geometry_msgs::Pose throw_target_point;                                             // 投掷目标点

bool takeoff_result;                                                                //M100 起飞结果判断
int Thres_count_takeoff;                                                            //起飞前等待时间
float fly_height;                                                                   //飞往搜索区域的飞行高度
float fly_height_delta;                                                             //飞往搜索区域的飞行高度delta
float fly_yaw;                                                                      // 飞行保持偏航角

/* //-----------------------------------------追踪相关----------------------------------------------------
float kpx_track,kpy_track,kpz_track,kpyaw_track;                                    //控制参数 - 比例参数 （追踪程序）
float Max_velx_track, Max_vely_track, Max_velz_track, Max_velyaw_track;             //控制参数 - 最大值限幅 （追踪程序）
float Thres_velx_track, Thres_vely_track, Thres_velz_track, Thres_velyaw_track;     //控制参数 - 死区限幅 （追踪程序）
float Thres_distance_track,Thres_count_track;                                       //控制参数 - 阈值（追踪程序）
//-----------------------------------------搜索策略相关----------------------------------------------------
//float Max_velx_search, Max_vely_search, Max_velz_search, Max_velyaw_search;             // 控制参数 - 最大值限幅(search)
*/

//-----------------------------------------发布及服务声明----------------------------------------------------
ros::Publisher Pospub;                                      // 发布期望位置给DJI SDK
ros::Publisher Velpub;                                      // 发布期望速度给DJI SDK
ros::Publisher SendDatapub;
ros::Publisher drop_pub;
ros::Publisher Throw_target_pub;
ros::Publisher Num_state_machine_pub;
ros::Publisher Data_Drone_State_pub;

ros::ServiceClient drone_task_service;                      // 起飞、降落等特殊服务
ros::ServiceClient sdk_ctrl_authority_service;              // 获取控制权限服务
ros::ServiceClient set_local_pos_reference;                 // 设置本地坐标参考点服务

//enum M100FlightStatus
//{
//    M100_STATUS_ON_GROUND        = DJI::OSDK::VehicleStatus::M100FlightStatus::ON_GROUND_STANDBY,
//    M100_STATUS_TAKINGOFF        = DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF,
//    M100_STATUS_IN_AIR           = DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY,
//    M100_STATUS_LANDING          = DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING,
//    M100_STATUS_FINISHED_LANDING = DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING
//};


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool takeoff_land(int task);
bool M100monitoredTakeoff();
float abs_distance(geometry_msgs::Pose pos_sp,geometry_msgs::Point current_local_pos);
void printf_M100_info();
void M100_pidcontrol();
bool obtain_control();
bool set_local_position();
void quaternion_2_euler(float quat[4], float angle[3]);
float satfunc(float data, float Max, float Thres);
void printf_param();

//void track();
void wait();    //悬停指令?
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    current_attitude = msg->quaternion;
    float q[4];

    q[0] = current_attitude.w;
    q[1] = current_attitude.x;
    q[2] = current_attitude.y;
    q[3] = current_attitude.z;
    quaternion_2_euler(q, euler_drone);
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    current_local_pos = msg->point;
}

void local_velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    //这里，由于DJI SDK发布的速度方向 xy莫名其妙反向，所以这里反号转换为ENU系
    current_local_vel.x = - msg->vector.x;
    current_local_vel.y = - msg->vector.y;
    current_local_vel.z = msg->vector.z;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_gps = *msg;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
    flight_status = msg->data;
}

void drop_flag_callback(const std_msgs::UInt16::ConstPtr& msg)
{
    drop.data = msg->data;
}


//////////////mavros::Data_Throw_target//////////////////
void receive_mission_callback(const mavros::Data_Throw_target::ConstPtr& msg)
{   received_mission = *msg;

    if (received_mission.flag_Return == 1 )
    {  //返航指令
        //如果当前无人机没有进入返航指令
        if (Num_StateMachine_Last < 700)
        {   //则直接切换任务到777
            Num_StateMachine = 777;
        }
        //待补充!!!
        //若被要求的无人机正在投掷任务下降阶段(即准备投掷)，是否可以等投掷完两再返回 ~_~!
    }
    else
    {//不是返航指令

        target_ID = received_mission.target_ID;    //投掷目标圆 编号
        target_type = received_mission.target_type;//投掷目标圆 类型
        target_Num = received_mission.target_Num;  //投掷目标圆 已投掷次数
        target_Lat = received_mission.latitude;    //投掷目标圆 纬度
        target_Lon = received_mission.longitude;   //投掷目标圆 经度

        Num_StateMachine = 1;
        //待补充!!!
        //重点目标3的时候，怎么办
    }

}
//////////////////////////////////////////









//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_thrower");
    ros::NodeHandle nh("~");

    //这个频率对吗 飞控接受控制指令的频率是多少？
    ros::Rate rate(50.0);

    // 订阅来自M100的消息
    // 无人机状态 50Hz 定义见dji_sdk.h
    ros::Subscriber flightStatusSub = nh.subscribe<std_msgs::UInt8>("/dji_sdk/flight_status", 10, flight_status_callback);

    // GPS 50Hz
    ros::Subscriber gpsSub   = nh.subscribe<sensor_msgs::NavSatFix>("/dji_sdk/gps_position", 10, gps_callback);

    //    ros::Subscriber GPS_INFO = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 100, state_cb);


    // 无人机姿态 100Hz ENU
    ros::Subscriber attitudeSub = nh.subscribe<geometry_msgs::QuaternionStamped>("/dji_sdk/attitude", 10, attitude_callback);
    // 无人机位置 50Hz？ ENU
    ros::Subscriber localPosition = nh.subscribe<geometry_msgs::PointStamped>("/dji_sdk/local_position", 10, local_position_callback);
    // 无人机速度 50Hz WSU gpsHealth>=3
    ros::Subscriber localVelocity = nh.subscribe<geometry_msgs::Vector3Stamped>("/dji_sdk/velocity", 10, local_velocity_callback);



    //////////////mavros::Data_Throw_target//////////////////

    //需要执行的指令
    ros::Subscriber receive_missionSub = nh.subscribe<mavros::Data_Throw_target>("/Mission_Data_Throw_target", 10, receive_mission_callback);


    //////////////////////////////////////////






    // 订阅 投掷 标志 1-开;0-关     /0->1 开    /1->0 装沙包；
    ros::Subscriber drop_flag_Sub = nh.subscribe<std_msgs::UInt16>("/drop_flag", 10, drop_flag_callback);
    // 发布投掷指令   1-  开 投掷;
    drop_pub = nh.advertise<std_msgs::UInt16>("/drop_flag", 10);
    // 初始值设为0
    drop.data = 0;

    // 要通过通信模块发布的本机状态指令
    Data_Drone_State_pub = nh.advertise<mavros::Data_Drone_State>("/Waitsend_Data_Drone_State", 10);
    // 发布 投掷点 ENU 信息
    Throw_target_pub = nh.advertise<geometry_msgs::Point>("/throw_target_point", 10);

    //  发布 本机当前状态编号
    Num_state_machine_pub = nh.advertise<std_msgs::UInt16>("/Num_state_machine", 10);
    std_msgs::UInt16 Num_StateMachine_client;

    // 发布控制M100的消息 位置控制模式 和 速度控制模式 可参看dji_sdk_node.cpp & dji_sdk_node_control.cpp
    // 本任务中只使用了 速度控制模式
    Pospub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
    Velpub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);

    //DJI权限、设置原点、起飞降落等服务
    drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("/dji_sdk/drone_task_control");
    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("/dji_sdk/sdk_control_authority");
    set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("/dji_sdk/set_local_pos_ref");

    //读取参数表中的参数
    nh.param<float>("kpx", kpx, 1.0);
    nh.param<float>("kpy", kpy, 1.0);
    nh.param<float>("kpz", kpz, 0.8);
    nh.param<float>("kpyaw", kpyaw, 0.5);

    nh.param<float>("kdx", kdx, 0.0);
    nh.param<float>("kdy", kdy, 0.0);
    nh.param<float>("kdz", kdz, 0.0);

    nh.param<float>("Max_velx", Max_velx, 5.0);
    nh.param<float>("Max_vely", Max_vely, 5.0);
    nh.param<float>("Max_velz", Max_velz, 5.0);
    nh.param<float>("Max_velyaw", Max_velyaw, 1.0);

    nh.param<float>("Max_velx_return", Max_velx_return, 5.0);
    nh.param<float>("Max_vely_return", Max_vely_return, 5.0);
    nh.param<float>("Max_velz_return", Max_velz_return, 5.0);
    nh.param<float>("Max_velyaw_return", Max_velyaw_return, 1.0);

    nh.param<float>("Thres_velx", Thres_velx, 0.05);
    nh.param<float>("Thres_vely", Thres_vely, 0.05);
    nh.param<float>("Thres_velz", Thres_velz, 0.05);
    nh.param<float>("Thres_velyaw", Thres_velyaw, 0.05);

    nh.param<float>("Thres_distance", Thres_distance, 0.2);
    nh.param<float>("Thres_count", Thres_count, 50);

    /*    暂时用不到 param   search track
    nh.param<float>("Max_velx_search", Max_velx_search, 3.0);
    nh.param<float>("Max_vely_search", Max_vely_search, 3.0);
    nh.param<float>("Max_velz_search", Max_velz_search, 2.0);
    nh.param<float>("Max_velyaw_search", Max_velyaw_search, 1.0);

    nh.param<float>("kpx_track", kpx_track, 0.5);
    nh.param<float>("kpy_track", kpy_track, 0.5);
    nh.param<float>("kpz_track", kpz_track, 0.4);
    nh.param<float>("kpyaw_track", kpyaw_track, 0.5);

    nh.param<float>("Max_velx_track", Max_velx_track, 2.0);
    nh.param<float>("Max_vely_track", Max_vely_track, 2.0);
    nh.param<float>("Max_velz_track", Max_velz_track, 1.0);
    nh.param<float>("Max_velyaw_track", Max_velyaw_track, 0.5);

    nh.param<float>("Thres_velx_track", Thres_velx_track, 0.05);
    nh.param<float>("Thres_vely_track", Thres_vely_track, 0.05);
    nh.param<float>("Thres_velz_track", Thres_velz_track, 0.05);
    nh.param<float>("Thres_velyaw_track", Thres_velyaw_track, 0.05);

    nh.param<float>("Thres_distance_track", Thres_distance_track, 0.6);
    nh.param<float>("Thres_count_track", Thres_count_track, 50);
  */

    nh.param<float>("Throw_height", Throw_height, 25);              //投掷高度

    nh.param<int>("Drone_Number", Drone_Number, 3);                 //M100编号

    nh.param<int>("Thres_count_takeoff",  Thres_count_takeoff, 3);  //M100起飞前等待时间

    nh.param<float>("fly_height_init", fly_height, 40);             //飞行高度init
    nh.param<float>("fly_height_delta", fly_height_delta, 5);       //飞行高度delta
    //fly_hight = fly_height(init) + fly_height_delta * (Drone ID-1);

    nh.param<float>("fly_yaw", fly_yaw, 0);                         //飞行时 保持的偏航角    [-180, 180]

    //--------------------------------------根据param, 计算部分变量----------------------------------------------------
    /*  后期需要根据GPS点范围    计算出 飞机的偏航角-- yaw
    *
    *nh.param<double>("LonEN", LonEN, 115.697536);       //搜索区 东北角 目标点 GPS 经纬度
    *nh.param<double>("LatEN", LatEN, 39.536098);
    *nh.param<double>("LonWS", LonWS, 115.696449);       //搜索区 西南角 目标点 GPS 经纬度
    *nh.param<double>("LatWS", LatWS, 39.535300);
    */
    fly_yaw = fly_yaw /180 * PI;                  // rad

    fly_height = fly_height + (Drone_Number-1) * fly_height_delta;

    //-----------------------------------------起飞返航降落点设置----------------------------------------------------
    //  爬升点
    climb_point.position.x = 0;
    climb_point.position.y = 0;
    climb_point.position.z = fly_height;
    climb_point.orientation.w = fly_yaw;;            // rad
    //  返回点
    rerun_point.position.x = 0;
    rerun_point.position.y = 0;
    rerun_point.position.z = fly_height;
    rerun_point.orientation.w = fly_yaw;
    //  降落点
    land_point.position.x =  0;
    land_point.position.y =  0;
    land_point.position.z =  2.5;
    land_point.orientation.w = fly_yaw;

    //后期需要取消!!!!
    //打印参数 检查确认
    printf_param();
    int check_flag;
    //输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting, 1 for go on, else for quit:  ";
    cin >> check_flag;
    if(check_flag != 1)
    {
        return -1;
    }

    //后期需要取消!!!!
    int test_flag;
    //输入233,继续，其他，退出程序
    cout << "Please input test_flag, 233 for test, else for normal(Obtain Control & Set local_position /Need-GPS_meg): "<<endl;
    cin >> test_flag;
    if (test_flag != 233)
    {
        //获取控制权限，需要遥控器拨到F档
        obtain_control();
        //设置本地坐标原点 这一步必须有GPS信息
        if (!set_local_position())
        {
            ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.[Set local position failed]");
            return 1;
        }
    }

    //后期需要取消!!!!
    //选择模式 - /测试模式(方便做单步测试) 或 /任务模式 或 /打印模式
    cout << "Please choose mode(m for mission mode, p for only printf mode): "<<endl;
    cin >> modeflag;
    // 打印模式 -- 无实际用途 用于打印相关信息测试
    if (modeflag == 'p')
    {
        while (ros::ok())
        {
            ros::spinOnce();

            printf_M100_info();
            rate.sleep();
        }
    }

    //等待状态发送标志
    int flag_send = 1;

    //任务模式
    if(modeflag == 'm')
    {
        Num_StateMachine = 0;

        while (ros::ok())
        {
            ros::spinOnce();

            Num_StateMachine_client.data = Num_StateMachine;
            Num_state_machine_pub.publish(Num_StateMachine_client);

            switch (Num_StateMachine)
            {

            case 0:
                //等待接收指令!
                if (Num_StateMachine_Last != 0)
                {
                    flag_send = 1;
                }

                Num_StateMachine_Last = Num_StateMachine;

                if ( (target_type == 2) && (target_Num == 1) && (drop.data == 0))
                {   //普通目标2     已完成一次投掷,且本机返回起飞点,完成重新上沙包
                    Num_StateMachine = 1;
                    flag_send = 0;
                }
                else if ( (target_type == target_Num) && (drop.data == 0))
                {
                    //上一个任务 投掷次数和其类型相同，且完成重新上沙包了
                    //通过通信模块告知  地面站0x0a
                    if ( flag_send == 1)
                    {
                        wait_Send_Data.Drone_ID = CONFIG_ID;
                        wait_Send_Data.Accepted_ID = 0x0a;      //地面站0x0a 接收本指令
                        wait_Send_Data.Drone_state = 111;       //本机可接收 目标指令
                        wait_Send_Data.target_ID = 0;           //投掷目标 编号
                        wait_Send_Data.target_type = 0;         //投掷目标 类型
                        wait_Send_Data.target_Num = 0;          //投掷目标 已投掷次数

                        Data_Drone_State_pub.publish(wait_Send_Data);
                        flag_send = 0;
                    }
                }

                printf_M100_info();

                rate.sleep();
                break;

                // 起飞s
            case 1:

                if(Num_StateMachine_Last != 1)
                {   //起飞前的  参数计算
                    Lon1 = current_gps.longitude * PI/180;  //记录起飞点GPS
                    Lat1 = current_gps.latitude * PI/180;

                    //计算 投掷目标点GPS 在ENU系下坐标---------------------------------------------------
                    Lon2 = target_Lon * PI/180;
                    Lat2 = target_Lat * PI/180;

                    double a = 6378137.0;
                    double b = 6356752.31414;
                    double e2 = (pow((a*a-b*b),0.5)/a)*(pow((a*a-b*b),0.5)/a);
                    double dB = Lat2-Lat1;
                    double dL = Lon2-Lon1;
                    double Bm = (Lat1+Lat2)/2;
                    double mm = a*(1-e2)*pow(1-e2*sin(Bm)*sin(Bm),-1.5);
                    double nm = a*pow(1-e2*sin(Bm)*sin(Bm),-0.5);

                    dZ = cos(Bm)*nm*dL;  //指向正东
                    dX = mm*dB;          //指向正北

                    throw_target_point.position.x = dZ;
                    throw_target_point.position.y = dX;

                    throw_target_point.position.z = Throw_height;
                    throw_target_point.orientation.w = fly_yaw;
                    //  发布投掷目标点 ENU系下坐标-----------------------------------------------------
                    geometry_msgs::Point Target_point;

                    Target_point.x = throw_target_point.position.x;
                    Target_point.y = throw_target_point.position.y;

                    Throw_target_pub.publish(Target_point);

                    //起飞前 倒计时
                    int count_takeoff = 0;
                    int remain_time = 0;
                    while(count_takeoff < Thres_count_takeoff && ros::ok())
                    {
                        remain_time = Thres_count_takeoff - count_takeoff;
                        count_takeoff = count_takeoff + 1;
                        cout << "Waiting for takeoff: "<< remain_time << "[s]"<<endl;
                        sleep(1);
                    }
                }
                //  起飞
                takeoff_result = M100monitoredTakeoff();
                //takeoff_result = M100monitoredTakeoff();

                printf_M100_info();

                Num_StateMachine_Last = Num_StateMachine;

                //起飞成功，进入下一过程
                Num_StateMachine = 2;

                rate.sleep();
                break;

                // 爬升至目标高度
            case 2:
                if (Num_StateMachine_Last != 2)
                {
                    pos_sp.position.x = climb_point.position.x;
                    pos_sp.position.y = climb_point.position.y;
                    pos_sp.position.z = climb_point.position.z;
                    pos_sp.orientation.w = climb_point.orientation.w;   //这里使用w来代表期望的偏航角 单位：rad
                }

                M100_pidcontrol();
                printf_M100_info();
                Num_StateMachine_Last = Num_StateMachine;

                if(abs_distance(pos_sp,current_local_pos) < Thres_distance)
                {
                    num_count++;
                    cout<< "Distance_count: " << num_count<<endl;
                }else
                {
                    num_count = 0;
                    cout<< "Distance_count: " << num_count<<endl;
                }

                //爬升成功，进入下一过程
                if(abs_distance(pos_sp,current_local_pos) < Thres_distance && num_count > Thres_count)
                {
                    Num_StateMachine = 3;
                    num_count=0;
                }

                rate.sleep();
                break;

                // 飞行至投掷目标点上方
            case 3:

                if (Num_StateMachine_Last != 3)
                {
                    // 飞行至搜索区域起点上方
                    pos_sp.position.x = throw_target_point.position.x;
                    pos_sp.position.y = throw_target_point.position.y;
                    pos_sp.position.z = fly_height;
                    pos_sp.orientation.w = throw_target_point.orientation.w;
                }

                M100_pidcontrol();
                printf_M100_info();
                Num_StateMachine_Last = Num_StateMachine;

                if(abs_distance(pos_sp,current_local_pos) < Thres_distance)
                {
                    num_count++;
                    cout<< "Distance_count: " << num_count<<endl;
                }else
                {
                    num_count = 0;
                    cout<< "Distance_count: " << num_count<<endl;
                }

                //飞到投掷目标上方成功，进入下一过程
                if(abs_distance(pos_sp,current_local_pos) < Thres_distance && num_count > Thres_count)
                {
                    Num_StateMachine = 4;
                    num_count=0;
                }

                rate.sleep();
                break;

                // 下降至 设定投掷高度
            case 4:
                if (Num_StateMachine_Last != 4)
                {
                    // 飞行至搜索区域起点
                    pos_sp.position.x = throw_target_point.position.x;
                    pos_sp.position.y = throw_target_point.position.y;
                    pos_sp.position.z = throw_target_point.position.z;
                    pos_sp.orientation.w = throw_target_point.orientation.w;
                }

                M100_pidcontrol();
                printf_M100_info();
                Num_StateMachine_Last = Num_StateMachine;

                if(abs_distance(pos_sp,current_local_pos) < Thres_distance)
                {
                    num_count++;
                    cout<< "Distance_count: " << num_count<<endl;
                }else
                {
                    num_count = 0;
                    cout<< "Distance_count: " << num_count<<endl;
                }

                //下降至 设定投掷高度 成功，进入下一过程
                if(abs_distance(pos_sp,current_local_pos) < Thres_distance && num_count > Thres_count)
                {
                    Num_StateMachine = 5;
                    num_count=0;
                }

                rate.sleep();
                break;

                // 投掷
            case 5:

                //待完成!!!!!!!!!!!!!!!!!!!!!!!!!
                //是否需要修改
                //如投掷后的悬停等待时间
                //未考虑重点目标3的情况
                //悬停指令
                //wait();

                //sleep(1);

                drop.data = 1;
                drop_pub.publish(drop);
                cout<< "Droping............." << endl;

                target_Num = target_Num + 1;

                //发布当前投掷点的信息，需要告知 地面站0x0a
                wait_Send_Data.Drone_ID = CONFIG_ID;
                wait_Send_Data.Accepted_ID = 0x0a;               //地面站0x0a 接收本指令
                wait_Send_Data.Drone_state = 0;                  //本机正在执行任务
                wait_Send_Data.target_ID = target_ID;            //本机当前 投掷目标 编号
                wait_Send_Data.target_type = target_type;        //本机当前 投掷目标 类型
                wait_Send_Data.target_Num = target_Num;          //本机当前 投掷目标 已投掷次数

                Data_Drone_State_pub.publish(wait_Send_Data);

                //发布安投掷指令后，悬停等待
                sleep(5);

                Num_StateMachine_Last = Num_StateMachine;

                printf_M100_info();

                Num_StateMachine = 6;
                rate.sleep();
                break;

                // 高度返回至投掷目标点上方
            case 6:

                if (Num_StateMachine_Last != 6)
                {
                    // 飞行至搜索区域起点上方
                    pos_sp.position.x = throw_target_point.position.x;
                    pos_sp.position.y = throw_target_point.position.y;
                    pos_sp.position.z = fly_height;
                    pos_sp.orientation.w = throw_target_point.orientation.w;
                }

                M100_pidcontrol();
                printf_M100_info();
                Num_StateMachine_Last = Num_StateMachine;

                if(abs_distance(pos_sp,current_local_pos) < Thres_distance)
                {
                    num_count++;
                    cout<< "Distance_count: " << num_count<<endl;
                }else
                {
                    num_count = 0;
                    cout<< "Distance_count: " << num_count<<endl;
                }

                //高度返回至投掷目标点上方成功，进入下一过程
                if(abs_distance(pos_sp,current_local_pos) < Thres_distance && num_count > Thres_count)
                {
                    Num_StateMachine = 7;
                    num_count=0;
                }

                rate.sleep();
                break;

                //  返航
            case 7:
                if (Num_StateMachine_Last != 7)
                {
                    pos_sp.position.x = rerun_point.position.x ;
                    pos_sp.position.y = rerun_point.position.y ;
                    pos_sp.position.z = rerun_point.position.z ;
                    pos_sp.orientation.w = rerun_point.orientation.w;
                }

                M100_pidcontrol();
                printf_M100_info();
                Num_StateMachine_Last = Num_StateMachine;

                if(abs_distance(pos_sp,current_local_pos) < Thres_distance)
                {
                    num_count++;
                    cout<< "Distance_count: " << num_count<<endl;
                }else
                {
                    num_count = 0;
                    cout<< "Distance_count: " << num_count<<endl;
                }

                // 降落
                if(abs_distance(pos_sp,current_local_pos) < Thres_distance && num_count > Thres_count)
                {
                    Num_StateMachine = 8;
                    num_count=0;
                }

                rate.sleep();
                break;

                // 降落
            case 8:
                if (Num_StateMachine_Last != 8)
                {
                    pos_sp.position.x = land_point.position.x ;
                    pos_sp.position.y = land_point.position.y ;
                    pos_sp.position.z = land_point.position.z ;
                    pos_sp.orientation.w = land_point.orientation.w;
                }

                M100_pidcontrol();
                printf_M100_info();
                Num_StateMachine_Last = Num_StateMachine;

                if(abs_distance(pos_sp,current_local_pos) < Thres_distance)
                {
                    num_count++;
                    cout<< "Distance_count: " << num_count<<endl;
                }else
                {
                    num_count = 0;
                    cout<< "Distance_count: " << num_count<<endl;
                }

                // 降落
                if(abs_distance(pos_sp,current_local_pos) < Thres_distance && num_count > Thres_count)
                {
                    takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND);
                    sleep(1);
                    takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND);
                    num_count=0;
                    Num_StateMachine = 0;
                }

                rate.sleep();
                break;


                //------------------------------------------强制降落返航------------------------------//
                //强制返航
                // 高度返回至投掷目标点上方
            case 777:
                if (Num_StateMachine_Last != 777)
                {
                    // 飞行至当前位置的飞行高度
                    //获取当前点ENU位置信息
                    pos_sp.position.x = current_local_pos.x;
                    pos_sp.position.y = current_local_pos.y;
                    pos_sp.position.z = current_local_pos.z;
                    pos_sp.orientation.w = throw_target_point.orientation.w;

                    //发布当前飞机的执行任务信息，需要告知 地面站0x0a
                    wait_Send_Data.Drone_ID = CONFIG_ID;
                    wait_Send_Data.Accepted_ID = 0x0a;               //地面站0x0a 接收本指令
                    wait_Send_Data.Drone_state = 777;                //本机强制返航
                    wait_Send_Data.target_ID = target_ID;            //本机返航前 投掷目标 编号
                    wait_Send_Data.target_type = target_type;        //本机返航前 投掷目标 类型
                    wait_Send_Data.target_Num = target_Num;          //本机返航前 投掷目标 已投掷次数

                    Data_Drone_State_pub.publish(wait_Send_Data);
                }

                M100_pidcontrol();
                printf_M100_info();
                Num_StateMachine_Last = Num_StateMachine;

                if(abs_distance(pos_sp,current_local_pos) < Thres_distance)
                {
                    num_count++;
                    cout<< "Distance_count: " << num_count<<endl;
                }else
                {
                    num_count = 0;
                    cout<< "Distance_count: " << num_count<<endl;
                }

                //高度返回至投掷目标点上方成功，进入下一过程
                if(abs_distance(pos_sp,current_local_pos) < Thres_distance && num_count > Thres_count)
                {
                    Num_StateMachine = 778;
                    num_count=0;
                }

                rate.sleep();
                break;

                //  返航
            case 778:
                if (Num_StateMachine_Last != 778)
                {
                    pos_sp.position.x = rerun_point.position.x ;
                    pos_sp.position.y = rerun_point.position.y ;
                    pos_sp.position.z = rerun_point.position.z ;
                    pos_sp.orientation.w = rerun_point.orientation.w;
                }

                M100_pidcontrol();
                printf_M100_info();
                Num_StateMachine_Last = Num_StateMachine;

                if(abs_distance(pos_sp,current_local_pos) < Thres_distance)
                {
                    num_count++;
                    cout<< "Distance_count: " << num_count<<endl;
                }else
                {
                    num_count = 0;
                    cout<< "Distance_count: " << num_count<<endl;
                }

                // 降落
                if(abs_distance(pos_sp,current_local_pos) < Thres_distance && num_count > Thres_count)
                {
                    Num_StateMachine = 779;
                    num_count=0;
                }

                rate.sleep();
                break;

                // 降落
            case 779:
                if (Num_StateMachine_Last != 8)
                {
                    pos_sp.position.x = land_point.position.x ;
                    pos_sp.position.y = land_point.position.y ;
                    pos_sp.position.z = land_point.position.z ;
                    pos_sp.orientation.w = land_point.orientation.w;
                }

                M100_pidcontrol();
                printf_M100_info();
                Num_StateMachine_Last = Num_StateMachine;

                if(abs_distance(pos_sp,current_local_pos) < Thres_distance)
                {
                    num_count++;
                    cout<< "Distance_count: " << num_count<<endl;
                }else
                {
                    num_count = 0;
                    cout<< "Distance_count: " << num_count<<endl;
                }

                // 降落
                if(abs_distance(pos_sp,current_local_pos) < Thres_distance && num_count > Thres_count)
                {
                    takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND);
                    sleep(1);
                    takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND);
                    num_count=0;
                    return 1;
                }

                rate.sleep();
                break;
                //------------------------------------------强制降落返航------------------------------//


            }
        }
    }


}

//M100 位置环控制函数
void M100_pidcontrol()
{
    sensor_msgs::Joy controlVelYawRate;

    //计算期望速度
    //xy位置控制 PD
    xCmd = kpx*(pos_sp.position.x - current_local_pos.x) + kdx * (0 - current_local_vel.x);
    yCmd = kpy*(pos_sp.position.y - current_local_pos.y) + kdy * (0 - current_local_vel.y);
    zCmd = kpz*(pos_sp.position.z - current_local_pos.z) + kdz * (0 - current_local_vel.z);

    //yaw control
    float euler_sp[3];
    euler_sp[2] = pos_sp.orientation.w;

    float yaw_error;
    yaw_error = (euler_sp[2] - euler_drone[2]);

    if(yaw_error>=PI)
    {
        yaw_error = yaw_error -2*PI;
    }
    if(yaw_error<-PI)
    {
        yaw_error = yaw_error +2*PI;
    }

    yawCmd = kpyaw*yaw_error;

    /*if(Num_StateMachine == 4 || Num_StateMachine == 6 )
    {
        //设置期望速度的死区和上限
        xCmd = satfunc(xCmd, Max_velx_search, Thres_velx);
        yCmd = satfunc(yCmd, Max_vely_search, Thres_vely);
        zCmd = satfunc(zCmd, Max_velz_search, Thres_velz);
        yawCmd = satfunc(yawCmd, Max_velyaw_search, Thres_velyaw);
    }else
   */
    if(Num_StateMachine == 6 || Num_StateMachine == 7 || Num_StateMachine == 8)
    {
        //设置期望速度的死区和上限
        xCmd = satfunc(xCmd, Max_velx_return, Thres_velx);
        yCmd = satfunc(yCmd, Max_vely_return, Thres_vely);
        zCmd = satfunc(zCmd, Max_velz_return, Thres_velz);
        yawCmd = satfunc(yawCmd, Max_velyaw_return, Thres_velyaw);
    }
    else if(Num_StateMachine > 700)
    {
        //设置期望速度的死区和上限
        xCmd = satfunc(xCmd, Max_velx_return, Thres_velx);
        yCmd = satfunc(yCmd, Max_vely_return, Thres_vely);
        zCmd = satfunc(zCmd, Max_velz_return, Thres_velz);
        yawCmd = satfunc(yawCmd, Max_velyaw_return, Thres_velyaw);
    }
    else
    {
        //设置期望速度的死区和上限
        xCmd = satfunc(xCmd, Max_velx, Thres_velx);
        yCmd = satfunc(yCmd, Max_vely, Thres_vely);
        zCmd = satfunc(zCmd, Max_velz, Thres_velz);
        yawCmd = satfunc(yawCmd, Max_velyaw, Thres_velyaw);
    }

    //发布期望速度给M100 SDK
    controlVelYawRate.axes.push_back(xCmd);
    controlVelYawRate.axes.push_back(yCmd);
    controlVelYawRate.axes.push_back(zCmd);
    controlVelYawRate.axes.push_back(yawCmd);

    Velpub.publish(controlVelYawRate);
}

//M100 track控制函数
/*
void track()
{
    sensor_msgs::Joy controlVelYawRate_track;

    //机体系控制指令
    float xCmd_body, yCmd_body;
    //xy位置控制 - 如果目标在前方，x大于0； 如果目标在左方， y大于0
    xCmd_body = kpx_track * Target_position.x;
    yCmd_body = kpy_track * Target_position.y;

    //根据机体系期望位置计算ENU系期望位置
    xCmd = xCmd_body * cos(euler_drone[2]) - yCmd_body * sin(euler_drone[2]);
    yCmd = xCmd_body * sin(euler_drone[2]) + yCmd_body * cos(euler_drone[2]);
    //高度用无人机自身高度反馈
    zCmd = kpz_track * (drop_max_z - 1 - current_local_pos.z);

    //yaw control
    float euler_sp[3];
    euler_sp[2] = pos_sp.orientation.w;

    float yaw_error;
    yaw_error = (euler_sp[2] - euler_drone[2]);

    if(yaw_error>=PI)
    {
        yaw_error = yaw_error -2*PI;
    }


    if(yaw_error<-PI)
    {
        yaw_error = yaw_error +2*PI;
    }

    yawCmd = kpyaw*yaw_error;


    //设置期望速度的死区和上限
    xCmd = satfunc(xCmd, Max_velx_track, Thres_velx_track);
    yCmd = satfunc(yCmd, Max_vely_track, Thres_vely_track);
    zCmd = satfunc(zCmd, Max_velz_track, Thres_velz_track);
    yawCmd = satfunc(yawCmd, Max_velyaw_track, Thres_velyaw_track);

    //发布期望速度给M100 SDK

    controlVelYawRate_track.axes.push_back(xCmd);
    controlVelYawRate_track.axes.push_back(yCmd);
    controlVelYawRate_track.axes.push_back(zCmd);
    controlVelYawRate_track.axes.push_back(yawCmd);

    Velpub.publish(controlVelYawRate_track);
}
*/

//M100 wait控制函数
void wait()
{
    sensor_msgs::Joy controlVelYawRate_wait;

    xCmd = 0;
    yCmd = 0;
    zCmd = 0;
    yawCmd = 0;

    //发布期望速度给M100 SDK
    controlVelYawRate_wait.axes.push_back(xCmd);
    controlVelYawRate_wait.axes.push_back(yCmd);
    controlVelYawRate_wait.axes.push_back(zCmd);
    controlVelYawRate_wait.axes.push_back(yawCmd);

    Velpub.publish(controlVelYawRate_wait);
}


bool set_local_position()
{
    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference.call(localPosReferenceSetter);
    return localPosReferenceSetter.response.result;
}

bool takeoff_land(int task)
{
    dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = task;

    drone_task_service.call(droneTaskControl);

    if(!droneTaskControl.response.result)
    {
        ROS_ERROR("takeoff_land fail");
        return false;
    }

    return true;
}

//起飞程序 节省时间 优化？
bool M100monitoredTakeoff()
{
    ros::Time start_time = ros::Time::now();

    float home_altitude = current_gps.altitude;

    if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
    {
        return false;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();

    // Step 1: If M100 is not in the air after 10 seconds, fail.
    while (ros::Time::now() - start_time < ros::Duration(10))
    {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
            current_gps.altitude - home_altitude < 1.0)
    {
        ROS_ERROR("Takeoff failed.");
        return false;
    }
    else
    {
        start_time = ros::Time::now();
        ROS_INFO("Successful takeoff!");
        ros::spinOnce();
    }

    return true;
}


float abs_distance(geometry_msgs::Pose pos_sp,geometry_msgs::Point current_local_pos)
{
    float abs_distance1;
    abs_distance1 = sqrt((pos_sp.position.x - current_local_pos.x) * (pos_sp.position.x - current_local_pos.x) + (pos_sp.position.y - current_local_pos.y) * (pos_sp.position.y - current_local_pos.y) + (pos_sp.position.z - current_local_pos.z) * (pos_sp.position.z - current_local_pos.z));
    cout<< "Abs_distance:" << abs_distance1 << "[m]" << endl;
    return abs_distance1;
}

bool obtain_control()
{
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable=1;
    sdk_ctrl_authority_service.call(authority);

    if(!authority.response.result)
    {
        ROS_ERROR("obtain control failed!");
        return false;
    }

    return true;
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

//M100 打印函数
void printf_M100_info()
{
    cout.setf(ios::fixed);
    cout << endl;
    cout <<"################## The Data Below is new Data. #####################" <<endl;
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    switch(Num_StateMachine)
    {
    case 0:
        cout <<"StateMachine State: "<< " 0 Wait for Mission" <<endl;
        break;
    case 1:
        cout <<"StateMachine State: "<< " 1 Take off" <<endl;
        break;
    case 2:
        cout <<"StateMachine State: "<< " 2 Climb up" <<endl;
        break;
    case 3:
        cout <<"StateMachine State: "<< " 3 Fly to target point" <<endl;
        break;
    case 4:
        cout <<"StateMachine State: "<< " 4 Land to throw point" <<endl;
        break;
    case 5:
        cout <<"StateMachine State: "<< " 5 Throw " <<endl;
        break;
    case 6:
        cout <<"StateMachine State: "<< " 6 Climb up for return  " <<endl;
        break;
    case 7:
        cout <<"StateMachine State: "<< " 7 Return land zone " <<endl;
        break;
    case 8:
        cout <<"StateMachine State: "<< " 8 Land " <<endl;
        break;

    case 777:
        cout <<"StateMachine State: "<< " 777 Forced Climb up for return  " <<endl;
        break;
    case 778:
        cout <<"StateMachine State: "<< " 778 Forced Return land zone " <<endl;
        break;
    case 779:
        cout <<"StateMachine State: "<< " 779 Forced Land " <<endl;
        break;
    }
    cout << "GPS: "<< fixed <<setprecision(9) <<"Lat: "<<current_gps.latitude <<", Lon: "<<current_gps.longitude<<", high: "<<current_gps.altitude<<endl;

    cout << "Local Position: [X Y Z][ENU] : " << fixed <<setprecision(5) << " " << current_local_pos.x << " [m] "<< current_local_pos.y<<" [m] "<< current_local_pos.z<<" [m] "<<endl;

    cout << "Velocity: [X Y Z][ENU] : " << " " << fixed <<setprecision(5) <<  current_local_vel.x << " [m/s] " <<  current_local_vel.y <<" [m/s] " << current_local_vel.z <<" [m/s] "<<endl;

    cout << "Euler: [X Y Z][ENU] : " << " " << fixed <<setprecision(5) << euler_drone[0]/PI*180  << " [du] "<< euler_drone[1]/PI*180  <<" [du] "<< euler_drone[2]/PI*180 <<" [du] "<<endl;

    cout << "drop : " << " " << fixed <<setprecision(5) <<  drop.data <<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>M100 Control State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout.setf(ios::fixed);
    cout << "pos_sp[ENU]: [x y z yaw] : " << " " << fixed << setprecision(5) << pos_sp.position.x <<" [m] "<< pos_sp.position.y <<" [m] "<< pos_sp.position.z <<" [m] " << pos_sp.orientation.w /3.415926 *180 <<" [du] "<<endl;
    cout << "Cmd: [xCmd yCmd zCmd yawCmd] : " << " " << fixed <<setprecision(5) << xCmd << " [m/s] "<< yCmd <<" [m/s] "<< zCmd <<" [m/s] "<< yawCmd <<" [rad/s] "<<endl;
    cout << endl << endl;
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

void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Control Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Drone_Number : " << Drone_Number <<endl;
    cout << "kpx : "<< kpx << endl;
    cout << "kpy : "<< kpy << endl;
    cout << "kpz : "<< kpz << endl;
    cout << "kpyaw : "<< kpyaw << endl;

    cout << "kdx : "<< kdx << endl;
    cout << "kdy : "<< kdy << endl;
    cout << "kdz : "<< kdz << endl;

    cout << "Max_velx : "<< Max_velx << endl;
    cout << "Max_vely : "<< Max_vely << endl;
    cout << "Max_velz : "<< Max_velz << endl;
    cout << "Max_velyaw : "<< Max_velyaw << endl;

    cout << "Thres_velx : "<< Thres_velx << endl;
    cout << "Thres_vely : "<< Thres_vely << endl;
    cout << "Thres_velz : "<< Thres_velz << endl;
    cout << "Thres_velyaw : "<< Thres_velyaw << endl;

    cout << "Thres_distance : "<< Thres_distance << endl;
    cout << "Thres_count : "<< Thres_count << endl;

    /*cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Track Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "kpx_track : "<< kpx_track << endl;
    cout << "kpy_track : "<< kpy_track << endl;
    cout << "kpz_track : "<< kpz_track << endl;
    cout << "kpyaw_track : "<< kpyaw_track << endl;

    cout << "Max_velx_track : "<< Max_velx_track << endl;
    cout << "Max_vely_track : "<< Max_vely_track << endl;
    cout << "Max_velz_track : "<< Max_velz_track << endl;
    cout << "Max_velyaw_track : "<< Max_velyaw_track << endl;

    cout << "Thres_velx : "<< Thres_velx << endl;
    cout << "Thres_vely : "<< Thres_vely << endl;
    cout << "Thres_velz : "<< Thres_velz << endl;
    cout << "Thres_velyaw : "<< Thres_velyaw << endl;

    cout << "Thres_distance_track : "<< Thres_distance_track << endl;
    cout << "Thres_count_track : "<< Thres_count_track << endl;
*/

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Mission Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "Throw_height : "<< Throw_height << endl;
    cout << "Thres_count_takeoff : "<< Thres_count_takeoff << endl;
    cout << "fly_height: "<< " " << fly_height << " [m] " <<endl;
    cout << "fly_yaw: "<< " " << fly_yaw << " [m] " <<endl;

    cout << "climb_point [x y z yaw] : "<< " " << climb_point.position.x << " [m] " << climb_point.position.y << " [m] "<< climb_point.position.z<<" [m] "<< climb_point.orientation.w/PI*180<<" [du] "<<endl;
    cout << "rerun_point [x y z yaw] : "<< " " << rerun_point.position.x << " [m] " << rerun_point.position.y << " [m] "<< rerun_point.position.z<<" [m] "<< rerun_point.orientation.w/PI*180<<" [du] "<<endl;
    cout << "land_point [x y z yaw] : "<< " " << land_point.position.x << " [m] " << land_point.position.y << " [m] "<< land_point.position.z<<" [m] "<< land_point.orientation.w/PI*180<<" [du] "<<endl;
}
