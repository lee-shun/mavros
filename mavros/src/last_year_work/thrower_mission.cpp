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


///////////////////////////////

int Num_StateMachine = 0;                                  // 状态机编号

////////////////////////////////


/*int Num_StateMachine_Last = 0;                             // 状态机编号_last
int num_count = 0;                                         // 计数器
//-----------------------------------------投掷相关----------------------------------------------------
std_msgs::UInt16 drop;              // 投掷信息 0-未投    1-开     0->1投掷  1->0装载 // 允许投掷flag
float Throw_height;                 //投掷高度
*/
//-----------------------------------------电台通讯相关----------------------------------------------------

mavros::Data_Throw_target received_mission;
mavros::Data_Drone_State wait_Send_Data;
sensor_msgs::NavSatFix current_state;



int target_ID;//投掷目标圆 编号
int target_type;//投掷目标圆 类型
int target_Num;//投掷目标圆 已投掷次数
double target_Lat, target_Lon;                              //投掷目标圆GPS 纬度&经度




//-----------------------------------------发布及服务声明----------------------------------------------------
//ros::Publisher Pospub;                                      // 发布期望位置给DJI SDK
//ros::Publisher Velpub;                                      // 发布期望速度给DJI SDK
//ros::Publisher SendDatapub;
//ros::Publisher drop_pub;
//ros::Publisher Throw_target_pub;
//ros::Publisher Num_state_machine_pub;
ros::Publisher Data_Drone_State_pub;

//ros::Subscriber GPS_INFO = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 100, state_cb);


//ros::ServiceClient drone_task_service;                      // 起飞、降落等特殊服务
//ros::ServiceClient sdk_ctrl_authority_service;              // 获取控制权限服务
//ros::ServiceClient set_local_pos_reference;                 // 设置本地坐标参考点服务


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
//{
//    current_attitude = msg->quaternion;
//    float q[4];

//    q[0] = current_attitude.w;
//    q[1] = current_attitude.x;
//    q[2] = current_attitude.y;
//    q[3] = current_attitude.z;
//    quaternion_2_euler(q, euler_drone);
//}



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

    cout << "现在在回调函数里面!!!"  << endl;
    cout <<fixed<< setprecision(10)<< current_gps.latitude   << endl;
    cout <<fixed<< setprecision(10)<< current_gps.longitude  << endl;
    cout <<fixed<< setprecision(10)<< current_gps.altitude   << endl;
    cout <<fixed<< setprecision(10)<< endl<<endl;
}




void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
    flight_status = msg->data;
}

//void drop_flag_callback(const std_msgs::UInt16::ConstPtr& msg)
//{
//    drop.data = msg->data;
//}





//////////////mavros::Data_Throw_target//////////////////
void receive_mission_callback(const mavros::Data_Throw_target::ConstPtr& msg)
{   received_mission = *msg;

//    if (received_mission.flag_Return == 1 )
//    {  //返航指令
//        //如果当前无人机没有进入返航指令
//        if (Num_StateMachine_Last < 700)
//        {   //则直接切换任务到777
//            Num_StateMachine = 777;
//        }
//        //待补充!!!
//        //若被要求的无人机正在投掷任务下降阶段(即准备投掷)，是否可以等投掷完两再返回 ~_~!
//    }
//    else
//    {//不是返航指令

        target_ID = received_mission.target_ID;    //投掷目标圆 编号
        target_type = received_mission.target_type;//投掷目标圆 类型
        target_Num = received_mission.target_Num;  //投掷目标圆 已投掷次数
        target_Lat = received_mission.latitude;    //投掷目标圆 纬度
        target_Lon = received_mission.longitude;   //投掷目标圆 经度

//        Num_StateMachine = 1;
//         cout << "target_ID=" << target_ID <<endl<<endl;
//         cout << "target_type=" << target_type <<endl<<endl;
//         cout << "target_Num=" << target_Num <<endl<<endl;
//         cout << "target_Lat=" << target_Lat <<endl<<endl;
//         cout << "target_Lon=" << target_Lon <<endl<<endl;








//        //待补充!!!
//        //重点目标3的时候，怎么办
//    }

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
    //    ros::Subscriber gpsSub   = nh.subscribe<sensor_msgs::NavSatFix>("/dji_sdk/gps_position", 10, gps_callback);

    ros::Subscriber gpsSub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 100, gps_callback);

//    ros::Subscriber gpsSub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 100, gps_callback);

    ros::Publisher Data_Drone_State_pub;

    //    // 无人机姿态 100Hz ENU
    //    ros::Subscriber attitudeSub = nh.subscribe<geometry_msgs::QuaternionStamped>("/dji_sdk/attitude", 10, attitude_callback);
    //    // 无人机位置 50Hz？ ENU
    //    ros::Subscriber localPosition = nh.subscribe<geometry_msgs::PointStamped>("/dji_sdk/local_position", 10, local_position_callback);
    //    // 无人机速度 50Hz WSU gpsHealth>=3
    //    ros::Subscriber localVelocity = nh.subscribe<geometry_msgs::Vector3Stamped>("/dji_sdk/velocity", 10, local_velocity_callback);



    //////////////mavros::Data_Throw_target//////////////////

    //需要执行的指令
    ros::Subscriber receive_missionSub = nh.subscribe<mavros::Data_Throw_target>("/Mission_Data_Throw_target", 10, receive_mission_callback);

    //////////////////////////////////////////






    //    // 订阅 投掷 标志 1-开;0-关     /0->1 开    /1->0 装沙包；
    //    ros::Subscriber drop_flag_Sub = nh.subscribe<std_msgs::UInt16>("/drop_flag", 10, drop_flag_callback);
    //    // 发布投掷指令   1-  开 投掷;
    //    drop_pub = nh.advertise<std_msgs::UInt16>("/drop_flag", 10);
    //    // 初始值设为0
    //    drop.data = 0;

    ros::Rate loop_rate(10);
    //////////////////////

    // 要通过通信模块发布的本机状态指令
    Data_Drone_State_pub = nh.advertise<mavros::Data_Drone_State>("/Waitsend_Data_Drone_State", 10);









    /////////////////




    //    // 发布 投掷点 ENU 信息
    //    Throw_target_pub = nh.advertise<geometry_msgs::Point>("/throw_target_point", 10);

    //    //  发布 本机当前状态编号
    //    Num_state_machine_pub = nh.advertise<std_msgs::UInt16>("/Num_state_machine", 10);
    //    std_msgs::UInt16 Num_StateMachine_client;

    //    // 发布控制M100的消息 位置控制模式 和 速度控制模式 可参看dji_sdk_node.cpp & dji_sdk_node_control.cpp
    //    // 本任务中只使用了 速度控制模式
    //    Pospub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
    //    Velpub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);

    //    //DJI权限、设置原点、起飞降落等服务
    //    drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("/dji_sdk/drone_task_control");
    //    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("/dji_sdk/sdk_control_authority");
    //    set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("/dji_sdk/set_local_pos_ref");

    while(ros::ok())
    {
        ////////////////////////////////////////////

        cout << "现在在主程序里面!!!"  << endl;
        cout <<fixed<< setprecision(10)<< current_gps.latitude   << endl;
        cout <<fixed<< setprecision(10)<< current_gps.longitude  << endl;
        cout <<fixed<< setprecision(10)<< current_gps.altitude   << endl;
        cout <<fixed<< setprecision(10)<< endl<<endl;

        //     ros::spinOnce();
        loop_rate.sleep();

    }






    return 0;
}





