/***************************************************************************************************************************
 * position_control.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2018.8.17
 *
 * 说明: mavros位置控制程序
 *      1. 订阅导航信息（飞机当前位置及速度）
 *      2. 订阅上层控制指令（惯性系移动、机体系移动、降落等）
 *      3. 位置环串级PID算法实现（移植PX4位置环串级PID算法）
 *      4. 发布加速度期望值给飞控（本代码的最终输出是期望加速度值[a][b]）
 *
 *                   [a]: 这个期望加速度将会在飞控中mc_pos_control_main.cpp中转换为期望姿态角，然后才会进入mc_att_contol_main.cpp中进行下一步计算；
 *                   [b]: 飞控本身提供的接口是有 期望位置、期望速度、期望加速度、期望姿态角、期望姿态角速度、期望推力，本程序选用期望加速度作为演示；
***************************************************************************************************************************/

//头文件
#include <ros/ros.h>
#include <pid.h>                                                        //PID头文件

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

//话题头文件
#include <mavros_msgs/Command.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
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

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define PI 3.1415926535898
//---------------------------------------相关参数-----------------------------------------------
float takeoff_height;                                   //起飞高度
float trim_az;                                          //平衡点推力值 - 平衡推力的选取(手飞大概选取，稍微选小0.01)
//---------------------------------------无人机状态---------------------------------------------
mavros_msgs::State current_state;                       //无人机当前状态[包含上锁状态 模式] (从飞控中读取)

geometry_msgs::Pose pos_drone;                          //无人机当前位置 姿态(从飞控中读取)
geometry_msgs::Pose vel_drone;                          //无人机当前速度(从飞控中读取)

float PIX_Euler[3];                                     //无人机当前欧拉角(从飞控中读取)
float PIX_Euler_target[3];                              //无人机当前期望欧拉角(从飞控中读取)
float Thrust_target;                                    //期望推力(从飞控中读取)
//---------------------------------------上层指令-----------------------------------------------
mavros_msgs::Command Command_Now;                      //无人机当前执行命令
mavros_msgs::Command Command_Last;                     //无人机上一条执行命令

//自定义的Command变量
//相应的命令分别为 待机 起飞 悬停 降落 移动(惯性系ENU) 上锁 移动(机体系)
//但目前 起飞和待机 并没有正式使用
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
//---------------------------------------控制相关-----------------------------------------------
ros::Publisher accel_pub;
mavros_msgs::PositionTarget pos_setpoint;               //即将发给无人机的控制指令  ----  加速度设定值，偏航角设定值 (发送给飞控,通过setpoint_saw.cpp)

geometry_msgs::Pose takeoff_pos;                        //无人机起飞位置


//--------------------------------------位置环控制算法相关------------------------------------------------------
PID PIDX, PIDY, PIDZ, PIDVX, PIDVY ,PIDVZ;                      //声明PID类

// PID参数
float x_p,y_p,z_p;                                                          //位置环 控制参数 - 比例参数
float vx_p,vy_p,vz_p;                                                       //速度环 控制参数 - 比例参数
float vx_i,vy_i,vz_i;                                                       //速度环 控制参数 - 积分参数
float vx_d,vy_d,vz_d;                                                       //速度环 控制参数 - 微分参数
float Output_Max_x, Output_Max_y, Output_Max_z;                             //控制参数 - 最大值限幅
float I_Max_vx, I_Max_vy, I_Max_vz;                                         //控制参数 - 积分限幅
float Output_Max_vx, Output_Max_vy, Output_Max_vz;                          //控制参数 - 最大值限幅
float Thres_vx, Thres_vy, Thres_vz;                                         //控制参数 - 死区限幅

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void quaternion_2_euler(float quat[4], float angle[3]);                              //四元数转欧拉角
float get_ros_time(ros::Time begin);                                                 //获取ros当前时间
int pix_controller(float cur_time);                                                  //位置环控制程序
void prinft_drone_state(float current_time);                                         //打印无人机状态函数
void prinft_command_state();                                                         //打印控制指令函数
void rotation_yaw(float yaw_angle, float input[2], float output[2]);                 //坐标转换
void printf_param();                                                                 //打印各项参数以供检查
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

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
//命令回调函数
void Command_cb(const mavros_msgs::Command::ConstPtr& msg)
{
    Command_Now = *msg;
}


void euler_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg)
{
    float q[4];
    q[0] = msg->orientation.w;
    q[1] = msg->orientation.x;
    q[2] = msg->orientation.y;
    q[3] = msg->orientation.z;
    quaternion_2_euler(q, PIX_Euler_target);

    Thrust_target = msg->thrust;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh("~");

    // 【订阅】无人机当前状态 - 来自飞控
    //  本话题来自飞控(通过/plugins/sys_status.cpp)
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    // 【订阅】指令
    //  本话题来自根据需求自定义的上层模块，比如track_land.cpp 比如move.cpp
    ros::Subscriber Command_sub = nh.subscribe<mavros_msgs::Command>("/mavros/Command", 10, Command_cb);

    // 【订阅】无人机当前位置 坐标系:NED系
    //  本话题来自飞控(通过/plugins/local_position.cpp发送), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);

    // 【订阅】无人机当前速度 坐标系:NED系
    //  本话题来自飞控(通过/plugins/local_position.cpp发送), 对应Mavlink消息为LOCAL_POSITION_NED (#32), 对应的飞控中的uORB消息为vehicle_local_position.msg
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 100, vel_cb);

    // 【订阅】无人机当前欧拉角 坐标系:NED系
    //  本话题来自飞控(通过/plugins/imu.cpp发送), 对应Mavlink消息为ATTITUDE (#30), 对应的飞控中的uORB消息为vehicle_attitude.msg
    ros::Subscriber euler_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, euler_cb);

    // 【订阅】无人机期望欧拉角 坐标系:NED系
    //  本话题来自飞控(通过/plugins/setpoint_raw.cpp发送), 对应Mavlink消息为ATTITUDE_TARGET (#83), 对应的飞控中的uORB消息为vehicle_attitude_setpoint.msg
    ros::Subscriber euler_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 10, euler_target_cb);

    // 【发布】加速度期望值 坐标系 NED系
    //  本话题要发送飞控(通过/plugins/setpoint_raw.cpp发送), 对应Mavlink消息为SET_POSITION_TARGET_LOCAL_NED (#84), 对应的飞控中的uORB消息为position_setpoint_triplet.msg
    accel_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // 【服务】解锁上锁
    //  本服务通过 /plugins/command.cpp 实现
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    // 【服务】修改系统模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    mavros_msgs::SetMode mode_cmd;

    mavros_msgs::CommandBool arm_cmd;

    // 频率 [50Hz]
    ros::Rate rate(50.0);

    // 如果使用 roslaunch mavros position_control.launch 启动本节点,则可以读取参数文档position_control_config.yaml中的参数值；
    // 否则如果用 rosrun mavros position_control 启动本节点，则会使用下列默认参数

    // 位置环比例参数
    nh.param<float>("x_p", x_p, 1.0);
    nh.param<float>("y_p", y_p, 1.0);
    nh.param<float>("z_p", z_p, 0.8);

    // 速度环比例参数
    nh.param<float>("vx_p", vx_p, 1.0);
    nh.param<float>("vy_p", vy_p, 1.0);
    nh.param<float>("vz_p", vz_p, 0.8);

    // 速度环积分参数
    nh.param<float>("vx_i", vx_i, 0.0);
    nh.param<float>("vy_i", vy_i, 0.0);
    nh.param<float>("vz_i", vz_i, 0.0);

    // 速度环微分参数
    nh.param<float>("vx_d", vx_d, 0.0);
    nh.param<float>("vy_d", vy_d, 0.0);
    nh.param<float>("vz_d", vz_d, 0.0);

    // 位置环输出最大值 (相当于速度限制幅度)
    nh.param<float>("Output_Max_x", Output_Max_x, 2.0);
    nh.param<float>("Output_Max_y", Output_Max_y, 2.0);
    nh.param<float>("Output_Max_z", Output_Max_z, 2.0);

    // 速度环积分最大值
    nh.param<float>("I_Max_vx", I_Max_vx, 3.0);
    nh.param<float>("I_Max_vy", I_Max_vy, 3.0);
    nh.param<float>("I_Max_vz", I_Max_vz, 3.0);

    // 速度环输出最大值 (相当于加速度限制幅度)
    nh.param<float>("Output_Max_vx", Output_Max_vx, 0.2);
    nh.param<float>("Output_Max_vy", Output_Max_vy, 0.2);
    nh.param<float>("Output_Max_vz", Output_Max_vz, 0.5);

    // 速度环死区
    nh.param<float>("Thres_vx", Thres_vx, 0.0);
    nh.param<float>("Thres_vy", Thres_vy, 0.0);
    nh.param<float>("Thres_vz", Thres_vz, 0.0);

    // 起飞高度
    nh.param<float>("takeoff_height", takeoff_height, 1.0);

    // 平衡推力
    nh.param<float>("trim_az", trim_az, -0.25);

    // 设置PID参数 比例参数 积分参数 微分参数 微分项滤波增益
    PIDX.setPID( x_p,  0,  0,  0);
    PIDY.setPID( y_p,  0,  0,  0);
    PIDZ.setPID( z_p,  0,  0,  0);
    PIDVX.setPID( vx_p,  vx_i,  vx_d,  0);
    PIDVY.setPID( vy_p,  vy_i,  vy_d,  0);
    PIDVZ.setPID( vz_p,  vz_i,  vz_d,  0);

    // 设置积分上限 控制量最大值 误差死区
    PIDX.set_sat(0, Output_Max_x, 0.0);
    PIDY.set_sat(0, Output_Max_y, 0.0);
    PIDZ.set_sat(0, Output_Max_z, 0.0);
    PIDVX.set_sat(I_Max_vx, Output_Max_vx, Thres_vx);
    PIDVY.set_sat(I_Max_vy, Output_Max_vy, Thres_vy);
    PIDVZ.set_sat(I_Max_vz, Output_Max_vz, Thres_vz);

    //打印现实检查参数
    printf_param();

    int check_flag;
    // 这一步是为了程序运行前检查一下参数是否正确
    // 输入1,继续，其他，退出程序
    cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
    cin >> check_flag;

    if(check_flag != 1)
    {
        return -1;
    }

    // 等待和飞控的连接
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Not Connected");
    }

    // 连接成功
    ROS_INFO("Connected!!");

    // 先读取一些飞控的数据
    int i =0;
    for(i=0;i<50;i++)
    {
        ros::spinOnce();
        rate.sleep();

    }

    //读取起飞点的位置（当前飞机位置）
    takeoff_pos = pos_drone;

    //初始化命令-
    // 默认设置：move模式 子模式：位置控制 起飞到当前位置点上方

    Command_Now.comid = 0;
    Command_Now.command = Move;
    Command_Now.sub_mode = 0;
    Command_Now.pos_sp[0] = takeoff_pos.position.x;          //NED Frame
    Command_Now.pos_sp[1] = takeoff_pos.position.y;          //NED Frame
    Command_Now.pos_sp[2] = takeoff_pos.position.z - takeoff_height;         //NED Frame
    Command_Now.vel_sp[0] = 0;          //NED Frame
    Command_Now.vel_sp[1] = 0;          //NED Frame
    Command_Now.vel_sp[2] = 0;          //NED Frame
    Command_Now.yaw_sp = 0;

    //初始化悬停的位置值
    float Hold_position_NED[3];

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();
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
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        // 当前时间
        float cur_time = get_ros_time(begin_time);

        //无人机一旦接受到Land指令，则会屏蔽其他指令
        if(Command_Last.command == Land)
        {
            Command_Now.command = Land;
        }

        switch (Command_Now.command)
        {

        // 【待机指令】
        // 这个状态暂时未启用，留作以后开发
        case Standby:

            break;

        // 【起飞指令】
        // 这个状态暂时未启用，留作以后开发
        case Takeoff:

            break;

        // 【悬停指令】
        //  无人机立刻记录此时自身位置，然后悬停于该位置
        case Hold:
            // 如果上一次执行的命令不是Hold，记录此时进入的飞机位置（即第一次执行hold指令时飞机的位置）
            if (Command_Last.command != Hold)
            {
              Hold_position_NED[0] = pos_drone.position.x;
              Hold_position_NED[1] = pos_drone.position.y;
              Hold_position_NED[2] = pos_drone.position.z;
            }

            // xyz均使用位置控制子模式
            Command_Now.sub_mode = 0;
            Command_Now.pos_sp[0] = Hold_position_NED[0];
            Command_Now.pos_sp[1] = Hold_position_NED[1];
            Command_Now.pos_sp[2] = Hold_position_NED[2];
            Command_Now.yaw_sp = 0 ;

            //位置控制器
            pix_controller(cur_time);
            //打印无人机当前状态信息
            prinft_drone_state(cur_time);
            //打印控制指令信息
            prinft_command_state();

            break;

        // 【降落指令】
        //  无人机立刻记录此时自身位置，然后降落于该位置
        case Land:
            if (Command_Last.command != Land)
            {
                Command_Now.sub_mode = 0;
                Command_Now.pos_sp[0] = pos_drone.position.x;
                Command_Now.pos_sp[1] = pos_drone.position.y;
                Command_Now.pos_sp[2] = takeoff_pos.position.z;
                Command_Now.yaw_sp = 0 ;
            }

            //如果距离起飞高度小于20厘米，则直接上锁并切换为手动模式；
            if(abs(pos_drone.position.z - takeoff_pos.position.z) < ( 0.2))
            {
                if(current_state.mode == "OFFBOARD")
                {
                    mode_cmd.request.custom_mode = "MANUAL";
                    set_mode_client.call(mode_cmd);
                }

                if(current_state.armed)
                {
                    arm_cmd.request.value = false;
                    arming_client.call(arm_cmd);

                }

                if (arm_cmd.response.success)
                {
                    cout<<"Disarm successfully!"<<endl;
                }
            }else
            {
                //位置控制器
                 pix_controller(cur_time);
            }

            //打印无人机当前状态信息
            prinft_drone_state(cur_time);
            //打印控制指令信息
            prinft_command_state();
            break;

        // 【紧急上锁指令】
        case Disarm:

            if(current_state.mode == "OFFBOARD")
            {
                mode_cmd.request.custom_mode = "MANUAL";
                set_mode_client.call(mode_cmd);
            }

            if(current_state.armed)
            {
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);

            }

            if (arm_cmd.response.success)
            {
                cout<<"Disarm successfully!"<<endl;
            }

            break;

        // 【惯性系移动指令】
        case Move:

            //位置控制器
            pix_controller(cur_time);
            //打印无人机当前状态信息
            prinft_drone_state(cur_time);
            //打印控制指令信息
            prinft_command_state();

            break;

        // 【机体系移动指令】
        //  实现方法就是：
        //      1、将机体系期望位置转化到惯性系下
        //      2、同惯性系移动控制指令
        case Moving_Body:

            //记录进入时,飞机的高度
            static double desire_position_NED[3] = {0, 0, pos_drone.position.z};
            static float desire_velocity_NED[3] = {0, 0, 0};
            static bool pz_is_nzero = 1;

            //只有在comid增加时才会进入解算
            if( Command_Now.comid  !=  Command_Last.comid )
            {
              //xy velocity mode
              if( Command_Now.sub_mode & 0b10 )
              {
                float d_vel_body[2] = {Command_Now.vel_sp[0], Command_Now.vel_sp[1]};         //the desired xy velocity in Body Frame
                float d_vel_ned[2];                                                           //the desired xy velocity in NED Frame
                //机体系到NED系
                rotation_yaw( PIX_Euler[2], d_vel_body, d_vel_ned);
                desire_velocity_NED[0] = d_vel_ned[0];
                desire_velocity_NED[1] = d_vel_ned[1];
              }
              //xy position mode
              else
              {
                float d_pos_body[2] = {Command_Now.pos_sp[0], Command_Now.pos_sp[1]};         //the desired xy position in Body Frame
                float d_pos_ned[2];                                                           //the desired xy position in NED Frame (The origin point is the drone)
                rotation_yaw( PIX_Euler[2], d_pos_body, d_pos_ned);

                desire_position_NED[0] = pos_drone.position.x + d_pos_ned[0];
                desire_position_NED[1] = pos_drone.position.y + d_pos_ned[1];                           //在当前位置上累加

              }

              //z velocity mode
              if( Command_Now.sub_mode & 0b01 )
              {
                desire_velocity_NED[2] = Command_Now.vel_sp[2];
              }
              //z position mode
              else
              {
                if(Command_Now.pos_sp[2] != 0)
                {
                  // the pos_sp[2] is not zero
                  pz_is_nzero = 1;
                  desire_position_NED[2] = pos_drone.position.z + Command_Now.pos_sp[2];
                }
                else
                {
                  //??
                  if( (Command_Now.pos_sp[2] == 0) && (pz_is_nzero == 1) )
                  {
                    // the pos_sp[2] is not zero
                    pz_is_nzero = 0;
                    desire_position_NED[2] = pos_drone.position.z + Command_Now.pos_sp[2];
                  }
                }

              }

            }

            Command_Now.pos_sp[0] = desire_position_NED[0];
            Command_Now.pos_sp[1] = desire_position_NED[1];
            Command_Now.pos_sp[2] = desire_position_NED[2];
            Command_Now.vel_sp[0] = desire_velocity_NED[0];
            Command_Now.vel_sp[1] = desire_velocity_NED[1];
            Command_Now.vel_sp[2] = desire_velocity_NED[2];
            Command_Now.yaw_sp = 0;


            //位置控制器
            pix_controller(cur_time);
            //打印无人机当前状态信息
            prinft_drone_state(cur_time);
            //打印控制指令信息
            prinft_command_state();

            break;
        }

        //记录上一时刻执行的指令
        Command_Last = Command_Now;
        //执行回调函数
        ros::spinOnce();
        //周期休眠
        rate.sleep();

    }

    return 0;

}




// 【位置环控制函数】
int pix_controller(float cur_time)
{
    // 积分标志位.未进入OFFBOARD时,不累积积分项;进入OFFBOARD时,开始积分.
    PIDX.start_intergrate_flag = 1;
    PIDY.start_intergrate_flag = 1;
    PIDZ.start_intergrate_flag = 1;
    PIDVX.start_intergrate_flag = 1;
    PIDVY.start_intergrate_flag = 1;
    PIDVZ.start_intergrate_flag = 1;
    if(current_state.mode != "OFFBOARD")
    {
        PIDX.start_intergrate_flag = 0;
        PIDY.start_intergrate_flag = 0;
        PIDZ.start_intergrate_flag = 0;
        PIDVX.start_intergrate_flag = 0;
        PIDVY.start_intergrate_flag = 0;
        PIDVZ.start_intergrate_flag = 0;
    }
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>位 置 环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // xy通道
    if((Command_Now.sub_mode & 0b10) == 0)
    {
        //xy位置控制
        //计算误差
        float error_x = Command_Now.pos_sp[0] - pos_drone.position.x;
        float error_y = Command_Now.pos_sp[1] - pos_drone.position.y;

        //传递误差
        PIDX.add_error(error_x, cur_time);
        PIDY.add_error(error_y, cur_time);

        //计算输出
        PIDX.pid_output();
        PIDY.pid_output();
    }
    else
    {
        //xy速度控制
        //直接赋值
        PIDX.Output = Command_Now.vel_sp[0];
        PIDY.Output = Command_Now.vel_sp[1];
    }

    // z通道
    if((Command_Now.sub_mode & 0b01) == 0)
    {
        //z位置控制
        //计算误差
        float error_z = Command_Now.pos_sp[2] - pos_drone.position.z;
        //传递误差
        PIDZ.add_error(error_z, cur_time);
        //计算输出
        PIDZ.pid_output();
    }
    else
    {
        //z速度控制
        //直接赋值
        PIDZ.Output = Command_Now.vel_sp[2];
    }

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>速 度 环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //计算误差
    float error_vx = PIDX.Output - vel_drone.position.x;
    float error_vy = PIDY.Output - vel_drone.position.y;
    float error_vz = PIDZ.Output - vel_drone.position.z;
    //传递误差
    PIDVX.add_error(error_vx, cur_time);
    PIDVY.add_error(error_vy, cur_time);
    PIDVZ.add_error(error_vz, cur_time);
    //计算输出
    PIDVX.pid_output();
    PIDVY.pid_output();
    PIDVZ.pid_output();

    //赋值到期望加速度

    //pos_setpoint.type_mask = 0b100000111111;  // 100 000 111 111   加速度 + yaw
    pos_setpoint.type_mask = (2 << 10) | (7 << 3) | (7 << 0);  // 100 000 111 111   加速度 + yaw

    //【注意】
    // 这里我经常遇到一个坑 x和y莫名其妙会交换。。且z的方向也会莫名变反 找不到原因
    // 解决方法： 试飞前先手动检查xyz指令方向是否正确
    pos_setpoint.acceleration_or_force.x = PIDVX.Output;   //x轴加速度控制指令
    pos_setpoint.acceleration_or_force.y = PIDVY.Output;   //y轴加速度控制指令
    pos_setpoint.acceleration_or_force.z = trim_az + PIDVZ.Output;   //Z轴加速度控制指令
    pos_setpoint.yaw = Command_Now.yaw_sp * 3.1415926/180;  //偏航角控制指令

    //【发布】 加速度指令
    accel_pub.publish(pos_setpoint);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>打   印   数   据<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Control State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "Control result [ax ay az]: " << pos_setpoint.acceleration_or_force.x << "  "<< pos_setpoint.acceleration_or_force.y <<" " << pos_setpoint.acceleration_or_force.z <<endl;
    cout << "Control result [psai]: " << pos_setpoint.yaw*180/3.1415 <<endl;

    return 0;
}

// 【打印无人机状态函数】
void prinft_drone_state(float current_time)
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "Time: " << current_time <<" [s] "<<endl;

    //是否和飞控建立起连接
    if (current_state.connected == true)
    {
        cout << "Connected State: [ "<< "Connected" <<" ]" <<endl;
    }
    else
    {
        cout << "Connected State: [ "<< "Unconnected" <<" ]" <<endl;
    }

    //是否上锁
    if (current_state.armed == true)
    {
        cout << "Connected State: [ "<< "Armed" <<" ]" <<endl;
    }
    else
    {
        cout << "Connected State: [ "<< "DisArmed" <<" ]" <<endl;
    }

    cout << "Mode : [ " << current_state.mode<<" ]" <<endl;

    cout << "Position: [X Y Z] : " << " " << pos_drone.position.x << " [m] "<< pos_drone.position.y<<" [m] "<<pos_drone.position.z<<" [m] "<<endl;
    cout << "Velocity: [X Y Z] : " << " " << vel_drone.position.x << " [m/s] "<< vel_drone.position.y<<" [m/s] "<<vel_drone.position.z<<" [m/s] "<<endl;

    cout << "Attitude: [roll pitch yaw] : " << PIX_Euler[0] * 180/3.1415 <<" [°] "<<PIX_Euler[1] * 180/3.1415 << " [°] "<< PIX_Euler[2] * 180/3.1415<<" [°] "<<endl;

    cout << "Attitude_target: [roll pitch yaw] : " << PIX_Euler_target[0] * 180/3.1415 <<" [°] "<<PIX_Euler_target[1] * 180/3.1415 << " [°] "<< PIX_Euler_target[2] * 180/3.1415<<" [°] "<<endl;

    cout << "Thrust_target[0 - 1] : " << Thrust_target <<endl;

}


// 【打印控制指令函数】
void prinft_command_state()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>Command State<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    switch(Command_Now.command)
    {
    case Standby:
        cout << "Command: [ Standby ] " <<endl;
        break;
    case Hold:
        cout << "Command: [ Hold ] " <<endl;
        break;
    case Move:
        cout << "Command: [ Move ] " <<endl;
        break;
    case Takeoff:
        cout << "Command: [ Takeoff ] " <<endl;
        break;
    case Land:
        cout << "Command: [ Land ] " <<endl;
        break;
    case Disarm:
        cout << "Command: [ Disarm ] " <<endl;
        break;
    case Moving_Body:
        cout << "Command: [ Moving_Body ] " <<endl;
        break;
    }

    int sub_mode;
    sub_mode = Command_Now.sub_mode;
    if((sub_mode>>1) == 0) //xy channel
    {
        cout << "Submode: xy position control "<<endl;
        cout << "X_desired : " << Command_Now.pos_sp[0] << " [m]"   <<  "  Y_desired : " << Command_Now.pos_sp[1] << " [m]"   <<endl;
    }
    else{
        cout << "Submode: xy velocity control "<<endl;
        cout << "X_desired : " << Command_Now.vel_sp[0] << " [m/s]" << "  Y_desired : "  << Command_Now.vel_sp[1] << " [m/s]" <<endl;
    }
    if((sub_mode & 0b01) == 0) //z channel
    {
        cout << "Submode:  z position control "<<endl;
        cout << "Z_desired : "<< Command_Now.pos_sp[2] << " [m]" << endl;
    }
    else
    {
        cout << "Submode:  z velocity control -- Z_desired [NED] : "<< Command_Now.vel_sp[2] <<" [m/s]"<<endl;
        cout << "Z_desired : "<< Command_Now.vel_sp[2] << " [m/s]" <<endl;
    }

    cout << "Yaw is locked. " <<endl;
    cout << "Yaw Desired:  "  << Command_Now.yaw_sp << " [°] "  << "Current Yaw: "<< PIX_Euler[2]/ 3.1415926 *180 << " [°]  "<<endl;

}

// 【打印参数函数】
void printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout << "x_p : "<< x_p << endl;
    cout << "y_p : "<< y_p << endl;
    cout << "z_p : "<< z_p << endl;

    cout << "vx_p : "<< vx_p << endl;
    cout << "vy_p : "<< vy_p << endl;
    cout << "vz_p : "<< vz_p << endl;

    cout << "vx_i : "<< vx_i << endl;
    cout << "vy_i : "<< vy_i << endl;
    cout << "vz_i : "<< vz_i << endl;

    cout << "vx_d : "<< vx_d << endl;
    cout << "vy_d : "<< vy_d << endl;
    cout << "vz_d : "<< vz_d << endl;

    cout << "Output_Max_x : "<< Output_Max_x << endl;
    cout << "Output_Max_y : "<< Output_Max_y << endl;
    cout << "Output_Max_z : "<< Output_Max_z << endl;

    cout << "I_Max_vx : "<< I_Max_vx << endl;
    cout << "I_Max_vy : "<< I_Max_vy << endl;
    cout << "I_Max_vz : "<< I_Max_vz << endl;

    cout << "Output_Max_vx : "<< Output_Max_vx << endl;
    cout << "Output_Max_vy : "<< Output_Max_vy << endl;
    cout << "Output_Max_vz : "<< Output_Max_vz << endl;

    cout << "Thres_vx : "<< Thres_vx << endl;
    cout << "Thres_vy : "<< Thres_vy << endl;
    cout << "Thres_vz : "<< Thres_vz << endl;

    cout << "takeoff_height : "<< takeoff_height << endl;
    cout << "trim_az : "<< trim_az << endl;

}

// 【坐标系旋转函数】- 机体系到NED系
// input是机体系,output是惯性系，yaw_angle是当前偏航角
void rotation_yaw(float yaw_angle, float input[2], float output[2])
{
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}

// 【获取当前时间函数】 单位：秒
float get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

// 【四元数转Euler角函数】
// q0 q1 q2 q3
// w x y z
void quaternion_2_euler(float quat[4], float angle[3])
{
    angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
}
