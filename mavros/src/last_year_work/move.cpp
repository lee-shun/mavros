/*
 * move.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2018.8.17
 *
 * 说明: 无人机控制测试节点
 *      1. 发送惯性系移动指令
 *      2. 发送机体系移动指令
 *      3. 发送降落指令
 *      4. 留给用户自己使用的测试借口
 */

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
#include <std_msgs/Bool.h>
#include <mavros_msgs/Command.h>

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
ros::Publisher move_pub;

mavros_msgs::Command Command_now;
void generate_com(int sub_mode, float state_desired[4]);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move");
    ros::NodeHandle nh;

    move_pub = nh.advertise<mavros_msgs::Command>("/mavros/Command", 10);


    int flag_command;
    float state_desired[4];
    int sub_mode;
    int Num_StateMachine;
    Num_StateMachine = 0;

    while(ros::ok())
    {
        switch (Num_StateMachine)
        {
            // 输入
            case 0:
                cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
                cout << "Input the flag:  0 for move_ned，1 for move_body，2 for land,3 for user "<<endl;
                cin >> flag_command;

                // 降落指令
                if (flag_command == 2)
                {
                    Num_StateMachine = 3;
                    break;
                }
                // 自定义指令
                else if (flag_command == 3)
                {
                    Num_StateMachine = 4;
                    break;
                }
                // 机体系移动指令
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
                    break;
                }

                //输入控制子模式
                cout << "Input the sub_mode:  # 0 for xy/z position control; 3 for xy/z velocity control"<<endl;
                cin >> sub_mode;

                cout << "Please input next setpoint [x y z yaw]: "<< endl;

                //输入控制期望值
                cout << "setpoint_t[0] --- x [m] : "<< endl;
                cin >> state_desired[0];
                cout << "setpoint_t[1] --- y [m] : "<< endl;
                cin >> state_desired[1];
                cout << "setpoint_t[2] --- z [m] : "<< endl;
                cin >> state_desired[2];
                cout << "setpoint_t[3] --- yaw [du] : "<< endl;
                cout << "500 for input again: "<< endl;
                cin >> state_desired[3];

                //500  重新输入各数值
                if (state_desired[3] == 500)
                {
                    Num_StateMachine = 0;
                }

                break;

        case 1:
            Command_now.command =Move;
            generate_com(sub_mode, state_desired);
            move_pub.publish(Command_now);
            Num_StateMachine = 0;
            break;

        case 2:
            Command_now.command = Moving_Body;
            generate_com(sub_mode, state_desired);
            move_pub.publish(Command_now);
            Num_StateMachine = 0;
            break;

        case 3:
            Command_now.command = Land;
            move_pub.publish(Command_now);
            Num_StateMachine = 0;
            break;

        case 4:

            Num_StateMachine = 0;
            break;

        }

        cout << "desired state: [ " << Command_now.pos_sp[0] << "  "<< Command_now.pos_sp[1] << "  "<<Command_now.pos_sp[2] << "  " \
              << Command_now.vel_sp[0]<<"  "<< Command_now.vel_sp[1]<< "  "<< Command_now.vel_sp[2]<<"  " << Command_now.yaw_sp << "]  "  <<endl;

        sleep(2);
    }

    return 0;
}

// float32[3] pos_sp
// float32[3] vel_sp
// float32 yaw_sp
void generate_com(int sub_mode, float state_desired[4]){

    static int comid = 1;
    Command_now.sub_mode = sub_mode; //0b00
    if((sub_mode>>1) == 0) //xy channel
    {
        Command_now.pos_sp[0] = state_desired[0];
        Command_now.pos_sp[1] = state_desired[1];
        Command_now.vel_sp[0] = 0.0;
        Command_now.vel_sp[1] = 0.0;
        cout << "submode: xy position control "<<endl;
    }
    else{
        Command_now.pos_sp[0] = 0.0;
        Command_now.pos_sp[1] = 0.0;

        Command_now.vel_sp[0] = state_desired[0];
        Command_now.vel_sp[1] = state_desired[1];
        cout << "submode: xy velocity control "<<endl;
    }
    if((sub_mode & 0b01) == 0) //z channel
    {
        Command_now.pos_sp[2] = state_desired[2];
        Command_now.vel_sp[2] = 0.0;
        cout << "submode: z position control "<<endl;
    }
    else
    {
        Command_now.pos_sp[2] = 0.0;
        Command_now.vel_sp[2] = state_desired[2];
        cout << "submode: z velovity control "<<endl;
    }
    Command_now.yaw_sp = state_desired[3];
    Command_now.comid = comid;
    comid++;
}
