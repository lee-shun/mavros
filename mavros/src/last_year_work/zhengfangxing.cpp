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
    ros::init(argc, argv, "zhengfangxing");
    ros::NodeHandle nh;
    // 频率 [10hz]
    ros::Rate rate(10.0);

    move_pub = nh.advertise<mavros_msgs::Command>("/mavros/Command", 10);

    // 【订阅】无人机当前位置 坐标系:NED系 [室外：GPS，室内：自主定位或mocap等]
    //ros::Subscriber position_sub = nh.subscribe<geometry_msgs::Pose>("/drone/pos", 10, pos_cb);

    float state_desired[4];
    int sub_mode;

    float size;
    size = 1.5;
    float height;
    height = -1.2;

    float sleep_time;
    sleep_time = 4;

    int i = 0;

    //起点 左下
    cout << "point 1"<<endl;
    Command_now.command = 4;
    sub_mode = 0;
    state_desired[0] = -size;
    state_desired[1] = -size;
    state_desired[2] = height;
    state_desired[3] = 0;
    generate_com(sub_mode, state_desired);

    i = 0;
    while (i < 10*sleep_time)
    {
        move_pub.publish(Command_now);

        rate.sleep();

        i++;

    }



    //左上
    cout << "point 1"<<endl;
    Command_now.command = 4;
    sub_mode = 0;
    state_desired[0] = size;
    state_desired[1] = -size;
    state_desired[2] = height;
    state_desired[3] = 0;
    generate_com(sub_mode, state_desired);

    i = 0;
    while (i < 10*sleep_time)
    {
        move_pub.publish(Command_now);

        rate.sleep();

        i++;

    }

    //右上
    Command_now.command = 4;
    sub_mode = 0;
    state_desired[0] = size;
    state_desired[1] = size;
    state_desired[2] = height;
    state_desired[3] = 0;
    generate_com(sub_mode, state_desired);

    i = 0;
    while (i < 10*sleep_time)
    {
        move_pub.publish(Command_now);

        rate.sleep();

        i++;

    }

    //左下
    Command_now.command = 4;
    sub_mode = 0;
    state_desired[0] = -size;
    state_desired[1] = size;
    state_desired[2] = height;
    state_desired[3] = 0;
    generate_com(sub_mode, state_desired);

    i = 0;
    while (i < 10*sleep_time)
    {
        move_pub.publish(Command_now);

        rate.sleep();

        i++;

    }

    //回到起点
    Command_now.command = 4;
    sub_mode = 0;
    state_desired[0] = -size;
    state_desired[1] = -size;
    state_desired[2] = height;
    state_desired[3] = 0;
    generate_com(sub_mode, state_desired);

    move_pub.publish(Command_now);

    sleep(sleep_time);

    Command_now.command = 4;
    sub_mode = 0;
    state_desired[0] = 0;
    state_desired[1] = 0;
    state_desired[2] = height;
    state_desired[3] = 0;
    generate_com(sub_mode, state_desired);

    i = 0;
    while (i < 10*sleep_time)
    {
        move_pub.publish(Command_now);

        rate.sleep();

        i++;

    }

    Command_now.command = 3;
    sub_mode = 0;
    state_desired[0] = 0;
    state_desired[1] = 0;
    state_desired[2] = height;
    state_desired[3] = 0;
    generate_com(sub_mode, state_desired);

    move_pub.publish(Command_now);


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
