/******************************************************
 * mavros_example_1.cpp                                *
 *                                                     *
 * Author: Qyp                                         *
 *                                                     *
 * Time: 2018.3.25                                     *
 *                                                     *
 * 说明: mavros示例程序1                                 *
 *      1. 系统状态订阅 更改模式                          *
 *      2. 纯做演示用,无实际用途                          *
 *                                                     *
 ******************************************************/

// ros程序必备头文件
#include <ros/ros.h>

//mavros相关头文件

#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>

using namespace std;

mavros_msgs::State current_state;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_example_1");
    ros::NodeHandle nh;

    // 频率 [50Hz]
    ros::Rate rate(50.0);

    // topic订阅 [消息流向: 飞控 通过mavlink发送到 mavros包, mavros包发布, 我们订阅使用]
    // topic发布 [消息流向: 本程序发布特定的topic,mavros接收指定的topic,封装成mavlink协议的消息发送给飞控, 飞控通过mavlink模块解码接收,存为本地UORB消息并发布给其他模块使用 ]

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>查询飞控状态及更改系统模式<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // home/nuc/mavros/src/mavros/mavros/src/plugins/sys_status.cpp
    // 作用:接收并发布 飞控系统的状态  例如 版本号 是否上锁 当前飞行模式 等 以及提供了 更改飞行模式 的服务

    // 订阅 系统当前状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    // 服务 修改系统模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode mode_cmd;

    while(ros::ok())
    {
        int flag;
        cout << "Input the mode:  # 0 for Offboard Mode;# 1 for AltControl Mode; #else for Manual Mode"<<endl;
        cin >> flag;
        if (flag == 0 )
        {
            mode_cmd.request.custom_mode = "OFFBOARD";
            set_mode_client.call(mode_cmd);
        }
        else if(flag == 1)
        {
            mode_cmd.request.custom_mode = "ALTCTL";
            set_mode_client.call(mode_cmd);
        }
        else
        {
            mode_cmd.request.custom_mode = "MANUAL";
            set_mode_client.call(mode_cmd);
        }

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

        //飞控(FCU)模式
        cout << "FCU Mode : [ " << current_state.mode << " ]" <<endl;

        //回调
        ros::spinOnce();
        //挂起一段时间(rate为 50HZ)
        rate.sleep();
    }
     return 0;


}
