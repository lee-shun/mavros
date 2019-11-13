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
#include <sensor_msgs/NavSatFix.h>

//SSS_CHANGE_TIME

#include <iostream>
#include <ctime>

//SSS_CHANGE

using namespace std;


//*******
//mavros_msgs::State current_state;
sensor_msgs::NavSatFix current_state;


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
/*state_cb 这个名字随意，但是要与订阅语句ros::Subscriber中的最后一个参数对应
//
//void msgCallback ( const ros_tutorials_topic::MsgTutorial::ConstPtr & msg )
//{}
*/
/*这是一个回调函数，当接收到 chatter 话题的时候就会被调用。
 * 消息是以 boost shared_ptr 指针的形式传输，
 * 这就意味着你可以存储它而又不需要复制数据。
 * /
*/

void state_cb(const sensor_msgs::NavSatFix::ConstPtr & msg)
{
    current_state = *msg;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int main(int argc, char **argv)
{

    /* You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    * 初始化 ROS 。它允许 ROS 通过命令行进行名称重映射——然而这并不是现在讨论的重点。
    * 在这里，我们也可以指定节点的名称——运行过程中，节点的名称必须唯一。
    * 这里的名称必须是一个 base name ，也就是说，名称内不能包含 / 等符号。
    */

    ros::init(argc, argv, "GPS_test");

    /*
     * NodeHandle是与ROS系统通信的主要访问点。
     *构造的第一个NodeHandle将完全初始化此节点，最后
     * NodeHandle被破坏将关闭节点。
     */
    ros::NodeHandle nh;

    // 频率 [50Hz]
    /*ros::Rate 对象可以允许你指定自循环的频率。
    它会追踪记录自上一次调用 Rate::sleep() 后时间的流逝，
    并休眠直到一个频率周期的时间。
    */
    ros::Rate rate(50.0);

    // topic订阅 [消息流向: 飞控 通过mavlink发送到 mavros包, mavros包发布, 我们订阅使用]
    // topic发布 [消息流向: 本程序发布特定的topic,mavros接收指定的topic,封装成mavlink协议的消息发送给飞控, 飞控通过mavlink模块解码接收,存为本地UORB消息并发布给其他模块使用 ]

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>查询飞控状态及更改系统模式<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    // home/nuc/mavros/src/mavros/mavros/src/plugins/sys_status.cpp
    // 作用:接收并发布 飞控系统的状态  例如 版本号 是否上锁 当前飞行模式 等 以及提供了 更改飞行模式 的服务

    // 订阅 系统当前状态
    /*subscribe<sensor_msgs::NavSatFix>中的<>中的内容填写的是笑死的类型，
    例如<std_msgs::String>表示的是 std_msgs/String 消息类型的消息
    <mavros_msgs::State>表示的是 mavros_msgs/State的消息
    第一个参数（带冒号的那个）表示话题的名称
    如果我们发布的消息的频率太高，缓冲区中的消息在大于10个的时候就会开始丢弃先前发布的消息。
    */

    ros::Subscriber GPS_INFO = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 100, state_cb);

    // 服务 修改系统模式
    //ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //mavros_msgs::SetMode mode_cmd;

    /*roscpp 会默认生成一个 SIGINT 句柄，它负责处理 Ctrl-C 键盘操作——使得 ros::ok() 返回 false。
    如果下列条件之一发生，ros::ok() 返回false：
    SIGINT 被触发 (Ctrl-C)
    被另一同名节点踢出 ROS 网络
    ros::shutdown() 被程序的另一部分调用
    节点中的所有 ros::NodeHandles 都已经被销毁
    一旦 ros::ok() 返回 false, 所有的 ROS 调用都会失效。
    */
    while(ros::ok())
    {/*
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
        */

        //Below SSS changed
        cout << "Latitude:  "<< current_state.latitude   <<endl;
        cout << "Longitude:  "<< current_state.longitude <<endl;
        cout << "Altitude:  "<< current_state.altitude   <<endl;
        //      cout << "Time:  "<< ros::Time begin  <<endl;


        time_t now = time(0);
        //cout << "本地日期和时间：" << now << endl;
        // 把 now 转换为字符串形式
        char* dt = ctime(&now);
        cout << "本地日期和时间：" << dt << endl;
        cout << endl;


        //Above SSS changed

        /*

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

        */
        //回调
        /*
         * 在这个例子中并不是一定要调用 ros::spinOnce()，因为我们不接受回调。
         * 然而，如果你的程序里包含其他回调函数，
         * 最好在这里加上 ros::spinOnce()这一语句，否则你的回调函数就永远也不会被调用了。
         */
        ros::spinOnce();

        //挂起一段时间(rate为 50HZ)


        rate.sleep();
        /*这条语句是调用 ros::Rate 对象来
         * 休眠一段时间以使得发布频率为 10Hz。
        */
    }

    return 0;


}
