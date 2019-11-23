#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_receiver");
    ros::NodeHandle nh;

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          串口 打开           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
    //串口serial设置&开启
    serial::Serial sp;                                       //创建一个serial类
    serial::Timeout to = serial::Timeout::simpleTimeout(10); //创建timeout
    sp.setPort("/dev/ttyACM0");                              //设置要打开的串口名称
    sp.setBaudrate(115200);                                  //设置串口通信的波特率
    sp.setTimeout(to);                                       //串口设置timeout
    //打开串口
    try
    {
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    //判断串口是否打开成功
    if (sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    //----------------------------------------          串口打开 完成         --------------------------------------------------//

    ros::Rate loop_rate(20);
}
