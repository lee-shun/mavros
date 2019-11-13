// ros程序必备头文件
#include <ros/ros.h>

//mavros相关头文件
#include<fstream>
#include <iomanip>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include<math.h>

using namespace std;

//sensor_msgs::NavSatFix current_state;
nav_msgs::Odometry ned_vel;
geometry_msgs::TwistStamped gps_vel;
geometry_msgs::TwistStamped imu_vel;




void NED_vel_callback(const nav_msgs::Odometry::ConstPtr & msg1)
{
    ned_vel = *msg1;
//    cout <<fixed<<"ned_vel.twist.linear.x="<< setprecision(10)<< ned_vel.twist.twist.linear.x  << endl;
//    cout <<fixed<<"ned_vel.twist.linear.y="<< setprecision(10)<< ned_vel.twist.twist.linear.y  << endl;
//    cout <<fixed<<"ned_vel.twist.linear.z="<< setprecision(10)<< ned_vel.twist.twist.linear.z  << endl;
    //cout << endl;

}


void gps_vel_callback(const geometry_msgs::TwistStamped::ConstPtr & msg1)
{
    gps_vel = *msg1;
    cout <<fixed<<"gps_vel.twist.linear.x="<< setprecision(10)<< gps_vel.twist.linear.x  << endl;
    cout <<fixed<<"gps_vel.twist.linear.y="<< setprecision(10)<< gps_vel.twist.linear.y  << endl;
    cout <<fixed<<"gps_vel.twist.linear.z="<< setprecision(10)<< gps_vel.twist.linear.z  << endl;
    cout << endl;

}

//void imu_vel_callback(const geometry_msgs::TwistStamped::ConstPtr & msg2)
//{
//    gps_vel = *msg2;
//    cout <<fixed<<"imu_vel.twist.linear.x="<< setprecision(10)<< imu_vel.twist.linear.x  << endl;
//    cout <<fixed<<"imu_vel.twist.linear.y="<< setprecision(10)<< imu_vel.twist.linear.y  << endl;
//    cout <<fixed<<"imu_vel.twist.linear.z="<< setprecision(10)<< imu_vel.twist.linear.z  << endl;
//    cout << endl;

//}



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int main(int argc, char **argv)
{

    int i=1;

    ros::init(argc, argv, "vel_monitor");
    ros::NodeHandle nh;
    ros::Rate rate(30.0);

    ros::Subscriber NED_VEL = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 100, NED_vel_callback);
    //ros::Subscriber GPS_VEL = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/global_position/raw/gps_vel", 100, gps_vel_callback);
    //ros::Subscriber IMU_VEL = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 100, imu_vel_callback);

    double theta;
    double vx,vy;
    double d1,d2,dx,dy;
    dx =-10; //北向偏差
    dy =-10; //东向偏差
    while(ros::ok())
    {
        vy=ned_vel.twist.twist.linear.y;
        vx=ned_vel.twist.twist.linear.x;
        cout <<fixed<<"ned_vel.twist.twist.linear.x="<< setprecision(10)<< ned_vel.twist.twist.linear.x  << endl;
        cout <<fixed<<"ned_vel.twist.twist.linear.y="<< setprecision(10)<< ned_vel.twist.twist.linear.y  << endl;
        cout <<fixed<<"ned_vel.twist.twist.linear.z="<< setprecision(10)<< ned_vel.twist.twist.linear.z  << endl;

//        enu_x =
//        enu_y =

        cout <<fixed<<"ned_vel.twist.twist.linear.x="<< setprecision(10)<< ned_vel.twist.twist.linear.x  << endl;
        cout <<fixed<<"ned_vel.twist.twist.linear.y="<< setprecision(10)<< ned_vel.twist.twist.linear.y  << endl;
        cout <<fixed<<"ned_vel.twist.twist.linear.z="<< setprecision(10)<< ned_vel.twist.twist.linear.z  << endl;



        theta = atan(vy/vx);//弧度制
        printf("atan=%.2lf degrees\n",theta * 180.0/3.1416);

        d1 = cos(theta)*dx-sin(theta)*dy;
        d2 = sin(theta)*dx+cos(theta)*dy;

        cout <<fixed<<"d1="<< setprecision(10)<< d1  << endl;
        cout <<fixed<<"d2="<< setprecision(10)<< d2  << endl;

        cout<<endl;


        ros::spinOnce();
        rate.sleep();
    }

    return 0;


}
