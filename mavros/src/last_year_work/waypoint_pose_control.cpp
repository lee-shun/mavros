/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

/************************************************

注意事项:
1 注意经纬度转化
2 注意在push的时候一定要注意尺寸!!
3 不能对地址进行cout,否则会出现0等错误




************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>
#include<iomanip>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/CommandCode.h>



using namespace std;
using namespace mavros_msgs;




///注意!!!!!
/// 最后所有的经纬转米的公式里面的参数一定要改!!!

//一些常值，暂定为全局常值变量
const int NUAV = 6;   //给出天上盘旋的无人机数量
const int Npoint = 6; //给出目标点数量+
const double g = 9.8;  //重力加速度
bool mission_received = 0;
int r = 30; //盘旋半径
//投弹区中心点的经纬度
double wc;
double jc;
//jing0,wei0 的含义待定
double jing0,wei0;
double xc,yc,goal_x,goal_y;

//waypoint list的大小
int size;

double waypoint_lat ;
double waypoint_lon ;



double *xy2ll(double x, double y)
{
    double LL[2];
    double *ret= LL ;

    //纬度
    LL[0] = wei0 + y / 111177.0;
    //经度
    LL[1] = jing0 + x / 85155.0;

    //    cout<<fixed<< setprecision(10)<<"y[0] ="<< y[0] <<endl;
    //    cout<<fixed<< setprecision(10)<<"y[1] ="<< y[1]<<endl;

    return ret;

}

double *ll2xy(double w, double j)
{
    double XY[2];
    double *ret= XY;
    //X
    XY[0] = (j - jing0) * 85155.0;
    //Y
    XY[1] = (w - wei0) * 111177.0;

    //    cout<<fixed<< setprecision(10)<<"y[0] ="<< y[0] <<endl;
    //    cout<<fixed<< setprecision(10)<<"y[1] ="<< y[1]<<endl;

    return ret;

}




//===========================用于计算切点的子函数===================
//输入的数据为纬经
double *point_tangency(double g[2])
{
    //传入目标点
    //===========================圆心坐标为侦查区的中心===================
    double x0 = xc;
    double y0 = yc;

    //==========================================================================

    double r0 = r;
    double y[2];
    double x_1,y_1,x_2,y_2;
    double k1,k2,*res= y ;
    double x[2];
    double *add;
    //将用纬度和经度表示的转化为ENU系
    //    cout<<"x[0]="<< x[0] <<endl;
    //    cout<<"wei0="<< wei0 <<endl;

    //纬经高转化为ENU(xyz)
    //    x[1]= (g[0] - wei0) * 111177.0;
    //    x[0]= (g[1] - jing0) * 85155.0;
    //    cout<<"纬经高转化为ENU(xyz)1"<<endl;
    //    cout<<fixed<< setprecision(10)<<"x[0] ="<< x[0] <<endl;
    //    cout<<fixed<< setprecision(10)<<"x[1] ="<< x[1] <<endl;

    //此处代码已经测试成功!!
    add=ll2xy(g[0],g[1]);
    x[0]=add[0];
    x[1]=add[1];

    //    cout<<"纬经高转化为ENU(xyz)2"<<endl;
    //    cout<<fixed<< setprecision(10)<<"x[0] ="<< x[0] <<endl;
    //    cout<<fixed<< setprecision(10)<<"x[1] ="<< x[1] <<endl;


    //找出两个切点（x_1,y_1）(x_2,y_2)
    k1 = (y0*x0 + x[1]*x[0] - y0*x[0] - x[1]*x0 + sqrt(r0*r0*(-2*y0*x[1] - 2*x0*x[0] + x[1]*x[1] +
            y0*y0 + x0*x0 - r0*r0 + x[0]*x[0]))) / (-r0*r0 + x0*x0 - 2*x0*x[1] + x[0]*x[0]);
    k2= (y0*x0 + x[1]*x[0] - y0*x[0] - x[1]*x0 - sqrt(r0*r0*(-2*y0*x[1] - 2*x0*x[0] + x[1]*x[1] +
            y0*y0 + x0*x0 - r0*r0 + x[0]*x[0])))/(-r0*r0 + x0*x0 - 2*x0*x[0] + x[0]*x[0]);
    x_1 = (-k1*x[1] + x0 + k1*k1*x[0] + y0*k1) / (1 + k1*k1);
    y_1 =-(-x[1] - k1*x0 - y0*k1*k1 + k1*x[0]) / (1 + k1*k1);
    x_2 = (-k2*x[1] + x0 + k2*k2*x[0] + y0*k2) / (1 + k2*k2);
    y_2 =-(-x[1] - k2*x0 - y0*k2*k2 + k2*x[0]) / (1 + k2*k2);

    //%%%%判断逆时针是先到哪个切点%%%%%%%%
    double w[9] = {0,-1,y0,1,0,-x0,-y0,x0,0};
    double r1[3] = {x_1 - x0, y_1-y0, 0};
    double r2[3] = {x_2 - x0, y_2-y0, 0};
    double v1[3],v2[3];
    double s[3] = {x[0] - x0, x[1] - y0, 0};
    double s1=0,s2=0;
    for (int i = 0;i<3;i++)
    {
        v1[i] = w[i*3+0]*r1[0] + w[i*3+1]*r1[1] + w[i*3+2]*r1[2];
        v2[i] = w[i*3+0]*r2[0] + w[i*3+1]*r2[1] + w[i*3+2]*r2[2];
    }
    for (int i = 0;i < 3;i++)
    {
        s1 = s1 + s[i]*v1[i];
        s2 = s2 + s[i]*v2[i];
    }
    if (s1 > 0)
    {
        y[0] = x_1;
        y[1] = y_1;
        cout<<fixed<< setprecision(10)<<"y[0] ="<< x_1 <<endl;
        cout<<fixed<< setprecision(10)<<"y[1] ="<< y_1 <<endl;

    }
    if (s2 > 0)
    {
        y[0] = x_2;
        y[1] = y_2;
        cout<<fixed<< setprecision(10)<<"y[0] ="<< x_2 <<endl;
        cout<<fixed<< setprecision(10)<<"y[1] ="<< y_2 <<endl;

    }
    cout<<fixed<< setprecision(10)<<"res ="<< res <<endl;

    //ENU(xyz)转化为纬经高
    add=xy2ll(y[0],y[1]);
    y[0] = add[0];
    y[1] = add[1];

    //    y[0] = wei0 + y[1] / 111177.0;
    //    y[1] = jing0 + y[0] / 85155.0;
    //    cout<<"求切点的函数!"<<endl;
    //    cout<<fixed<< setprecision(10)<<"y[0] ="<< y[0] <<endl;
    //    cout<<fixed<< setprecision(10)<<"y[1] ="<< y[1] <<endl;

    return res;
}


mavros_msgs::State current_state;
sensor_msgs::NavSatFix gps_state;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void gps_cb(const sensor_msgs::NavSatFix::ConstPtr & msg)
{
    gps_state = *msg;

}




//geometry_msgs::Twist att_tw;
//void get_throttle(const geometry_msgs::Twist::ConstPtr& msg)
//{
//    att_tw = *msg;
//    ROS_INFO("%f %f %f %f", att_tw.angular.x, att_tw.angular.y, att_tw.angular.z,att_tw.linear.z);
//}

/**
# Waypoint.msg
#
# ROS representation of MAVLink MISSION_ITEM
# See mavlink documentation



# see enum MAV_FRAME
uint8 frame
uint8 FRAME_GLOBAL = 0
uint8 FRAME_LOCAL_NED = 1
uint8 FRAME_MISSION = 2
uint8 FRAME_GLOBAL_REL_ALT = 3
uint8 FRAME_LOCAL_ENU = 4

# see enum MAV_CMD and CommandCode.msg
uint16 command

bool is_current
bool autocontinue
# meaning of this params described in enum MAV_CMD
float32 param1
float32 param2
float32 param3
float32 param4
float64 x_lat
float64 y_long
float64 z_alt


*/

//用于打印航迹点的子函数
void printwaypoint(const mavros_msgs::WaypointList points)
{
    cout<<"count:"<<points.waypoints.size()<<endl;
    for (size_t i = 0; i < points.waypoints.size(); i++)
    {

        cout<<i<<" "<<points.waypoints[i].command<<" ";
        cout<<fixed<< setprecision(10)<<points.waypoints[i].x_lat<<" ";
        cout<<fixed<< setprecision(10)<<points.waypoints[i].y_long<<" ";
        cout<<fixed<< setprecision(10)<<points.waypoints[i].z_alt<<endl;

    }

}




mavros_msgs::WaypointList current_waypoints;
void get_waypoints(const mavros_msgs::WaypointList::ConstPtr& msg)
{
    current_waypoints = *msg;
    
    printwaypoint(current_waypoints);
}

//////////////////////////////////
////////// 主函数//////////////////
//////////////////////////////////
int main(int argc, char **argv)
{
    int i=1;              //飞机的序号
    double hmin = 30;      //飞机最小盘旋高度
    double h = 5,H;          //飞机高度间隔
    double v=15;            //飞机的飞行速度(单位:m/s)
    double paolenth;
    double R,pointlenth;
    double tan_x,tan_y;
    double *add;
    //由四个角点计算中心点的程序
    //输入四个角点的值
    double w1= 39.9897585,  w2=39.9897833,  w3=39.9882652,  w4= 39.9882500;
    double j1= 116.3526900, j2=116.3541295, j3=116.3542219, j4=116.3527874;
    //double wc,jc;
    wc = ( w1+w2+w3+w4 ) / 4;
    jc = ( j1+j2+j3+j4 ) / 4;

    //目前暂时以投弹区的中心点为wei0和jing0
    wei0 =wc;
    jing0=jc;

    cout<<"wc="<<wc<<endl;
    cout<<"jc="<<jc<<endl;

    //    double aaa[2],bbb[2];

    ros::init(argc, argv, "waypoints_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>("mavros/mission/waypoints", 10, get_waypoints);

    ros::Subscriber GPS_INFO = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 100, gps_cb);


    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient waypoint_setcurrent_client = nh.serviceClient<mavros_msgs::WaypointSetCurrent>
            ("mavros/mission/set_current");
    ros::ServiceClient waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>
            ("mavros/mission/pull");
    ros::ServiceClient waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>
            ("mavros/mission/push");
    ros::ServiceClient waypoint_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>
            ("mavros/mission/clear");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(0.20);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        cout<<"THE CONNECTION STATE OF FCU IS "<< current_state.connected <<endl;
        ros::spinOnce();
        rate.sleep();
    }



    //    //不知道是做什么用的
    //    geometry_msgs::PoseStamped pose;
    //    //send a few setpoints before starting
    //    for(int i = 20; ros::ok() && i > 0; --i){
    //        local_pos_pub.publish(pose);
    //        ros::spinOnce();
    //        rate.sleep();
    //    }


    // lat 47.397748, long 8.545596, alt 487.9
    mavros_msgs::WaypointList waypoint_list;


    //将当前的目标航点设为0号航点
    mavros_msgs::WaypointSetCurrent Current_wp;
    Current_wp.request.wp_seq=0;
    if (waypoint_setcurrent_client.call(Current_wp)&&Current_wp.response.success)
    {
        ROS_INFO("%d", Current_wp.response.success);
        ROS_INFO("Waypoint set to 0 success");
    }


    //暂时设定30个航点
    mavros_msgs::Waypoint waypoint[30];


    //参数表
    //改变速度 command=178  param1 = 未知 ：0/1  param2 = 速度 ：15 param3 = 未知 ：-1 param3 = 未知 ：0
    //改变速度的经纬度似乎可有可无
    //起飞 command=22  param1 = pitch : 45.0
    //飞航点 command=16  param1 = HOLD ：单位 秒
    //盘旋 command=17  param3 = 盘旋半径 ：单位 米
    // !!!!!!! 盘旋半径为正值时为顺时针盘旋,为负值时为逆时针盘旋



    ///设置起飞点
    //设置经纬高
    waypoint[0].x_lat = 39.9889384;
    waypoint[0].y_long = 116.3497164;
    waypoint[0].z_alt = 30;
    //设置命令类型
    waypoint[0].command = mavros_msgs::CommandCode::NAV_TAKEOFF;
    //设置坐标系，通常选FRAME_GLOBAL_REL_ALT（3）
    waypoint[0].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    //第一个点的autocontinue必设置为true()默认为false
    waypoint[0].autocontinue = true;
    waypoint[0].is_current = true;
    //设置飞机的起飞爬升角(目前还有待研究)
    waypoint[0].param1 = 45.0;


    ///设置盘旋点(由比赛场地决定)



    //
    waypoint[1].x_lat = wc;
    waypoint[1].y_long = jc;

    ///盘旋点的高度 i是飞机的序号,h是飞机飞行高度差
    H = hmin + h * i;
    waypoint[1].z_alt = H;
    waypoint[1].param3 = -30;
    //    waypoint[1].param4 = nan;
    waypoint[1].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
    //    waypoint[1].command = 31;//(LOITER_ALTITUTE) 实际最后可能还是会用上面那种
    waypoint[1].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    waypoint[1].autocontinue = true;
    waypoint[1].is_current = false;

    waypoint[1].param1 = 1;
    waypoint[1].param2 = -30.0;
    waypoint[1].param3 = 0;
    waypoint[1].param4 = 1;


    ros::Time last_request = ros::Time::now();
    //放在前面作为全局变量
    waypoint_lat = waypoint[1].x_lat;
    waypoint_lon = waypoint[1].y_long;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.MISSION";

    //    mavros_msgs::CommandBool arm_cmd;
    //    arm_cmd.request.value = true;

    set_mode_client.call(offb_set_mode);
    cout<<"set_mode=Mission"<<offb_set_mode.response.mode_sent<<endl<<endl;


    if (waypoint_setcurrent_client.call(Current_wp)&&Current_wp.response.success)
    {
        ROS_INFO("%d", Current_wp.response.success);
        ROS_INFO("Waypoint set to 0 success");
    }


    mavros_msgs::WaypointClear waypoint_clear;

    if (waypoint_clear_client.call(waypoint_clear)&&waypoint_clear.response.success)
    {
        ROS_INFO("Waypoint clear success");
    }

    // !(waypoint[1].is_current)
    //&&(ros::Time::now() - last_request > ros::Duration(5.0))
    double current_x,current_y;

    current_x = xc;
    current_y = yc;

    //    add=ll2xy(waypoint[0].x_lat,waypoint[0].y_long);
    //    waypoint[0].x_lat  = add[0];
    //    waypoint[0].y_long = add[1];




    double wp1_x,wp1_y;
    while (ros::ok()){

        current_x=gps_state.latitude;
        current_y=gps_state.longitude;
        add=ll2xy(current_x,current_y);
        current_x = add[0];
        current_y = add[1];
        cout << "current_x="<< current_x<<endl;
        cout << "current_y="<< current_y<<endl;

        add=ll2xy(waypoint[1].x_lat,waypoint[1].y_long);
        wp1_x = add[0];
        wp1_y = add[1];
        cout << "wp1_x="<< wp1_x<<endl;
        cout << "wp1_y="<< wp1_y<<endl;



        //清除航点,写入新的航点

        if(sqrt( (current_x-wp1_x)*(current_x-wp1_x)+
                 (current_y-wp1_y)*(current_y-wp1_y) ) > 5){

            continue;
        }

        //        if (waypoint_clear_client.call(waypoint_clear)&&waypoint_clear.response.success)
        //        {
        //            ROS_INFO("Waypoint clear success");
        //        }




        waypoint[0].x_lat  = waypoint[1].x_lat;
        waypoint[0].y_long = waypoint[1].y_long;

        //        add=ll2xy(waypoint[0].x_lat,waypoint[0].y_long);
        //        waypoint[0].x_lat  = add[0];
        //        waypoint[0].y_long = add[1];

        //        waypoint[1].x_lat  = waypoint[0].x_lat+5;
        //        waypoint[1].y_long = waypoint[0].y_long;

        //        add=xy2ll(waypoint[0].x_lat,waypoint[0].y_long);
        //        waypoint[0].x_lat  = add[0];
        //        waypoint[0].y_long = add[1];
        waypoint[0].z_alt = H;

        waypoint[0].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[0].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[0].autocontinue = true;
        waypoint[0].is_current = true;
        //            cout<<"waypoint[0].x_lat  ="<< waypoint[0].x_lat <<endl;


        waypoint[1].x_lat = wp1_x+15;
        waypoint[1].x_lat = wp1_x+15;

        add=xy2ll(waypoint[1].x_lat,waypoint[1].y_long);
        waypoint[1].x_lat  = add[0];
        waypoint[1].y_long = add[1];

        waypoint[1].z_alt = H;
        waypoint[1].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[1].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[1].autocontinue = true;
        waypoint[1].is_current = false;


        cout<<"waypoint[0].x_lat="<<waypoint[0].x_lat<<endl;
        cout<<"waypoint[0].y_long="<<waypoint[0].y_long<<endl;
        cout<<"waypoint[1].x_lat="<<waypoint[1].x_lat<<endl;
        cout<<"waypoint[1].y_long="<<waypoint[1].y_long<<endl;



        //        waypoint_lat = waypoint[1].x_lat;
        //        waypoint_lon = waypoint[1].y_long;


        WaypointPull waypoint_pull;

        if (waypoint_pull_client.call(waypoint_pull)&&waypoint_pull.response.success)
        {
            ROS_INFO("%d", waypoint_pull.response.wp_received);
            ROS_INFO("Waypoint pull success");
        }


        //push
        WaypointPush waypoint_push;
        for(i = 0; i < 2; i++){
            cout<<i<<endl;
            waypoint_push.request.waypoints.push_back(waypoint[i]);

        }

        if (waypoint_push_client.call(waypoint_push)&&waypoint_push.response.success)
        {
            ROS_INFO("%d", waypoint_push.response.wp_transfered);
            ROS_INFO("Waypoint push success");
        }

        if (waypoint_setcurrent_client.call(Current_wp)&&Current_wp.response.success)
        {
            ROS_INFO("%d", Current_wp.response.success);
            ROS_INFO("Waypoint set to 0 success");
        }





        //    while (ros::Time::now() - lCast_request < ros::Duration(1.0))
        //    {
        //        /* code for True */
        //    }





        //        while (ros::ok())
        //        {

        //            ros::spinOnce();
        //            rate.sleep();
        //        }


        ros::spinOnce();
        rate.sleep();
        ros::Time last_request = ros::Time::now();


    }
    return 0;

}
























///**
// * @file offb_node.cpp
// * @brief offboard example node, written with mavros version 0.14.2, px4 flight
// * stack and tested in Gazebo SITL
// */

//#include <ros/ros.h>
//#include <std_msgs/String.h>
//#include <stdio.h>
//#include <math.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Vector3Stamped.h>
//#include <std_msgs/Float64.h>
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/SetMode.h>
//#include <mavros_msgs/State.h>
//#include <mavros_msgs/GlobalPositionTarget.h>
//#include <mavros_msgs/HomePosition.h>
//#include <mavros_msgs/CommandHome.h>
//#include <mavros_msgs/Waypoint.h>
//#include <mavros_msgs/WaypointList.h>
//#include <mavros_msgs/WaypointPull.h>
//#include <mavros_msgs/WaypointPush.h>
//#include <mavros_msgs/WaypointClear.h>
//#include <mavros_msgs/WaypointSetCurrent.h>
//#include <mavros_msgs/CommandCode.h>

//#include"iostream"
////#include "math.h"
//#include <cmath>
//#include <iomanip>
//#include <fstream>
//using namespace std;

//#include <iostream>
//using namespace std;
//using namespace mavros_msgs;


/////变量定义
/////
/////
//double t[20][3];
//const double pi = acos(-1.0);
////j0, w0为原点的经纬度
//double jing0, wei0;
//mavros_msgs::Waypoint waypoint[20];
/////



//mavros_msgs::State current_state;
//void state_cb(const mavros_msgs::State::ConstPtr& msg){
//    current_state = *msg;
//}

//geometry_msgs::Twist att_tw;
//void get_throttle(const geometry_msgs::Twist::ConstPtr& msg)
//{
//    att_tw = *msg;
//    ROS_INFO("%f %f %f %f", att_tw.angular.x, att_tw.angular.y, att_tw.angular.z,att_tw.linear.z);
//}

///*
//# Waypoint.msg
//#
//# ROS representation of MAVLink MISSION_ITEM
//# See mavlink documentation



//# see enum MAV_FRAME
//uint8 frame
//uint8 FRAME_GLOBAL = 0
//uint8 FRAME_LOCAL_NED = 1
//uint8 FRAME_MISSION = 2
//uint8 FRAME_GLOBAL_REL_ALT = 3
//uint8 FRAME_LOCAL_ENU = 4

//# see enum MAV_CMD and CommandCode.msg
//uint16 command

//bool is_current
//bool autocontinue
//# meaning of this params described in enum MAV_CMD
//float32 param1
//float32 param2
//float32 param3
//float32 param4
//float64 x_lat
//float64 y_long
//float64 z_alt


//*/
//void printwaypoint(const mavros_msgs::WaypointList points)
//{
//    cout<<"count:"<<points.waypoints.size()<<endl;
//    for (size_t i = 0; i < points.waypoints.size(); i++)
//    {

//        cout<<i<<" "<<points.waypoints[i].command<<" ";
//        cout<<fixed<< setprecision(10)<<points.waypoints[i].x_lat<<" ";
//        cout<<fixed<< setprecision(10)<<points.waypoints[i].y_long<<" ";
//        cout<<fixed<< setprecision(10)<<points.waypoints[i].z_alt<<endl;



//    }

//}


//////将米转换为经纬度
////void m2L(double &x, double &y)
////{
////    x = jing0 + x / 85155.0;
////    y = wei0 + y / 111177.0;
////}


//////将经纬度转换为米(东北天)
////void L2m(double &w, double &j)
////{
////    w = (w - wei0) * 111177.0;
////    j = (j - jing0) * 85155.0;
////}


////将米转换为经纬度
//void m2L(double &x, double &y)
//{

//    x = jing0 + x / 85155.0;
//    y = wei0 + y / 111177.0;

//}
////将经纬度转换为米
//void L2m(double &j, double &w)
//{
//    j = (j - jing0) * 85155.0;
//    w = (w - wei0) * 111177.0;
//}


///*
//double cauy1(double x, double y, double u, double v, double r)
//{
//double y1, y2;
//y1 = v + sqrt(r*r - (x - u)*(x - u));
//y2 = v - sqrt(r*r - (x - u)*(x - u));
//if (abs(y1 - y) > abs(y2 - y))
//return y2;
//else return y1;
//}
//double cauy2(double x, double y, double u, double v, double r)
//{
//double x1, x2;
//x1 = u + sqrt(r*r - (y - v)*(y - v));
//x2 = u - sqrt(r*r - (y - v)*(y - v));
//if (abs(x1 - x) > abs(x2 - x))
//return x2;
//else return x1;
//}
//*/

//double dis(double x1, double y1, double x2, double y2)
//{
//    return ((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
//}

//void cau(double x, double y, double o1, double o2, double R,double *t)
//{
//    double m, n, a, b, r;
//    m = x;
//    n = y;
//    a = o1;
//    b = o2;
//    r = R;
//    double d2 = (m - a) * (m - a) + (n - b) * (n - b);
//    // 点到圆心距离
//    double d = sqrt(d2);
//    // 半径的平方
//    double r2 = r * r;
//    if (d2 < r2)
//    {
//        cout << "点在圆内，无切点" << endl;
//    }
//    else if (d2 == r2)
//    {
//        cout << "点在圆上，切点为给定点：(" << m <<
//                ", " << n << ")" << endl;
//    }
//    else
//    {
//        // 点到切点距离
//        double l = sqrt(d2 - r2);
//        // 点->圆心的单位向量
//        double x0 = (a - m) / d;
//        double y0 = (b - n) / d;
//        // 计算切线与点心连线的夹角
//        double f = asin(r / d);
//        // 向正反两个方向旋转单位向量
//        double x1 = x0 * cos(f) - y0 * sin(f);
//        double y1 = x0 * sin(f) + y0 * cos(f);
//        double x2 = x0 * cos(-f) - y0 * sin(-f);
//        double y2 = x0 * sin(-f) + y0 * cos(-f);
//        // 得到新座标
//        t[0] = (x1)* l + m;
//        t[1] = (y1)* l + n;
//        t[2] = (x2)* l + m;
//        t[3] = (y2)* l + n;

//        //cout << "点在圆外，切点有两个：(" << x1 << ", "
//        //<< y1 << ")和(" << x2 << ", " << y2 << ")" << endl;
//    }
//}


//void dub(double x0, double y0, double m, double a0, double b0, double n, double r)
//{

//    double x1, y1, x, y, a1, b1, a, b, q, u, v, k, xb, yb;
//    double e[4];


//    jing0 = x0;
//    wei0 = y0;
//    L2m(x0, y0);
//    L2m(a0, b0);
//    cout << x0 << endl << y0 << endl;
//    cout << a0 << endl << b0 << endl;



//    x = x0 + r*cos((m + 90) / 57.29578);//
//    y = y0 + r*sin((m + 90) / 57.29578);

//    x1 = x0 + r*cos((m - 90) / 57.29578);//
//    y1 = y0 + r*sin((m - 90) / 57.29578);


//    a = a0 + r*cos((n + 90) / 57.29578);//逆时针终止圆
//    b = b0 + r*sin((n + 90) / 57.29578);

//    a1 = a0 + r*cos((n - 90) / 57.29578);//顺时针终止圆
//    b1 = b0 + r*sin((n - 90) / 57.29578);



//    if (((a1 - x0)*(a1 - x0) + (b1 - y0)*(b1 - y0)) < ((a - x0)*(a - x0) + (b - y0)*(b - y0)))//shun
//    {
//        u = a1;
//        v = b1;
//        cau(0, 0, a1, b1, 30, e);
//        xb = e[0];
//        yb = e[1];

//    }
//    else
//    {
//        u = a;
//        v = b;
//        cau(0, 0, a, b, 30, e);
//        xb = e[2];
//        yb = e[3];

//    }



//    k = (yb - y0) / (xb - x0);
//    for (int i = 0; i < 5; i++)
//    {
//        t[i][0] = x0 + i*(xb - x0) / 4;
//        t[i][1] = y0 + i*(yb - y0) / 4;
//    }

//    /*
//    int j[10] = { 25, 21, 17, 13, 9, 5, 4, 3, 2, 1 };
//    t[5][0] = xb + j[0] * (a0 - xb) / 100;
//    t[5][1] = cauy1(t[5][0], yb, u, v, r);

//    for (int i = 1; i < 9; i++)
//    {
//        t[5 + i][0] = t[i + 5 - 1][0] + j[i] * (a0 - xb) / 100;
//        t[5 + i][1] = cauy1(t[i + 5][0], t[i + 5 - 1][1], u, v, r);
//        if (dis(t[5 + i][0], t[5 + i][1], a0, b0) > dis(t[5 + i - 1][0], t[5 + i - 1][1], a0, b0))
//        {
//            t[5 + i][1] = t[i + 5 - 1][1] + j[i] * (b0 - yb) / 100;
//            t[5 + i][0] = cauy2(t[i + 5 - 1][0], t[i + 5][1], u, v, r);
//        }
//    }
//    t[14][0] = a0; t[14][1] = b0;
//    */


//    //航迹点参数
//    int j[10] = { 17, 30, 41, 51, 61, 70, 75, 79, 82, 85 };
//    //t[5][0] = xb + j[0] * (a0 - xb) / 100;
//    //t[5][1] = cauy1(t[5][0], yb, u, v, r);
//    double s0, s1, s2, s3;
//    if (u == a)
//    {
//        s2 = atan2(b0 - b, a0 - a);
//        s1 = atan2(yb - b, xb - a);
//        if (abs(s2 - s1) > pi)
//        {
//            s3 = 2*pi - abs(s2 - s1);
//        }
//        else s3 = abs(s2 - s1);
//        for (int i = 0; i <= 9; i++)
//        {
//            s0 = s1 + j[i] * s3 / 85;
//            t[5 + i][0] = a + r*cos(s0);
//            t[5 + i][1] = b + r*sin(s0);
//        }
//    }
//    else
//    {
//        s2 = atan2(b0 - b1, a0 - a1);
//        s1 = atan2(yb - b1, xb - a1);
//        if (abs(s2 - s1) > pi)
//        {
//            s3 = 2 * pi - abs(s2 - s1);
//        }
//        else s3 = abs(s2 - s1);
//        for (int i = 0; i <=9; i++)
//        {
//            s0 = s1 - j[i] * s3 / 85;
//            t[5 + i][0] = a1 + r*cos(s0);
//            t[5 + i][1] = b1 + r*sin(s0);
//        }
//    }
//    //t[14][0] = a0; t[14][1] = b0;



//    //转换经纬高输出

//    for (int i = 0; i < 15; i++)
//    {
//        m2L(t[i][0], t[i][1]);
//    }

//    cout << endl << xb << endl << yb << endl;




//    //t[0][0] = 0; t[1][0] = 0;
//}


//double sita(double j1, double w1, double j2, double w2)
//{
//    double a;
//    L2m(j1, w1);
//    L2m(j2, w2);
//    a= atan2((w2 - w1), (j2 - j1));
//    if (((a - pi / 2) < pi) && ((a - pi / 2) > 0))
//    {
//        return (a-pi/2)*57.29578;
//    }
//    else return (2 * pi + a - pi / 2)*57.29578;
//}


///*
//double sita(double j1, double w1, double j2, double w2)
//{

//    L2m(j1, w1);
//    L2m(j2, w2);
//    return  atan2(abs((w2 - w1)), abs((j2 - j1)))*57.29578+270;

//}
//*/





////}

//void exchange()
//{
//        double q[20];
//        for (int i = 0; i < 20; i++)
//        {
//                q[i] = t[i][0];
//                t[i][0] = t[i][1];
//                t[i][1] = q[i];
//        }
//}

//void dubins(double b,double r, double w0, double j0, double w1, double j1, double w2, double j2, double w3, double j3, double w4, double j4)
//{
//        double c;//调节平飞的距离的参数
//        if (dis(j1, w1, j2, w2) > dis(j2, w2, j3, w3))
//        {
//                c = 5;
//        }
//        else c = 15;
//        j1 = j1 + (j1 - j4) / c;
//        j2 = j2 + (j2 - j3) / c;
//        w1 = w1 + (w1 - w4) / c;
//        w2 = w2 + (w2 - w3) / c;
//        double a = sita(j1, w1, j2, w2);
//        cout << a;
//        dub(j0, w0, 0, b*j2 + (1 - b)*j1, b*w2 + (1 - b)*w1, a, r);
//        t[19][0] = (1 - b)*j4 + b*j3;// (j4 * 2 + j3) / 3;
//        t[19][1] = (1 - b)*w4 + b*w3;// (w4 * 2 + w3) / 3;
//        for (int i = 15; i < 19; i++)
//        {
//                t[i][0] = t[14][0]+(i - 14)*(t[19][0] - t[14][0]) / 5;
//                t[i][1] = t[14][1]+(i - 14)*(t[19][1] - t[14][1]) / 5;
//        }
//        exchange();//如果注释掉的话就是输出经纬高
//        for (int i = 0; i < 20; i++)
//        {
//                t[i][2] = 40;
//        }

//        //保存数据文件
//        ofstream f1("5.txt");
//        for (int i = 0; i < 20; i++)
//        {
//                f1 << setiosflags(ios::fixed) << setprecision(6) << t[i][1] << "\t" << t[i][0] << "\t" << t[i][2] << endl;
//                cout << setprecision(9) << t[i][0] << ",";
//                cout << setprecision(8) << t[i][1] << endl;
//        }
//        f1.close();


//        for (int i = 0; i < 20; i++){
//            cout<< "t["<<i<<"][0]"<< t[i][0] <<endl;
//            cout<< "t["<<i<<"][1]"<< t[i][1] <<endl;
//            cout<< "t["<<i<<"][2]"<< t[i][2] <<endl;
//            cout<<endl;

//        }




//        for (int i = 0; i < 20; i++)
//            {
//                if (i == 0)
//                {
//                    //大部分的参数在没有设置的情况下都是默认为0的，因此一些参数是必须设置的
//                    //设置经纬高
//                    //    waypoint[0].x_lat = 39.717300415;
//                    //    waypoint[0].y_long = 116.495895386;
//                    //    waypoint[0].z_alt = 30;

//                    //用于设置QGC上的1 起飞那个点，用于驱动起飞程序，以及表示起飞后到达的第一个航点，并非真实的起飞点
//                    //真实的起飞点在环境变量中设置
//                    waypoint[i].x_lat = t[i][0];
//                    waypoint[i].y_long = t[i][1];
//                    //
//                    //此处注意以后的起飞高度要自己设置
//                    waypoint[i].z_alt = t[i][2];
//                    //设置命令类型
//                    waypoint[i].command = mavros_msgs::CommandCode::NAV_TAKEOFF;
//                    //设置坐标系，通常选FRAME_GLOBAL_REL_ALT（3）
//                    waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;

//                    waypoint[i].autocontinue = true;
//                    //第一个点的autocontinue必设置为true()默认为false
//                    waypoint[i].is_current = true;
//                    //            waypoint[i].is_current = true;
//                    waypoint[i].param1 = 45.0;
//                }

//                else
//                {
//                    waypoint[i].x_lat = t[i][0];
//                    waypoint[i].y_long = t[i][1];
//                    waypoint[i].z_alt = t[i][2];

//                    //                     cout<<"这个程序到底有没有改过??!!!" << endl;
//                    cout<< "waypoint["<<i<<"].x_lat "<< waypoint[i].x_lat<<endl;
//                    cout<< "waypoint["<<i<<"].y_long "<< waypoint[i].y_long<<endl;
//                    cout<< "waypoint["<<i<<"].z_alt "<< waypoint[i].z_alt<<endl;
//                    cout<<endl;

//                    //                    cout<< "waypoint["<<i<<"].x_lat""<<waypoint[i].x_lat;
//                    //                           cout<< "waypoint["<<i<<"].x_lat""<<waypoint[i].x_lat;

//                    //                           waypoint[i].y_long = t[i][1];
//                    //                    waypoint[i].z_alt = t[i][2];


//                    waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
//                    waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
//                    waypoint[i].autocontinue = true;
//                }
//            }










//}




//mavros_msgs::WaypointList current_waypoints;
//void get_waypoints(const mavros_msgs::WaypointList::ConstPtr& msg)
//{
//    current_waypoints = *msg;

//    printwaypoint(current_waypoints);
//}



//int main(int argc, char **argv)
//{
//    ros::init(argc, argv, "waypoints_node");
//    ros::NodeHandle nh;

//    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
//            ("mavros/state", 10, state_cb);
//    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
//            ("mavros/setpoint_position/local", 10);
//    ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>("mavros/mission/waypoints", 10, get_waypoints);

//    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
//            ("mavros/cmd/arming");
//    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
//            ("mavros/set_mode");
//    ros::ServiceClient waypoint_setcurrent_client = nh.serviceClient<mavros_msgs::WaypointSetCurrent>
//            ("mavros/mission/set_current");
//    ros::ServiceClient waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>
//            ("mavros/mission/pull");
//    ros::ServiceClient waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>
//            ("mavros/mission/push");
//    ros::ServiceClient waypoint_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>
//            ("mavros/mission/clear");

//    //the setpoint publishing rate MUST be faster than 2Hz
//    ros::Rate rate(20.0);



//    // wait for FCU connection
//    while(ros::ok() && current_state.connected){
//        ros::spinOnce();
//        rate.sleep();
//    }

//    geometry_msgs::PoseStamped pose;
//    //send a few setpoints before starting
//    for(int i = 20; ros::ok() && i > 0; --i){
//        local_pos_pub.publish(pose);
//        ros::spinOnce();
//        rate.sleep();
//    }
//    // lat 47.397748, long 8.545596, alt 487.9
//    mavros_msgs::WaypointList waypoint_list;
//    //mavros_msgs::Waypoint waypoint0,waypoint1,waypoint2,waypoint3,waypoint4,waypoint_setvelocity;
//    //    mavros_msgs::Waypoint waypoint[20];
//    mavros_msgs::WaypointSetCurrent Current_wp;
//    //    Current_wp.re.wp_seq=0;
//    //    waypoint_pull_client.call(waypoint_pull)&&waypoint_pull.response.success

//    Current_wp.request.wp_seq=0;

//    if (waypoint_setcurrent_client.call(Current_wp)&&Current_wp.response.success)
//    {
//        ROS_INFO("%d", Current_wp.response.success);
//        ROS_INFO("Waypoint set to 0 success");
//    }


//    double a;

//    dubins((double)5 / 6,   30,             39.9889384, 116.3497164, 39.9853533,    116.3519498,    39.9854672 ,    116.3552630, 39.9806438, 116.3554348, 39.9805485, 116.3520746);

//    ros::Time last_request = ros::Time::now();

//    last_request = ros::Time::now();

//    WaypointPush waypoint_push;

//    for (int i = 0; i < 20; i++)
//    {
//        waypoint_push.request.waypoints.push_back(waypoint[i]);
//    }


//    //    waypoint_push.request.waypoints.push_back(waypoint[0]);
//    //    waypoint_push.request.waypoints.push_back(waypoint[1]);
//    //waypoint_push.request.waypoints.push_back(waypoint[2]);
//    //    waypoint_push.request.waypoints.push_back(waypoint[1]);


//    //    waypoint_push.request.waypoints.push_back(waypoint_setvelocity);

//    //    waypoint_push.request.waypoints.push_back(waypoint0);
//    //    waypoint_push.request.waypoints.push_back(waypoint1);
//    //    waypoint_push.request.waypoints.push_back(waypoint2);
//    //    waypoint_push.request.waypoints.push_back(waypoint3);
//    //    waypoint_push.request.waypoints.push_back(waypoint4);

//    if (waypoint_push_client.call(waypoint_push)&&waypoint_push.response.success)
//    {
//        ROS_INFO("%d", waypoint_push.response.wp_transfered);
//        ROS_INFO("Waypoint push success");
//    }

//    while (ros::Time::now() - last_request < ros::Duration(1.0))
//    {
//        /* code for True */
//    }

//    WaypointPull waypoint_pull;
//    //Request update waypoint list.
//    if (waypoint_pull_client.call(waypoint_pull)&&waypoint_pull.response.success)
//    {
//        ROS_INFO("WaypointPull received is %d", waypoint_pull.response.wp_received);
//        ROS_INFO("Waypoint pull success");
//    }


//    while (ros::Time::now() - last_request < ros::Duration(1.0))
//    {
//        /* code for True */
//    }


//    while (ros::ok())
//    {

//        ros::spinOnce();
//        rate.sleep();
//    }

//    return 0;
//}



















































//// while (ros::Time::now() - last_request < ros::Duration(3.0))
//// {
////     /* code for True */
//// }

//// mavros_msgs::WaypointClear waypoint_clear;
//// if (waypoint_clear_client.call(waypoint_clear)&&waypoint_clear.response.success)
//// {
////     ROS_INFO("Waypoint clear success");
//// }
//last_request = ros::Time::now();

//WaypointPush waypoint_push;
///**
//    // for (size_t i = 0; i < waypoint_list.waypoints.size(); i++)
//    // {
//    //     // code for loop body waypoint_push.request.waypoints.push_back()
//    // }

//    //points.waypoints.size()
////    cout<<"waypoint_list.waypoints.size()"<<waypoint_list.waypoints.size()<<endl;
//*/
//for(i = 0; i < size; i++){
//    cout<<i<<endl;
//    waypoint_push.request.waypoints.push_back(waypoint[i]);

//}

//if (waypoint_push_client.call(waypoint_push)&&waypoint_push.response.success)
//{
//    ROS_INFO("%d", waypoint_push.response.wp_transfered);
//    ROS_INFO("Waypoint push success");
//}

////    while (ros::Time::now() - last_request < ros::Duration(1.0))
////    {
////        /* code for True */
////    }

//WaypointPull waypoint_pull;

//if (waypoint_pull_client.call(waypoint_pull)&&waypoint_pull.response.success)
//{
//    ROS_INFO("%d", waypoint_pull.response.wp_received);
//    ROS_INFO("Waypoint pull success");
//}

////    while (ros::Time::now() - last_request < ros::Duration(1.0))
////    {
////        /* code for True */
////    }


//// WaypointSetCurrent waypoint_setcurrent;
//// waypoint_setcurrent.request.wp_seq = 1;
//// if (waypoint_setcurrent_client.call(waypoint_setcurrent)&&waypoint_setcurrent.response.success)
//// {
////     ROS_INFO("Waypoint setcurrent success");
//// }
//// while (ros::Time::now() - last_request < ros::Duration(1.0))
//// {
////     /* code for True */
//// }


///**

////    mavros_msgs::SetMode offb_set_mode;
////    offb_set_mode.request.custom_mode = "AUTO.MISSION";

////    mavros_msgs::CommandBool arm_cmd;
////    arm_cmd.request.value = true;

////    set_mode_client.call(offb_set_mode);
////    cout<<offb_set_mode.response.success<<endl<<endl;


////    while (ros::ok())
////    {
////        //std::cout << current_state.mode << std::endl;
////        // if (current_state.mode != "OFFBOARD" &&
////        //     (ros::Time::now() - last_request > ros::Duration(1.0)))
////        // {
////        //     if( set_mode_client.call(offb_set_mode) &&
////        //         offb_set_mode.response.success){
////        //         ROS_INFO("Offboard enabled");
////        //     }
////        //     last_request = ros::Time::now();
////        // } else {
////        //     if( !current_state.armed &&
////        //         (ros::Time::now() - last_request > ros::Duration(1.0))){
////        //         if( arming_client.call(arm_cmd) &&
////        //             arm_cmd.response.success){
////        //             ROS_INFO("Vehicle armed");
////        //         }
////        //         last_request = ros::Time::now();
////        //     }
////        // }

////        // //pose.pose.position.x = r * cos(n/200);
////        // //pose.pose.position.y = r * sin(n/200);
////        // n=n+1;
////        // cout << pose.pose.orientation.w << " " << pose.pose.orientation.x << " " << pose.pose.orientation.y << " " << pose.pose.orientation.z << " ";

////        // local_pos_pub.publish(pose);
////        // ROS_INFO("%f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);


////    }
//*/
//ros::spinOnce();
//rate.sleep();

//return 0;
//}
