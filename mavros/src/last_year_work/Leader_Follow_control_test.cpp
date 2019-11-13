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

#include<string>
#include<stdlib.h>
#include<fstream>
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
#include <sensor_msgs/NavSatFix.h>



using namespace std;
using namespace mavros_msgs;

sensor_msgs::NavSatFix leadergps;
sensor_msgs::NavSatFix followergps;
geometry_msgs::PoseStamped leaderpose;
geometry_msgs::PoseStamped followerpose;
geometry_msgs::PoseStamped desiredpose;


///注意!!!!!
/// 最后所有的经纬转米的公式里面的参数一定要改!!!

sensor_msgs::NavSatFix current_gps;
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr & msg)
{
    current_gps = *msg;
    //cout << "gps回调函数"<<endl<<current_gps <<endl;

}

sensor_msgs::NavSatFix num1_gps;
void num1gps_callback(const sensor_msgs::NavSatFix::ConstPtr & msg)
{
    num1_gps = *msg;
    //    cout << "mother_latitude"<<endl<<num1_gps.latitude <<endl;
    //    cout << "mother_longitude"<<endl<<num1_gps.longitude <<endl;

}


//一些常值，暂定为全局常值变量
const int NUAV = 6;   //给出天上盘旋的无人机数量
const int Npoint = 6; //给出目标点数量+
const double g = 9.8;  //重力加速度
bool mission_received = 0;
int r = 30; //盘旋半径
double PI = 3.1415926;
double K = 0.05;                                   //parameter of gain
double jing0 = 0;
double wei0 = 0;
int inter_time = 8;



//投弹区中心点的经纬度
double wc;
double jc;

double xc,yc,goal_x,goal_y;

//waypoint list的大小
int size;


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

double CalcuVelAngel(double V_x, double V_y)                            //calculate the angel between navigation coordinate and velocity coordinate
{
    double theta;
    if ( V_x == 0)
    {
        if (V_y > 0)
        {
            theta = PI;
        }
        theta = -PI;
        return theta;
    }
    theta = atan(V_y / V_x);
    if (V_x <0)
    {
        if (V_y < 0)
        {
            theta = theta - PI;
        }
        else
        {
            theta = theta + PI;
        }
    }
    return theta;
}




mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//geometry_msgs::Twist att_tw;
//void get_throttle(const geometry_msgs::Twist::ConstPtr& msg)
//{
//    att_tw = *msg;
//    ROS_INFO("%f %f %f %f", att_tw.angular.x, att_tw.angular.y, att_tw.angular.z,att_tw.linear.z);
//}

/*
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
//    cout<<"count:"<<points.waypoints.size()<<endl;
//    for (size_t i = 0; i < points.waypoints.size(); i++)
//    {

//        cout<<i<<" "<<points.waypoints[i].command<<" ";
//        cout<<fixed<< setprecision(10)<<points.waypoints[i].x_lat<<" ";
//        cout<<fixed<< setprecision(10)<<points.waypoints[i].y_long<<" ";
//        cout<<fixed<< setprecision(10)<<points.waypoints[i].z_alt<<endl;
//    }

}


mavros_msgs::WaypointList current_waypoints;

void get_waypoints(const mavros_msgs::WaypointList::ConstPtr& msg)
{
    current_waypoints = *msg;
    printwaypoint(current_waypoints);
}






/////////////////////////////////////////////////////////
///////////////////////////////// 主函数//////////////////
////////////////////////////////////////////////////////



int main(int argc, char **argv)
{
    int i=1;              //飞机的序号
    double hmin = 30;      //飞机最小盘旋高度
    double h = 5,H;          //飞机高度间隔
    H =  hmin+i*h;
    double v=15;            //飞机的飞行速度(单位:m/s)
    double paolenth;
    double R,pointlenth;
    double tan_x,tan_y;
    double *add;
    //由四个角点计算中心点的程序
    //输入四个角点的值

    double leader_latitude[2], leader_longitude[2], time[2], follower_latitude, follower_longitude, desired_gap_x_V, desired_gap_y_V, desired_position_latitude, desired_position_longitude, desired_V;
    int init_flag = 1;

    double w1= 39.9897585,  w2=39.9897833,  w3=39.9882652,  w4= 39.9882500;
    double j1= 116.3526900, j2=116.3541295, j3=116.3542219, j4=116.3527874;
    //double wc,jc;
    wc = ( w1+w2+w3+w4 ) / 4;
    jc = ( j1+j2+j3+j4 ) / 4;

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

    //python 监视节点需要订阅的消息
    //仔鸡自己的GPS消息
    ros::Subscriber GPS_INFO = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 100, gps_callback);
    //母鸡的GPS
    ros::Subscriber gps_Sub = nh.subscribe<sensor_msgs::NavSatFix>("/gps_receiver/tranfered_gps", 10,num1gps_callback);

    //################################
    ros::Publisher gps_PubL = nh.advertise<sensor_msgs::NavSatFix>("leader_gps", 10);

    ros::Publisher gps_PubF = nh.advertise<sensor_msgs::NavSatFix>("follower_gps", 10);

    ros::Publisher pose_PubL = nh.advertise<geometry_msgs::PoseStamped>("leader_pose", 10);

    ros::Publisher pose_PubF = nh.advertise<geometry_msgs::PoseStamped>("follower_pose", 10);

    ros::Publisher pose_PubD = nh.advertise<geometry_msgs::PoseStamped>("desired_pose", 10);



    //python 以上是监视节点需要订阅的消息

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
    ros::Rate rate(0.8);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        cout<<"THE CONNECTION STATE OF FCU IS "<< current_state.connected <<endl;
        ros::spinOnce();
        rate.sleep();
    }

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

    ros::Time last_request = ros::Time::now();

//    while(ros::Time::now()-last_request<ros::Duration(5)){

//    }
    ros::Time now_request = ros::Time::now();

    ros::Time time_0,time_1;

    double leader_x[2], leader_y[2], follower_x, follower_y, leader_Vx, leader_Vy;
    double *xy,*ll;
    double desired_gap_x, desired_gap_y, desired_x, desired_y;


    //#####################################
    leader_latitude[0] = 39.9889318;
    leader_longitude[0] = 116.3514164;
    //#####################################



    ofstream outfile;//创建文件
    outfile.open("/home/sss/catkin_ws/Leader_Follower_sim.txt");
    outfile <<"" <<endl;
    while(ros::ok())
    {
        ros::spinOnce();
        last_request = ros::Time::now();

        //##########################
        //        leader_latitude[0]=num1_gps.latitude;
        //        leader_longitude[0]=num1_gps.longitude;

        //#######################################


        while(ros::Time::now()-last_request<ros::Duration(1)){

        }
        now_request = ros::Time::now();

        ros::spinOnce();
        //传入leader的位置
        //std::cout << 'Please input the leader\'s latitude and longitude and time' << std::endl;
        //std::cin >> leader_latitude[1] >> leader_longitude[1];


        //#############very important!!!
//        leader_latitude[1]=num1_gps.latitude;
//        leader_longitude[1]=num1_gps.longitude;
        //#############very important!!!

        //##########################################


        cout << "leader_latitude[0]=" <<leader_latitude[0]<<endl;
        cout << "leader_longitude[0]=" <<leader_longitude[0]<<endl;

        xy = ll2xy(leader_latitude[0], leader_longitude[0]);                        //leader position in time0
        leader_x[0] = xy[0];
        leader_y[0] = xy[1];

        cout << "leader_x[0]=" <<leader_x[0]<<endl;
        cout << "leader_y[0]=" <<leader_y[0]<<endl;


//################################################
        leader_x[1] = leader_x[0]+15;
        leader_y[1] = leader_y[0];

        cout << "leader_x[1]=" <<leader_x[1]<<endl;
        cout << "leader_y[1]=" <<leader_y[1]<<endl;

        ll = xy2ll(leader_x[1],leader_y[1]);
        leader_latitude[1] = ll[0];
        leader_longitude[1] = ll[1];
//###############################################

        leadergps.latitude= leader_latitude[1];
        leadergps.longitude= leader_longitude[1];

        gps_PubL.publish(leadergps);



        //########################################3

//        leader_latitude[0] = leader_latitude[1];
//        leader_longitude[0] = leader_longitude[1];
         now_request = ros::Time::now();

//        time[0] = time[1];
//        time[0] = last_request;
//        time[1] = now_request;
        time_0 = last_request;
        time_1 = now_request;

        //cout << "time_0=" <<time_0<<endl;
        //cout << "time_1=" <<time_1<<endl;

        //订阅follower此时的位置
        //        std::cout << 'Please input the follower\'s latitude and longitude' << std::endl;
        //        std::cin >> follower_latitude >> follower_longitude;
        //cout << "time_0=" <<time_0<<endl;
        //cout << "time_1=" <<time_1<<endl;

        ros::spinOnce();
        follower_latitude = current_gps.latitude;
        follower_longitude = current_gps.longitude;
        if((follower_latitude ==0)||(follower_longitude ==0)){
            continue;
        }
        //cout << "follower_latitude=" <<follower_latitude<<endl;
        //cout << "follower_longitude=" <<follower_longitude<<endl;
        followergps.latitude= follower_latitude;
        followergps.longitude= follower_longitude;

        gps_PubF.publish(followergps);



        //设定一个偏差的的位置
        //        std::cout << 'Please input the desired gap x and y in velocity direction' << std::endl;
        //        std::cin >> desired_gap_x_V >> desired_gap_y_V;
        desired_gap_x_V = 10;
        desired_gap_y_V = -10;



        if (init_flag)
        {
            init_flag = 0;
            continue;
        }



        xy = ll2xy(leader_latitude[0], leader_longitude[0]);                        //leader position in time0
        leader_x[0] = xy[0];
        leader_y[0] = xy[1];

        //cout << "leader_x[0]=" <<leader_x[0]<<endl;
        //cout << "leader_y[0]=" <<leader_y[0]<<endl;


        xy = ll2xy(leader_latitude[1], leader_longitude[1]);                        //leader position in time1
        leader_x[1] = xy[0];
        leader_y[1] = xy[1];

        leaderpose.pose.position.x = leader_x[1];
        leaderpose.pose.position.y = leader_y[1];
        pose_PubD.publish(leaderpose);

        cout << "leader_x[1]=" <<leader_x[1]<<endl;
        cout << "leader_y[1]=" <<leader_y[1]<<endl;

//        leader_Vx = (leader_x[1] - leader_x[0]) / (time[1] - time[0]);          //leader velocity
//        leader_Vy = (leader_y[1] - leader_y[0]) / (time[1] - time[0]);

        double duration =double(time_1.sec - time_0.sec);
        cout << "duration=" << (time_1.sec - time_0.sec) <<endl;
        leader_Vx = (leader_x[1] - leader_x[0]) / (time_1.sec - time_0.sec);          //leader velocity
        leader_Vy = (leader_y[1] - leader_y[0]) / (time_1.sec - time_0.sec);

        cout << "leader_Vx=" <<leader_Vx<<endl;
        cout << "leader_Vy=" <<leader_Vy<<endl;

        double theta_V;
        theta_V = CalcuVelAngel(leader_Vx, leader_Vx);

        xy = ll2xy(follower_latitude, follower_longitude);                      //follower position
        follower_x = xy[0];
        follower_y = xy[1];
        cout << "follower_x=" <<follower_x<<endl;
        cout << "follower_y=" <<follower_y<<endl;

        //##########################

        followerpose.pose.position.x = follower_x;
        followerpose.pose.position.y = follower_y;
        pose_PubF.publish(followerpose);

        //##########################


        double desired_gap_x, desired_gap_y, desired_x, desired_y;
        desired_gap_x = cos(theta_V) * desired_gap_x_V + sin(theta_V) * desired_gap_y_V;
        desired_gap_y = ( -sin(theta_V) ) * desired_gap_x_V + cos(theta_V) * desired_gap_y_V;
        desired_x = leader_x[1] - desired_gap_x;
        desired_y = leader_y[1] - desired_gap_y;


        desiredpose.pose.position.x = desired_x;
        desiredpose.pose.position.y = desired_y;
        pose_PubD.publish(desiredpose);


        //输出dx dy
        outfile <<desired_x-follower_x<<"  ";
        outfile <<desired_y-follower_y<<endl;

        ll = xy2ll(desired_x,desired_y);
        desired_position_latitude = ll[0];
        desired_position_longitude = ll[1];

        cout << "dx=" <<leader_x[1]-follower_x<<endl;
        cout << "dy=" <<leader_y[1]-follower_y<<endl;

        double distance;
        distance = sqrt( (desired_x - follower_x) * (desired_x - follower_x) + (desired_y - follower_y) * (desired_y - follower_y) );

        double leader_V;
        leader_V = sqrt( leader_Vx * (leader_Vx) + (leader_Vy) * (leader_Vy) );
        //--------------------------------
        desired_V = (leader_V) + K * distance;



        //        //################
        //        //赋值给目前的16个航点
        //        for (int i = 0; i < (inter_time*2);i=i+2)
        ////        for (int i = 0; i < (inter_time);i++)
        //        {

        //            double current_latitude, current_longitude;
        //            current_latitude =  follower_latitude+(i + 1) * (desired_position_latitude-follower_latitude)/ inter_time;
        //            current_longitude = follower_longitude + (i + 1)*(desired_position_longitude - follower_longitude)/ inter_time;
        //            waypoint[i].x_lat  = current_latitude;
        //            waypoint[i].y_long = current_longitude;
        //            waypoint[i].z_alt = H;
        //            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        //            waypoint[i].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        //            waypoint[i].autocontinue = true;
        //            if(i == 0){
        //                waypoint[i].is_current = true;
        //            }
        //            else {
        //                waypoint[i].is_current = false;
        //            }


        //            waypoint[i+1].x_lat  = 0;
        //            waypoint[i+1].y_long = 0;
        //            waypoint[i+1].z_alt = 0;
        //            waypoint[i+1].command = mavros_msgs::CommandCode::CMD_DO_CHANGE_SPEED;
        //            waypoint[i+1].frame = 2;
        //            cout<<"desired_V="<<desired_V<<endl;
        //            waypoint[i+1].param1 = 0;
        //            waypoint[i+1].param2 = desired_V;
        //            waypoint[i+1].param3 = -1;
        //            waypoint[i+1].param4 = 0;
        //            waypoint[i+1].autocontinue = true;
        //            waypoint[i+1].is_current = false;
        //        }
//#############只设置速度和目标点位置##########################

//            waypoint[0].x_lat  = 0;
//            waypoint[0].y_long = 0;
//            waypoint[0].z_alt = 0;
//            waypoint[0].command = mavros_msgs::CommandCode::CMD_DO_CHANGE_SPEED;
//            waypoint[0].frame = 2;
//            cout<<"desired_V="<<desired_V<<endl;
//            waypoint[0].param1 = 0;
//            waypoint[0].param2 = desired_V;
//            waypoint[0].param3 = -1;
//            waypoint[0].param4 = 0;
//            waypoint[0].autocontinue = true;
//            waypoint[0].is_current = true;


//            double current_latitude, current_longitude;
//            //current_latitude =  follower_latitude+(i + 1) * (desired_position_latitude-follower_latitude)/ inter_time;
//            //current_longitude = follower_longitude + (i + 1)*(desired_position_longitude - follower_longitude)/ inter_time;
//            waypoint[1].x_lat  = desired_position_latitude;
//            waypoint[1].y_long = desired_position_longitude;
//            waypoint[1].z_alt = H;
//            waypoint[1].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
//            waypoint[1].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
//            waypoint[1].autocontinue = true;
//            waypoint[1].is_current = false;



            double current_latitude, current_longitude;
            //current_latitude =  follower_latitude+(i + 1) * (desired_position_latitude-follower_latitude)/ inter_time;
            //current_longitude = follower_longitude + (i + 1)*(desired_position_longitude - follower_longitude)/ inter_time;
            waypoint[0].x_lat  = desired_position_latitude;
            waypoint[0].y_long = desired_position_longitude;
            waypoint[0].z_alt = H;
            waypoint[0].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[0].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[0].autocontinue = true;
            waypoint[0].is_current = false;


            waypoint[1].x_lat  = 0;
            waypoint[1].y_long = 0;
            waypoint[1].z_alt  = 0;
            waypoint[1].command = mavros_msgs::CommandCode::CMD_DO_CHANGE_SPEED;
            waypoint[1].frame = 2;
            cout<<"desired_V="<<desired_V<<endl;
            waypoint[1].param1 = 0;
            waypoint[1].param2 = desired_V;
            waypoint[1].param3 = -1;
            waypoint[1].param4 = 0;
            waypoint[1].autocontinue = true;
            waypoint[1].is_current = true;


            //            setPoint result(current_latitude, current_longitude,desired_V);
            //            result.show();

        ros::Time last_request = ros::Time::now();
        // while (ros::Time::now() - last_request < ros::Duration(3.0))
        // {
        //     /* code for True */
        // }

        // mavros_msgs::WaypointClear waypoint_clear;
        // if (waypoint_clear_client.call(waypoint_clear)&&waypoint_clear.response.success)
        // {
        //     ROS_INFO("Waypoint clear success");
        // }
        //last_request = ros::Time::now();

        WaypointPush waypoint_push;
        //cout<<"waypoint_list.waypoints.size()"<<waypoint_list.waypoints.size()<<endl;

//

        for(i = 0; i < 2; i=i+1){
//      for(i = 0; i < (inter_time*2); i++){
            //cout<<i<<endl;
            waypoint_push.request.waypoints.push_back(waypoint[i]);
        }

        if (waypoint_push_client.call(waypoint_push)&&waypoint_push.response.success)
        {
            //ROS_INFO("%d", waypoint_push.response.wp_transfered);
            ROS_INFO("Waypoint push success");
        }

        WaypointPull waypoint_pull;

        if (waypoint_pull_client.call(waypoint_pull)&&waypoint_pull.response.success)
        {
            //ROS_INFO("%d", waypoint_pull.response.wp_received);
            ROS_INFO("Waypoint pull success");
        }
        /**
//        while (ros::Time::now() - last_request < ros::Duration(1.0))
//        {
//
//        }


        // WaypointSetCurrent waypoint_setcurrent;
        // waypoint_setcurrent.request.wp_seq = 1;
        // if (waypoint_setcurrent_client.call(waypoint_setcurrent)&&waypoint_setcurrent.response.success)
        // {
        //     ROS_INFO("Waypoint setcurrent success");
        // }
        // while (ros::Time::now() - last_request < ros::Duration(1.0))
        // {
        //
         }
*/

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "AUTO.MISSION";
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
            ROS_INFO("AUTO.MISSION ENABLE!");
        }

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        if( !current_state.armed){
            if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
        }
            //    cout<<offb_set_mode.response.success<<endl<<endl;

        Current_wp.request.wp_seq=0;
        if (waypoint_setcurrent_client.call(Current_wp)&&Current_wp.response.success)
        {
            //ROS_INFO("%d", Current_wp.response.success);
            ROS_INFO("Waypoint set to 0 success");
        }


            /**
//        while (ros::ok())
//        {
//            //std::cout << current_state.mode << std::endl;
//            // if (current_state.mode != "OFFBOARD" &&
//            //     (ros::Time::now() - last_request > ros::Duration(1.0)))
//            // {
//            //     if( set_mode_client.call(offb_set_mode) &&
//            //         offb_set_mode.response.success){
//            //         ROS_INFO("Offboard enabled");
//            //     }
//            //     last_request = ros::Time::now();
//            // } else {
//            //     if( !current_state.armed &&
//            //         (ros::Time::now() - last_request > ros::Duration(1.0))){
//            //         if( arming_client.call(arm_cmd) &&
//            //             arm_cmd.response.success){
//            //             ROS_INFO("Vehicle armed");
//            //         }
//            //         last_request = ros::Time::now();
//            //     }
//            // }

//            // //pose.pose.position.x = r * cos(n/200);
//            // //pose.pose.position.y = r * sin(n/200);
//            // n=n+1;
//            // cout << pose.pose.orientation.w << " " << pose.pose.orientation.x << " " << pose.pose.orientation.y << " " << pose.pose.orientation.z << " ";

//            // local_pos_pub.publish(pose);
//            // ROS_INFO("%f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);


//        }
*/


            //####################
            leader_latitude[0]=leader_latitude[1];
            leader_longitude[0]=leader_longitude[1];
            //###########################



            ros::spinOnce();
            rate.sleep();
    }
    outfile.close();
    return 0;
}



