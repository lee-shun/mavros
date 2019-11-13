/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>
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


#include <iostream>
using namespace std;
using namespace mavros_msgs;


//一些常值，暂定为全局常值变量
const int NUAV = 6;   //给出天上盘旋的无人机数量
const int Npoint = 6; //给出目标点数量
const float g = 9.8;  //重力加速度
bool mission_received = 0;
int r = 30; //盘旋半径
//由四个角点计算中心点的程序
//输入四个角点的值
double w1= 39.9897585,  w2=39.9897833,  w3=39.9882652,  w4= 39.9882500;
float j1= 116.3526900, j2=116.3541295, j3=116.3542219, j4=116.3527874;
//float wc,jc;
float wc = ( w1+w2+w3+w4 ) / 4;
float jc ;
jc = ( j1+j2+j3+j4 ) / 4;



//===========================用于计算切点的子函数===================

float *point_tangency(float x[2])
{
    //===========================圆心坐标为侦查区的中心===================
    float x0 = wc;
    float y0 = jc;

    //==========================================================================

    float r0 = r;
    float y[2]={9,0};
    float x_1,y_1,x_2,y_2;
    float k1,k2,*res=y;

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
    float w[9] = {0,-1,y0,1,0,-x0,-y0,x0,0};
    float r1[3] = {x_1 - x0, y_1-y0, 0};
    float r2[3] = {x_2 - x0, y_2-y0, 0};
    float v1[3],v2[3];
    float s[3] = {x[0] - x0, x[1] - y0, 0};
    float s1=0,s2=0;
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
    }
    if (s2 > 0)
    {
        y[0] = x_2;
        y[1] = y_2;
    }






    return res;
}


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::Twist att_tw;
void get_throttle(const geometry_msgs::Twist::ConstPtr& msg)
{
    att_tw = *msg;
    ROS_INFO("%f %f %f %f", att_tw.angular.x, att_tw.angular.y, att_tw.angular.z,att_tw.linear.z);
}

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
    cout<<"count:"<<points.waypoints.size()<<endl;
    for (size_t i = 0; i < points.waypoints.size(); i++)
    {
        cout<<i<<" "<<points.waypoints[i].command<<" "<<points.waypoints[i].x_lat<<" "<<points.waypoints[i].y_long<<" "<<points.waypoints[i].z_alt<<endl;
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


    float hmin = 30;      //飞机最小盘旋高度
    float h = 5;          //飞机高度间隔
    int i=1;              //飞机的序号
    float v=15            //飞机的飞行速度(单位:m/s)

            ros::init(argc, argv, "waypoints_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Subscriber waypoints_sub = nh.subscribe<mavros_msgs::WaypointList>("mavros/mission/waypoints", 10, get_waypoints);
    
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
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        cout<<"THE CONNECTION STATE OF FCU IS "<< current_state.connected <<endl;
        ros::spinOnce();
        rate.sleep();
    }



    //不知道是做什么用的
    geometry_msgs::PoseStamped pose;
    //send a few setpoints before starting
    for(int i = 20; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }


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
    waypoint[1].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
    waypoint[1].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    waypoint[1].autocontinue = true;
    waypoint[1].is_current = false;


    cin>>mission_received;
    float goal[2] = {0, 0};
    float bbb[2] = {0, 0};


    //接受到消息之后,开始规划航迹从 waypoint[2]开始之后

    while (ros::ok() && mission_received ) {

        ///目标点的坐标接收
        goal[1]= 39.9895180;
        goal[2]= 116.3529902;
        cout << "mission received!!!" <<endl;


        ///计算飞机飞行高度对应平抛距离
        paolenth = v * sqrt(2 * H / g);

        ///计算飞机可以直接平抛的大圆半径
        R = sqrt( r*r + paolenth*paolenth);

        //        goal[0] = Prank[i][0];
        //        goal[1] = Prank[i][1];


        //        pointlenth为goal到盘旋圆心的距离

        if (pointlenth >= R)     //目标点在圆外且切线长足够平抛
        {   cout << "pointlenth >= R!!!" <<endl;

            tan = point_tangency(goal);
            waypoint[2].x_lat  = tan[0];
            waypoint[2].y_long = tan[1];
            waypoint[2].z_alt = H;
            waypoint[2].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
            waypoint[2].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[2].autocontinue = true;
            waypoint[2].is_current = false;


            waypoint[3].x_lat  = goal[0]-(goal[0]-tan[0])/sqrt((goal[0]-tan[0])*(goal[0]-tan[0])+(goal[1]-tan[1])*(goal[1]-tan[1]))*paolenth;
            waypoint[3].y_long = goal[1]-(goal[1]-tan[1])/sqrt((goal[0]-tan[0])*(goal[0]-tan[0])+(goal[1]-tan[1])*(goal[1]-tan[1]))*paolenth;
            waypoint[3].z_alt = H;
            waypoint[3].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
            waypoint[3].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[3].autocontinue = true;
            waypoint[3].is_current = false;

            //            waypoint[3].x_lat  = goal[0]-(goal[0]-tan[0])/sqrt((goal[0]-tan[0])*(goal[0]-tan[0])+(goal[1]-tan[1])*(goal[1]-tan[1]))*paolenth;
            //            waypoint[3].y_long = goal[1]-(goal[1]-tan[1])/sqrt((goal[0]-tan[0])*(goal[0]-tan[0])+(goal[1]-tan[1])*(goal[1]-tan[1]))*paolenth;
            //            waypoint[3].z_alt = H;
            //            waypoint[3].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
            //            waypoint[3].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            //            waypoint[3].autocontinue = true;
            //            waypoint[3].is_current = false;

            //            waypoint[3].x_lat  = goal[0]-(goal[0]-tan[0])/sqrt((goal[0]-tan[0])*(goal[0]-tan[0])+(goal[1]-tan[1])*(goal[1]-tan[1]))*paolenth;
            //            waypoint[3].y_long = goal[1]-(goal[1]-tan[1])/sqrt((goal[0]-tan[0])*(goal[0]-tan[0])+(goal[1]-tan[1])*(goal[1]-tan[1]))*paolenth;
            //            waypoint[3].z_alt = H;
            //            waypoint[3].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
            //            waypoint[3].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            //            waypoint[3].autocontinue = true;
            //            waypoint[3].is_current = false;

            //            waypoint[3].x_lat  = goal[0]-(goal[0]-tan[0])/sqrt((goal[0]-tan[0])*(goal[0]-tan[0])+(goal[1]-tan[1])*(goal[1]-tan[1]))*paolenth;
            //            waypoint[3].y_long = goal[1]-(goal[1]-tan[1])/sqrt((goal[0]-tan[0])*(goal[0]-tan[0])+(goal[1]-tan[1])*(goal[1]-tan[1]))*paolenth;
            //            waypoint[3].z_alt = H;
            //            waypoint[3].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
            //            waypoint[3].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            //            waypoint[3].autocontinue = true;
            //            waypoint[3].is_current = false;





        }

        if (pointlenth < R && pointlenth >= r)     //目标点在圆外然而切线长不够平抛
        {
            cout << "pointlenth < R 且 pointlenth >= r!!!" <<endl;

            tan = point_tangency(goal);
            tanlenth = sqrt((goal[0]-tan[0])*(goal[0]-tan[0])+(goal[1]-tan[1])*(goal[1]-tan[1]));

            //            point1[0] = -1*tan[0];
            //            point1[1] = -1*tan[1];
            //            aaa = (paolenth - tanlenth)/tanlenth;
            //            point2[0] = point1[0] + aaa*(tan[0] - goal[0]);
            //            point2[1] = point1[1] + aaa*(tan[1] - goal[1]);
            //            point3[0] = tan[0] + aaa*(tan[0] - goal[0]);
            //            point3[1] = tan[1] + aaa*(tan[1] - goal[1]);


            waypoint[2].x_lat  = -1*tan[0];
            waypoint[2].y_long = -1*tan[1];
            waypoint[2].z_alt = H;
            waypoint[2].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
            waypoint[2].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[2].autocontinue = true;
            waypoint[2].is_current = false;

            aaa = (paolenth - tanlenth)/tanlenth; //什么意思???

            waypoint[3].x_lat  = point1[0] + aaa*(tan[0] - goal[0]);
            waypoint[3].y_long = point1[1] + aaa*(tan[1] - goal[1]);
            waypoint[3].z_alt = H;
            waypoint[3].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
            waypoint[3].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[3].autocontinue = true;
            waypoint[3].is_current = false;




            waypoint[4].x_lat  = tan[0] + aaa*(tan[0] - goal[0]);
            waypoint[4].y_long = tan[1] + aaa*(tan[1] - goal[1]);
            waypoint[4].z_alt = H;
            waypoint[4].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
            waypoint[4].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[4].autocontinue = true;
            waypoint[4].is_current = false;










        }

        if (pointlenth < r)     //目标点在圆外且切线长足够平抛
        {
            cout << "pointlenth < r!!!" <<endl;

            if (pointlenth < 0.0000001)
            {
                goal[0] = goal[0] + 0.0000001;
                goal[1] = goal[1] + 0.0000001;
            }

            //            point1[0] = -goal[0]/pointlenth*r;
            //            point1[1] = -goal[1]/pointlenth*r;

            //            bbb[0] = -1*point1[1];
            //            bbb[1] = point1[0];

            //            point2[0] = point1[0] + bbb[0];
            //            point2[1] = point1[1] + bbb[1];

            //            point3[0] = 2*bbb[0];
            //            point3[1] = 2*bbb[1];

            //            point5[0] = goal[0]/pointlenth*(pointlenth+paolenth);
            //            point5[1] = goal[1]/pointlenth*(pointlenth+paolenth);

            //            point4[0] = point3[0] + point5[0];
            //            point4[1] = point3[1] + point5[1];

            waypoint[2].x_lat  = -goal[0]/pointlenth*r;
            waypoint[2].y_long = -goal[1]/pointlenth*r;
            waypoint[2].z_alt = H;
            waypoint[2].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
            waypoint[2].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[2].autocontinue = true;
            waypoint[2].is_current = false;

            bbb[0] = -1*point1[1];
            bbb[1] = point1[0];

            waypoint[3].x_lat  = point1[0] + bbb[0];
            waypoint[3].y_long = point1[1] + bbb[1];
            waypoint[3].z_alt = H;
            waypoint[3].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
            waypoint[3].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[3].autocontinue = true;
            waypoint[3].is_current = false;

            waypoint[4].x_lat  = 2*bbb[0];
            waypoint[4].y_long = 2*bbb[1];
            waypoint[4].z_alt = H;
            waypoint[4].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
            waypoint[4].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[4].autocontinue = true;
            waypoint[4].is_current = false;

            waypoint[6].x_lat  = goal[0]/pointlenth*(pointlenth+paolenth);
            waypoint[6].y_long = goal[0]/pointlenth*(pointlenth+paolenth);
            waypoint[6].z_alt = H;
            waypoint[6].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
            waypoint[6].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[6].autocontinue = true;
            waypoint[6].is_current = false;

            waypoint[5].x_lat  = waypoint[4].x_lat  + waypoint[6].x_lat;
            waypoint[5].y_long = waypoint[4].y_long + waypoint[6].y_long;
            waypoint[5].z_alt = H;
            waypoint[5].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
            waypoint[5].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
            waypoint[5].autocontinue = true;
            waypoint[5].is_current = false;
        }
    }


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
    last_request = ros::Time::now();

    WaypointPush waypoint_push;
    // for (size_t i = 0; i < waypoint_list.waypoints.size(); i++)
    // {
    //     /* code for loop body */waypoint_push.request.waypoints.push_back()
    // }

    for(i=1:i<7:i++){
        waypoint_push.request.waypoints.push_back(waypoint[i]);

    }

    if (waypoint_push_client.call(waypoint_push)&&waypoint_push.response.success)
    {
        ROS_INFO("%d", waypoint_push.response.wp_transfered);
        ROS_INFO("Waypoint push success");
    }

    while (ros::Time::now() - last_request < ros::Duration(1.0))
    {
        /* code for True */
    }


    WaypointPull waypoint_pull;

    if (waypoint_pull_client.call(waypoint_pull)&&waypoint_pull.response.success)
    {
        ROS_INFO("%d", waypoint_pull.response.wp_received);
        ROS_INFO("Waypoint pull success");
    }

    while (ros::Time::now() - last_request < ros::Duration(1.0))
    {
        /* code for True */
    }


    // WaypointSetCurrent waypoint_setcurrent;
    // waypoint_setcurrent.request.wp_seq = 1;
    // if (waypoint_setcurrent_client.call(waypoint_setcurrent)&&waypoint_setcurrent.response.success)
    // {
    //     ROS_INFO("Waypoint setcurrent success");
    // }
    // while (ros::Time::now() - last_request < ros::Duration(1.0))
    // {
    //     /* code for True */
    // }
    



    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    

    while (ros::ok())
    {
        //std::cout << current_state.mode << std::endl;
        // if (current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(1.0)))
        // {
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.success){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(1.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }

        // //pose.pose.position.x = r * cos(n/200);
        // //pose.pose.position.y = r * sin(n/200);
        // n=n+1;
        // cout << pose.pose.orientation.w << " " << pose.pose.orientation.x << " " << pose.pose.orientation.y << " " << pose.pose.orientation.z << " ";

        // local_pos_pub.publish(pose);
        // ROS_INFO("%f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
