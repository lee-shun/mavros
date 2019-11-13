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



int main(int argc, char **argv)
{
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
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    //send a few setpoints before starting
    for(int i = 20; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    // lat 47.397748, long 8.545596, alt 487.9
    mavros_msgs::WaypointList waypoint_list;
    //mavros_msgs::Waypoint waypoint0,waypoint1,waypoint2,waypoint3,waypoint4,waypoint_setvelocity;
    mavros_msgs::Waypoint waypoint[20];

    mavros_msgs::WaypointSetCurrent Current_wp;


    //    Current_wp.re.wp_seq=0;
    //    waypoint_pull_client.call(waypoint_pull)&&waypoint_pull.response.success

    Current_wp.request.wp_seq=0;

    if (waypoint_setcurrent_client.call(Current_wp)&&Current_wp.response.success)
    {
        ROS_INFO("%d", Current_wp.response.success);
        ROS_INFO("Waypoint set to 0 success");
    }

    //WaypointSetCurrent
    //# Some MAV_CMD command codes.
    //# Actual meaning and params you may find in MAVLink documentation
    //# http://mavlink.org/messages/common#ENUM_MAV_CMD

    //# some common MAV_CMD
    //uint16 CMD_DO_SET_MODE = 176
    //uint16 CMD_DO_JUMP = 177
    //uint16 CMD_DO_CHANGE_SPEED = 178
    //uint16 CMD_DO_SET_HOME = 179
    //uint16 CMD_DO_SET_RELAY = 181
    //uint16 CMD_DO_REPEAT_RELAY = 182
    //uint16 CMD_DO_SET_SERVO = 183
    //uint16 CMD_DO_REPEAT_SERVO = 184
    //uint16 CMD_DO_CONTROL_VIDEO = 200
    //uint16 CMD_DO_SET_ROI = 201
    //uint16 CMD_DO_MOUNT_CONTROL = 205
    //uint16 CMD_DO_SET_CAM_TRIGG_DIST = 206
    //uint16 CMD_DO_FENCE_ENABLE = 207
    //uint16 CMD_DO_PARACHUTE = 208
    //uint16 CMD_DO_INVERTED_FLIGHT = 210
    //uint16 CMD_DO_MOUNT_CONTROL_QUAT = 220
    //uint16 CMD_PREFLIGHT_CALIBRATION = 241
    //uint16 CMD_MISSION_START = 300
    //uint16 CMD_COMPONENT_ARM_DISARM = 400
    //uint16 CMD_GET_HOME_POSITION = 410
    //uint16 CMD_START_RX_PAIR = 500
    //uint16 CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520
    //uint16 CMD_DO_TRIGGER_CONTROL = 2003

    //# Waypoint related commands
    //uint16 NAV_WAYPOINT = 16
    //uint16 NAV_LOITER_UNLIM = 17
    //uint16 NAV_LOITER_TURNS = 18
    //uint16 NAV_LOITER_TIME = 19
    //uint16 NAV_RETURN_TO_LAUNCH = 20
    //uint16 NAV_LAND = 21
    //uint16 NAV_TAKEOFF = 22

    //    0 22 39.7173 116.496 50
    //    1 16 39.716 116.496 50
    //    2 16 39.716 116.499 50
    //    3 16 39.7168 116.499 50
    //    4 16 39.7173 116.5 50

    //# see enum MAV_FRAME
    //uint8 frame
    //uint8 FRAME_GLOBAL = 0
    //uint8 FRAME_LOCAL_NED = 1
    //uint8 FRAME_MISSION = 2
    //uint8 FRAME_GLOBAL_REL_ALT = 3
    //uint8 FRAME_LOCAL_ENU = 4

    //参数表
    //改变速度 command=178  param1 = 未知 ：0/1  param2 = 速度 ：15 param3 = 未知 ：-1 param3 = 未知 ：0
    //改变速度的经纬度似乎可有可无
    //起飞 command=22  param1 = pitch : 45.0
    //飞航点 command=16  param1 = HOLD ：单位 秒
    //盘旋 command=17  param3 = 盘旋半径 ：单位 米



    //    waypoint[0].x_lat = 39.717300415;
    //    waypoint[0].y_long = 116.495895386;
    //    waypoint[0].z_alt = 50;

    //投弹组飞机第一阶段计划拟用航点
    //waypoint[0]起飞点 命令22
    //waypoint[1]盘旋点 命令17 x_lat为侦查区中心纬度 y_long为侦查区中心经度 z为各架飞机的飞行高度

    //    waypoint[0].x_lat = 0;
    //    waypoint[0].y_long = 0;
    //    waypoint[0].z_alt = 0;
    //    waypoint[0].command = mavros_msgs::CommandCode::CMD_DO_CHANGE_SPEED;
    //    waypoint[0].frame = mavros_msgs::Waypoint::FRAME_MISSION;
    //    waypoint[0].autocontinue = true;
    //    waypoint[0].is_current = true;
    //    //  waypoint[0].param1 = 1;
    //    waypoint[0].param2 = 15;
    //    waypoint[0].param3 = -1;

    //大部分的参数在没有设置的情况下都是默认为0的，因此一些参数是必须设置的
    //设置经纬高
    waypoint[0].x_lat = 39.717300415;
    waypoint[0].y_long = 116.495895386;
    waypoint[0].z_alt = 30;
    //设置命令类型
    waypoint[0].command = mavros_msgs::CommandCode::NAV_TAKEOFF;
    //设置坐标系，通常选FRAME_GLOBAL_REL_ALT（3）
    waypoint[0].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;

    waypoint[0].autocontinue = true;
    //第一个点的autocontinue必设置为true()默认为false
    waypoint[0].is_current = true;


    waypoint[1].x_lat = 39.716;
    waypoint[1].y_long = 116.496;
    waypoint[1].z_alt = 80;
    waypoint[1].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
    waypoint[1].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    waypoint[1].autocontinue = true;





    //    waypoint[2].param1 = 5;


    /*
    waypoint_setvelocity.x_lat = 47.397748;
    waypoint_setvelocity.y_long = 116.496;
    waypoint_setvelocity.z_alt = 50;
    waypoint_setvelocity.command = mavros_msgs::CommandCode::CMD_DO_CHANGE_SPEED;
    waypoint_setvelocity.frame = mavros_msgs::Waypoint::FRAME_MISSION;
    waypoint_setvelocity.autocontinue = true;
    waypoint_setvelocity.is_current = true;
    waypoint_setvelocity.param1 = 1;
    waypoint_setvelocity.param2 = 2;
    waypoint_setvelocity.param3 = -1;

    waypoint0.x_lat = 39.7173;
    waypoint0.y_long = 8.545596;
    waypoint0.z_alt = 5.0;
    waypoint0.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
    waypoint0.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    waypoint0.autocontinue = true;
    waypoint0.is_current = false;

    waypoint1.x_lat = 47.397948;
    waypoint1.y_long = 8.545596;
    waypoint1.z_alt = 5.0;
    waypoint1.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
    waypoint1.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    waypoint1.autocontinue = true;
    waypoint1.param1 = 5;


    waypoint2.x_lat = 47.397948;
    waypoint2.y_long = 8.545996;
    waypoint2.z_alt = 5.0;
    waypoint2.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
    waypoint2.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    waypoint2.autocontinue = true;
    waypoint2.param1 = 5;

    waypoint3.x_lat = 47.397748;
    waypoint3.y_long = 8.545996;
    waypoint3.z_alt = 5.0;
    waypoint3.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
    waypoint3.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    waypoint3.autocontinue = true;

    waypoint4.x_lat = 0.0;
    waypoint4.y_long = 0.0;
    waypoint4.z_alt = 5.0;
    waypoint4.command = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
    waypoint4.frame = mavros_msgs::Waypoint::FRAME_MISSION;
    waypoint4.autocontinue = true;
*/

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
    //Send new waypoint table.
    // for (size_t i = 0; i < waypoint_list.waypoints.size(); i++)
    // {
    //     /* code for loop body */waypoint_push.request.waypoints.push_back()
    // }

    waypoint_push.request.waypoints.push_back(waypoint[0]);
    waypoint_push.request.waypoints.push_back(waypoint[1]);
    //waypoint_push.request.waypoints.push_back(waypoint[2]);
    //    waypoint_push.request.waypoints.push_back(waypoint[1]);


    //    waypoint_push.request.waypoints.push_back(waypoint_setvelocity);

    //    waypoint_push.request.waypoints.push_back(waypoint0);
    //    waypoint_push.request.waypoints.push_back(waypoint1);
    //    waypoint_push.request.waypoints.push_back(waypoint2);
    //    waypoint_push.request.waypoints.push_back(waypoint3);
    //    waypoint_push.request.waypoints.push_back(waypoint4);

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
    //Request update waypoint list.
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
