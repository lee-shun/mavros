#include <cstdlib>
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

int main(int argc, char **argv)
{
    using namespace std;
    int rate = 10;

    ros::init(argc, argv, "mavros_takeoff");
    ros::NodeHandle n;

/*
    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
*/


    ros::Rate r(rate);

    ////////////////////////////////////////////
    /////////////////OFFBOARD//////////////////
    ////////////////////////////////////////////
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;

/*
    srv_setMode.request.custom_mode = "OFFBOARD";
    if(cl.call(srv_setMode)){
        //ROS_INFO("setmode send ok %d value:", srv_setMode.response.success);
        cout<< srv_setMode.response.success) << endl;

    }else{
        ROS_ERROR("Failed SetMode");
        return -1;
    }
*/
    ////////////////////////////////////////////
    ///////////////////ARM//////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_INFO("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }

    ////////////////////////////////////////////
    /////////////////TAKEOFF////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 39.7285473;
    srv_takeoff.request.latitude = 116.1636696;
    srv_takeoff.request.longitude = 30.9433943088;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 90;
    if(takeoff_cl.call(srv_takeoff)){
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
    }
/*
    ////////////////////////////////////////////
    /////////////////SSS REVISED////////////////
    ////////////////////////////////////////////
    sleep(30);

    cout << srv_setMode.request.custom_mode << endl;


    srv_setMode.request.custom_mode = "AUTO.MISSION";
    cl.call(srv_setMode);
    cout << srv_setMode.response << endl;

    //srv_setMode.request.custom_mode = "LOITER";
    //    sleep(10);
    cout << srv_setMode.request.custom_mode << endl;

*/
    ////////////////////////////////////////////
    /////////////////DO STUFF///////////////////
    ////////////////////////////////////////////
    sleep(30);

    ////////////////////////////////////////////
    ///////////////////LAND/////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

    mavros_msgs::CommandTOL srv_land;

    srv_land.request.altitude = 39.7285473;
    srv_land.request.latitude = 116.1636696;
    srv_land.request.longitude = 30.9433943088;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if(land_cl.call(srv_land)){
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }else{
        ROS_ERROR("Failed Land");
    }



    while (n.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}
