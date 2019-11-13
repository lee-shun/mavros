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
    ///


    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
//  创建一个调用服务的客户端
//  n.serviceClient    是ros 中创建客户端的正常操作
//  mavros_msgs是包的名称 SetMode的是服务的名称
//  /mavros/set_mode是话题名

//    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

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
//    cmd/arming 是服务的名称
//    ~cmd/arming (mavros_msgs/CommandBool)

//    Change Arming status.





    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_INFO("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }
//    这段代码是在调用service。由于service的调用是模态过程（调用的时候占用进程阻止其他代码的执行），
//    一旦调用完成，将返回调用结果。如果service调用成功，
//    call()函数将返回true，srv.response里面的值将是合法的值。
//    如果调用失败，call()函数将返回false，srv.response里面的值将是非法的。

/*
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
*/
    ////////////////////////////////////////////
    /////////////////SSS REVISED////////////////
    ////////////////////////////////////////////
//    sleep(30);

    cout << srv_setMode.request.custom_mode << endl;


    srv_setMode.request.custom_mode = "AUTO.MISSION";
    cl.call(srv_setMode);
    cout << srv_setMode.response << endl;

    //srv_setMode.request.custom_mode = "LOITER";
    //    sleep(10);
    cout << srv_setMode.request.custom_mode << endl;


    ////////////////////////////////////////////
    /////////////////DO STUFF///////////////////
    ////////////////////////////////////////////
//    sleep(30);
/*
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


    */
    while (n.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}
