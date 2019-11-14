/* this software is developed by the CGNC_LAB BIT, all rights reserved.*/
/* 作者：CGNC_LAB,BIT
 * 时间：2019.11.09
 */
#include "fixed_wing_formation_control.hpp" //记得修改一下路径

void _FIXED_WING_FORMATION_CONTROL::write_to_files(string file_path_name, float time_stamp, float data)
{
    //打开一个文件，将它的值以及时间戳写进去，文件命名为值的名字

    fstream oufile;

    oufile.open(file_path_name.c_str(), ios::app | ios::out);
    oufile << fixed << setprecision(4) << time_stamp << "\t" << data << endl;

    if (!oufile)
        cout << file_path_name << "-->"
             << "something wrong to open or write" << endl;
    oufile.close();
}

float _FIXED_WING_FORMATION_CONTROL::get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void _FIXED_WING_FORMATION_CONTROL::handle_status_from_receiver()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::handle_message_from_px4()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::send_the_command_to_px4()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::send_message_to_sender()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::ros_sub_and_pub(_FIXED_WING_SUB_PUB *fixed_wing_sub_pub_poiter)
{
    fixed_wing_states_sub                  //
        = nh.subscribe<mavros_msgs::State> //
          ("mavros/state", 10, &_FIXED_WING_SUB_PUB::state_cb, fixed_wing_sub_pub_poiter);

    // 【订阅】无人机imu信息，
    fixed_wing_imu_sub                   //
        = nh.subscribe<sensor_msgs::Imu> //
          ("mavros/imu/data", 10, &_FIXED_WING_SUB_PUB::imu_cb, fixed_wing_sub_pub_poiter);

    // 【订阅】无人机gps位置
    fixed_wing_global_position_form_px4_sub    //
        = nh.subscribe<sensor_msgs::NavSatFix> //
          ("mavros/global_position/global", 10, &_FIXED_WING_SUB_PUB::global_position_form_px4_cb, fixed_wing_sub_pub_poiter);

    //【订阅】无人机gps相对alt
    fixed_wing_global_rel_alt_from_px4_sub //
        = nh.subscribe<std_msgs::Float64>  //
          ("/mavros/global_position/rel_alt", 10, &_FIXED_WING_SUB_PUB::fixed_wing_global_rel_alt_from_px4_cb, fixed_wing_sub_pub_poiter);

    // 【订阅】无人机ump位置
    fixed_wing_umt_position_from_px4_sub   //
        = nh.subscribe<nav_msgs::Odometry> //
          ("mavros/global_position/local", 10, &_FIXED_WING_SUB_PUB::umt_position_from_px4_cb, fixed_wing_sub_pub_poiter);

    // 【订阅】无人机gps三向速度
    fixed_wing_velocity_global_fused_from_px4_sub   //
        = nh.subscribe<geometry_msgs::TwistStamped> //
          ("mavros/global_position/raw/gps_vel", 10, &_FIXED_WING_SUB_PUB::velocity_global_fused_from_px4_cb, fixed_wing_sub_pub_poiter);

    // 【订阅】无人机ned位置
    fixed_wing_local_position_from_px4             //
        = nh.subscribe<geometry_msgs::PoseStamped> //
          ("mavros/local_position/pose", 10, &_FIXED_WING_SUB_PUB::local_position_from_px4_cb, fixed_wing_sub_pub_poiter);

    // 【订阅】无人机ned三向速度
    fixed_wing_velocity_ned_fused_from_px4_sub      //
        = nh.subscribe<geometry_msgs::TwistStamped> //
          ("/mavros/local_position/velocity_local", 10, &_FIXED_WING_SUB_PUB::velocity_ned_fused_from_px4_cb, fixed_wing_sub_pub_poiter);

    // 【订阅】无人机ned三向加速度
    fixed_wing_acc_ned_from_px4_sub                               //
        = nh.subscribe<geometry_msgs::AccelWithCovarianceStamped> //
          ("/mavros/local_position/accel", 10, &_FIXED_WING_SUB_PUB::acc_ned_from_px4_cb, fixed_wing_sub_pub_poiter);

    // 【订阅】无人机ned三向加速度
    fixed_wing_wind_estimate_from_px4_sub                         //
        = nh.subscribe<geometry_msgs::TwistWithCovarianceStamped> //
          ("/mavros/wind_estimation", 10, &_FIXED_WING_SUB_PUB::wind_estimate_from_px4_cb, fixed_wing_sub_pub_poiter);

    //                                  // 【订阅】无人机ned三向加速度
    fixed_wing_battrey_state_from_px4_sub         //
        = nh.subscribe<sensor_msgs::BatteryState> //
          ("/mavros/battery", 10, &_FIXED_WING_SUB_PUB::battrey_state_from_px4_cb, fixed_wing_sub_pub_poiter);

    fixed_wing_waypoints_sub                      //// 【订阅】无人机当前航点
        = nh.subscribe<mavros_msgs::WaypointList> //
          ("mavros/mission/waypoints", 10, &_FIXED_WING_SUB_PUB::waypointlist_from_px4_cb, fixed_wing_sub_pub_poiter);

    fixed_wing_waypointsreach_sub                    //// 【订阅】无人机到达的航点
        = nh.subscribe<mavros_msgs::WaypointReached> //
          ("mavros/mission/reached", 10, &_FIXED_WING_SUB_PUB::waypoints_reached_from_px4_cb, fixed_wing_sub_pub_poiter);

    //##########################################订阅消息###################################################//

    //##########################################发布消息###################################################//

    fixed_wing_local_pos_sp_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

    fixed_wing_global_pos_sp_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 10);

    fixed_wing_local_att_sp_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);

    //##########################################发布消息###################################################//

    //##########################################服务###################################################//
    // 服务 修改系统模式
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    waypoint_setcurrent_client = nh.serviceClient<mavros_msgs::WaypointSetCurrent>("mavros/mission/set_current");

    waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>("mavros/mission/pull");

    waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");

    waypoint_clear_client = nh.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");
    //##########################################服务###################################################//
    //
}
void _FIXED_WING_FORMATION_CONTROL::control_formation()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::send_setpoint_to_px4(_FIXED_WING_SUB_PUB *fixed_wing_sub_pub_pointer)
{

    _FIXED_WING_MATHLIB math;

    float angle[3], quat[4];

    angle[0] = follower_setpoint.roll_angle;
    angle[1] = follower_setpoint.pitch_angle;
    angle[2] = follower_setpoint.yaw_angle;

    math.euler_2_quaternion(angle, quat);

    fixed_wing_sub_pub_pointer->att_sp.type_mask = 7;
    fixed_wing_sub_pub_pointer->att_sp.orientation.w = quat[0];
    fixed_wing_sub_pub_pointer->att_sp.orientation.x = quat[1];
    fixed_wing_sub_pub_pointer->att_sp.orientation.y = quat[2];
    fixed_wing_sub_pub_pointer->att_sp.orientation.z = quat[3];
    fixed_wing_sub_pub_pointer->att_sp.thrust = follower_setpoint.thrust;

    fixed_wing_local_att_sp_pub.publish(fixed_wing_sub_pub_pointer->att_sp);
}

void _FIXED_WING_FORMATION_CONTROL::send_setpoint_to_ground_station()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::test()
{
    follower_setpoint.pitch_angle = -0.2; //还有就是要注意正负号问题

    follower_setpoint.roll_angle = 0.0;

    follower_setpoint.yaw_angle = 0.0;

    follower_setpoint.thrust = 0.8;
}

bool _FIXED_WING_FORMATION_CONTROL::set_fixed_wing_mode(_FIXED_WING_SUB_PUB *fixed_wing_sub_pub_pointer, string setpoint_mode)
{

    if (follower_status.mode != setpoint_mode)
    {
        fixed_wing_sub_pub_pointer->mode_cmd.request.custom_mode = setpoint_mode;

        set_mode_client.call(fixed_wing_sub_pub_pointer->mode_cmd);
    }
}

void _FIXED_WING_FORMATION_CONTROL::show_fixed_wing_status(int PlaneID)
{

    _s_fixed_wing_status *p;
    if (2 == PlaneID)
    {
        p = &follower_status;
    }
    else if (1 == PlaneID)
    {
        p = &leader_status;
    }

    cout << "***************以下是" << PlaneID << "号飞机状态******************" << endl;
    cout << "***************以下是" << PlaneID << "号飞机状态******************" << endl;
    cout << "Time: " << current_time << " [s] " << endl;
    cout << "Mode : [ " << p->mode << " ]" << endl;

    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;
    cout << "飞机当前姿态欧美系【pitch，roll，yaw】" << p->pitch_angle * 57.3 << " [deg] "
         << p->roll_angle * 57.3 << " [deg] "
         << p->yaw_angle * 57.3 << " [deg] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "ned下的位置【XYZ】" << p->ned_pos_x << " [m] " //待完成
         << p->ned_pos_y << " [m] "
         << p->ned_pos_z << " [m] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "ned下的速度【XYZ】" << p->ned_vel_x << " [m/s] " //待完成
         << p->ned_vel_y << " [m/s] "
         << p->ned_vel_z << " [m/s] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "ned下的加速度【XYZ】(旋翼和固定翼都没有)" << p->ned_acc_x << " [m/ss] " //待完成
         << p->ned_acc_y << " [m/ss] "
         << p->ned_acc_z << " [m/ss] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "GPS位置【lat,long,alt,rel_alt】" << p->latitude << " [] " //待完成
         << p->longtitude << " [] "
         << p->altitude << " [] "
         << p->relative_alt << " [] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "GPS融合速度linear【x,y,z】" << p->global_vel_x << " [m/s] " //待完成
         << p->global_vel_y << " [m/s] "
         << p->global_vel_z << " [m/s] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "风估计【x,y,z】" << p->wind_estimate_x << " [m/s] " //待完成
         << p->wind_estimate_y << " [m/s] "
         << p->wind_estimate_z << " [m/s] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "电池状态【电压,比例,电流】" << p->battery_voltage << " [伏特] " //待完成
         << p->battery_precentage << " [%] "
         << p->battery_current << " [安培] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "***************以上是" << PlaneID << "号飞机状态******************" << endl;
    cout << "***************以上是" << PlaneID << "号飞机状态******************" << endl;
    for (int i = 1; i <= the_space_between_blocks; i++)
        cout << endl;
}
bool _FIXED_WING_FORMATION_CONTROL::update_leader_status()
{
    //
}
bool _FIXED_WING_FORMATION_CONTROL::update_follwer_status(_FIXED_WING_SUB_PUB *fixed_wing_sub_pub_pointer)
{

    //给结构体赋值；更新飞机状态

    follower_status.mode = fixed_wing_sub_pub_pointer->current_state.mode;
    //以下为GPS信息
    follower_status.altitude = fixed_wing_sub_pub_pointer->global_position_form_px4.altitude;
    follower_status.latitude = fixed_wing_sub_pub_pointer->global_position_form_px4.latitude;
    follower_status.longtitude = fixed_wing_sub_pub_pointer->global_position_form_px4.longitude;

    follower_status.global_vel_x = fixed_wing_sub_pub_pointer->velocity_global_fused_from_px4.twist.linear.x;
    follower_status.global_vel_y = fixed_wing_sub_pub_pointer->velocity_global_fused_from_px4.twist.linear.y;
    follower_status.global_vel_z = fixed_wing_sub_pub_pointer->velocity_global_fused_from_px4.twist.linear.z;

    follower_status.relative_alt = fixed_wing_sub_pub_pointer->global_rel_alt_from_px4.data;

    //以下为机体系和地面系的夹角，地面系选东向为x，2019.11.11暂时理解
    follower_status.roll_angle = fixed_wing_sub_pub_pointer->att_angle_Euler[0];  //这里拿到的roll是对的，
    follower_status.pitch_angle = fixed_wing_sub_pub_pointer->att_angle_Euler[1]; //以欧美系建立机体坐标系，前向x，向下y，向左z
    follower_status.yaw_angle = fixed_wing_sub_pub_pointer->att_angle_Euler[2];   //东向为零，向北转动为正，

    //以下为ned坐标系
    follower_status.ned_pos_x = fixed_wing_sub_pub_pointer->local_position_from_px4.pose.position.x;
    follower_status.ned_pos_y = fixed_wing_sub_pub_pointer->local_position_from_px4.pose.position.y;
    follower_status.ned_pos_z = fixed_wing_sub_pub_pointer->local_position_from_px4.pose.position.z;

    follower_status.ned_vel_x = fixed_wing_sub_pub_pointer->velocity_ned_fused_from_px4.twist.linear.x;
    follower_status.ned_vel_y = fixed_wing_sub_pub_pointer->velocity_ned_fused_from_px4.twist.linear.y;
    follower_status.ned_vel_z = fixed_wing_sub_pub_pointer->velocity_ned_fused_from_px4.twist.linear.z;

    follower_status.ned_acc_x = fixed_wing_sub_pub_pointer->acc_ned_from_px4.accel.accel.linear.x;
    follower_status.ned_acc_y = fixed_wing_sub_pub_pointer->acc_ned_from_px4.accel.accel.linear.y;
    follower_status.ned_acc_z = fixed_wing_sub_pub_pointer->acc_ned_from_px4.accel.accel.linear.z;

    follower_status.air_speed = 0;
    //write_to_files("/home/lee/airspeed", current_time, follower_status.air_speed);
    follower_status.wind_estimate_x = fixed_wing_sub_pub_pointer->wind_estimate_from_px4.twist.twist.linear.x;
    follower_status.wind_estimate_y = fixed_wing_sub_pub_pointer->wind_estimate_from_px4.twist.twist.linear.y;
    follower_status.wind_estimate_z = fixed_wing_sub_pub_pointer->wind_estimate_from_px4.twist.twist.linear.z;

    follower_status.battery_current = fixed_wing_sub_pub_pointer->battrey_state_from_px4.current;
    follower_status.battery_precentage = fixed_wing_sub_pub_pointer->battrey_state_from_px4.percentage;
    follower_status.battery_voltage = fixed_wing_sub_pub_pointer->battrey_state_from_px4.voltage;
}

void _FIXED_WING_FORMATION_CONTROL::run(int argc, char **argv)
{

    ros::Rate rate(10.0);
    ros::Time begin_time = ros::Time::now(); // 记录启控时间

    _FIXED_WING_SUB_PUB fixed_wing_sub_pub;

    ros_sub_and_pub(&fixed_wing_sub_pub);

    while (ros::ok())
    {
        current_time = get_ros_time(begin_time);

        update_follwer_status(&fixed_wing_sub_pub);
        update_leader_status();

        show_fixed_wing_status(1);
        show_fixed_wing_status(2);

        test(); //这里面对att_sp赋值；
        send_setpoint_to_px4(&fixed_wing_sub_pub);
        follower_setpoint.mode = "OFFBOARD";
        set_fixed_wing_mode(&fixed_wing_sub_pub, follower_setpoint.mode);

        control_formation();

        send_setpoint_to_ground_station();

        ros::spinOnce(); //挂起一段时间，保证周期的速度
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fixed_wing_formation_control");

    _FIXED_WING_FORMATION_CONTROL formation_control;
    if (true)
    {
        formation_control.run(argc, argv);
    }
}
