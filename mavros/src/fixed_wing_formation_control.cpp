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

float _FIXED_WING_FORMATION_CONTROL::get_air_speed(int type)
{
    if (type == 2)
    {
        return follower_status.air_speed;
    }
    else
    {
        return sqrt(follower_status.global_vel_x * follower_status.global_vel_x   //
                    + follower_status.global_vel_y * follower_status.global_vel_y //
                    + follower_status.global_vel_z * follower_status.global_vel_z //
        );
    }
}

void _FIXED_WING_FORMATION_CONTROL::update_rotmat()
{
    float angle[3], quat[4];

    angle[0] = follower_status.roll_angle;
    angle[1] = follower_status.pitch_angle;
    angle[2] = follower_status.yaw_angle;

    euler_2_quaternion(angle, quat);

    quat_2_rotmax(quat, follower_status.rotmat);
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

    fixed_wing_altitude_from_px4_sub          //订阅高度
        = nh.subscribe<mavros_msgs::Altitude> //
          ("/mavros/altitude", 10, &_FIXED_WING_SUB_PUB::altitude_from_px4_cb, fixed_wing_sub_pub_poiter);

    fixed_wing_air_ground_speed_from_px4_sub //订阅空速、地速
        = nh.subscribe<mavros_msgs::VFR_HUD> //
          ("/mavros/vfr_hud", 10, &_FIXED_WING_SUB_PUB::air_ground_speed_from_px4_cb, fixed_wing_sub_pub_poiter);

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
void _FIXED_WING_FORMATION_CONTROL::trans_the_sp_for_send()
{
    //在这里要进行中间转换，欧美系的期望值存储在follower_setpoint，转换后的存在sp_to_send中
    sp_to_send.roll_angle = follower_setpoint.roll_angle;

    if (-follower_setpoint.yaw_angle + deg_2_rad(90.0) < 0)
        sp_to_send.yaw_angle = -follower_setpoint.yaw_angle + deg_2_rad(90.0);
    else
        sp_to_send.yaw_angle = -follower_setpoint.yaw_angle + deg_2_rad(90.0) + deg_2_rad(360.0);

    sp_to_send.pitch_angle = -follower_setpoint.pitch_angle;

    sp_to_send.thrust = follower_setpoint.thrust;
}
void _FIXED_WING_FORMATION_CONTROL::send_setpoint_to_px4(_FIXED_WING_SUB_PUB *fixed_wing_sub_pub_pointer)
{
    trans_the_sp_for_send(); //将期望值转换一下坐标系

    float angle[3], quat[4];

    angle[0] = sp_to_send.roll_angle;
    angle[1] = sp_to_send.pitch_angle;
    angle[2] = sp_to_send.yaw_angle;

    euler_2_quaternion(angle, quat);

    fixed_wing_sub_pub_pointer->att_sp.type_mask = 7; //1+2+4+64+128 body.rate_x,body.rate_y,body.rate_z thrust..
    fixed_wing_sub_pub_pointer->att_sp.orientation.w = quat[0];
    fixed_wing_sub_pub_pointer->att_sp.orientation.x = quat[1];
    fixed_wing_sub_pub_pointer->att_sp.orientation.y = quat[2];
    fixed_wing_sub_pub_pointer->att_sp.orientation.z = quat[3];
    fixed_wing_sub_pub_pointer->att_sp.thrust = sp_to_send.thrust;

    fixed_wing_local_att_sp_pub.publish(fixed_wing_sub_pub_pointer->att_sp);

    fixed_wing_sub_pub_pointer->att_sp_Euler[0] = angle[0];
    fixed_wing_sub_pub_pointer->att_sp_Euler[1] = angle[1];
    fixed_wing_sub_pub_pointer->att_sp_Euler[2] = angle[2];
    fixed_wing_sub_pub_pointer->thrust_sp = sp_to_send.thrust;

    //show_control_state(fixed_wing_sub_pub_pointer);
}

void _FIXED_WING_FORMATION_CONTROL::send_setpoint_to_ground_station()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::test()
{

    //当前是屏蔽角速度控制的mavros
    follower_setpoint.roll_angle = 0.0;

    follower_setpoint.yaw_angle = 0.0;
    //将期望高度和期望空速赋值
    follower_setpoint.air_speed = 18.5;

    follower_setpoint.altitude = 580.0;
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
    cout << "飞机当前姿态欧美系【roll，pitch，yaw】" << rad_2_deg(p->roll_angle) << " [deg] "
         << rad_2_deg(p->pitch_angle) << " [deg] "
         << rad_2_deg(p->yaw_angle) << " [deg] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "飞机当前姿态的旋转矩阵【第2行】" << p->rotmat[2][0] << " [] "
         << p->rotmat[2][1] << " [] "
         << p->rotmat[2][2] << " [] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "body下的加速度【XYZ】" << p->body_acc_x << " [m/ss] " //待完成
         << p->body_acc_y << " [m/ss] "
         << p->body_acc_z << " [m/ss] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "ned下的位置【XYZ】" << p->ned_pos_x << " [m] " //待完成
         << p->ned_pos_y << " [m] "
         << p->ned_pos_z << " [m] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "高度【local,relative】" << p->ned_altitude << " [m] " //待完成
         << p->relative_hight << " [m] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "ned下的速度【XYZ】" << p->ned_vel_x << " [m/s] " //待完成
         << p->ned_vel_y << " [m/s] "
         << p->ned_vel_z << " [m/s] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "ned下的加速度【XYZ】(由旋转矩阵得来)" << p->ned_acc_x << " [m/ss] " //待完成
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

    cout << "空速以及地速【air,ground】" << p->air_speed << " [m/s] " //待完成
         << p->ground_speed << " [m/s] " << endl;
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

void _FIXED_WING_FORMATION_CONTROL::show_fixed_wing_setpoint(int PlaneID)
{
    _s_follower_setpiont *p;
    if (2 == PlaneID)
    {
        p = &follower_setpoint;
    }
    else if (1 == PlaneID)
    {
        p = &leader_setpoint;
    }
    cout << "***************以下是" << PlaneID << "号飞机期望值******************" << endl;
    cout << "***************以下是" << PlaneID << "号飞机期望值******************" << endl;
    cout << "Time: " << current_time << " [s] " << endl;
    cout << "Mode : [ " << p->mode << " ]" << endl;

    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;
    cout << "飞机期望高度与期望空速【altitude(absolute),airspeed】" << p->altitude << " [m] "
         << p->air_speed << " [m/s] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "飞机姿态期望值【roll，pitch，yaw，thrust】" << p->roll_angle << " [rad] "
         << p->pitch_angle << " [rad] "
         << p->yaw_angle << " [rad] "
         << p->thrust << " [] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "***************以上是" << PlaneID << "号飞机期望值******************" << endl;
    cout << "***************以上是" << PlaneID << "号飞机期望值******************" << endl;
    for (int i = 1; i <= the_space_between_blocks; i++)
        cout << endl;
}

void _FIXED_WING_FORMATION_CONTROL::show_control_state(_FIXED_WING_SUB_PUB *fixed_wing_sub_pub_pointer)
{
    for (int i = 1; i <= the_space_between_blocks; i++)
        cout << endl;
    cout << "***************以下是从机的控制状态***************" << endl;
    cout << "att_sp.type_mask = " << float(fixed_wing_sub_pub_pointer->att_sp.type_mask) << endl;
    cout << "att_sp_Euler[0] = " << fixed_wing_sub_pub_pointer->att_sp_Euler[0] << endl;
    cout << "att_sp_Euler[1] = " << fixed_wing_sub_pub_pointer->att_sp_Euler[1] << endl;
    cout << "att_sp_Euler[2] = " << fixed_wing_sub_pub_pointer->att_sp_Euler[2] << endl;
    cout << "thrust_sp = " << fixed_wing_sub_pub_pointer->thrust_sp << endl;
    cout << "***************以上是从机的控制状态***************" << endl;
}

bool _FIXED_WING_FORMATION_CONTROL::update_leader_status()
{
    //
}
bool _FIXED_WING_FORMATION_CONTROL::update_follwer_status(_FIXED_WING_SUB_PUB *fixed_wing_sub_pub_pointer)
{

    //给结构体赋值；更新飞机状态

    follower_status.mode = fixed_wing_sub_pub_pointer->current_state.mode; //px4->mavros->node坐标没问题

    //以下为GPS信息//px4->mavros->node坐标没问题
    follower_status.altitude = fixed_wing_sub_pub_pointer->global_position_form_px4.altitude;
    follower_status.latitude = fixed_wing_sub_pub_pointer->global_position_form_px4.latitude;
    follower_status.longtitude = fixed_wing_sub_pub_pointer->global_position_form_px4.longitude;

    //GPS速度是在ned下的，
    follower_status.global_vel_x = fixed_wing_sub_pub_pointer->velocity_global_fused_from_px4.twist.linear.y;
    follower_status.global_vel_y = fixed_wing_sub_pub_pointer->velocity_global_fused_from_px4.twist.linear.x;
    follower_status.global_vel_z = -fixed_wing_sub_pub_pointer->velocity_global_fused_from_px4.twist.linear.z;

    follower_status.relative_alt = fixed_wing_sub_pub_pointer->global_rel_alt_from_px4.data;

    //以下为机体系和地面系的夹角，姿态角
    follower_status.roll_angle = fixed_wing_sub_pub_pointer->att_angle_Euler[0];
    follower_status.pitch_angle = -fixed_wing_sub_pub_pointer->att_angle_Euler[1]; //添加负号转换到px4的系

    if (-fixed_wing_sub_pub_pointer->att_angle_Euler[2] + deg_2_rad(90.0) > 0)
        follower_status.yaw_angle = -fixed_wing_sub_pub_pointer->att_angle_Euler[2] + deg_2_rad(90.0); //添加符号使增加方向相同，而且领先于px490°
    else
        follower_status.yaw_angle = -fixed_wing_sub_pub_pointer->att_angle_Euler[2] + deg_2_rad(90.0) + deg_2_rad(360.0);

    //以下为旋转矩阵（ned和body）
    update_rotmat();

    //以下为ned坐标系
    follower_status.ned_pos_x = fixed_wing_sub_pub_pointer->local_position_from_px4.pose.position.y;
    follower_status.ned_pos_y = fixed_wing_sub_pub_pointer->local_position_from_px4.pose.position.x;
    follower_status.ned_pos_z = -fixed_wing_sub_pub_pointer->local_position_from_px4.pose.position.z;

    follower_status.ned_vel_x = fixed_wing_sub_pub_pointer->velocity_ned_fused_from_px4.twist.linear.y;
    follower_status.ned_vel_y = fixed_wing_sub_pub_pointer->velocity_ned_fused_from_px4.twist.linear.x;
    follower_status.ned_vel_z = -fixed_wing_sub_pub_pointer->velocity_ned_fused_from_px4.twist.linear.z;

    //以下为体轴系//体轴系当中的加速度是对的
    follower_status.body_acc_x = fixed_wing_sub_pub_pointer->imu.linear_acceleration.x;
    follower_status.body_acc_y = fixed_wing_sub_pub_pointer->imu.linear_acceleration.y;
    follower_status.body_acc_z = fixed_wing_sub_pub_pointer->imu.linear_acceleration.z;

    follower_status.body_acc[0] = follower_status.body_acc_x;
    follower_status.body_acc[1] = follower_status.body_acc_y;
    follower_status.body_acc[2] = follower_status.body_acc_z;

    //把体轴系下的加速度转到ned下
    matrix_plus_vector_3(follower_status.ned_acc, follower_status.rotmat, follower_status.body_acc);

    follower_status.ned_acc_x = follower_status.ned_acc[0];
    follower_status.ned_acc_y = follower_status.ned_acc[1];
    follower_status.ned_acc_z = follower_status.ned_acc[2];

    //以下来自altitude
    follower_status.relative_hight = fixed_wing_sub_pub_pointer->altitude_from_px4.relative;
    follower_status.ned_altitude = fixed_wing_sub_pub_pointer->altitude_from_px4.local;

    //空速和地速
    follower_status.air_speed = fixed_wing_sub_pub_pointer->air_ground_speed_from_px4.airspeed;
    follower_status.ground_speed = fixed_wing_sub_pub_pointer->air_ground_speed_from_px4.groundspeed;

    //write_to_files("/home/lee/airspeed", current_time, follower_status.air_speed);
    follower_status.wind_estimate_x = fixed_wing_sub_pub_pointer->wind_estimate_from_px4.twist.twist.linear.y;
    follower_status.wind_estimate_y = fixed_wing_sub_pub_pointer->wind_estimate_from_px4.twist.twist.linear.x;
    follower_status.wind_estimate_z = -fixed_wing_sub_pub_pointer->wind_estimate_from_px4.twist.twist.linear.z;

    follower_status.battery_current = fixed_wing_sub_pub_pointer->battrey_state_from_px4.current;
    follower_status.battery_precentage = fixed_wing_sub_pub_pointer->battrey_state_from_px4.percentage;
    follower_status.battery_voltage = fixed_wing_sub_pub_pointer->battrey_state_from_px4.voltage;
}

void _FIXED_WING_FORMATION_CONTROL::foramtion_demands_update(int formation_type)
{
    cout << "当前队形--->" << formation_type << endl;

    switch (formation_type)
    {
    case 1: //一对一，前后为10m，左右为0.1m

        break;

    case 2:

        break;
    };

    follower_setpoint.altitude = leader_status.altitude + formation_params.altitude_offset;

    follower_setpoint.latitude = leader_status.latitude + formation_params.latitude_offset;

    follower_setpoint.longtitude = leader_status.longtitude + formation_params.longtitude_offset;

    calculate_the_distance_error();//计算一下距离error，得到地速期望

    follower_setpoint.ned_vel_x = leader_status.ned_vel_x + formation_params.v_kp * error_follwer1.distance_3d;

    follower_setpoint.ned_vel_y = leader_status.ned_vel_y + formation_params.v_kp * error_follwer1.distance_3d;
}

void _FIXED_WING_FORMATION_CONTROL::calculate_the_distance_error()
{
    float follwer_pos[2];
    float follower_sp_pos[2];

    follwer_pos[0] = follower_status.latitude;
    follwer_pos[1] = follower_status.longtitude;
    follower_sp_pos[0] = follower_setpoint.latitude;
    follower_sp_pos[1] = follower_setpoint.longtitude;

    error_follwer1.altitude = follower_setpoint.altitude - follower_status.altitude;

    error_follwer1.distance_level = cov_lat_long_2_m(follwer_pos, follower_sp_pos);

    error_follwer1.distance_3d = sqrt(error_follwer1.altitude * error_follwer1.altitude +
                                      error_follwer1.distance_level * error_follwer1.distance_level);
}

void _FIXED_WING_FORMATION_CONTROL::calculate_the_desire_airspeed()
{
    //TECS控制的是空速，编队控制是地速，因而用风估计将期望地速转换成期望空速，注意正负号
    follower_setpoint.air_speed = sqrt((follower_setpoint.ned_vel_x - follower_status.wind_estimate_x) * (follower_setpoint.ned_vel_x - follower_status.wind_estimate_x) +
                                       (follower_setpoint.ned_vel_y - follower_status.wind_estimate_y) * (follower_setpoint.ned_vel_y - follower_status.wind_estimate_y));
}

void _FIXED_WING_FORMATION_CONTROL::control_vertical(float current_time)
{
    //不同模式切换的时候需要进行重置tecs
    control_mode_current = follower_status.mode;

    //从领机状态获得飞机的期望高度与期望空速

    if (control_mode_current != control_mode_prev)
    {
        _tecs.reset_state();
    }

    //设置参数,真实的飞机还需要另外调参
    _tecs.set_speed_weight(params.speed_weight);
    _tecs.set_time_const_throt(params.time_const_throt); //这个值影响到总能量-》油门的（相当于Kp，他越大，kp越小）
    _tecs.set_time_const(params.time_const);             //这个值影响到能量分配-》俯仰角他越大，kp越小
    _tecs.enable_airspeed(true);

    if (follower_setpoint.altitude - follower_status.altitude >= 5) //判断一下是否要进入爬升

    {
        params.climboutdem = true;
    }
    else
    {
        params.climboutdem = false;
    }

    calculate_the_desire_airspeed();

    _tecs.update_state(current_time, follower_status.altitude,
                       follower_status.air_speed, follower_status.rotmat,
                       follower_status.body_acc, follower_status.ned_acc, follower_status.altitude_lock, follower_status.in_air);

    _tecs.update_pitch_throttle(current_time, follower_status.rotmat, follower_status.pitch_angle,
                                follower_status.altitude, follower_setpoint.altitude, follower_setpoint.air_speed,
                                follower_status.air_speed, params.EAS2TAS, params.climboutdem,
                                params.climbout_pitch_min_rad, params.throttle_min, params.throttle_max,
                                params.throttle_cruise, params.pitch_min_rad, params.pitch_max_rad);

    _tecs.get_tecs_state(tecs_outputs); //这个是状态，可以作为调试的窗口用

    follower_setpoint.pitch_angle = _tecs.get_pitch_demand(); //添加负号保证,
    follower_setpoint.thrust = _tecs.get_throttle_demand();

    control_mode_prev = control_mode_current;
}

void _FIXED_WING_FORMATION_CONTROL::show_tecs_status()
{
    for (int i = 1; i <= the_space_between_blocks; i++)
        cout << endl;
    cout << "&&&&&&&&&&&&&&&&&&&以下是tecs控制器的状态打印#####################" << endl;

    cout << "mode" << tecs_outputs.mode << endl;
    cout << "airspeed_filtered"
         << "=" << tecs_outputs.airspeed_filtered << endl;
    cout << "airspeed_rate"
         << "=" << tecs_outputs.airspeed_rate << endl;
    cout << "airspeed_sp"
         << "=" << tecs_outputs.airspeed_sp << endl;
    cout << "altitude_filtered"
         << "=" << tecs_outputs.altitude_filtered << endl;
    cout << "altitude_rate"
         << "=" << tecs_outputs.altitude_rate << endl;
    cout << "altitude_rate_sp"
         << "=" << tecs_outputs.altitude_rate_sp << endl;
    cout << "altitude_sp"
         << "=" << tecs_outputs.altitude_sp << endl;
    cout << "pitch_integ"
         << "=" << tecs_outputs.pitch_integ << endl;
    cout << "throttle_integ"
         << "=" << tecs_outputs.throttle_integ << endl;
    cout << "total_energy_error"
         << "=" << tecs_outputs.total_energy_error << endl;
    cout << "total_energy_rate_error"
         << "=" << tecs_outputs.total_energy_rate_error << endl;
    cout << "energy_distribution_error"
         << "=" << tecs_outputs.energy_distribution_error << endl;
    cout << "energy_distribution_error_integ"
         << "=" << tecs_outputs.energy_distribution_error_integ << endl;
    cout << "energy_distribution_rate_error"
         << "=" << tecs_outputs.energy_distribution_rate_error << endl;
    cout << "energy_error_integ"
         << "=" << tecs_outputs.energy_error_integ << endl;

    cout << "&&&&&&&&&&&&&&&&&&&以上是tecs控制器的状态打印#####################" << endl;
    for (int i = 1; i <= the_space_between_blocks; i++)
        cout << endl;
}

void _FIXED_WING_FORMATION_CONTROL::control_lateral(float current_time)
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::run(int argc, char **argv)
{

    ros::Rate rate(60.0);
    begin_time = ros::Time::now(); // 记录启控时间

    ros_sub_and_pub(&fixed_wing_sub_pub);

    while (ros::ok())
    {
        current_time = get_ros_time(begin_time);

        update_follwer_status(&fixed_wing_sub_pub);

        update_leader_status();

        if (follower_status.mode != "OFFBOARD")
        {
            cout << "当前不是OFFBOARD,请用遥控器切换,不进行控制，监视模式" << endl;
            cout << "|             |             |" << endl;
            cout << "|             |             |" << endl;
            cout << "|             |             |" << endl;
            cout << "V             V             V" << endl;
            show_fixed_wing_status(2);
        }

        else
        {
            cout << "当前是OFFBOARD,进行编队控制，控制模式" << endl;
            cout << "|             |             |" << endl;
            cout << "|             |             |" << endl;
            cout << "|             |             |" << endl;
            cout << "V             V             V" << endl;
            show_fixed_wing_status(2);

            //从机的期望值从这里开始被赋值
            //test(); //在这里面将期望高度，期望空速赋值

            foramtion_demands_update(1); //根据队形的需要，计算出编队从机的期望水平位置，即经纬高，以及编队的“地速”

            control_vertical(current_time); //控制高度，空速

            control_lateral(current_time); //控制水平位置（速度方向）

            show_tecs_status();

            show_fixed_wing_setpoint(2); //打印从机期望值

            send_setpoint_to_px4(&fixed_wing_sub_pub);

            control_formation();

            send_setpoint_to_ground_station();
        }

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
