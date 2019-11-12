/* this software is developed by the CGNC_LAB BIT, all rights reserved.*/
/* 作者：CGNC_LAB,BIT
 * 时间：2019.11.09
 */
#include "fixed_wing_formation_control.hpp" //记得修改一下路径

void write_to_files()
{
    //打开一个文件，将它的值以及时间戳写进去，文件命名为值的名字
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

void _FIXED_WING_FORMATION_CONTROL::ros_sub_and_pub()
{

    //
}
void _FIXED_WING_FORMATION_CONTROL::control_formation()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::send_setpoint_to_px4()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::send_setpoint_to_ground_station()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::test(int argc, char **argv)
{
}

void _FIXED_WING_FORMATION_CONTROL::show_fixed_wing_status(const float current_time, int PlaneID)
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
    cout << "GPS位置【lat,long,alt】" << p->latitude << " [] " //待完成
         << p->longtitude << " [] "
         << p->altitude << " [] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;
    cout << "GPS融合速度linear【x,y,z】" << p->global_vel_x << " [m/s] " //待完成
         << p->global_vel_y << " [m/s] "
         << p->global_vel_z << " [m/s] " << endl;
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

    follower_status.air_speed = 0;
}

void _FIXED_WING_FORMATION_CONTROL::run(int argc, char **argv)
{
    ros::init(argc, argv, "fixed_wing_formation_control");
    ros::NodeHandle nh;
    ros::Rate rate(10.0);
    ros::Time begin_time = ros::Time::now(); // 记录启控时间

    _FIXED_WING_SUB_PUB fixed_wing_sub_pub; //定义订阅发布对象，所有的发布声明，以及回调函数声明（直接定义），回调函数结果在这里面。

    ros::Subscriber // 【订阅】无人机当前模式
        fixed_wing_states_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &_FIXED_WING_SUB_PUB::state_cb, &fixed_wing_sub_pub);
    ros::Subscriber // 【订阅】无人机imu信息，
        fixed_wing_imu_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, &_FIXED_WING_SUB_PUB::imu_cb, &fixed_wing_sub_pub);
    ros::Subscriber // 【订阅】无人机gps位置
        fixed_wing_global_position_form_px4_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, &_FIXED_WING_SUB_PUB::global_position_form_px4_cb, &fixed_wing_sub_pub);
    ros::Subscriber // 【订阅】无人机ump位置
        fixed_wing_umt_position_from_px4_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("mavros/global_position/local", 10, &_FIXED_WING_SUB_PUB::umt_position_from_px4_cb, &fixed_wing_sub_pub);
    ros::Subscriber // 【订阅】无人机gps三向速度
        fixed_wing_velocity_global_fused_from_px4_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/global_position/gp_vel", 10, &_FIXED_WING_SUB_PUB::velocity_global_fused_from_px4_cb, &fixed_wing_sub_pub);
    ros::Subscriber // 【订阅】无人机ned位置
        fixed_wing_local_position_from_px4 = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &_FIXED_WING_SUB_PUB::local_position_from_px4_cb, &fixed_wing_sub_pub);
    ros::Subscriber // 【订阅】无人机ned三向速度
        fixed_wing_velocity_ned_fused_from_px4_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 10, &_FIXED_WING_SUB_PUB::velocity_ned_fused_from_px4_cb, &fixed_wing_sub_pub);

    while (ros::ok())
    {
        float current_time = get_ros_time(begin_time);

        update_follwer_status(&fixed_wing_sub_pub);
        update_leader_status();

        show_fixed_wing_status(current_time, 2);
        show_fixed_wing_status(current_time, 1);

        control_formation();

        send_setpoint_to_px4();

        send_setpoint_to_ground_station();

        ros::spinOnce(); //挂起一段时间，保证周期的速度
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    _FIXED_WING_FORMATION_CONTROL formation_control;
    if (true)
    {
        formation_control.run(argc, argv);
    }
}
