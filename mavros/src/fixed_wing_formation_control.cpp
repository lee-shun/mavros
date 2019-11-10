/* this software is developed by the CGNC_LAB BIT, all rights reserved.*/
/* 作者：CGNC_LAB,BIT
 * 时间：2019.11.09
 */
#include "fixed_wing_formation_control.hpp" //记得修改一下路径


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

void _FIXED_WING_FORMATION_CONTROL::run()
{
    //
}

void _FIXED_WING_FORMATION_CONTROL::test(int argc, char **argv)
{

    ros::init(argc, argv, "fixed_wing_formation_control");
    ros::NodeHandle nh;

    // 频率 [50Hz]
    ros::Rate rate(50.0);

    _FIXED_WING_SUB_PUB fixed_wing_sub_pub;

    // 【订阅】无人机当前状态
    ros::Subscriber fixed_wing_states_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &_FIXED_WING_SUB_PUB::state_cb,&fixed_wing_sub_pub);

    // 【订阅】无人机imu信息
    ros::Subscriber fixed_wing_imu_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data", 10, &_FIXED_WING_SUB_PUB::imu_cb,&fixed_wing_sub_pub);

    // 【订阅】无人机位置期望值[飞控中读取]
    ros::Subscriber fixed_wing_pos_setpoints_from_px4_sub = nh.subscribe<mavros_msgs::PositionTarget>("mavros/setpoint_raw/target_local", 10, &_FIXED_WING_SUB_PUB::pos_target_cb,&fixed_wing_sub_pub);

    // 【订阅】无人机姿态期望值[飞控中读取]
    ros::Subscriber fixed_wing_att_setpoints_from_px4_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/target_attitude", 10, &_FIXED_WING_SUB_PUB::att_target_cb,&fixed_wing_sub_pub);

    // 服务 修改系统模式
    ros::ServiceClient fixed_wing_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode mode_cmd;

    // 【发布】位置控制期望值 对应的mavlink消息类型是 SET_POSITION_TARGET_LOCAL_NED ( #84 )
    ros::Publisher fixed_wing_pos_setpoints_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

    // 记录启控时间
    ros::Time begin_time = ros::Time::now();

    while (ros::ok())
    {
        // 当前时间
        float current_time = get_ros_time(begin_time);

        //如果当前模式不是OFFBOARD模式, 则切换为OFFBOARD模式

        // if (fixed_wing_sub_pub.current_state.mode != "OFFBOARD")
        // {
        //     mode_cmd.request.custom_mode = "OFFBOARD";
        //     set_mode_client.call(mode_cmd);
        // }
        //对即将发给飞控的指令进行赋值
        fixed_wing_sub_pub.pos_setpoint.type_mask = (2 << 10) | (7 << 6) | (7 << 3); //100 111 111 000  位置 + yaw
        //fixed_wing_sub_pub.pos_setpoint.type_mask = (1 << 10) | (7 << 6) | (7 << 3);  //010 111 111 000  位置 + yaw_rate
        //fixed_wing_sub_pub.pos_setpoint.type_mask = (2 << 10) | (7 << 3) | (7 << 0);  // 100 000 111 111   加速度 + yaw
        fixed_wing_sub_pub.pos_setpoint.position.x = 10;
        fixed_wing_sub_pub.pos_setpoint.position.y = 10;
        fixed_wing_sub_pub.pos_setpoint.position.z = 0;
        fixed_wing_sub_pub.pos_setpoint.velocity.x = 0;
        fixed_wing_sub_pub.pos_setpoint.velocity.y = 0;
        fixed_wing_sub_pub.pos_setpoint.velocity.z = 0;
        fixed_wing_sub_pub.pos_setpoint.acceleration_or_force.x = 0;
        fixed_wing_sub_pub.pos_setpoint.acceleration_or_force.y = 0;
        fixed_wing_sub_pub.pos_setpoint.acceleration_or_force.z = 0;
        fixed_wing_sub_pub.pos_setpoint.yaw = 0 * 3.1415 / 180.0;      //注意 : 单位为弧度
        fixed_wing_sub_pub.pos_setpoint.yaw_rate = 0 * 3.1415 / 180.0; //注意 : 单位为弧度

        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>SEND State<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
        cout << "fixed_wing_sub_pub.pos_setpoint上位机设定值: [X Y Z] : "
             << " " << fixed_wing_sub_pub.pos_setpoint.position.x << " [m] " << fixed_wing_sub_pub.pos_setpoint.position.y << " [m] " << fixed_wing_sub_pub.pos_setpoint.position.z << " [m] " << endl;

        cout << "fixed_wing_sub_pub.pos_setpoint: [yaw] : "
             << " " << fixed_wing_sub_pub.pos_setpoint.yaw * 180 / 3.1415 << " [°] " << endl;

        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Drone State<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;

        cout << "Time: " << current_time << " [s] " << endl;

        cout << "Mode : [ " << fixed_wing_sub_pub.current_state.mode << " ]" << endl;

        cout << "pos_target从飞控中直接读到的: [X Y Z] : "
             << " " << fixed_wing_sub_pub.pos_target.position.x << " [m] " << fixed_wing_sub_pub.pos_target.position.y << " [m] " << fixed_wing_sub_pub.pos_target.position.z << " [m] " << endl;

        cout << "Attitude: [roll pitch yaw] "
             << " " << fixed_wing_sub_pub.PIX_Euler[0] * 180 / 3.1415 << " ° " << fixed_wing_sub_pub.PIX_Euler[1] * 180 / 3.1415 << " ° " << fixed_wing_sub_pub.PIX_Euler[2] * 180 / 3.1415 << " ° " << endl;

        cout << "Attitude_target: [roll pitch yaw] : " << fixed_wing_sub_pub.PIX_Euler_target[0] * 180 / 3.1415 << " [°] " << fixed_wing_sub_pub.PIX_Euler_target[1] * 180 / 3.1415 << " [°] " << fixed_wing_sub_pub.PIX_Euler_target[2] * 180 / 3.1415 << " [°] " << endl;

        cout << "Thrust_target[0 - 1] : " << fixed_wing_sub_pub.Thrust_target << endl;

        //发布
        fixed_wing_pos_setpoints_pub.publish(fixed_wing_sub_pub.pos_setpoint);
        //回调
        ros::spinOnce();
        //挂起一段时间(rate为 50HZ)
        rate.sleep();
    }

    //
}

int main(int argc, char **argv)
{
    _FIXED_WING_FORMATION_CONTROL formation_control;

    if (true)
    {

        formation_control.test( argc,  argv);
    }
}
