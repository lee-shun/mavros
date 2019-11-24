#include "vir_leader.hpp"

void VIR_LEADER::ros_sub_pub()
{
    vir_leader_pub = nh.advertise<mavros_msgs::Formation_fixed_wing>("/mavros/fixed_wing_formation/status", 10);
}

void VIR_LEADER::run(int argc, char **argv)
{
    ros::Rate rate(10.0);
    begin_time = ros::Time::now(); // 记录启控时间
    ros_sub_pub();
    while (ros::ok())
    {

        cout << "this is in the vir_leader.cpp" << endl;
        
        fixed_wing_sub_pub.fixed_wing_states_tran.air_speed = 18.5;

        vir_leader_pub.publish(fixed_wing_sub_pub.fixed_wing_states_tran);
        
        ros::spinOnce(); //挂起一段时间，保证周期的速度

        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vir_leader");

    VIR_LEADER _vir_leader;

    if (true)
    {
        _vir_leader.run(argc, argv);
    }

    return 0;
}