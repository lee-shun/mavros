#include "vir_leader.hpp"

void VIR_LEADER::run(int argc, char **argv)
{
    ros::Rate rate(10.0);
    begin_time = ros::Time::now(); // 记录启控时间
    while (ros::ok())
    {

        cout << "this is in the vir_leader.cpp" << endl;
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