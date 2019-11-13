#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <mavros/Key.h>

using namespace std;
int main(int argc, char **argv)
{

  ros::init(argc, argv, "IamPublisher");

  ros::NodeHandle nh;
  ros::Publisher commandPublisher = nh.advertise<mavros::Key>("/keyboard/keydown", 1);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    mavros::Key key;

    //cin.get(key.code);
    cin>>key.code;
    cout<<"key== "<<key<<endl<<endl;

    //ROS_INFO("key== %s", key.code);


    commandPublisher.publish(key);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
