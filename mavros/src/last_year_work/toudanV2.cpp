/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

/************************************************

注意事项:
1 注意经纬度转化
2 注意在push的时候一定要注意尺寸!!
3 不能对地址进行cout,否则会出现0等错误




************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>
#include<iomanip>
#include <iostream>

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



using namespace std;
using namespace mavros_msgs;


///注意!!!!!
/// 最后所有的经纬转米的公式里面的参数一定要改!!!

//一些常值，暂定为全局常值变量
const int NUAV = 6;   //给出天上盘旋的无人机数量
const int Npoint = 6; //给出目标点数量+
const double g = 9.80665;  //重力加速度
const float pi = 3.141593;
bool mission_received = 0;
int r = 30; //盘旋半径
//投弹区中心点的经纬度
double wc;
double jc;
//jing0,wei0 的含义待定
double jing0,wei0;
double xc,yc,goal_x,goal_y;

//waypoint list的大小
int size;


//计算原理
//2*pi*R(地球半径)/360
//经度的话要乘以一个cos纬度

//将ENU(xyz)转换为经纬度(LLA)
//使用方式
//add=ll2xy(x[0],x[1]);
//x[0]=add[0];
//x[1]=add[1];

double *xy2ll(double x, double y)
{
    double LL[2];
    double *ret= LL ;

    //纬度
    LL[0] = wei0 + y / 111177.0;                                                                       ///?????????????
    //经度
    LL[1] = jing0 + x / 85155.0;

    //    cout<<fixed<< setprecision(10)<<"y[0] ="<< y[0] <<endl;
    //    cout<<fixed<< setprecision(10)<<"y[1] ="<< y[1]<<endl;

    return ret;

}

//将经纬度(LLA)转换为ENU(xyz)
//使用方式
//add=xy2ll(y[0],y[1]);
//y[0] = add[0];
//y[1] = add[1];
double *ll2xy(double w, double j)
{
    double XY[2];
    double *ret= XY;
    //X
    XY[0] = (j - jing0) * 85155.0;
    //Y
    XY[1] = (w - wei0) * 111177.0;

    //    cout<<fixed<< setprecision(10)<<"y[0] ="<< y[0] <<endl;
    //    cout<<fixed<< setprecision(10)<<"y[1] ="<< y[1]<<endl;

    return ret;

}
WaypointPush waypoint_push;



//===========================用于计算切点的子函数===================
//输入的数据为纬经
double *point_tangency(double g[2])
{
    //传入目标点
    //===========================圆心坐标为侦查区的中心===================
    double x0 = xc;
    double y0 = yc;

    //==========================================================================

    double r0 = r;
    double y[2];
    double x_1,y_1,x_2,y_2;
    double k1,k2,*res= y ;
    double x[2];
    double *add;
    //将用纬度和经度表示的转化为ENU系
    //    cout<<"x[0]="<< x[0] <<endl;
    //    cout<<"wei0="<< wei0 <<endl;

    //纬经高转化为ENU(xyz)
    //    x[1]= (g[0] - wei0) * 111177.0;
    //    x[0]= (g[1] - jing0) * 85155.0;
    //    cout<<"纬经高转化为ENU(xyz)1"<<endl;
    //    cout<<fixed<< setprecision(10)<<"x[0] ="<< x[0] <<endl;
    //    cout<<fixed<< setprecision(10)<<"x[1] ="<< x[1] <<endl;

    //此处代码已经测试成功!!
    add=ll2xy(g[0],g[1]);
    x[0]=add[0];
    x[1]=add[1];

    //    cout<<"纬经高转化为ENU(xyz)2"<<endl;
    //    cout<<fixed<< setprecision(10)<<"x[0] ="<< x[0] <<endl;
    //    cout<<fixed<< setprecision(10)<<"x[1] ="<< x[1] <<endl;


    //找出两个切点（x_1,y_1）(x_2,y_2)                                             ///此处采取的圆的半径为r0。值为r，盘旋半径
    k1 = (y0*x0 + x[1]*x[0] - y0*x[0] - x[1]*x0 + sqrt(r0*r0*(-2*y0*x[1] - 2*x0*x[0] + x[1]*x[1] +
            y0*y0 + x0*x0 - r0*r0 + x[0]*x[0]))) / (-r0*r0 + x0*x0 - 2*x0*x[1] + x[0]*x[0]);
    k2= (y0*x0 + x[1]*x[0] - y0*x[0] - x[1]*x0 - sqrt(r0*r0*(-2*y0*x[1] - 2*x0*x[0] + x[1]*x[1] +
            y0*y0 + x0*x0 - r0*r0 + x[0]*x[0])))/(-r0*r0 + x0*x0 - 2*x0*x[0] + x[0]*x[0]);
    x_1 = (-k1*x[1] + x0 + k1*k1*x[0] + y0*k1) / (1 + k1*k1);
    y_1 =-(-x[1] - k1*x0 - y0*k1*k1 + k1*x[0]) / (1 + k1*k1);
    x_2 = (-k2*x[1] + x0 + k2*k2*x[0] + y0*k2) / (1 + k2*k2);
    y_2 =-(-x[1] - k2*x0 - y0*k2*k2 + k2*x[0]) / (1 + k2*k2);

    //%%%%判断逆时针是先到哪个切点%%%%%%%%
    double w[9] = {0,-1,y0,1,0,-x0,-y0,x0,0};
    double r1[3] = {x_1 - x0, y_1-y0, 0};
    double r2[3] = {x_2 - x0, y_2-y0, 0};
    double v1[3],v2[3];
    double s[3] = {x[0] - x0, x[1] - y0, 0};
    double s1=0,s2=0;
    for (int i = 0;i<3;i++)
    {
        v1[i] = w[i*3+0]*r1[0] + w[i*3+1]*r1[1] + w[i*3+2]*r1[2];
        v2[i] = w[i*3+0]*r2[0] + w[i*3+1]*r2[1] + w[i*3+2]*r2[2];
    }
    for (int i = 0;i < 3;i++)
    {
        s1 = s1 + s[i]*v1[i];
        s2 = s2 + s[i]*v2[i];
    }
    if (s1 > 0)
    {
        y[0] = x_1;
        y[1] = y_1;
        cout<<fixed<< setprecision(10)<<"y[0] ="<< x_1 <<endl;
        cout<<fixed<< setprecision(10)<<"y[1] ="<< y_1 <<endl;

    }
    if (s2 > 0)
    {
        y[0] = x_2;
        y[1] = y_2;
        cout<<fixed<< setprecision(10)<<"y[0] ="<< x_2 <<endl;
        cout<<fixed<< setprecision(10)<<"y[1] ="<< y_2 <<endl;

    }
    cout<<fixed<< setprecision(10)<<"res ="<< res <<endl;

    //ENU(xyz)转化为纬经高
    add=xy2ll(y[0],y[1]);
    y[0] = add[0];
    y[1] = add[1];

    //    y[0] = wei0 + y[1] / 111177.0;
    //    y[1] = jing0 + y[0] / 85155.0;
    //    cout<<"求切点的函数!"<<endl;
    //    cout<<fixed<< setprecision(10)<<"y[0] ="<< y[0] <<endl;
    //    cout<<fixed<< setprecision(10)<<"y[1] ="<< y[1] <<endl;

    return res;
}


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//geometry_msgs::Twist att_tw;
//void get_throttle(const geometry_msgs::Twist::ConstPtr& msg)
//{
//    att_tw = *msg;
//    ROS_INFO("%f %f %f %f", att_tw.angular.x, att_tw.angular.y, att_tw.angular.z,att_tw.linear.z);
//}

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

//用于打印航迹点的子函数
void printwaypoint(const mavros_msgs::WaypointList points)
{
    cout<<"count:"<<points.waypoints.size()<<endl;
    for (size_t i = 0; i < points.waypoints.size(); i++)
    {

        cout<<i<<" "<<points.waypoints[i].command<<" ";
        cout<<fixed<< setprecision(10)<<points.waypoints[i].x_lat<<" ";
        cout<<fixed<< setprecision(10)<<points.waypoints[i].y_long<<" ";
        cout<<fixed<< setprecision(10)<<points.waypoints[i].z_alt<<endl;



    }

}




mavros_msgs::WaypointList current_waypoints;
void get_waypoints(const mavros_msgs::WaypointList::ConstPtr& msg)
{
    current_waypoints = *msg;
    
    printwaypoint(current_waypoints);
}






//////////////////////////////////
////////// 主函数//////////////////
//////////////////////////////////
int main(int argc, char **argv)
{
    int i=1;              //飞机的序号,                                                          ///如何换飞机，即如何分辨实在给第几架飞机生成waypoint
    double hmin = 30;      //飞机最小盘旋高度
    double h = 5,H;          //飞机高度间隔
    double v=15;            //飞机的飞行速度(单位:m/s)
    double paolenth;
    double R,pointlenth,desire_ds,ds;
    double tan_x,tan_y;
    double *add;
    //由四个角点计算中心点的程序
    //输入四个角点的值
    double w1= 39.9897585,  w2=39.9897833,  w3=39.9882652,  w4= 39.9882500;
    double j1= 116.3526900, j2=116.3541295, j3=116.3542219, j4=116.3527874;
    //double wc,jc;
    wc = ( w1+w2+w3+w4 ) / 4;
    jc = ( j1+j2+j3+j4 ) / 4;

    //目前暂时以投弹区的中心点为wei0和jing0
    wei0 =wc;
    jing0=jc;

    cout<<"wc="<<wc<<endl;
    cout<<"jc="<<jc<<endl;

    //    double aaa[2],bbb[2];

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
        cout<<"THE CONNECTION STATE OF FCU IS "<< current_state.connected <<endl;
        ros::spinOnce();
        rate.sleep();
    }



    //    //不知道是做什么用的
    //    geometry_msgs::PoseStamped pose;
    //    //send a few setpoints before starting
    //    for(int i = 20; ros::ok() && i > 0; --i){
    //        local_pos_pub.publish(pose);
    //        ros::spinOnce();
    //        rate.sleep();
    //    }


    // lat 47.397748, long 8.545596, alt 487.9
    mavros_msgs::WaypointList waypoint_list;


    //将当前的目标航点设为0号航点
    mavros_msgs::WaypointSetCurrent Current_wp;
    Current_wp.request.wp_seq=0;
    if (waypoint_setcurrent_client.call(Current_wp)&&Current_wp.response.success)
    {
        ROS_INFO("%d", Current_wp.response.success);
        ROS_INFO("Waypoint set to 0 success");
    }


    //暂时设定30个航点
    mavros_msgs::Waypoint waypoint[30];


    //参数表
    //改变速度 command=178  param1 = 未知 ：0/1  param2 = 速度 ：15 param3 = 未知 ：-1 param3 = 未知 ：0
    //改变速度的经纬度似乎可有可无
    //起飞 command=22  param1 = pitch : 45.0
    //飞航点 command=16  param1 = HOLD ：单位 秒
    //盘旋 command=17  param3 = 盘旋半径 ：单位 米
    // !!!!!!! 盘旋半径为正值时为顺时针盘旋,为负值时为逆时针盘旋
    /*waypoint[] 的参数表     类型  mavros_msga::Waypoint
     * .x_lat           x坐标，经度
     * .y_long          y坐标，纬度
     * .z_alt           z坐标，高度
     * .command         设置命令类型  mavros_msgs::CommandCode::NAV_TAKEOFF   mavros_msgs::CommandCode::NAV_LOITER_UNLIM
     * .frame           设置坐标系   mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT
     * .autocontinue    true    第一个点的autocontinue必设置为true()默认为false
     * .is_current      true    （除起飞点，其他的暂设为false）
     *
     * 依据command来定
     * .param1          未知 ：0/1
     * .param2          速度 ：15
     * .param3          未知 ：-1
     * .param4          未知 ：0
     *
     *
     * */





    ///设置起飞点
    //设置经纬高
    waypoint[0].x_lat = 39.9889384;
    waypoint[0].y_long = 116.3497164;
    waypoint[0].z_alt = 30;
    //设置命令类型
    waypoint[0].command = mavros_msgs::CommandCode::NAV_TAKEOFF;
    //设置坐标系，通常选FRAME_GLOBAL_REL_ALT（3）
    waypoint[0].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    //第一个点的autocontinue必设置为true()默认为false
    waypoint[0].autocontinue = true;
    waypoint[0].is_current = true;
    //设置飞机的起飞爬升角(目前还有待研究)
    waypoint[0].param1 = 45.0;


    ///设置盘旋点(由比赛场地决定)



    //
    waypoint[1].x_lat = wc;
    waypoint[1].y_long = jc;

    ///盘旋点的高度 i是飞机的序号,h是飞机飞行高度差
    H = hmin + h * i;
    waypoint[1].z_alt = H;
    waypoint[1].param3 = -30;                                                                 ///说明param3为盘旋半径？？
    //    waypoint[1].param4 = nan;
    waypoint[1].command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
    //    waypoint[1].command = 31;//(LOITER_ALTITUTE) 实际最后可能还是会用上面那种
    waypoint[1].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    waypoint[1].autocontinue = true;
    waypoint[1].is_current = false;

    //    waypoint[1].param1 = 1;
    //    waypoint[1].param2 = -30.0;
    //    waypoint[1].param3 = 0;
    //    waypoint[1].param4 = 1;




    //    cin>>mission_received;
    double goal[2] = {0, 0};
    double bbb[2] = {0, 0};
    double tanlenth,aaa;


    //接受到消息之后,开始规划航迹从 waypoint[2]开始之后
    //    cout << "mission received!!!" <<endl;
    //    while (ros::ok() && mission_received ) {

    ///目标点的坐标接收
//    //目标点在盘旋园外的测试点
//        goal[0]= 39.9916445;
//        goal[1]= 116.3564010;

//    //目标点在盘旋圆外大圆内的测试点
//    goal[0]=  39.9893311;
//    goal[1]= 116.3537461;

//        目标点在盘旋园内的测试点
        goal[0]= 39.9889683;
        goal[1]= 116.3536919;

    cout<<"goal[0]=" << goal[0]<<endl;
    cout<<"goal[1]=" << goal[1]  <<endl;


    ///计算飞机飞行高度对应平抛距离
    paolenth = v * sqrt((2 * H) / g);

    ///计算飞机可以直接平抛的大圆半径
    desire_ds = 80;                          //在投掷之前必须先飞多少的直线
    ds = desire_ds + paolenth;
    R = sqrt( r*r + ds*ds);                        ///????
 /*   //    cout<<"R=" << R  <<endl;
    //        goal[0] = Prank[i][0];
    //        goal[1] = Prank[i][1];


    //        pointlenth为goal到盘旋圆心的距离

    ///要把经纬度转化为xy坐标
    ///
    //        ///  //计算原理
    //        //2*pi*R(地球半径)/360
    //        //经度的话要乘以一个cos纬度

    //        //将米转换为经纬度
    //        double m2L(double x, double y)
    //        {
    //            lat = jing0 + x / 85155.0;
    //            lon = wei0 + y / 111177.0;


    //        }
    //        //将经纬度转换为米(东北天)
    //        void L2m(double &w, double &j)
    //        {
    //            xc = (w - wei0) * 111177.0;
    //            yc = (j - jing0) * 85155.0;
    //        }

*/
    //把纬经高转化为ENU
    add=ll2xy(wc,jc);

    //        xc= (jc - jing0) * 85155.0;
    //        yc= (wc - wei0) * 111177.0;
    xc=add[0];
    yc=add[1];

    //        cout<<"xc=" << xc  <<endl;
    //        cout<<"yc=" << yc  <<endl;


    //        goal_x= ( goal[1] - jing0) * 85155.0;
    //        goal_y= ( goal[0] - wei0) * 111177.0;

    //把纬经高转化为ENU
    add=ll2xy(goal[0],goal[1]);
    goal_x=add[0];
    goal_y=add[1];




    //至此goal的值都是正确的
    cout<<"goal[0]=" << goal[0]<<endl;
    cout<<"goal[1]=" << goal[1]  <<endl;

    pointlenth=sqrt( ( xc-goal_x )*( xc-goal_x ) +( yc-goal_y )*( yc-goal_y ));

    cout<<"pointlenth="<<pointlenth<<endl;
    double *tan;

    //航点转化
    //        add=xy2ll(waypoint[].x_lat,waypoint[].y_long);
    //        waypoint[].x_lat  = add[0];
    //        waypoint[].y_long = add[1];

    //    cout<<r<<endl;

    //下面一行命令似乎会改变goal的值!!!
    //以下函数的输入输出都是纬经高
    tan = point_tangency(goal);
    tanlenth = sqrt((goal_x-tan_x)*(goal_x-tan_x)+(goal_y-tan_y)*(goal_y-tan_y));
    //转化为ENU
    add=ll2xy(tan[0],tan[1]);
    tan_x=add[0];
    tan_y=add[1];
    cout<<"tan_x  ="<< tan_x <<endl;
    cout<<"tan_y  ="<< tan_y <<endl;

    double n1_x,n1_y;
    if (pointlenth >= R)     //目标点在圆外且切线长足够平抛
    {   cout << "pointlenth >= R!!!" <<endl;

/*
        //            cout<<"tan="<< tan <<endl;



        //            tan_x= ( tan[1] - jing0) * 85155.0;
        //            tan_y= ( tan[0] - wei0) * 111177.0;


        //            cout<<"tan="<< tan <<endl;
        //            cout<<"tan="<< *tan <<endl;

        //            cout<<"tan[0]="<< tan[0] <<endl;
        //            cout<<"tan[1]="<< tan[1] <<endl;


        //            cout<<"goal[0]=" << goal[0]<<endl;
        //            cout<<"goal[1]=" << goal[1]  <<endl;
*/

        //            设置切点
        //waypoint[2].x_lat  = tan_x ;                                                                  ///这里赋值意义？？？
        //waypoint[2].y_long = tan_y ;
        //add=xy2ll(waypoint[2].x_lat,waypoint[2].y_long);
        add=xy2ll(tan_x,tan_y);
        waypoint[2].x_lat  = add[0];
        waypoint[2].y_long = add[1];

//        waypoint[2].x_lat  = tan[0] ;
//        waypoint[2].y_long = tan[1] ;

        //            cout<<"waypoint[2].x_lat=" << waypoint[2].x_lat <<endl;
        //            cout<<"waypoint[2].y_long="<< waypoint[2].y_long  <<endl;

        waypoint[2].z_alt = H;
        waypoint[2].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[2].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[2].autocontinue = true;
        waypoint[2].is_current = false;

        //            cout<<"goal[0]=" << goal[0]<<endl;
        //            cout<<"goal[1]=" << goal[1]  <<endl;

        for (int i = 3;i <= 39; i++)
        {
            double x = tan_x + (tanlenth- paolenth)/tanlenth*(goal_x - tan_x)*(i-2)/37;
            double y = tan_y + (tanlenth- paolenth)/tanlenth*(goal_y - tan_y)*(i-2)/37;
            add = xy2ll(x,y);
            waypoint[i].x_lat = add[0];
            waypoint[i].y_long = add[1];
            waypoint[i].z_alt = H;
            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[i].autocontinue = true;
            waypoint[i].is_current = false;

        }
        waypoint[40].x_lat = waypoint[39].x_lat;    //和第39个完全一致
        waypoint[40].y_long = waypoint[39].y_long;
        waypoint[40].z_alt = H;
        waypoint[40].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[40].autocontinue = true;
        waypoint[40].is_current = false;
        //设置投弹点
        //得到投弹点的xy坐标
        /*waypoint[3].x_lat  = goal_x-(goal_x-tan_x)/sqrt((goal_x-tan_x)*(goal_x-tan_x)+(goal_y-tan_y)*(goal_y-tan_y))*paolenth;
        waypoint[3].y_long = goal_y-(goal_y-tan_y)/sqrt((goal_x-tan_x)*(goal_x-tan_x)+(goal_y-tan_y)*(goal_y-tan[1]))*paolenth;

        //转化为经纬度
        add=xy2ll(waypoint[3].x_lat,waypoint[3].y_long);
        waypoint[3].x_lat  =  add[0];
        waypoint[3].y_long =  add[1];



        //            cout<<"waypoint[3].x_lat=" << waypoint[3].x_lat <<endl;
        //            cout<<"waypoint[3].y_long="<< waypoint[3].y_long  <<endl;

        waypoint[3].z_alt = H;
        waypoint[3].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[3].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[3].autocontinue = true;
        waypoint[3].is_current = false;
        */

        //            cout<<"goal[0]=" << goal[0]<<endl;
        //            cout<<"goal[1]=" << goal[1]  <<endl;


        //            设置目标点
        //            goal[0]= 39.9916445;
        //            goal[1]= 116.3564010;

        waypoint[41].x_lat  = goal[0];
        waypoint[41].y_long = goal[1];



        //            cout<<"waypoint[4].x_lat=" << waypoint[4].x_lat <<endl;
        //            cout<<"waypoint[4].y_long="<< waypoint[4].y_long  <<endl;

        waypoint[41].z_alt = H;
        waypoint[41].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[41].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[41].autocontinue = true;
        waypoint[41].is_current = false;




        //            用于最后的push
        size=41;




    }

    if (pointlenth < R && pointlenth >= r)     //目标点在圆外然而切线长不够平抛
    {
        cout << "pointlenth < R 且 pointlenth >= r!!!" <<endl;

//        tan = point_tangency(goal);


        //            point1[0] = -1*tan[0];
        //            point1[1] = -1*tan[1];
        //            aaa = (paolenth - tanlenth)/tanlenth;
        //            point2[0] = point1[0] + aaa*(tan[0] - goal[0]);
        //            point2[1] = point1[1] + aaa*(tan_[1] - goal[1]);
        //            point3[0] = tan[0] + aaa*(tan[0] - goal[0]);
        //            point3[1] = tan[1] + aaa*(tan[1] - goal[1]);


        //ENU系下的操作
        //如果中心不是00怎么办??
        cout<<"tan_x  ="<< tan_x <<endl;
        cout<<"tan_y  ="<< tan_y <<endl;

        //waypoint[2].x_lat  = -1*tan_x;
        //waypoint[2].y_long = -1*tan_y;

        n1_x = xc - tan_x;
        n1_y = yc - tan_y;


        //cout<<"waypoint[2].x_lat  ="<< waypoint[2].x_lat <<endl;
        //cout<<"waypoint[2].y_long  ="<< waypoint[2].y_long <<endl;
        for (int i=2;i<= 11;i++)
        {
            double x,y,x1,y1;

            x = -1*r;
            y = (2 - i)*ds/11;
            x1 = (-1*n1_x*x + n1_y*y)/r+xc;
            y1 = -1*(n1_y*x + n1_x*y)/r+yc;


            add = xy2ll(x1,y1);
            waypoint[i].x_lat = add[0];
            waypoint[i].y_long = add[1];
            waypoint[i].z_alt = H;
            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[i].autocontinue = true;
            waypoint[i].is_current = false;

        }

        for (int i=12;i<=27 ;i++)
        {

            double x,y,x1,y1;

            x = cos((i-27)*pi/15)*r;
            y = -1*ds+sin((i-27)*pi/15)*r;
            x1 = (-1*n1_x*x + n1_y*y)/r+xc;
            y1 = -1*(n1_y*x + n1_x*y)/r+yc;

            add = xy2ll(x1,y1);
            waypoint[i].x_lat = add[0];
            waypoint[i].y_long = add[1];
            waypoint[i].z_alt = H;
            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[i].autocontinue = true;
            waypoint[i].is_current = false;

        }

        for (int i=28;i<= 39;i++)
        {
            double x,y,x1,y1;

            x = r;
            y = (tanlenth - paolenth + ds) * (i - 27)/12 - ds;
            x1 = (-1*n1_x*x + n1_y*y)/r+xc;
            y1 = -1*(n1_y*x + n1_x*y)/r+yc;


            add = xy2ll(x1,y1);
            waypoint[i].x_lat = add[0];
            waypoint[i].y_long = add[1];
            waypoint[i].z_alt = H;
            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[i].autocontinue = true;
            waypoint[i].is_current = false;

        }
        waypoint[40].x_lat = waypoint[39].x_lat;    //和第39个完全一致
        waypoint[40].y_long = waypoint[39].y_long;
        waypoint[40].z_alt = H;
        waypoint[40].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[40].autocontinue = true;
        waypoint[40].is_current = false;



        waypoint[41].x_lat  = goal[0];
        waypoint[41].y_long = goal[1];



        //            cout<<"waypoint[4].x_lat=" << waypoint[4].x_lat <<endl;
        //            cout<<"waypoint[4].y_long="<< waypoint[4].y_long  <<endl;

        waypoint[41].z_alt = H;
        waypoint[41].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[41].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[41].autocontinue = true;
        waypoint[41].is_current = false;



 /*





        aaa = (paolenth - tanlenth)/tanlenth; //什么意思??? 得到了一个系数
        cout<<"aaa  ="<< aaa <<endl;

        waypoint[3].x_lat  = waypoint[2].x_lat  + aaa*(tan_x - goal_x);
        waypoint[3].y_long = waypoint[2].y_long + aaa*(tan_y - goal_y);

        cout<<"waypoint[3].x_lat  ="<< waypoint[3].x_lat <<endl;
        cout<<"waypoint[3].x_lat  ="<< waypoint[3].y_long <<endl;


        waypoint[4].x_lat  = tan_x + aaa*(tan_x - goal_x);
        waypoint[4].y_long = tan_y + aaa*(tan_y - goal_y);



        //转为LLA系
        add=xy2ll(waypoint[2].x_lat,waypoint[2].y_long);
        waypoint[2].x_lat  = add[0];
        waypoint[2].y_long = add[1];
        waypoint[2].z_alt = H;

        waypoint[2].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[2].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[2].autocontinue = true;
        waypoint[2].is_current = false;
        //            cout<<"waypoint[2].x_lat  ="<< waypoint[2].x_lat <<endl;


        add=xy2ll(waypoint[3].x_lat,waypoint[3].y_long);
        waypoint[3].x_lat  = add[0];
        waypoint[3].y_long = add[1];
        waypoint[3].z_alt = H;
        waypoint[3].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[3].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[3].autocontinue = true;
        waypoint[3].is_current = false;


        add=xy2ll(waypoint[4].x_lat,waypoint[4].y_long);
        waypoint[4].x_lat  = add[0];
        waypoint[4].y_long = add[1];
        waypoint[4].z_alt = H;
        waypoint[4].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[4].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[4].autocontinue = true;
        waypoint[4].is_current = false;

        waypoint[5].x_lat  = goal[0];
        waypoint[5].y_long = goal[1];
        waypoint[5].z_alt = H;
        waypoint[5].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[5].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[5].autocontinue = true;
        waypoint[5].is_current = false;

*/
        size=41;

    }
    //pointlenth为goal到盘旋圆心的距离
    if (pointlenth < r)     //目标点在圆外且切线长足够平抛
    {
        cout << "pointlenth < r!!!" <<endl;

        // 这个判断有什么作用???
        if (pointlenth < 0.0000001)
        {
            goal_x = goal_x + 0.0000001;
            goal_y = goal_y + 0.0000001;
        }

        n1_x = goal_x - xc;
        n1_y = goal_y - yc;

        /*//            point1[0] = -goal[0]/pointlenth*r;
        //            point1[1] = -goal[1]/pointlenth*r;

        //            bbb[0] = -1*point1[1];
        //            bbb[1] = point1[0];

        //            point2[0] = point1[0] + bbb[0];
        //            point2[1] = point1[1] + bbb[1];

        //            point3[0] = 2*bbb[0];
        //            point3[1] = 2*bbb[1];

        //            point5[0] = goal[0]/pointlenth*(pointlenth+paolenth);
        //            point5[1] = goal[1]/pointlenth*(pointlenth+paolenth);

        //            point4[0] = point3[0] + point5[0];
        //            point4[1] = point3[1] + point5[1];

        //ENU系下的操作
        waypoint[2].x_lat  = -goal_x/pointlenth*r;
        waypoint[2].y_long = -goal_y/pointlenth*r;

        bbb[0] = -1*waypoint[2].y_long;
        bbb[1] = waypoint[2].x_lat;

        waypoint[3].x_lat  = waypoint[2].x_lat + bbb[0];
        waypoint[3].y_long = waypoint[2].y_long + bbb[1];

        waypoint[4].x_lat  = 2*bbb[0];
        waypoint[4].y_long = 2*bbb[1];

        waypoint[6].x_lat  = goal_x/pointlenth*(pointlenth+paolenth);
        waypoint[6].y_long = goal_y/pointlenth*(pointlenth+paolenth);

        waypoint[5].x_lat  = waypoint[4].x_lat  + waypoint[6].x_lat;
        waypoint[5].y_long = waypoint[4].y_long + waypoint[6].y_long;
*/

        double a = r + ds + pointlenth;
        for (int i=2;i<=6 ;i++)
        {

            double x,y,x1,y1;

            x = a * (i - 2) / 5;
            y = -1*r;
            x1 = (n1_y*x + n1_x*y)/pointlenth+xc;
            y1 = (-1*n1_x*x + n1_y*y)/pointlenth+yc;

            add = xy2ll(x1,y1);
            waypoint[i].x_lat = add[0];
            waypoint[i].y_long = add[1];
            waypoint[i].z_alt = H;
            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[i].autocontinue = true;
            waypoint[i].is_current = false;

        }
        for (int i=7;i<=31 ;i++)
        {

            double x,y,x1,y1;

            x = a + cos((i-15)*pi/16)*a;
            y = ds+pointlenth+sin((i-15)*pi/16)*a;
            x1 = (n1_y*x + n1_x*y)/pointlenth+xc;
            y1 = (-1*n1_x*x + n1_y*y)/pointlenth+yc;

            add = xy2ll(x1,y1);
            waypoint[i].x_lat = add[0];
            waypoint[i].y_long = add[1];
            waypoint[i].z_alt = H;
            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[i].autocontinue = true;
            waypoint[i].is_current = false;

        }
        for (int i=32;i<=39 ;i++)
        {

            double x,y,x1,y1;

            x = 0;
            y = ds+pointlenth - ds * (i - 31)/8;
            x1 = (n1_y*x + n1_x*y)/pointlenth+xc;
            y1 = (-1*n1_x*x + n1_y*y)/pointlenth+yc;

            add = xy2ll(x1,y1);
            waypoint[i].x_lat = add[0];
            waypoint[i].y_long = add[1];
            waypoint[i].z_alt = H;
            waypoint[i].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
            waypoint[i].autocontinue = true;
            waypoint[i].is_current = false;

        }
        waypoint[40].x_lat = waypoint[39].x_lat;    //和第39个完全一致
        waypoint[40].y_long = waypoint[39].y_long;
        waypoint[40].z_alt = H;
        waypoint[40].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[40].autocontinue = true;
        waypoint[40].is_current = false;



        waypoint[41].x_lat  = goal[0];
        waypoint[41].y_long = goal[1];



        //            cout<<"waypoint[4].x_lat=" << waypoint[4].x_lat <<endl;
        //            cout<<"waypoint[4].y_long="<< waypoint[4].y_long  <<endl;

        waypoint[41].z_alt = H;
        waypoint[41].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[41].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[41].autocontinue = true;
        waypoint[41].is_current = false;


/*
        add=xy2ll(waypoint[2].x_lat,waypoint[2].y_long);
        waypoint[2].x_lat  = add[0];
        waypoint[2].y_long = add[1];

        waypoint[2].z_alt = H;
        waypoint[2].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[2].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[2].autocontinue = true;
        waypoint[2].is_current = false;
        //            cout<<"waypoint[2].x_lat  ="<< waypoint[2].x_lat <<endl;


        add=xy2ll(waypoint[3].x_lat,waypoint[3].y_long);
        waypoint[3].x_lat  = add[0];
        waypoint[3].y_long = add[1];

        waypoint[3].z_alt = H;
        waypoint[3].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[3].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[3].autocontinue = true;
        waypoint[3].is_current = false;


        add=xy2ll(waypoint[4].x_lat,waypoint[4].y_long);
        waypoint[4].x_lat  = add[0];
        waypoint[4].y_long = add[1];

        waypoint[4].z_alt = H;
        waypoint[4].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[4].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[4].autocontinue = true;
        waypoint[4].is_current = false;



        add=xy2ll(waypoint[6].x_lat,waypoint[6].y_long);
        waypoint[6].x_lat  = add[0];
        waypoint[6].y_long = add[1];

        waypoint[6].z_alt = H;
        waypoint[6].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[6].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[6].autocontinue = true;
        waypoint[6].is_current = false;



        add=xy2ll(waypoint[5].x_lat,waypoint[5].y_long);
        waypoint[5].x_lat  = add[0];
        waypoint[5].y_long = add[1];

        waypoint[5].z_alt = H;
        waypoint[5].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[5].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[5].autocontinue = true;
        waypoint[5].is_current = false;

        //waypoint[5]和waypoint[6]起码还得再有几个点

        waypoint[7].x_lat  = goal[0];
        waypoint[7].y_long = goal[1];
        waypoint[7].z_alt = H;
        waypoint[7].command = mavros_msgs::CommandCode::NAV_WAYPOINT;
        waypoint[7].frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        waypoint[7].autocontinue = true;
        waypoint[7].is_current = false;

*/
        size=41;
    }
    //        break;
    //    }


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
    // for (size_t i = 0; i < waypoint_list.waypoints.size(); i++)
    // {
    //     /* code for loop body */waypoint_push.request.waypoints.push_back()
    // }

    //points.waypoints.size()
    cout<<"waypoint_list.waypoints.size()"<<waypoint_list.waypoints.size()<<endl;

    for(i = 0; i < size; i++){                                                                     //??????
        cout<<i<<endl;
        waypoint_push.request.waypoints.push_back(waypoint[i]);

    }



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
    offb_set_mode.request.custom_mode = "AUTO.MISSION";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    set_mode_client.call(offb_set_mode);
//    cout<<offb_set_mode.response.success<<endl<<endl;


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
