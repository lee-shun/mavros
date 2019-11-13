/* Leader Follower position control
*
* * Author : LZX
**Date: 2019-4-27
**Function:
*           1.get the leader's position (in latitude and longitude), estimate the velocity of leader.
*           2.give the desired position (in latitude and longitude) and desired velocity.
*Coordinates:
*           ENU: x directs in east; y in north; z in up;
*Input argurments:
*                       1. leader latitude 2. leader longitude 3. time
*                       4. follower latitude 5. follower longitude
*                       6. desired gap in velocity coordinate
*/

#include<stdlib.h>
#include<iostream>
#include<string>
#include<math.h>

double PI = 3.1415926;
double K = 1;                                   //parameter of gain
double jing0 = 0;
double wei0 = 0;
int inter_time = 8;

double *xy2ll(double x, double y);
double *ll2xy(double w, double j);
double CalcuVelAngel(double V_x, double V_y);

class setPoint
{
public:
    setPoint(double latitude_in, double longitude_in, double V_in);
    void show();
    ~setPoint(){};

private:
    double latitude;
    double longitude;
    double V;

};

setPoint::setPoint(latitude_in, longitude_in, V_in)
{
    latitude = latitude_in;
    longitude = longitude_in;
    V = V_in;
}

void setPoint::show()
{
    std::cout << 'desired latitude = ' << latitude << 'desired longitude = ' << longitude << 'desired V = ' << V << std::endl;
}

int main(int argc, char const *argv[])
{
    double leader_latitude[2], leader_longitude[2], time[2], follower_latitude, follower_longitude, desired_gap_x_V, desired_gap_y_V, desired_position_latitude, desired_position_longitude, desired_V;
    int init_flag = 1;


    double w1= 39.9897585,  w2=39.9897833,  w3=39.9882652,  w4= 39.9882500;
    double j1= 116.3526900, j2=116.3541295, j3=116.3542219, j4=116.3527874;
    //double wc,jc;
    wc = ( w1+w2+w3+w4 ) / 4;
    jc = ( j1+j2+j3+j4 ) / 4;
    wei0 =wc;
    jing0=jc;


    do
    {
        //传入leader的位置
        std::cout << 'Please input the leader\'s latitude and longitude and time' << std::endl;
        std::cin >> leader_latitude[1] >> leader_longitude[1];
        leader_latitude[0] = leader_latitude[1];
        leader_longitude[0] = leader_longitude[1];
        time[0] = time[1];

        //订阅follower此时的位置
        std::cout << 'Please input the follower\'s latitude and longitude' << std::endl;
        std::cin >> follower_latitude >> follower_longitude;

        //设定一个偏差的的位置
        std::cout << 'Please input the desired gap x and y in velocity direction' << std::endl;
        std::cin >> desired_gap_x_V >> desired_gap_y_V;

        if (init_flag)
        {
            init_flag = 0;
            continue;
        }

        double leader_x[2], leader_y[2], follower_x, follower_y, leader_Vx, leader_Vy;
        double xy[2];


        xy = ll2xy(leader_latitude[0], leader_longitude[0]);                        //leader position in time0
        leader_x[0] = xy[0];
        leader_y[0] = xy[1];
        xy = ll2xy(leader_latitude[1], leader_longitude[1]);                        //leader position in time1
        leader_x[1] = xy[0];
        leader_y[1] = xy[1];

        leader_Vx = (leader_x[1] - leader_x[0]) / (time[1] - time[0]);          //leader velocity
        leader_Vy = (leader_y[1] - leader_y[0]) / (time[1] - time[0]);
        double theta_V;
        theta_V = CalcuVelAngel(leader_Vx, leader_Vx);

        xy = ll2xy(follower_latitude, follower_longitude);                      //follower position
        follower_x = xy[0];
        follower_y = xy[0];

        double desired_gap_x, desired_gap_y, desired_x, desired_y;
        desired_gap_x = cos(theta_V) * desired_gap_x_V + sin(theta_V) * desired_gap_y_V;
        desired_gap_y = ( -sin(theta_V) ) * desired_gap_x_V + cos(theta_V) * desired_gap_y_V;
        desired_x = leader_x[1] - desired_gap_x;
        desired_y = leader_y[1] - desired_gap_y;

        double ll[2];
        ll = xy2ll(desired_x,desired_y);
        desired_position_latitude = ll[0];
        desired_position_longitude = ll[1];

        double distance;
        distance = sqrt( (leader_x[1] - follower_x) * (leader_x[1] - follower_x) + (leader_y[1] - follower_y) * (leader_y[1] - follower_y) );

        double leader_V;
        leader_V = sqrt( leader_Vx * (leader_Vx) + (leader_Vy) * (leader_Vy) );
        desired_V = leader_V + K * distance;

        for (int i = 0; i < inter_time; ++i)
        {
            double current_latitude, current_longitude;
            current_latitude = follower_latitude + (i + 1)*(desired_position_latitude - follower_latitude)/inter_time;
            current_longitude = follower_longitude + (i + 1)*(desired_position_longitude - follower_longitude)/inter_time;
            setPoint result(current_latitude, current_longitude,desired_V);
            result.show();
        }

    } while (1);

    return 0;
}


double CalcuVelAngel(double V_x, double V_y)
{
    double theta;
    theta = atan(V_y / V_x);
    if (V_x <0)
    {
        if (V_y < 0)
        {
            theta = theta - PI;
        }
        else
        {
            theta = theta + PI;
        }
    }
    return theta;
}



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
    LL[0] = wei0 + y / 111177.0;
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
