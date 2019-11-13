#include <iostream>
#include <math.h>

using namespace std;

//一些常值，暂定为全局常值变
const int NUAV = 6;   //给出天上盘旋的无人机数量
const int Npoint = 6; //给出目标点数量
const float g = 9.8;  //重力加速度

int r = 30; //全局变量

int main()
{
    float v = 10;         //飞机速度
    float hmin = 30;      //飞机最小盘旋高度
    float h = 5;          //飞机高度间隔
    float H[NUAV];        //生成每架飞机高度列表
    float paolenth[NUAV]; //每架飞机对应平抛距离
    float P[Npoint][3];   //目标点坐标
    float Prank[NUAV][3];
    float pointlenth[NUAV]; //存贮点到圆心的距离

    float *point_tangency(float x[2]);

    for (int i = 0; i < NUAV; i++)
    {
        H[i] = hmin + h * i;                  //生成每架飞机高度列表
        paolenth[i] = v * sqrt(2 * H[i] / g); //计算每架飞机对应平抛距离
    }
    ///*生成场地和目标点坐标,侦查组所需传递的信息

    //===============================P的值由侦察组提供，具体还要细化============================
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < Npoint; j++)
        {
            P[j][i] = (i + j) / (Npoint + 3.0);
        }
    }
    //=======================================================================================

    for (int j = 0; j < Npoint; j++)
    {
        P[j][0] = P[j][0] * 100 - 50;
        P[j][1] = P[j][1] * 300 - 150;
        P[j][2] = 0;
    }

    //把目标点按离圆心的距离排序,先排够打一轮的
    for (int i = 0; i < NUAV; i++)
    {
        int n;
        float s0 = 0;
        for (int j = 0; j < Npoint; j++)
        {
            float s = sqrt(P[j][0] * P[j][0] + P[j][1] * P[j][1]); //找出离圆心最近点

            if (j == 0)
            {
                s0 = s;
                n = 0;
            }
            if (s < s0)
            {
                s0 = s;
                n = j; //目标点n为飞机i的攻击点，与i水平距离为s0
            }
        }

        Prank[i][0] = P[n][0];
        Prank[i][1] = P[n][1];

        P[n][0] = 10000; //把找到的好点弄出去
        P[n][1] = 10000;

        Prank[i][2] = H[i];
        pointlenth[i] = s0; //存储点到圆心距离
    }

    float R, tanlenth, aaa;
    float goal[2] = {0, 0},
            bbb[2] = {0, 0};
    float *tan;
    float point1[2] = {-1,-1},
            point2[2] = {-1,-1},
            point3[2] = {-1,-1},
            point4[2] = {-1,-1},
            point5[2] = {-1,-1};

    for (int i = 0; i < NUAV; i++)
    {
        R = sqrt(r*r + paolenth[i]*paolenth[i]);
        goal[0] = Prank[i][0];
        goal[1] = Prank[i][1];
        
        
        if (pointlenth[i] >= R)     //目标点在圆外且切线长足够平抛
        {
            tan = point_tangency(goal);
            point1[0] = tan[0];
            point1[1] = tan[1];
            point2[0]=goal[0]-(goal[0]-tan[0])/sqrt((goal[0]-tan[0])*(goal[0]-tan[0])+(goal[1]-tan[1])*(goal[1]-tan[1]))*paolenth[i];
            point2[1]=goal[1]-(goal[1]-tan[1])/sqrt((goal[0]-tan[0])*(goal[0]-tan[0])+(goal[1]-tan[1])*(goal[1]-tan[1]))*paolenth[i];
        }

        if (pointlenth[i] < R && pointlenth[i] >= r)     //目标点在圆外且切线长足够平抛
        {
            tan = point_tangency(goal);
            tanlenth = sqrt((goal[0]-tan[0])*(goal[0]-tan[0])+(goal[1]-tan[1])*(goal[1]-tan[1]));
            point1[0] = -1*tan[0];
            point1[1] = -1*tan[1];
            aaa = (paolenth[i] - tanlenth)/tanlenth;
            point2[0] = point1[0] + aaa*(tan[0] - goal[0]);
            point2[1] = point1[1] + aaa*(tan[1] - goal[1]);
            point3[0] = tan[0] + aaa*(tan[0] - goal[0]);
            point3[1] = tan[1] + aaa*(tan[1] - goal[1]);
        }

        if (pointlenth[i] < r)     //目标点在圆外且切线长足够平抛
        {
            if (pointlenth[i] < 0.0000001)
            {
                goal[0] = goal[0] + 0.0000001;
                goal[1] = goal[1] + 0.0000001;
            }
            point1[0] = -goal[0]/pointlenth[i]*r;
            point1[1] = -goal[1]/pointlenth[i]*r;
            bbb[0] = -1*point1[1];
            bbb[1] = point1[0];
            point2[0] = point1[0] + bbb[0];
            point2[1] = point1[1] + bbb[1];
            point3[0] = 2*bbb[0];
            point3[1] = 2*bbb[1];
            point5[0] = goal[0]/pointlenth[i]*(pointlenth[i]+paolenth[i]);
            point5[1] = goal[1]/pointlenth[i]*(pointlenth[i]+paolenth[i]);
            point4[0] = point3[0] + point5[0];
            point4[1] = point3[1] + point5[1];
        }
    }

    ///*测试用句
    cout << "1:" << point1[0] << "  " << point2[0] << "  " << point3[0] << "  " << point4[0] << "  " << point5[0] << endl;
    cout << "2:" << point1[1] << "  " << point2[1] << "  " << point3[1] << "  " << point4[1] << "  " << point5[1] << endl;
    //    cout << "3:" << point1[2] << "  " << point2[2] << "  " << point3[1] << "  " << point4[1] << "  " << point5[1] << endl;
    //    cout << "4:" << point1[3] << "  " << point2[1] << "  " << point3[1] << "  " << point4[1] << "  " << point5[1] << endl;
    //    cout << "5:" << point1[4] << "  " << point2[1] << "  " << point3[1] << "  " << point4[1] << "  " << point5[1] << endl;
    //    cout << "6:" << point1[5] << "  " << point2[1] << "  " << point3[1] << "  " << point4[1] << "  " << point5[1] << endl;


    //*/

    return 0;
}

float *point_tangency(float x[2])
{
    //===========================数据来源，为圆心坐标，具体未知===================
    float x0 = 0;
    float y0 = 0;
    //==========================================================================

    float r0 = r;
    float y[2]={9,0};
    float x_1,y_1,x_2,y_2;
    float k1,k2,*res=y;

    //找出两个切点（x_1,y_1）(x_2,y_2)
    k1 = (y0*x0 + x[1]*x[0] - y0*x[0] - x[1]*x0 + sqrt(r0*r0*(-2*y0*x[1] - 2*x0*x[0] + x[1]*x[1] +
            y0*y0 + x0*x0 - r0*r0 + x[0]*x[0]))) / (-r0*r0 + x0*x0 - 2*x0*x[1] + x[0]*x[0]);
    k2= (y0*x0 + x[1]*x[0] - y0*x[0] - x[1]*x0 - sqrt(r0*r0*(-2*y0*x[1] - 2*x0*x[0] + x[1]*x[1] +
            y0*y0 + x0*x0 - r0*r0 + x[0]*x[0])))/(-r0*r0 + x0*x0 - 2*x0*x[0] + x[0]*x[0]);
    x_1 = (-k1*x[1] + x0 + k1*k1*x[0] + y0*k1) / (1 + k1*k1);
    y_1 =-(-x[1] - k1*x0 - y0*k1*k1 + k1*x[0]) / (1 + k1*k1);
    x_2 = (-k2*x[1] + x0 + k2*k2*x[0] + y0*k2) / (1 + k2*k2);
    y_2 =-(-x[1] - k2*x0 - y0*k2*k2 + k2*x[0]) / (1 + k2*k2);

    //%%%%判断逆时针是先到哪个切点%%%%%%%%
    float w[9] = {0,-1,y0,1,0,-x0,-y0,x0,0};
    float r1[3] = {x_1 - x0, y_1-y0, 0};
    float r2[3] = {x_2 - x0, y_2-y0, 0};
    float v1[3],v2[3];
    float s[3] = {x[0] - x0, x[1] - y0, 0};
    float s1=0,s2=0;
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
    }
    if (s2 > 0)
    {
        y[0] = x_2;
        y[1] = y_2;
    }
    return res;
}
