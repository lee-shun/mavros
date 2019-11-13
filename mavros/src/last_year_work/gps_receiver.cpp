/* 串口通信 程序 银河
 * 原作者：zzy 
 * 作者:  SSS & LS
 * 日期:    2019.04.29
 * 功能:
 *  1. 接受来自某一个通信模块的数据进行处理
    2. 将解包后的数据发布到本机的ros话题当中       
 *  3. 订阅自身的ros话题，通过串口传送出去        
 *                        
 */
#include <ros/ros.h>

#include <serial/serial.h>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <cstring>

#include <fstream>
#include <cstdlib>
#include <stdlib.h>

//topic msg头文件
#include "std_msgs/UInt16.h"
#include "std_msgs/String.h"
//#include <dji_sdk/Data_Throw_target.h>       //ROS -> serial    投掷机 需要 接收&执行 的指令信息
//#include <dji_sdk/Data_Drone_State.h>        //serial -> ROS    地面站 接收到的 投掷机的 状态信息



#include <iomanip>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/NavSatFix.h>

#include <mavros/Data_Drone_State.h>
#include <mavros/Data_Throw_target.h>



sensor_msgs::NavSatFix need_to_send_gps;




using namespace std;
void show_connectedDevice();
//##################################################            全局变量         ############################################################//
//----------------------------------------          无线模块相关          ----------------------------------------//
//通信模块配置 相关帧头
//上电节点配置
const uint8_t CONFIG_Header_0 = 0xEB;               //帧头0
const uint8_t CONFIG_Header_1 = 0x90;               //帧头1
const uint8_t CONFIG_ID =       0x03;               //通信模块编号 | 与飞机编号相同   0x01&0x02_侦察机    0x03-0x08_投掷机    0x0a_地面站

//接收数据
const uint8_t Data_HEADER_0	= 0X9F;                 //帧头0
const uint8_t Data_HEADER_1	= 0XE4;                 //帧头1

//自定义数据帧头帧尾
const uint8_t CustomData_Header_0	= 0X31;         //帧头0
const uint8_t CustomData_Header_1	= 0X6B;         //帧头1

//统计连接在内的模块
char16_t Connected_Num = 0x0001 << CONFIG_ID;       //统计连接在系统内的模块
int connected_Count = 0;                            //统计连接的模块计数

uint8_t Rbuffer_rx[512];                            //串口接收Raw  20Hz读取频率
size_t  Rn;                                         //串口接收Raw 字节数

int flag_find;                                      //帧头寻找 Flag
size_t n_last, n_present;                           //指针*
uint8_t buffer_rx_precent[11][128];                 //Rbuffer_rx 分割帧后   01-0a
int     n_rx_precent[11];                           //buffer_rx_precent[x] 中的有效字节数

int flag_complete;                                  //包完整性判断Flag
uint8_t buffer_rx[128];                             //串口接收 一个完整的自定义数据包
size_t  n;                                          //串口接收 一个完整的自定义数据包 字节数

uint8_t buffer_temp[512];                           //临时数组

//串口待发送信息缓存 20Hz 每次只发送一条信息
//后期需修改!!!!!!
uint8_t buffer_tx[16][256];                             //串口发送 缓存
size_t  n_tx[16];                                       //串口发送 缓存 对应字节数 0-15  0x0-0xF
int  n_tx_current_flag = -1;                              //串口发送 缓存 标志 - -1 没有

uint8_t Wbuffer_tx[256];                            //串口实际发送 [自定义 帧头*2 & 数据位*1 & 数据*n & 校验位*1]
size_t  Wn_tx;                                      //串口实际发送 字节数

//----------------------------------------        ROS&自定义信息相关       ----------------------------------------//
uint8_t sendID;                                     //接收到的信息的发送端ID
uint8_t Order_num;                                  //指令编号  -- 用于通信应答!!!! 待完成   [l3-0]

int Drone_ID;       //投掷机 编号
int Accepted_ID;   //接收机 编号
int Drone_state;    //##本机状态	/111 - 0x0a接收到此指令后，即得知该机可以接收新的目标
//          /777    被强制返航   /0	本机有正在执行的任务
int target_ID;     // 投掷目标编号 1-10 根据被识别到的顺序
int target_type;   //投掷目标 类型
int target_Num;    //对应投掷目标已经投掷次数	/无人机投掷时发布	/无人机被强制返航时发布



//####################################################################################################################
//##################################################            主函数         ############################################################//
int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_receiver");
    ros::NodeHandle nh("~");                                     //创建句柄
    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("tranfered_gps", 10);

    //需要通过 通信模块 发布出的信息
    //--通过 ROS 接收 mission_Ground.cpp 的话题
    //--包括 \接收的设备 \发布信信息[1-返航 2-投掷目标(投掷目标类型 投掷目标编号 本次投掷次数)]
    //ros::Subscriber WaitSendtoData_Throw_target_Sub = nh.subscribe<mavros::Data_Throw_target>("/WaitSendtoData_Throw_target", 10, WaitSendtoData_Throw_target_callback);

    //需要更新的状态  --通过 ROS 发布 给 mission_Ground.cpp
    //ros::Publisher received_Throw_state_Pub = nh.advertise<mavros::Data_Drone_State>("/Mission_Data_Drone_State_renew", 10);

    ros::Rate loop_rate(20);                                   //数据状态更新频率 [20Hz]

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          串口 打开           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
    //串口serial设置&开启
    serial::Serial sp;                                          //创建一个serial类
    serial::Timeout to = serial::Timeout::simpleTimeout(10);   //创建timeout
    sp.setPort("/dev/ttyUSB0");                                 //设置要打开的串口名称
    sp.setBaudrate(115200);                                     //设置串口通信的波特率
    sp.setTimeout(to);                                          //串口设置timeout
    //打开串口
    try
    {    sp.open();
    }
    catch(serial::IOException& e)
    {   ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    //判断串口是否打开成功
    if(sp.isOpen())
    {   ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {   return -1;
    }
    //----------------------------------------          串口打开 完成         --------------------------------------------------//

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          模块 初始配置           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
    // 串口打开后，配置模块 地址 EB 90 + 飞机编号
    Wbuffer_tx[0] = CONFIG_Header_0;     //配置帧头 0xEB
    Wbuffer_tx[1] = CONFIG_Header_1;     //配置帧头 0x90
    Wbuffer_tx[2] = CONFIG_ID;           //本机编号 0x03-0x09
    cout<<"Send_Configuration_Set: ";
    for(int i=0; i<3; i++)
    {
        cout << hex << (Wbuffer_tx[i] & 0xff) << " ";
    }
    cout << endl << "Please Wait!!" << endl << endl;

    int flag_config = 1;
    while(flag_config)
    {
        ros::spinOnce();

        sp.write(Wbuffer_tx, 3);
        usleep(25000);              //百万分之一秒        //5,0000 * 0.000 001 = 0.05
        //获取缓冲区内的字节数
        Rn = sp.available();
        if(Rn!=0)
        {
            Rn = sp.read(Rbuffer_rx, Rn);  //读出数据
            if (Rn==4)   //配置返回信息
            {
                if (Rbuffer_rx[0]==CONFIG_Header_0 && Rbuffer_rx[1]==CONFIG_Header_1 && Rbuffer_rx[2]==0x00 && Rbuffer_rx[3]==CONFIG_ID )
                {
                    flag_config = 0;
                    cout<<"Configuration transreceiver Sucessed!  Info  -- Start connected"<<endl;
                }
                else
                {
                    flag_config = 1;
                    cout<<"Configuration transreceiver Failed!  Info  -- n=4 Failed"<<endl;
                }
            } // End{ if (Rn==4)
            else
            {
                if (Rbuffer_rx[0]==Data_HEADER_0 && Rbuffer_rx[1]==Data_HEADER_1 && Rbuffer_rx[2]==CONFIG_ID )
                {
                    flag_config = 0;
                    cout<<"Configuration transreceiver Sucessed!  Info  -- Already connected"<<endl;
                }
                else
                {
                    flag_config = 1;
                    cout<<"Configuration transreceiver Failed!  Info  -- n<>4 Failed"<<endl;
                }
            }
            //接收信息打印
            cout<<"Set_Transceiver_Result: ";
            for(int i=0; i<Rn; i++)
            {
                cout << hex << (Rbuffer_rx[i] & 0xff) << " ";
            }
            cout << endl << endl;
        }   // if(Rn!=0)
        loop_rate.sleep();
    }   // while(flag_config)
    //----------------------------------------          模块 初始配置 完成         --------------------------------------------------//

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          模块数据 串口读取&发送&处理           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
    ofstream outfile;

    //部分变量&缓存数组初始化
    for(int i=1; i<11; i++)
    {
        buffer_rx_precent[i][0] = 0;
        n_rx_precent[i] = 0;
    }
    n_tx_current_flag = -1;

    while(ros::ok())
    {
        ros::spinOnce();
        // 获取缓冲区内的字节数
        Rn = sp.available();

        if(Rn!=0)
        {
            Rn = sp.read(Rbuffer_rx, Rn);      // 读取串口数据
            // 输出串口收到的数据
            cout << endl << endl << "-------------this is the new data-----------" <<endl;
            cout << "Rn = " << Rn << endl << "Receive_Data: ";
            for(int i=0; i<Rn; i++)
            {
                cout << hex << (Rbuffer_rx[i] & 0xff) << " ";
            }
            cout << endl;

            //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>          分割 Rbuffer_rx           <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
            n_present = 0;
            n_last = n_present;
            sendID = 0x00;
            Connected_Num = 0x0001 << CONFIG_ID;
            connected_Count = 0;

            //寻找最开始的 EB90 如果 不是0 则输出 error 到txt文件!!
            flag_find = 0;
            while (flag_find ==0)
            {
                if (n_present+2==Rn)
                {
                    flag_find = 1;
                    n_present = Rn;
                    //没有找到帧头 到最后了
                }
                else if (Rbuffer_rx[n_present+0]==Data_HEADER_0 && Rbuffer_rx[n_present+1]==Data_HEADER_1 && Rbuffer_rx[n_present+2]==CONFIG_ID )
                {
                    flag_find = 1;
                }
                else
                {
                    flag_find = 0;
                    n_present = n_present+1;
                }
            }
            //cout << "n_present init:" << n_present << ", n_last inti:" << n_last << endl;

            // 如果 EB 90 CONFIG_ID 不是 Rbuffer_rx[0][1][2] 则输出 error 到txt文件!!
            if (n_present != 0)
            {
                outfile.open("/home/ubuntu/serial_error/Rbuffer_rx.txt",ios::app);
                outfile << endl <<"-- EB 90 -- is not Rbuffer_rx[0][1]" << endl;
                outfile << "Rbuffer_rx : ";
                for(int i=0; i<Rn; i++)
                {
                    outfile << hex << (Rbuffer_rx[i] & 0xff) << " ";
                }
                outfile << endl;
                outfile.close();
            }

            //进行分割
            while( n_present<Rn )
            {
                n_last = n_present;         //下个包的开头
                n_present = n_present+1;    //寻找下下一个包的开头

                flag_find = 0;              //flag复位
                while (flag_find ==0 )
                {
                    if ( n_present+2 == Rn)
                    {
                        flag_find = 1;
                        n_present = Rn;     //没有找到帧头 到最后了
                    }
                    else if (Rbuffer_rx[n_present+0]==Data_HEADER_0 && Rbuffer_rx[n_present+1]==Data_HEADER_1 && Rbuffer_rx[n_present+2]==CONFIG_ID )
                    {
                        flag_find = 1;      //下下一个包的开头
                    }
                    else
                    {
                        flag_find = 0;
                        n_present = n_present+1;
                    }
                }

                sendID = Rbuffer_rx[n_last+3];
                cout <<  "n_last :" << n_last << ", n_present :" << n_present <<endl;
                cout << "Data From [" << dec << (int)sendID << "]";

                n = n_present - n_last;
                if (n > 8)
                {   //数据复制到对应的ID后
                    memcpy(buffer_rx_precent[sendID]+n_rx_precent[sendID], Rbuffer_rx+n_last+8, n-8);
                    n_rx_precent[sendID] = n_rx_precent[sendID] + n-8;

                    cout << ", n =" << dec << n << ", Receive_Data: ";
                    for(int i=n_last; i<n_present; i++)
                    {
                        cout << hex << (Rbuffer_rx[i] & 0xff) << " ";
                    }

                    outfile.open("/home/ubuntu/serial_error/Received_Data.txt",ios::app);
                    outfile << endl <<"n_rx_precent[" << dec << (int)sendID << "] , n=" << (int)n_rx_precent[sendID] << endl;
                    for(int i=0; i<n_rx_precent[sendID]; i++)
                    {
                        outfile << hex << (buffer_rx_precent[sendID][i] & 0xff) << " ";
                    }
                    outfile << endl;
                    outfile.close();
                }
                else if (n < 8)
                { // !!!!串口读取有错误
                    outfile.open("/home/ubuntu/serial_error/Rbuffer_rx.txt",ios::app);
                    outfile << endl <<"-- n < 8 -- Rbuffer_rx error -- ID: " << dec <<(int)sendID << endl;
                    outfile << "Rbuffer_rx : ";
                    for(int i=0; i<Rn; i++)
                    {
                        outfile << hex << (Rbuffer_rx[i] & 0xff) << " ";
                    }
                    outfile << endl;
                    outfile.close();
                }
                else
                { // n==8 !!!!上一回的数据可能有错误
                    if (n_rx_precent[sendID] >0)
                    {
                        outfile.open("/home/ubuntu/serial_error/Rbuffer_rx.txt",ios::app);
                        outfile << endl <<" -- n_rx_precent[sendID] >0 error -- ID: " << dec << (int)sendID << endl;
                        outfile << "n_rx_precent : ";
                        for(int i=0; i<n_rx_precent[sendID]; i++)
                        {
                            outfile << hex << (buffer_rx_precent[sendID][i] & 0xff) << " ";
                        }
                        outfile << endl;
                        outfile.close();
                    }
                }
                cout << endl;
                //----------------------------------------          分割 Rbuffer_rx  完成         --------------------------------------------------//


                if (sendID != 0x00) //统计已连接的模块
                {
                    Connected_Num = (Connected_Num | (0x0001 << sendID ));
                }
                connected_Count = connected_Count+1;
            }

            show_connectedDevice();     //显示已连接模块数量和编号

        }//if(Rn!=0)
        else
        {
            sendID = 0x00;  //记录已连接模块ID清零
            cout << endl << endl << "-------------this is the new data-----------" << endl << "Serial Received NO Data & No device connected !" <<endl;
        }

            



    //    for( int i=0;i<=n_rx_precent[sendID];i++)
    //         {
    //             cout<< hex << (buffer_rx_precent[sendID][i] & 0xff)<<"  ";

           
    //         }
    //         cout<<endl;

    //         for( int i=0;i<=n_rx_precent[sendID];i++)
    //         {
    //             cout<< hex << (buffer_rx_precent[sendID+1][i] & 0xff)<<"  ";
                
           
    //         }
    //         cout<<endl;

    //         for( int i=0;i<=n_rx_precent[sendID];i++)
    //         {
    //             cout<< hex << (buffer_rx_precent[sendID-1][i] & 0xff)<<"  ";
                
           
    //         }


              //---------------------------------解读GPS信息————第一部分-----------------------------------------------------
                double lat;
                double lon;
                int data_lat[5],data_lon[5];
                int i=0;
                //读取纬度
                for(i=0;i<5;i++){

                    data_lat[i] = buffer_rx_precent[sendID][n_last+3+i];

                }
                for(i=0;i<5;i++){

                    cout<< "data_lat"<<"["<<i<<"]="<<dec<< data_lat[i] <<";"<<endl;

                }


                //读取经度

                for(i=0;i<5;i++){
                    data_lon[i] = buffer_rx_precent[sendID][n_last+8+i];

                }
                for(i=0;i<5;i++){
                    cout<< "data_lon"<<"["<<i<<"]="<<dec<< data_lon[i] <<";"<<endl;
                }
                //-------------------------------------------------------------------------------------


                //---------------------------------解读GPS信息————第二部分-----------------------------------------------------


                        lat = 0;
                        lon = 0;


                        for(i=0;i<5;i++){

                            lat = lat + data_lat[i];
                            lat = lat*100;
                            //        cout<< "lat = " << lat <<endl;
                        }
                        lat = lat/(1e10);

                           cout<< "lat = " <<fixed<<setprecision(10)<< lat <<endl;



                        for(i=0;i<5;i++){

                            lon = lon + data_lon[i];
                            lon = lon*100;
                        }
                        lon = lon/(1e10);

                           cout<< "lon = "<<fixed<<setprecision(10)<< lon <<endl;


            //#################################################循环完成之后就归为
                                for(int i=1; i<11; i++)
                            {
                                buffer_rx_precent[i][0] = 0;
                                n_rx_precent[i] = 0;
                            }
                            n_tx_current_flag = -1;
            //#################################################循环完成之后就归为
            
            //---------------------------------复制发布到ros-----------------------------------------------------

            need_to_send_gps.latitude = lat;
            need_to_send_gps.longitude = lon;
            gps_pub.publish(need_to_send_gps);
















        loop_rate.sleep();
    }//ros::ok

    //关闭串口
    sp.close();

    return 0;
}





















//##################################################            其他函数         ############################################################//
void show_connectedDevice()
{
    //显示已连接模块数量和编号
    cout << "already connected " << dec << connected_Count << " devices, Num: " ;//<< hex << (Connected_Num & 0xFFFF) << endl;
    int temp_t = 1;
    while (temp_t <15)
    {   //cout << "hex" << hex << (0x0001 << temp_t) << endl;
        if ( (Connected_Num & (0x0001 << temp_t)) == (0x0001 << temp_t))
        {
            cout << temp_t << ", ";
        }
        temp_t = temp_t + 1;
    }
    cout << endl;
}

