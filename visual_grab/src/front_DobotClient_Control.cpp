#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PointStamped.h"
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetJOGCmd.h"
#include "dobot/SetEndEffectorGripper.h"
#include "dobot/GetEndEffectorGripper.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/SetPTPCmd.h"
#include "dobot/GetPose.h"
#include "dobot/SetEndEffectorSuctionCup.h"
#include "dobot/GetEndEffectorSuctionCup.h"

#include "dobot/SetHOMECmd.h"

#include <dynamic_reconfigure/server.h>
#include "dobot/DobotControlConfig.h" 


#include <cstdlib>

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include "eaitool.h"
#include "apriltag_ros/TargetAprilTag.h"

#define KEYCODE_O 0x6f
#define KEYCODE_H 0x68

#define KEYCODE_1 0x31
#define KEYCODE_2 0x32

int kfd = 0;
struct termios cooked, raw;

dobot::SetPTPCmd TargetDobotPoseSrv;
dobot::SetPTPCmd releaseDobotPoseSrv;
bool targetIsVaild=false;
double std_obs_height=135;
double obs_height=135;       //135;

double height_target= -45.0;  //-40.0;
double height_release=-34.193069458;
double height_robotSuckup=-38.193069458;

double dobot_min_x=280.0; //毫米
double offset;
double offset_array[2]={0};

double Observer_hight_offset_x;
double Observer_hight_offset_y;

ros::Publisher g_back_pub;

using namespace std;

void targetPointCallback(const apriltag_ros::TargetAprilTag& data)
{
    geometry_msgs::PointStamped msg = data.pose;
    float target_x;
    TargetDobotPoseSrv.request.ptpMode=0;
    if(msg.point.x==0.0 && msg.point.y==0.0)
    {
        //ROS_WARN("target point is 0.0, set targetIsVaild false");
        targetIsVaild=false;
    }
    else
    {
        //如果摄像头左右倾斜，经测试，观察高度每下降1毫米，x轴偏移-0.16107毫米，y轴偏移0.16107毫米
        //该参数每个摄像头都不一样！
#if 1   
        //这里可能还需要对机械臂x,y 坐标的正负进行判断，来进行加减处理     
        //double offset_x= (std_obs_height-obs_height)*Observer_hight_offset_x; //单位毫米
        //double offset_y= (std_obs_height-obs_height)*Observer_hight_offset_y;
        //TargetDobotPoseSrv.request.x=(msg.point.x)*1000-offset_x; 
        //TargetDobotPoseSrv.request.y=(msg.point.y)*1000-offset_y;
        
        TargetDobotPoseSrv.request.x=(msg.point.x)*1000; 
        TargetDobotPoseSrv.request.y=(msg.point.y)*1000;
#endif        
        targetIsVaild=true;
        TargetDobotPoseSrv.request.z=height_target; //-40.0;
        TargetDobotPoseSrv.request.r=-150;    
    }

}

//获取释放物体的位置
void releasePointCallback(const apriltag_ros::TargetAprilTag& data)
{
    geometry_msgs::PointStamped msg = data.pose;
    //ROS_WARN("get release pose");
    releaseDobotPoseSrv.request.ptpMode=1;
    
    releaseDobotPoseSrv.request.x=(msg.point.x)*1000;
    releaseDobotPoseSrv.request.y=(msg.point.y)*1000;

    releaseDobotPoseSrv.request.z=-25; //-40.0;
    releaseDobotPoseSrv.request.r=40.0;   
}

void GoToTargetLocation2()
{
    ros::NodeHandle n;
    ros::ServiceClient SetPTPCmdClient;
    SetPTPCmdClient = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd SetPTPCmdSrv;
    SetPTPCmdSrv.request.ptpMode = 1;
    SetPTPCmdSrv.request.x = TargetDobotPoseSrv.request.x;
    SetPTPCmdSrv.request.y = TargetDobotPoseSrv.request.y;
    SetPTPCmdSrv.request.z = TargetDobotPoseSrv.request.z+30;
    SetPTPCmdSrv.request.r = -10;                                   
    if (SetPTPCmdClient.call(SetPTPCmdSrv)) 
    {
        ROS_INFO("Result:%d", SetPTPCmdSrv.response.result);
    } else 
    {
        ROS_ERROR("Failed to go target pose");
    }
    //sleep(3);
}

void GoHome()
{
    ROS_WARN("start go home");
    ros::NodeHandle nh;
    ros::ServiceClient JumpParams;
    JumpParams = nh.serviceClient<dobot::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
    dobot::SetPTPJumpParams JumpParamsSrv;
    JumpParamsSrv.request.jumpHeight = 20.0;
    JumpParamsSrv.request.jumpHeight = 200.0;
	if (JumpParams.call(JumpParamsSrv)) 
    {
        ROS_INFO("Result:%d", JumpParamsSrv.response.result);
    } else 
    {
        ROS_ERROR("Failed to call JumpParams");
    }
    
    ros::ServiceClient SetPTPCmdClient;
    SetPTPCmdClient = nh.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd SetPTPCmdSrv;
    SetPTPCmdSrv.request.ptpMode = 1;
    SetPTPCmdSrv.request.x = 258;
    SetPTPCmdSrv.request.y = 0.0;
    SetPTPCmdSrv.request.z = 121;
    SetPTPCmdSrv.request.r = -10;
	if (SetPTPCmdClient.call(SetPTPCmdSrv)) 
    {
        ROS_INFO("Result:%d", SetPTPCmdSrv.response.result);
    } else 
    {
        ROS_ERROR("Failed to call JumpParams");
    }
    
}

//因为是机械臂放前面了，可能转动方向等需要不大一样，需要调试修改
//检查二维码与机械臂底座中心的位置，如果x偏差太大，那么转动一定的角度，缩小偏差。如果y偏差太大，那么y调整，缩小偏差。
//param:TargetDobotPoseSrv.x(y,z):二维码相对机械臂底座中心的坐标，为绝对植
//param:TargetDobotPoseSrv.request 机械臂请求，发出去的
double* targetPointIsTooFar()
{   //得到的坐标单位为毫米，因为机械臂接口需要毫米单位
    double sqrt_target_xy=sqrt(TargetDobotPoseSrv.request.x*TargetDobotPoseSrv.request.x+ \
                          TargetDobotPoseSrv.request.y*TargetDobotPoseSrv.request.y) ;
    //double tmp_x=TargetDobotPoseSrv.request.x/1.4265;
    double tmp_x=TargetDobotPoseSrv.request.x;           
    double tmp_y=TargetDobotPoseSrv.request.y;    //单位是毫米                   
    
    //ROS_WARN("to dobot x =%lf , y= %lf",TargetDobotPoseSrv.request.x,TargetDobotPoseSrv.request.y);
    //double robot_to_dobot_dist=197.0;
    //在y(左右)方向误差超过10cm时，先旋转

    if(sqrt_target_xy > fabs(dobot_min_x) || fabs(tmp_y)>100.0 )
    {        
        //double thea=atan2(tmp_y,fabs(tmp_x)+robot_to_dobot_dist); 
        double thea=atan2(tmp_y,fabs(tmp_x));             
        double yaw = thea;
        
        ROS_WARN("start turn %lf",yaw);
        ROS_WARN("thea =%lf , yaw= %lf",thea,yaw);
        offset_array[0]=1.0;
        offset_array[1]=yaw;
        return offset_array;
    } //在y(左右)方向误差比较小时，直接x方向前进
    else if(fabs(TargetDobotPoseSrv.request.x) > dobot_min_x)//x方向偏差大
    {        
        //让机械臂调整到290 毫米距离
        offset=(tmp_x-dobot_min_x)/1000.0;  //求出偏差的距离，单位为m
        
        ROS_WARN("distance too far,is %lf > 305, dobot can not go it,start  forward %lf",tmp_x,offset);
        offset_array[0]=2.0;
        offset_array[1]=offset;
        return offset_array;
        //return offset;
    }
    else//正常
    {
        offset_array[0]=0.0;
        offset_array[1]=0.0;
        return offset_array;
    }
}

void GoToTargetPoint()
{
    ros::NodeHandle n;
    //获取二维码的位置
    ROS_WARN("START get apriltag pose ");
    //ros::Subscriber target_point_sub=n.subscribe("/target_to_dobot_point",1,targetPointCallback);
    //让机械臂移动到二维码位置，并吸取
    ROS_WARN("****target_point=(%f,%f,%f)",TargetDobotPoseSrv.request.x,TargetDobotPoseSrv.request.y, \
                                       TargetDobotPoseSrv.request.z);
    ros::ServiceClient SetPTPCmdClient;
    SetPTPCmdClient = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd SetPTPCmdSrv;
    SetPTPCmdSrv.request.ptpMode = 1;
    SetPTPCmdSrv.request.x = TargetDobotPoseSrv.request.x;
    SetPTPCmdSrv.request.y = TargetDobotPoseSrv.request.y;
    SetPTPCmdSrv.request.z = TargetDobotPoseSrv.request.z;
    SetPTPCmdSrv.request.r = -10;                                   
    if (SetPTPCmdClient.call(SetPTPCmdSrv)) 
    {
        ROS_INFO("Result:%d", SetPTPCmdSrv.response.result);
    } else 
    {
        ROS_ERROR("Failed to go target pose");
    }
    
    //sleep(5);
}

void suckupObject()
{
    ros::NodeHandle n;
    //气泵吸取
    ros::ServiceClient SuctionCupClient;
    SuctionCupClient = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
    dobot::SetEndEffectorSuctionCup SuctionCupSrv;
    SuctionCupSrv.request.enableCtrl = 1;
    SuctionCupSrv.request.suck = 1;
     
    if (SuctionCupClient.call(SuctionCupSrv)) 
    {
       ROS_INFO("Result:%d", SuctionCupSrv.response.result);
    } 
    else 
    {
       ROS_ERROR("Failed to call Set SuctionCupSrv");
    }
}

bool GrabObject()
{
  //先去观察位置，观察获取二维码位置
    GoHome();
    sleep(2);
    
    if(!targetIsVaild)
    {
        ROS_WARN("can not look target,go home ");
        GoHome();
        //continue;
        //如果没有目标，在回到home 后，结束
        return false;
    }
    
    //判断目标离机械臂距离，如果过远，就移动走近
    double* dist_array=targetPointIsTooFar();
    stringstream ss;
    std_msgs::String msg_str;
    if(dist_array[0] == 1.0)
    {
        ROS_WARN("offset angule is %lf",dist_array[1]);
        //发现物体太远，底盘转动特定的角度
        double yaw_angule=(dist_array[1]/3.14159)*180 ;
        ROS_WARN("yaw_angule=%lf ",yaw_angule);
        ss<<"angule true "<<yaw_angule<<" 0.3 2";
        //ss<<"angule true "<<dist_array[1]<<" 0.2 2";
        msg_str.data=ss.str();
        g_back_pub.publish(msg_str);
        //ROS_WARN("start to go back");
        sleep(5); 
        
        //转完后，再判断臂展是否够长，能够抓到,如果不够再后退
        ROS_WARN("x=%lf, dobot_min_x=%lf ",TargetDobotPoseSrv.request.x,dobot_min_x);
        if(TargetDobotPoseSrv.request.x > dobot_min_x)
        {   //约大约28cm 就后退
            double x_offset=(TargetDobotPoseSrv.request.x-dobot_min_x)/1000.0;
            ROS_WARN("continue to forward %lf",x_offset);
            stringstream ss2;
            std_msgs::String msg_str2;
            ss2<<"line true "<<x_offset<<" 0.2 0.01";
            msg_str2.data=ss2.str();
            g_back_pub.publish(msg_str2);
            sleep(4);
        }
    }
    else if(dist_array[0] == 2.0 )
    {
        ROS_WARN("offset distance is %lf",dist_array[1]);
        //发现物体太远，先移动底盘到达合适的距离
        ss<<"line true "<<dist_array[1]<<" 0.2 0.01";
        msg_str.data=ss.str();
        g_back_pub.publish(msg_str);
        ROS_WARN("start to forward");
        sleep(6); 
        
    }	        
    //重置一下状态
    targetIsVaild=false;
    //获取二维码的位置,并移动到哪里
    GoToTargetPoint();
    sleep(4);
    //吸取物体
    suckupObject();
    sleep(2);     
    //先往上移动2cm
    GoToTargetLocation2();

    //抓取后，回到home 位置
    GoHome();
    sleep(2);

    return true;
}


double* releasePointIsTooFar()
{
    
    double sqrt_release_xy=sqrt(releaseDobotPoseSrv.request.x*releaseDobotPoseSrv.request.x+ \
                          releaseDobotPoseSrv.request.y*releaseDobotPoseSrv.request.y) ;
    //double tmp_x=releaseDobotPoseSrv.request.x/1.4265; 
    double tmp_x=releaseDobotPoseSrv.request.x;             
    double tmp_y=releaseDobotPoseSrv.request.y;                       
    
    ROS_WARN("to dobot x =%lf , y= %lf, sqrt_release_xy=%lf ",releaseDobotPoseSrv.request.x,releaseDobotPoseSrv.request.y,sqrt_release_xy);
    //double robot_to_dobot_dist=197.0;
    //在x(左右)方向误差超过10cm时，先旋转

    //if(sqrt_release_xy > fabs(dobot_min_y) || fabs(tmp_x)>(100.0/1.4265) )
    if(sqrt_release_xy > fabs(dobot_min_x) || fabs(tmp_y)>(100.0) )
    {        
        //double thea=atan2(tmp_x,fabs(tmp_y)+robot_to_dobot_dist);
        double thea=atan2(tmp_y,fabs(tmp_x));          
        double yaw = thea;
        
        ROS_WARN("start turn %lf",yaw);
        //ROS_WARN("thea =%lf , yaw= %lf",thea,yaw);
        offset_array[0]=1.0;
        offset_array[1]=yaw;
        return offset_array;
    } //在x(左右)方向误差比较小时，直接y方向后退
    else if(releaseDobotPoseSrv.request.x > dobot_min_x)//x方向偏差大
    {        
        //让机械臂调整到290 毫米距离
        //offset=(tmp_y-(dobot_min_y-25))/1000.0;  //求出偏差的距离，单位为m
        offset=(tmp_x-dobot_min_x)/1000.0;  //求出偏差的距离，单位为m
        ROS_WARN("distance too far,is %lf > 305 mm, dobot can not go it,start forward %lf",tmp_y,offset);
        offset_array[0]=2.0;
        offset_array[1]=offset;
        return offset_array;
        //return offset;
    }
    else//正常
    {
        offset_array[0]=0.0;
        offset_array[1]=0.0;
        return offset_array;
    }
    
}

//移动到释放物体位置
void GetAndGoToReleasePose()
{

        ROS_WARN("start go to release pose");
        ros::NodeHandle n;
        ros::ServiceClient SetPTPCmdClient;
        SetPTPCmdClient = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
        dobot::SetPTPCmd SetPTPCmdSrv;
        SetPTPCmdSrv.request.ptpMode = 1;
        SetPTPCmdSrv.request.x = releaseDobotPoseSrv.request.x;
        SetPTPCmdSrv.request.y = releaseDobotPoseSrv.request.y;

        SetPTPCmdSrv.request.z = releaseDobotPoseSrv.request.z;
        SetPTPCmdSrv.request.r = releaseDobotPoseSrv.request.r;                                   
        if (SetPTPCmdClient.call(SetPTPCmdSrv)) 
        {
            ROS_INFO("Result:%d", SetPTPCmdSrv.response.result);
        } else 
        {
            ROS_ERROR("Failed to go target pose");
        }

}

void releaseObject()
{
    ros::NodeHandle nh;
    //气泵释放
    ros::ServiceClient SuctionCupClient;
    SuctionCupClient = nh.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
    dobot::SetEndEffectorSuctionCup SuctionCupSrv;
    SuctionCupSrv.request.enableCtrl = 0;
    SuctionCupSrv.request.suck = 0;
     
    if (SuctionCupClient.call(SuctionCupSrv)) 
    {
       ROS_INFO("Result:%d", SuctionCupSrv.response.result);
    } 
    else 
    {
       ROS_ERROR("Failed to call Set SuctionCupSrv");
    }
     
}

void keyboardLoop(ros::NodeHandle &n)
{
    unsigned char c;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    
    ros::Subscriber target_point_sub=n.subscribe("/target_to_dobot_point",1,targetPointCallback);
    ros::Subscriber release_point_sub=n.subscribe("/release_tag_to_dobot_point",1,releasePointCallback);
    
    for(;;)
    {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;

        if ((num = poll(&ufd, 1, 250)) < 0) {
            perror("poll():");
            return;
        } else if(num > 0) {
            if(read(kfd, &c, 1) < 0) {
                perror("read():");
                return;
            }
        } else {
            continue;
        }
        
        ros::ServiceClient client;
        client = n.serviceClient<dobot::SetJOGCmd>("/DobotServer/SetJOGCmd");
        dobot::SetJOGCmd srv;
        srv.request.isJoint = false;
        
        if(c == KEYCODE_O)
	    {
	         ROS_INFO("O");
            //机械臂位置归0
            ROS_WARN("dobot start set 0");
            ros::ServiceClient SetHomeClient;
            SetHomeClient = n.serviceClient<dobot::SetHOMECmd>("/DobotServer/SetHOMECmd");
            dobot::SetHOMECmd SetHomeSrv;

            if (SetHomeClient.call(SetHomeSrv) == false) {
                ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
                //return -1;
            }
	    }
	    else if(c == KEYCODE_H)
	    {
	        ROS_INFO("H");
            GoHome();	
	    }
	    else if(c == KEYCODE_1)
	    {
	        ROS_WARN("111111111111111");
	        int count=0;
	        //开始抓取目标二维码
	        bool result=GrabObject();
	        if(!result)
	        {
	            continue;
	            ROS_WARN("****can not excute it");
	        }	
	        //再次观察判断是否有二维码被放下，如果没有重新抓取，最多再尝试2次
	        while(count<2)
	        {
	            count++;
	            ROS_WARN("start while grab,%d",count);	        
	            if(!targetIsVaild)
	            {
	                ROS_WARN("not grab target, grab again ");
                    GrabObject();
                    continue;
	            }
	            else //如果看到有二维码，直接跳出循环，不再抓取
	            {
	                break;
	            }
	        }        
	        
	    }
	    //2键，移动机械臂到目标位置，释放物体
	    else if(c == KEYCODE_2)
	    {
	        //返回home 位，并观察二维码的位置
	        GoHome();
	        //判断机械臂是否能到达释放位置，如果不行进行简单异常处理
	        double* dist_array=releasePointIsTooFar();
	        stringstream ss;
	        std_msgs::String msg_str;
	        if(dist_array[0] == 1.0)
	        {
	            ROS_WARN("offset angule is %lf",dist_array[1]);
	            //发现物体太远，地盘转动特定的角度
	            double yaw_angule=(dist_array[1]/3.14159)*180;
	            ROS_WARN("yaw_angule=%lf ",yaw_angule);
	            ss<<"angule true "<<yaw_angule<<" 0.2 2";
	            //ss<<"angule true "<<dist_array[1]<<" 0.2 2";
	            msg_str.data=ss.str();
	            g_back_pub.publish(msg_str);
	            //ROS_WARN("start to go back");
	            sleep(5); 
	            
	            //转完后，再判断臂展是否够长，能够抓到,如果不够再后退
	            ROS_WARN("x=%lf, dobot_min_x=%lf ",releaseDobotPoseSrv.request.x,dobot_min_x);
	            if(releaseDobotPoseSrv.request.x > dobot_min_x)
	            {   //约大于28cm 就后退
	                double x_offset=(releaseDobotPoseSrv.request.x-dobot_min_x)/1000.0;
	                ROS_WARN("continue to forward %lf",x_offset);
	                stringstream ss2;
	                std_msgs::String msg_str2;
	                ss2<<"line true "<<x_offset<<" 0.2 0.01";
	                msg_str2.data=ss2.str();
	                g_back_pub.publish(msg_str2);
	                sleep(4);
	            }
	            
	        }
	        else if(dist_array[0] == 2.0 )
	        {
	            ROS_WARN("offset distance is %lf",dist_array[1]);
	            //发现物体太远，先移动底盘到达合适的距离
	            ss<<"line true "<<dist_array[1]<<" 0.2 0.01";
	            msg_str.data=ss.str();
	            g_back_pub.publish(msg_str);
	            ROS_WARN("start to go back");
	            sleep(6); 
	            
	        }
	        
	        //获取释放二维码坐标,并去到释放坐标上
	        GetAndGoToReleasePose();	        
	        sleep(4);
	        
	        //释放二维码
	        releaseObject();
	        
	        //回到home位
	        GoHome();
	        
	    }
	    else
	    {
	        ROS_INFO("DEFAULT:0x%02x", c);
            srv.request.cmd = 0;
            if (client.call(srv)) 
	        {
	            ROS_INFO("Result:%d", srv.response.result);
            } else 
            {
                ROS_ERROR("Failed to call SetJOGCmd");
            }            
	    } 
        
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "front_DobotClient_Control");
    ros::NodeHandle n;
#if 0    
    bool isRegistered=bEAIHadAuthed();
    if(!isRegistered)
    {
        ROS_ERROR("robot is not registered, can not run visual grab");
        return -1;
    }
#endif 

#if 0   
    n.param<double>("Observer_hight_offset_x", Observer_hight_offset_x, -0.16107);
    n.param<double>("Observer_hight_offset_y", Observer_hight_offset_y, 0.1616);

    dynamic_reconfigure::Server<dobot::DobotControlConfig> server;
    dynamic_reconfigure::Server<dobot::DobotControlConfig>::CallbackType f;

    f = boost::bind(&callback, _1);
    server.setCallback(f);
    
    g_back_pub =n.advertise<std_msgs::String>("/check",5);
#endif    

    g_back_pub =n.advertise<std_msgs::String>("/check",5);
    // SetCmdTimeout
    ros::ServiceClient client;

    client = n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot::SetCmdTimeout srv;
    srv.request.timeout = 3000;
    if (client.call(srv) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }

    boost::thread t = boost::thread(boost::bind(&keyboardLoop, boost::ref(n)));
    
    //ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
    //spinner.start();
    //ros::waitForShutdown();
    ros::spin();
    //ros::spinOnce();
    t.interrupt();
    t.join();

    tcsetattr(kfd, TCSANOW, &cooked);
    return 0;
}















