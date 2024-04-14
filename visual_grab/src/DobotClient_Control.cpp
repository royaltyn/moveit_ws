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

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_U 0x75
#define KEYCODE_H 0x68

#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b

#define KEYCODE_O 0x6f
#define KEYCODE_P 0x70
#define KEYCODE_L 0x6c

#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33

//extend void GoHome();

int kfd = 0;
struct termios cooked, raw;

double height_target= -40.0;  //-40.0;
double height_release=-34.193069458;
double height_robotSuckup=-38.193069458;

double dobot_min_y=-305.0; //毫米
double offset;
double offset_array[2]={0};

ros::Publisher g_back_pub;

double error_incorret_x_1=-0.035;
double error_incorret_y_1=0.03;
double error_incorret_x_2=-0.01;
double error_incorret_y_2=0.04;
double error_incorret_x_3=0.015;
double error_incorret_y_3=0.033;
double error_incorret_x_4=-0.005;
double error_incorret_y_4=0.03;

dobot::SetPTPCmd TargetDobotPoseSrv;
dobot::SetPTPCmd releaseDobotPoseSrv;
bool targetIsVaild=false;
double std_obs_height=135;
double obs_height=135;       //135;

double Observer_hight_offset_x;
double Observer_hight_offset_y;

using namespace std;
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
    SetPTPCmdSrv.request.x = -2.2;
    SetPTPCmdSrv.request.y = 258;
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

void releaseInRobot()
{

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
    SetPTPCmdSrv.request.x = 3.32064580917;
    SetPTPCmdSrv.request.y = 208.527862549;  //228.527862549;
    //SetPTPCmdSrv.request.z = -53.193069458;
    SetPTPCmdSrv.request.z = height_release; //-34.193069458;
    SetPTPCmdSrv.request.r = -10;
	if (SetPTPCmdClient.call(SetPTPCmdSrv)) 
    {
        ROS_INFO("Result:%d", SetPTPCmdSrv.response.result);
    } else 
    {
        ROS_ERROR("Failed to call JumpParams");
    }
    
}

void GoObserverPose()
{
    ROS_WARN("start go observer pose");
    ros::NodeHandle nh;
   
    ros::ServiceClient SetPTPCmdClient;
    SetPTPCmdClient = nh.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd SetPTPCmdSrv;
    SetPTPCmdSrv.request.ptpMode = 1;
    SetPTPCmdSrv.request.x = 0;
    SetPTPCmdSrv.request.y = -256;
    SetPTPCmdSrv.request.z = obs_height;   //默认 134;
    SetPTPCmdSrv.request.r = -30;
	if (SetPTPCmdClient.call(SetPTPCmdSrv)) 
    {
        ROS_INFO("Result:%d", SetPTPCmdSrv.response.result);
    } else 
    {
        ROS_ERROR("Failed to call JumpParams");
    }
    
}

void  GetPose(dobot::SetPTPCmd& SetPoseSrv)
{
    ros::NodeHandle gh;
    ros::ServiceClient GetPoseClient;
    GetPoseClient = gh.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
    dobot::GetPose GetPoseSrv;
    if (GetPoseClient.call(GetPoseSrv)) 
    {
        ROS_INFO("Result:%d", GetPoseSrv.response.result);
    } else 
    {
        ROS_ERROR("Failed to call JumpParams");
    }
    

    SetPoseSrv.request.ptpMode=0;
    SetPoseSrv.request.x= GetPoseSrv.response.x;
    SetPoseSrv.request.y= GetPoseSrv.response.y;
    SetPoseSrv.request.z= GetPoseSrv.response.z;
    SetPoseSrv.request.r= GetPoseSrv.response.r;
   
}


//void targetPointCallback(const geometry_msgs::PointStamped& msg)
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
        double offset_x= (std_obs_height-obs_height)*Observer_hight_offset_x; //单位毫米
        double offset_y= (std_obs_height-obs_height)*Observer_hight_offset_y;
        TargetDobotPoseSrv.request.x=(msg.point.x)*1000-offset_x; 
        TargetDobotPoseSrv.request.y=(msg.point.y)*1000-offset_y;
#endif        
        targetIsVaild=true;
        //TargetDobotPoseSrv.request.x=(msg.point.x)*1000;//发个机械臂参数单位为毫米
        //TargetDobotPoseSrv.request.y=(msg.point.y)*1000;

        //TargetDobotPoseSrv.request.z=-50.0;
        TargetDobotPoseSrv.request.z=height_target; //-40.0;
        TargetDobotPoseSrv.request.r=-150;    
    }

/*    
    ROS_WARN("error_incorret_x_1=%lf,error_incorret_y_1=%lf, error_incorret_x_2=%lf,error_incorret_y_2=%lf,\
              error_incorret_x_3=%lf,error_incorret_y_3=%lf, error_incorret_x_4=%lf,error_incorret_y_4=%lf",\
              error_incorret_x_1,error_incorret_y_1,error_incorret_x_2,error_incorret_y_2,error_incorret_x_3,\
              error_incorret_y_3,error_incorret_x_4,error_incorret_y_4
              );
*/
}


//检查二维码与机械臂底座中心的位置，如果x偏差太大，那么转动一定的角度，缩小偏差。如果y偏差太大，那么y调整，缩小偏差。
//param:TargetDobotPoseSrv.x(y,z):二维码相对机械臂底座中心的坐标，为绝对植
//param:TargetDobotPoseSrv.request 机械臂请求，发出去的
double* targetPointIsTooFar()
{   //得到的坐标单位为毫米，因为机械臂接口需要毫米单位
    double sqrt_target_xy=sqrt(TargetDobotPoseSrv.request.x*TargetDobotPoseSrv.request.x+ \
                          TargetDobotPoseSrv.request.y*TargetDobotPoseSrv.request.y) ;
    double tmp_x=TargetDobotPoseSrv.request.x/1.4265;            
    double tmp_y=TargetDobotPoseSrv.request.y;                       
    
    //ROS_WARN("to dobot x =%lf , y= %lf",TargetDobotPoseSrv.request.x,TargetDobotPoseSrv.request.y);
    double robot_to_dobot_dist=197.0;
    //在x(左右)方向误差超过10cm时，先旋转

    if(sqrt_target_xy > fabs(dobot_min_y) || fabs(tmp_x)>100.0 )
    {        
        double thea=atan2(tmp_x,fabs(tmp_y)+robot_to_dobot_dist);        
        double yaw = thea;
        
        ROS_WARN("start turn %lf",yaw);
        //ROS_WARN("thea =%lf , yaw= %lf",thea,yaw);
        offset_array[0]=1.0;
        offset_array[1]=yaw;
        return offset_array;
    } //在x(左右)方向误差比较小时，直接y方向后退
    else if(TargetDobotPoseSrv.request.y < dobot_min_y)//y方向偏差大
    {        
        //让机械臂调整到290 毫米距离
        offset=(tmp_y-(dobot_min_y-25))/1000.0;  //求出偏差的距离，单位为m
        
        ROS_WARN("distance too far,is %lf < -305, dobot can not go it,start go back %lf",tmp_y,offset);
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

//获取释放物体的位置
//void releasePointCallback(const geometry_msgs::PointStamped& msg)
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

double* releasePointIsTooFar()
{
    
    double sqrt_release_xy=sqrt(releaseDobotPoseSrv.request.x*releaseDobotPoseSrv.request.x+ \
                          releaseDobotPoseSrv.request.y*releaseDobotPoseSrv.request.y) ;
    double tmp_x=releaseDobotPoseSrv.request.x/1.4265;            
    double tmp_y=releaseDobotPoseSrv.request.y;                       
    
    ROS_WARN("to dobot x =%lf , y= %lf, sqrt_release_xy=%lf ",releaseDobotPoseSrv.request.x,releaseDobotPoseSrv.request.y,sqrt_release_xy);
    double robot_to_dobot_dist=197.0;
    //在x(左右)方向误差超过10cm时，先旋转

    if(sqrt_release_xy > fabs(dobot_min_y) || fabs(tmp_x)>(100.0/1.4265) )
    {        
        double thea=atan2(tmp_x,fabs(tmp_y)+robot_to_dobot_dist);        
        double yaw = thea;
        
        ROS_WARN("start turn %lf",yaw);
        //ROS_WARN("thea =%lf , yaw= %lf",thea,yaw);
        offset_array[0]=1.0;
        offset_array[1]=yaw;
        return offset_array;
    } //在x(左右)方向误差比较小时，直接y方向后退
    else if(releaseDobotPoseSrv.request.y < dobot_min_y)//y方向偏差大
    {        
        //让机械臂调整到290 毫米距离
        offset=(tmp_y-(dobot_min_y-25))/1000.0;  //求出偏差的距离，单位为m
        
        ROS_WARN("distance too far,is %lf < -305, dobot can not go it,start go back %lf",tmp_y,offset);
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


dobot::SetPTPCmd StartPoseSrv,TargetPoseSrv;


//在抓取到目标后，向上移动 2cm
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

#if 0  //这里是先去抓取东西然后去释放
void GotoRobotSuckup()
{
    ros::NodeHandle n;
    ros::ServiceClient SetPTPCmdClient;
    SetPTPCmdClient = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd SetPTPCmdSrv;
    SetPTPCmdSrv.request.ptpMode = 1;
    SetPTPCmdSrv.request.x = 3.32064580917;
    SetPTPCmdSrv.request.y = 228.527862549;
    //SetPTPCmdSrv.request.z = -57.193069458;
    SetPTPCmdSrv.request.z = height_robotSuckup; //-38.193069458;
    SetPTPCmdSrv.request.r = -10;                                  
    if (SetPTPCmdClient.call(SetPTPCmdSrv)) 
    {
        ROS_INFO("Result:%d", SetPTPCmdSrv.response.result);
    } else 
    {
        ROS_ERROR("Failed to go target pose");
    }

}
#endif
#if 1
void GotoRobotSuckup()
{
    ros::NodeHandle n;
    ros::ServiceClient SetPTPCmdClient;
    SetPTPCmdClient = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd SetPTPCmdSrv;
    SetPTPCmdSrv.request.ptpMode = 1;
    SetPTPCmdSrv.request.x = TargetDobotPoseSrv.request.x;
    SetPTPCmdSrv.request.y = TargetDobotPoseSrv.request.y;
    //SetPTPCmdSrv.request.z = -57.193069458;
    SetPTPCmdSrv.request.z = height_robotSuckup; //-38.193069458;
    SetPTPCmdSrv.request.r = -10;                                  
    if (SetPTPCmdClient.call(SetPTPCmdSrv)) 
    {
        ROS_INFO("Result:%d", SetPTPCmdSrv.response.result);
    } else 
    {
        ROS_ERROR("Failed to go target pose");
    }

}
#endif

bool GrabObject()
{
  //先去观察位置，观察获取二维码位置
    GoObserverPose();
    sleep(5);
    
    if(!targetIsVaild)
    {
        ROS_WARN("can not look target,go home ");
        GoHome();
        //continue;
        //如果没有目标，在回到home 后，结束
        return false;
    }
    
    double* dist_array=targetPointIsTooFar();
    stringstream ss;
    std_msgs::String msg_str;
    if(dist_array[0] == 1.0)
    {
        ROS_WARN("offset angule is %lf",dist_array[1]);
        //发现物体太远，地盘转动特定的角度
        double yaw_angule=(dist_array[1]/3.14159)*180 ;
        ROS_WARN("yaw_angule=%lf ",yaw_angule);
        ss<<"angule true "<<yaw_angule<<" 0.2 2";
        //ss<<"angule true "<<dist_array[1]<<" 0.2 2";
        msg_str.data=ss.str();
        g_back_pub.publish(msg_str);
        //ROS_WARN("start to go back");
        sleep(5); 
        
        //转完后，再判断臂展是否够长，能够抓到,如果不够再后退
        ROS_WARN("y=%lf, dobot_min_y=%lf ",TargetDobotPoseSrv.request.y,dobot_min_y);
        if(TargetDobotPoseSrv.request.y < dobot_min_y)
        {   //约大约28cm 就后退
            double y_offset=(TargetDobotPoseSrv.request.y-dobot_min_y-25)/1000.0;
            ROS_WARN("continue to go back %lf",y_offset);
            stringstream ss2;
            std_msgs::String msg_str2;
            ss2<<"line true "<<y_offset<<" 0.2 0.01";
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
    //返回观察二维码的位置
    GoObserverPose();

    //移动到另外一个位置释放
    GoHome();
    sleep(2);
    //把物体放到机器人托盘上
    releaseInRobot();
    sleep(5);
    releaseObject();
    
    //移动到另外一个位置释放
    GoHome();
    sleep(2);
    return true;
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
	    // SetJOGCmdService
        ros::ServiceClient client;
        client = n.serviceClient<dobot::SetJOGCmd>("/DobotServer/SetJOGCmd");
        dobot::SetJOGCmd srv;
        srv.request.isJoint = false;
        
        ros::ServiceClient GripperClient;
        GripperClient = n.serviceClient<dobot::SetEndEffectorGripper>("/DobotServer/SetEndEffectorGripper");
        dobot::SetEndEffectorGripper GripperSrv;
        
    	ros::ServiceClient JumpParams;
        JumpParams = n.serviceClient<dobot::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
        dobot::SetPTPJumpParams JumpParamsSrv;        
        
        ros::ServiceClient SuctionCupClient;
        SuctionCupClient = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
        dobot::SetEndEffectorSuctionCup SuctionCupSrv;
        
    
	    if(c == KEYCODE_W)
	    {
	        ROS_INFO("W");
	        srv.request.cmd = 1;
	        if (client.call(srv)) 
	        {
	            ROS_INFO("Result:%d", srv.response.result);
            } else 
            {
                ROS_ERROR("Failed to call SetJOGCmd");
            }
	    }
	    else if(c == KEYCODE_S)
	    {
	        ROS_INFO("S");
	        srv.request.cmd = 2;
	        if (client.call(srv)) 
	        {
	            ROS_INFO("Result:%d", srv.response.result);
            	} else 
            	{
                	ROS_ERROR("Failed to call SetJOGCmd");
            	}
	    }
	    else if(c == KEYCODE_A)
	    {
	        ROS_INFO("A");
	        srv.request.cmd = 3;
	        if (client.call(srv)) 
	        {
	            ROS_INFO("Result:%d", srv.response.result);
            	} else 
            	{
                	ROS_ERROR("Failed to call SetJOGCmd");
            	}	
	    }
	    else if(c == KEYCODE_D)
	    {
	        ROS_INFO("D");
	        srv.request.cmd = 4;
	        if (client.call(srv)) 
	        {
	            ROS_INFO("Result:%d", srv.response.result);
            	} else 
            	{
                    ROS_ERROR("Failed to call SetJOGCmd");
            	}	
	    }
	    else if(c == KEYCODE_U)
	    {
	        ROS_INFO("U");
	        srv.request.cmd = 5;
	        if (client.call(srv)) 
	        {
	            ROS_INFO("Result:%d", srv.response.result);
            	} else 
            	{
                    ROS_ERROR("Failed to call SetJOGCmd");
            	}	
	    }
            else if(c == KEYCODE_H)
	    {
		ROS_INFO("H");
                GoHome();	
	    }
	    else if(c == KEYCODE_I)
	    {
	        ROS_INFO("I");
	        srv.request.cmd = 6;
	        if (client.call(srv)) 
	        {
	            ROS_INFO("Result:%d", srv.response.result);
                } else 
                {
                    ROS_ERROR("Failed to call SetJOGCmd");
                }	
	    }
	    else if(c == KEYCODE_J)
	    {
	        ROS_INFO("J");
	        srv.request.cmd = 7;
	        if (client.call(srv)) 
	        {
	            ROS_INFO("Result:%d", srv.response.result);
                } else 
                {
                    ROS_ERROR("Failed to call SetJOGCmd");
                }		
	    }
	    else if(c == KEYCODE_K)
	    {
	        ROS_INFO("K");
	        srv.request.cmd = 8;
	        if (client.call(srv)) 
	        {
	            ROS_INFO("Result:%d", srv.response.result);
                } else 
                {
                    ROS_ERROR("Failed to call SetJOGCmd");
                }	
	    }
	    //1键识别物体，并吸取起来
	    else if(c == KEYCODE_1)
	    { 
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
	        
#if 0	    
	      //先去观察位置，观察获取二维码位置
	        GoObserverPose();
	        sleep(5);
	        
	        if(!targetIsVaild)
	        {
	            ROS_WARN("can not look target,go home ");
	            GoHome();
	            continue;
	            ROS_WARN("can not execute it ");
	        }
	        
	        double* dist_array=targetPointIsTooFar();
	        stringstream ss;
	        std_msgs::String msg_str;
	        if(dist_array[0] == 1.0)
	        {
	            ROS_WARN("offset angule is %lf",dist_array[1]);
	            //发现物体太远，地盘转动特定的角度
	            double yaw_angule=(dist_array[1]/3.14159)*180 ;
	            ROS_WARN("yaw_angule=%lf ",yaw_angule);
	            ss<<"angule true "<<yaw_angule<<" 0.2 2";
	            //ss<<"angule true "<<dist_array[1]<<" 0.2 2";
	            msg_str.data=ss.str();
	            g_back_pub.publish(msg_str);
	            //ROS_WARN("start to go back");
	            sleep(5); 
	            
	            //转完后，再判断臂展是否够长，能够抓到,如果不够再后退
	            ROS_WARN("y=%lf, dobot_min_y=%lf ",TargetDobotPoseSrv.request.y,dobot_min_y);
	            if(TargetDobotPoseSrv.request.y < dobot_min_y)
	            {   //约大约28cm 就后退
	                double y_offset=(TargetDobotPoseSrv.request.y-dobot_min_y-25)/1000.0;
	                ROS_WARN("continue to go back %lf",y_offset);
	                stringstream ss2;
	                std_msgs::String msg_str2;
	                ss2<<"line true "<<y_offset<<" 0.2 0.01";
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
	        //返回观察二维码的位置
	        GoObserverPose();
		
	        //移动到另外一个位置释放
	        GoHome();
	        sleep(2);
	        //把物体放到机器人托盘上
	        releaseInRobot();
	        sleep(5);
	        releaseObject();
	        
	        //移动到另外一个位置释放
	        GoHome();
#endif	        
	        

	    }
	    //2键，移动机械臂到目标位置，释放物体
	    else if(c == KEYCODE_2)
	    {   
	        sleep(2);
	        if(!targetIsVaild)
	        {
	            ROS_WARN("222can not look target,go home ");
	            //GoHome();
	            continue;
	            ROS_WARN("222can not execute it ");
	        }
	        //重置状态
	        targetIsVaild=false;
	        
	        //先到机器人平台上
	        GotoRobotSuckup();
	        sleep(3);
	        //把物体吸取起来
	        suckupObject();
	        sleep(2);
	        
	        GoHome();
	        
	        //返回观察二维码的位置
	        GoObserverPose();
	        sleep(5);	        
	        
	        //判断机械臂是否能到达释放位置，如果不行进行简单异常处理
	        double* dist_array=releasePointIsTooFar();
	        stringstream ss;
	        std_msgs::String msg_str;
	        if(dist_array[0] == 1.0)
	        {
	            ROS_WARN("offset angule is %lf",dist_array[1]);
	            //发现物体太远，地盘转动特定的角度
	            double yaw_angule=(dist_array[1]/3.14159)*180 ;
	            ROS_WARN("yaw_angule=%lf ",yaw_angule);
	            ss<<"angule true "<<yaw_angule<<" 0.2 2";
	            //ss<<"angule true "<<dist_array[1]<<" 0.2 2";
	            msg_str.data=ss.str();
	            g_back_pub.publish(msg_str);
	            //ROS_WARN("start to go back");
	            sleep(5); 
	            
	            //转完后，再判断臂展是否够长，能够抓到,如果不够再后退
	            ROS_WARN("y=%lf, dobot_min_y=%lf ",releaseDobotPoseSrv.request.y,dobot_min_y);
	            if(releaseDobotPoseSrv.request.y < dobot_min_y)
	            {   //约大于28cm 就后退
	                double y_offset=(releaseDobotPoseSrv.request.y-dobot_min_y-25)/1000.0;
	                ROS_WARN("continue to go back %lf",y_offset);
	                stringstream ss2;
	                std_msgs::String msg_str2;
	                ss2<<"line true "<<y_offset<<" 0.2 0.01";
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
	        #if 0
	        if(dist != 0.0 )
	        {
	            ROS_WARN("offset distance is %lf",dist);
	            //发现物体太远，先移动底盘到达合适的距离
	            stringstream ss;
	            std_msgs::String msg_str;
	            ss<<"line true "<<dist<<" 0.2 0.01";
	            msg_str.data=ss.str();
	            g_back_pub.publish(msg_str);
	            ROS_WARN("start to go back");
	            sleep(4); 
	            
	        }
	        #endif
	        
	        //获取释放二维码坐标,并去到释放坐标上
	        GetAndGoToReleasePose();	        
	        sleep(4);
	        
	        //释放二维码
	        releaseObject();
	        
	        GoObserverPose();
             
	        sleep(2);
	        //移动到另外一个位置释放
	        GoHome();
	    }

	    else if(c == KEYCODE_O)
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

void callback(dobot::DobotControlConfig &config)
 { 
    ROS_WARN("***Reconfigure Request: %lf %lf %lf %lf %lf %lf %lf %lf %lf", 
                                    config.hight_to_grab,
                                    config.case_one_x,config.case_one_y, 
                                    config.case_two_x,config.case_two_y, 
                                    config.case_three_x,config.case_three_y,
                                    config.case_four_x,config.case_four_y
                                    ); 
    error_incorret_x_1=config.case_one_x;
    error_incorret_y_1=config.case_one_y;
    
    error_incorret_x_2=config.case_two_x;
    error_incorret_y_2=config.case_two_y;
    
    error_incorret_x_3=config.case_three_x;
    error_incorret_y_3=config.case_three_y;
    
    error_incorret_x_4=config.case_four_x;
    error_incorret_y_4=config.case_four_y;
        
 }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient");
    ros::NodeHandle n;
    
    bool isRegistered=bEAIHadAuthed();
    if(!isRegistered)
    {
        ROS_ERROR("robot is not registered, can not run visual grab");
        return -1;
    }
    
    n.param<double>("Observer_hight_offset_x", Observer_hight_offset_x, -0.16107);
    n.param<double>("Observer_hight_offset_y", Observer_hight_offset_y, 0.1616);

    dynamic_reconfigure::Server<dobot::DobotControlConfig> server;
    dynamic_reconfigure::Server<dobot::DobotControlConfig>::CallbackType f;

    f = boost::bind(&callback, _1);
    server.setCallback(f);
    
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

