#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "dobot/SetPTPCmd.h"
#include "dobot/SetEndEffectorSuctionCup.h"
#include "geometry_msgs/PointStamped.h"
#include <unistd.h>
#include "dobot/SetPTPJumpParams.h"
#include <boost/thread.hpp>
#include <string>
#include <sstream>
#include "std_msgs/String.h"
#include "eaitool.h"

#include "apriltag_ros/TargetAprilTag.h"

dobot::SetPTPCmd TargetDobotPoseSrv;
dobot::SetPTPCmd releaseDobotPoseSrv;
using namespace std;
ros::Publisher g_back_pub;

double height_target= -40.0;  //-40.0;
double height_release=-34.193069458;
double height_robotSuckup=-38.193069458;

double dobot_min_y=-305.0; //毫米
double offset;
double offset_array[2]={0};

bool targetIsVaild=false;

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
    SetPTPCmdSrv.request.z = 134;
    SetPTPCmdSrv.request.r = -30;
	if (SetPTPCmdClient.call(SetPTPCmdSrv)) 
    {
        ROS_INFO("Result:%d", SetPTPCmdSrv.response.result);
    } else 
    {
        ROS_ERROR("Failed to call JumpParams");
    }
    
}

//吸取物体
void suckupObject()
{
    ros::NodeHandle n;
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

void SuckDownObject()
{
    ros::NodeHandle n;
    ros::ServiceClient SuctionCupClient;
    SuctionCupClient = n.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
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

//移动到目标物体位置
void GoToTargetLocation()
{
    ros::NodeHandle n;
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
    //sleep(3);
}

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
    SetPTPCmdSrv.request.z = TargetDobotPoseSrv.request.z+20;
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
    SetPTPCmdSrv.request.y = 228.527862549;
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

#if 0
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


#if 0
void GoToReleaseLocation()
{
    ros::NodeHandle n;
    ros::ServiceClient SetPTPCmdClient;
    SetPTPCmdClient = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd SetPTPCmdSrv;
    SetPTPCmdSrv.request.ptpMode = 1;
    SetPTPCmdSrv.request.x = -0;
    SetPTPCmdSrv.request.y = -296;
    SetPTPCmdSrv.request.z = -45.0;
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
        targetIsVaild=true;
        TargetDobotPoseSrv.request.x=(msg.point.x)*1000;
        TargetDobotPoseSrv.request.y=(msg.point.y)*1000;
        //TargetDobotPoseSrv.request.z=-50.0;
        TargetDobotPoseSrv.request.z=height_target; //-40.0;
        TargetDobotPoseSrv.request.r=-150;    
    }
#if 0
    TargetDobotPoseSrv.request.x=(msg.point.x)*1000;
    TargetDobotPoseSrv.request.y=(msg.point.y)*1000;
    //TargetDobotPoseSrv.request.z=-50.0;
    TargetDobotPoseSrv.request.z=height_target; //-40.0;
    TargetDobotPoseSrv.request.r=-150;
#endif

}

bool GrabObject()
{
        ROS_WARN("start to suck");
        //ros::NodeHandle n;
        GoObserverPose();
        sleep(5);
        //判断目标点是否有效，为0.0,0.0 时无效
    	if(!targetIsVaild)
        {
            ROS_WARN("can not look target,go home ");
            GoHome();
            return false;

        }
        
        double* dist_array=targetPointIsTooFar();
        stringstream ss;
        std_msgs::String msg_str;
        if(dist_array[0] == 1.0) //先转动特定角度，然后在判断距离
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
        //获取二维码的位置

        GoToTargetLocation();

        sleep(4);
        //气泵吸取
        suckupObject();
        
         sleep(2);
         
         //在抓取到目标后，向上移动 2cm
        GoToTargetLocation2();
        //移动到另外一个位置
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

void executeGrabCallback(const std_msgs::Int32& msg)
{    
   
    if(msg.data ==1)
    {
        int count=0;
        bool result=GrabObject();
    	if(!result)
        {
            return;
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
        ROS_WARN("start to suck");
        //ros::NodeHandle n;
        GoObserverPose();
        sleep(5);
        //判断目标点是否有效，为0.0,0.0 时无效
    	if(!targetIsVaild)
        {
            ROS_WARN("can not look target,go home ");
            GoHome();
            //continue;
            return;
            ROS_WARN("can not execute it ");
        }
        
        double* dist_array=targetPointIsTooFar();
        stringstream ss;
        std_msgs::String msg_str;
        if(dist_array[0] == 1.0) //先转动特定角度，然后在判断距离
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
        //获取二维码的位置

        GoToTargetLocation();

        sleep(4);
        //气泵吸取
        suckupObject();
        
         sleep(2);
         
         //在抓取到目标后，向上移动 2cm
        GoToTargetLocation2();
        //移动到另外一个位置
        GoHome();
        sleep(2);
        
    	//把物体放到机器人托盘上
        releaseInRobot();
        sleep(5);
        releaseObject();
        
        //移动到另外一个位置释放
        GoHome();
        sleep(2);
#endif        
        
    }
    else
    {

    	if(!targetIsVaild)
        {
            ROS_WARN("222can not look target,go home ");
            //GoHome();
            //continue;
            return;
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
    
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"execute_grab");
    ros::NodeHandle nh;
    ros::Rate loop(1);
    
    g_back_pub =nh.advertise<std_msgs::String>("/check",5);
    ros::Subscriber target_point_sub=nh.subscribe("/target_to_dobot_point",5,targetPointCallback);
    ros::Subscriber release_point_sub=nh.subscribe("/release_tag_to_dobot_point",1,releasePointCallback);
    ros::Subscriber grab_sub=nh.subscribe("/move_to_grab/grab",5,executeGrabCallback);
    loop.sleep();
    
    //为了让两个订阅器同时执行
    ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
    spinner.start();
    ros::waitForShutdown(); 
    return 0;
    
}
