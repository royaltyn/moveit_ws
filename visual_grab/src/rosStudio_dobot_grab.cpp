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
#include "visual_grab/RosStudioDobotGrabMsg.h"
#include "visual_grab/RosStudioDobotGrabResultMsg.h"
#include "geometry_msgs/PoseStamped.h"
#include "eaitool.h"
#include "apriltag_ros/TargetAprilTag.h"

visual_grab::RosStudioDobotGrabResultMsg grab_result;

int excute_id;
geometry_msgs::PoseStamped targetDobotPose,releaseDobotPose,targetDobotPoseUp5, tmpPose;
int targetId=0;
bool targetIsVaild=false;

double offset;
double offset_array[2]={0};
double dobot_min_y=-305.0; //毫米
ros::Publisher g_back_pub,grab_result_pub, grab_targetId_pub;

float target_height=-40.0;

double std_obs_height=135;
double obs_height=135;       //观察位置高度;

double Observer_hight_offset_x;
double Observer_hight_offset_y;
double release_offset_x;
double release_offset_y;

using namespace std;
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
//气泵释放物体
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

/*让机械臂去到指定的目标点
*ObserverPose :(x,y,z,r)=(0,-256,134,-30)
* 
*/
void dobotGoToPose(geometry_msgs::PoseStamped targetPose)
{
    ROS_WARN("start go observer pose");
    ros::NodeHandle nh;
   
    ros::ServiceClient SetPTPCmdClient;
    SetPTPCmdClient = nh.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot::SetPTPCmd SetPTPCmdSrv;
    SetPTPCmdSrv.request.ptpMode = 1;
    SetPTPCmdSrv.request.x = targetPose.pose.position.x; 
    SetPTPCmdSrv.request.y = targetPose.pose.position.y; 
    SetPTPCmdSrv.request.z = targetPose.pose.position.z; 
    SetPTPCmdSrv.request.r = -20;
	if (SetPTPCmdClient.call(SetPTPCmdSrv)) 
    {
        ROS_INFO("Result:%d", SetPTPCmdSrv.response.result);
    } else 
    {
        ROS_ERROR("Failed to call JumpParams");
    }
    
}

//获取释放物体的位置
//void releasePointCallback(const geometry_msgs::PointStamped& msg)
void releasePointCallback(const apriltag_ros::TargetAprilTag& data)
{
    geometry_msgs::PointStamped msg = data.pose;
    if(msg.point.x==0.0 && msg.point.y==0.0)
    {
        releaseDobotPose.pose.position.x=0.0; 
        releaseDobotPose.pose.position.y=0.0;   
    }
    else
    {
        releaseDobotPose.pose.position.x=(msg.point.x)*1000; 
        releaseDobotPose.pose.position.y=(msg.point.y)*1000;
        releaseDobotPose.pose.position.z=-25;    
    }
}

double* releasePointIsTooFar()
{
    
    double sqrt_release_xy=sqrt(releaseDobotPose.pose.position.x*releaseDobotPose.pose.position.x+ \
                          releaseDobotPose.pose.position.y*releaseDobotPose.pose.position.y) ;
    double tmp_x=releaseDobotPose.pose.position.x/1.4265;            
    double tmp_y=releaseDobotPose.pose.position.y;                       
    
    ROS_WARN("to dobot x =%lf , y= %lf, sqrt_release_xy=%lf ",releaseDobotPose.pose.position.x,releaseDobotPose.pose.position.y,sqrt_release_xy);
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
    else if(releaseDobotPose.pose.position.y < dobot_min_y)//y方向偏差大
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

//获取抓取物体的位置
//void targetPointCallback(const geometry_msgs::PointStamped& msg)
void targetPointCallback(const apriltag_ros::TargetAprilTag& data)
{
    geometry_msgs::PointStamped msg = data.pose;
    targetId=data.id;
    if(msg.point.x==0.0 && msg.point.y==0.0)
    {
        //ROS_WARN("target point is 0.0, set targetIsVaild false");
        targetIsVaild=false;
        targetDobotPose.pose.position.x=0.0;
        targetDobotPose.pose.position.y=0.0;
    }
    else
    {
        targetIsVaild=true;
        targetDobotPose.pose.position.x=(msg.point.x)*1000;
        targetDobotPose.pose.position.y=(msg.point.y)*1000;
        targetDobotPose.pose.position.z= target_height; //-40.0;
    }
}


//检查二维码与机械臂底座中心的位置，如果x偏差太大，那么转动一定的角度，缩小偏差。如果y偏差太大，那么y调整，缩小偏差。
//param:TargetDobotPoseSrv.x(y,z):二维码相对机械臂底座中心的坐标，为绝对植
//param:TargetDobotPoseSrv.request 机械臂请求，发出去的
double* targetPointIsTooFar()
{   //得到的坐标单位为毫米，因为机械臂接口需要毫米单位
    double sqrt_target_xy=sqrt(targetDobotPose.pose.position.x*targetDobotPose.pose.position.x+ \
                          targetDobotPose.pose.position.y*targetDobotPose.pose.position.y) ;
    double tmp_x=targetDobotPose.pose.position.x/1.4265;            
    double tmp_y=targetDobotPose.pose.position.y;                       
    
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
    else if(targetDobotPose.pose.position.y < dobot_min_y)//y方向偏差大
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

/*机械臂在观察位看到目标后，根据机械臂是否能够到达目标位置，来调整机器人位置
*
*/
void robotAdjustPosition(double* dist_array,bool dobot_action_type)
{
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
        if(dobot_action_type) //机械臂抓取
        {
            ROS_WARN("y=%lf, dobot_min_y=%lf ",targetDobotPose.pose.position.y,dobot_min_y);
            if(targetDobotPose.pose.position.y < dobot_min_y)
            {   //约大约28cm 就后退
                double y_offset=(targetDobotPose.pose.position.y-dobot_min_y-25)/1000.0;
                ROS_WARN("continue to go back %lf",y_offset);
                stringstream ss2;
                std_msgs::String msg_str2;
                ss2<<"line true "<<y_offset<<" 0.2 0.01";
                msg_str2.data=ss2.str();
                g_back_pub.publish(msg_str2);
                sleep(4);
            }
        }
        else //机械臂是释放
        {
           //转完后，再判断臂展是否够长，能够抓到,如果不够再后退
            ROS_WARN("y=%lf, dobot_min_y=%lf ",releaseDobotPose.pose.position.y,dobot_min_y);
            if(releaseDobotPose.pose.position.y < dobot_min_y)
            {   //约大于28cm 就后退
                double y_offset=(releaseDobotPose.pose.position.y-dobot_min_y-25)/1000.0;
                ROS_WARN("continue to go back %lf",y_offset);
                stringstream ss2;
                std_msgs::String msg_str2;
                ss2<<"line true "<<y_offset<<" 0.2 0.01";
                msg_str2.data=ss2.str();
                g_back_pub.publish(msg_str2);
                sleep(4);
            }
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
}

bool GrabObject(const visual_grab::RosStudioDobotGrabMsg& msg)
{
//表示去观察位，true就去，false不去
    if(msg.dobot_action1.data)
    {
        obs_height=msg.observerPose.pose.position.z;
        dobotGoToPose(msg.observerPose);
        sleep(5);
    }
    else
    {
        //返回结果[1,0]
        std_msgs::Int32 data1,data2;
        data1.data=1;
        data2.data=0;
        grab_result.dobot_excute_result.push_back(data1);
        grab_result.dobot_excute_result.push_back(data2);
        //grab_result_pub.publish(grab_result);
        sleep(2);
        return false;
    }
    
    //表示去目标点位置(通过摄像头实际获取得到，如果没有就是0,0),并且气泵吸取，并上移到目标点位置5cm
    if(msg.dobot_action2.data)
    {   //判断目标点坐标是否为0.0, 0.0
        if(!targetIsVaild)
        {
            ROS_WARN("can not target ");
            //go home
            dobotGoToPose(msg.HomePose);
            //返回结果[2,0]
            std_msgs::Int32 data1,data2;
            data1.data=2;
            data2.data=0;
            grab_result.dobot_excute_result.push_back(data1);
            grab_result.dobot_excute_result.push_back(data2);

            //grab_result_pub.publish(grab_result);
            sleep(2);
            return false;
        }
        else
        {
            //获取目标点距离，判断机械臂是否能够到达，如果太远，进行调整
            double* dist_array=targetPointIsTooFar();
            //调整位置机器人位置，让机械臂能够到达目标位置
            robotAdjustPosition(dist_array,msg.dobot_action_type.data); //里面最大延时10s
            
            targetDobotPoseUp5.pose.position.x=targetDobotPose.pose.position.x;
            targetDobotPoseUp5.pose.position.y=targetDobotPose.pose.position.y;
            targetDobotPoseUp5.pose.position.z=targetDobotPose.pose.position.x+50;
            
            //纠正因摄像头倾斜，观察位置高度引起的误差
            double offset_x= (std_obs_height-obs_height)*Observer_hight_offset_x; //单位毫米
            double offset_y= (std_obs_height-obs_height)*Observer_hight_offset_y;
            targetDobotPose.pose.position.x -=offset_x;
            targetDobotPose.pose.position.y -=offset_y;
            
            //机械臂去目标(二维码)位置
            dobotGoToPose(targetDobotPose);
            sleep(4);
            
            //气泵吸取
            suckupObject();
            sleep(2);
            
            //让机械臂从目标位置上移动5cm
            //targetDobotPose.pose.position.z=targetDobotPose.pose.position.z+50; //50毫米
            //dobotGoToPose(targetDobotPoseUp5);
            ROS_WARN("will go to observerpose");
            dobotGoToPose(msg.observerPose);  
            sleep(3);              
        }
        
    }
    else
    {
        //go home
        dobotGoToPose(msg.HomePose);
        //返回结果[2,0]
        std_msgs::Int32 data1,data2;
        data1.data=2;
        data2.data=0;
        grab_result.dobot_excute_result.push_back(data1);
        grab_result.dobot_excute_result.push_back(data2);

        //grab_result_pub.publish(grab_result);
        sleep(2);
        return false;
    }
    
    //机械臂抓取到目标后， go home
    if(msg.dobot_action3.data)
    {
        //让机械臂 go home
        dobotGoToPose(msg.HomePose);
        sleep(3);
    }
    else
    {
        
        //返回结果[3,0]
        std_msgs::Int32 data1,data2;
        data1.data=3;
        data2.data=0;
        grab_result.dobot_excute_result.push_back(data1);
        grab_result.dobot_excute_result.push_back(data2);

        //grab_result_pub.publish(grab_result);
        sleep(2);
        return false;
       
    }
    
    //机械臂去robotReleasePose(机器人上的释放点，固定)，并且气泵释放
    if(msg.dobot_action4.data)
    {
        //让机械臂去机器人平台释放位置
        dobotGoToPose(msg.robotReleasePose);
        sleep(3);
        
        //气泵释放物体
        releaseObject();
    }
    else
    {
       
        //返回结果[4,0]
        std_msgs::Int32 data1,data2;
        data1.data=4;
        data2.data=0;
        grab_result.dobot_excute_result.push_back(data1);
        grab_result.dobot_excute_result.push_back(data2);

        //grab_result_pub.publish(grab_result);
        sleep(2);
        return false;
    }
    
    if(msg.dobot_action5.data)
    {
        //最后go home
        dobotGoToPose(msg.HomePose);
    }
    else
    {

        //返回结果[5,0]
        std_msgs::Int32 data1,data2;
        data1.data=5;
        data2.data=0;
        grab_result.dobot_excute_result.push_back(data1);
        grab_result.dobot_excute_result.push_back(data2);

        //grab_result_pub.publish(grab_result);
        sleep(2);
        return false;
    }

    //吸取物体操作全部成功，返回[1,1]
    std_msgs::Int32 data1,data2;
    data1.data=1;
    data2.data=1;
    grab_result.dobot_excute_result.push_back(data1);
    grab_result.dobot_excute_result.push_back(data2);

    //grab_result_pub.publish(grab_result);
    sleep(2);
    return true;
}

int64_t nowTime, delet_time;
void executeGrabCallback(const visual_grab::RosStudioDobotGrabMsg& msg)
{   
     ROS_WARN("in  executeGrabCallback");
     
     std_msgs::Int32 targetIdMsg;
     //判断一下指令时间，防止执行旧的缓存指令
     nowTime=ros::Time::now().toSec();
     delet_time=nowTime - msg.header.stamp.sec;
     //ROS_WARN("now_time=%lu,msg_time=%lu,so delet_time=%lu",nowTime,msg.header.stamp.sec,delet_time);
     if(delet_time > 10)
     {
        ROS_WARN("==========data is too old");
        return;
     }
     excute_id=msg.id.data;
     grab_result.id.data=msg.id.data;
     target_height=msg.target_height.data;
     
    //判断是要吸取物体，还是要释放物体，true 吸取，false 释放
    if(msg.dobot_action_type.data) 
    {  
        int count=0;
        bool result=GrabObject(msg);
        //表明没看到目标二维码，直接结束，返回结果
    	if(!result)
        {   //失败返回目标id 为0
            targetIdMsg.data=0;
            grab_targetId_pub.publish(targetIdMsg);
            
            grab_result_pub.publish(grab_result);
            return;
        }
        //表明第一次抓取看到二维码，并去抓了，但可能没抓成功，需要再判断，然后重新再抓
        while(count<2)
        {
            count++;
            ROS_WARN("start while grab,%d",count);	        
            if(!targetIsVaild)
            {
                ROS_WARN("not grab target, grab again ");
                result=GrabObject(msg);
                //表明没看到目标二维码，直接结束，返回结果
            	if(!result)
                {
                    grab_result_pub.publish(grab_result);
                    break;
                }
                continue;
            }
            else //表明抓成功了，在前面托盘上看到有二维码，直接跳出循环，不再抓取
            {
                //再次发布成功抓取结果
                grab_result_pub.publish(grab_result);
                break;
            }
        }
        //表明重复抓取都失败了，返回失败结果
        if(count >=2)
        {
            targetIdMsg.data=0;
            grab_targetId_pub.publish(targetIdMsg);
            grab_result_pub.publish(grab_result);
            
        }// 否则成功后返回抓取二维码的id 号
        else
        {
            
            targetIdMsg.data=targetId;
            grab_targetId_pub.publish(targetIdMsg);
        }

    }
    //释放物体流程
    else 
    {
        if(!targetIsVaild)
        {
            ROS_WARN("222can not look target,go home ");
            //返回结果[1,0]
            std_msgs::Int32 data1,data2;
            data1.data=1;
            data2.data=0;
            grab_result.dobot_excute_result.push_back(data1);
            grab_result.dobot_excute_result.push_back(data2);

            grab_result_pub.publish(grab_result);
            sleep(2);
            return;
        }
        //机械臂去robotReleasePose，吸取物体
        if(msg.dobot_action1.data)
        {
            dobotGoToPose(msg.robotReleasePose);
            sleep(3);
            //气泵吸取
            suckupObject();
            sleep(2);
        }
        else
        {

            //返回结果[1,0]
            std_msgs::Int32 data1,data2;
            data1.data=1;
            data2.data=0;
            grab_result.dobot_excute_result.push_back(data1);
            grab_result.dobot_excute_result.push_back(data2);

            grab_result_pub.publish(grab_result);
            sleep(2);
            return ;
        } 
        
        //dobot_action2 表示去Home点，true就去，false不去
        if(msg.dobot_action2.data)
        {
            dobotGoToPose(msg.HomePose);
            sleep(1);
        }
        else
        {

            //返回结果[2,0]
            std_msgs::Int32 data1,data2;
            data1.data=2;
            data2.data=0;
            grab_result.dobot_excute_result.push_back(data1);
            grab_result.dobot_excute_result.push_back(data2);

            grab_result_pub.publish(grab_result);
            sleep(2);
            return ;
        } 
        //表示去观察位，true就去，false不去，
        if(msg.dobot_action3.data)
        {
            dobotGoToPose(msg.observerPose);
            sleep(5);
        }
        else
        {

            //返回结果[3,0]
            std_msgs::Int32 data1,data2;
            data1.data=3;
            data2.data=0;
            grab_result.dobot_excute_result.push_back(data1);
            grab_result.dobot_excute_result.push_back(data2);

            grab_result_pub.publish(grab_result);
            sleep(2);
            return ;
        }
        
        //表示去释放位(靠摄像头看到6号二维码)，并且气泵释放，再上移动到释放位5cm
        if(msg.dobot_action4.data)
        {
            //获取释放位置，看是否有6号二维码
            double* dist_array=releasePointIsTooFar();
            //调整位置机器人位置，让机械臂能够到达目标位置
            robotAdjustPosition(dist_array,msg.dobot_action_type.data);           
            //去释放位置
            ROS_WARN("releaseDobotPose,(%lf,%lf,%lf)",releaseDobotPose.pose.position.x,\
                      releaseDobotPose.pose.position.y,releaseDobotPose.pose.position.z);
            releaseDobotPose.pose.position.x=releaseDobotPose.pose.position.x + release_offset_x*1000;
            releaseDobotPose.pose.position.y=releaseDobotPose.pose.position.y - release_offset_y*1000;

            dobotGoToPose(releaseDobotPose);
            sleep(4);
            //释放二维码
            releaseObject();
            sleep(2);           
        }
        else 
        {

            //返回结果[4,0]
            std_msgs::Int32 data1,data2;
            data1.data=4;
            data2.data=0;
            grab_result.dobot_excute_result.push_back(data1);
            grab_result.dobot_excute_result.push_back(data2);

            grab_result_pub.publish(grab_result);
            sleep(2);
            return ;
        } 
        
        //表示去Home点，true就去，false不去
        if(msg.dobot_action5.data)
        {
            //先返回观察位置
            dobotGoToPose(msg.observerPose);
            sleep(2);
            //go home
            dobotGoToPose(msg.HomePose);
        }
        else
        {

            //返回结果[5,0]
            std_msgs::Int32 data1,data2;
            data1.data=5;
            data2.data=0;
            grab_result.dobot_excute_result.push_back(data1);
            grab_result.dobot_excute_result.push_back(data2);

            grab_result_pub.publish(grab_result);
            sleep(2);
            return ;
        }
     
        //释放动作全部成功，返回[1,1]
        std_msgs::Int32 data1,data2;
        data1.data=1;
        data2.data=1;
        grab_result.dobot_excute_result.push_back(data1);
        grab_result.dobot_excute_result.push_back(data2);

        grab_result_pub.publish(grab_result);
        sleep(2);
        return ;       
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"rosStudio_dobot_grab");
    ros::NodeHandle nh;
    
    bool isRegistered=bEAIHadAuthed();
    if(!isRegistered)
    {
        ROS_ERROR("robot is not registered, can not run visual grab");
        return -1;
    }
    
    nh.param<double>("Observer_hight_offset_x", Observer_hight_offset_x, -0.16107);
    nh.param<double>("Observer_hight_offset_y", Observer_hight_offset_y, 0.1616);
    
    nh.param<double>("release_offset_x", release_offset_x, 0.0);
    nh.param<double>("release_offset_y", release_offset_y, 0.0);
    
    ros::Rate loop(1);
    
    g_back_pub =nh.advertise<std_msgs::String>("/check",5);
    ros::Subscriber target_point_sub=nh.subscribe("/target_to_dobot_point",1,targetPointCallback);
    ros::Subscriber release_point_sub=nh.subscribe("/release_tag_to_dobot_point",1,releasePointCallback);
    ros::Subscriber grab_sub=nh.subscribe("/RosStudioDobotGrab",1,executeGrabCallback);
    
    grab_result_pub =nh.advertise<visual_grab::RosStudioDobotGrabResultMsg>("/RosStudioDobotGrabResult",5);
    grab_targetId_pub =nh.advertise<std_msgs::Int32>("/GrabTargetId",5);
    loop.sleep();
    
    //为了让两个订阅器同时执行
    ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
    spinner.start();
    ros::waitForShutdown(); 
    return 0;
    
}
