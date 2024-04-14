#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PointStamped.h>
#include <visual_grab/LocalPointMsg.h> 
#include<fstream>
#include <cstring>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
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
#include <sstream>

#include "apriltag_ros/TargetAprilTag.h" 

using namespace std;

bool targetIsVaild=false;
bool haveGetTarget=false;
geometry_msgs::PoseStamped targetDobotPose ;

//去掉首部空格
string& ClearHeadSpace(string &str)   
{  
    if (str.empty())   
    {  
        return str;  
    }  
 
    str.erase(0,str.find_first_not_of(" "));  
    //str.erase(str.find_last_not_of(" ") + 1);  
    return str;  
} 

std::string& replace_all_distinct(std::string&  str,const std::string&   old_value,const std::string& new_value)
{
    for(std::string::size_type   pos(0);   pos!=std::string::npos;   pos+=new_value.length())
    {
        if((pos=str.find(old_value,pos))!=std::string::npos)
        {
            str.replace(pos,old_value.length(),new_value);
        }
        else
        {
            break;
        }
    }
    return  str;
}
/*
作用： 以字符delim 分割字符串s
参数:delim  子字符，如'\n'
    s 从文件中读出来的字符串
*/
std::vector<std::string> StringSplit2(const  std::string& s, const std::string& delim)
{
    std::vector<std::string> elems;
    size_t pos = 0;
    size_t len = s.length();
    size_t delim_len = delim.length();
    if (delim_len == 0) return elems;
    while (pos < len)
    {
        int find_pos = s.find(delim, pos);
        if (find_pos < 0)
        {
            elems.push_back(s.substr(pos, len - pos));
            break;
        }
        elems.push_back(s.substr(pos, find_pos - pos));
        pos = find_pos + delim_len;
    }
    return elems;
}

string ReadFile(string& filePath)
{
    string strData="";
    const char * p8File = (char*)filePath.c_str();

    FILE *fp;
    fp = fopen(p8File,"r");
    if(fp == NULL)
    {
      cout<<"cannot read file, open error"<<endl;
      return "false";
    }

    fseek(fp,0,SEEK_END);//将文件内部的指针指向文件末尾
    long lsize=ftell(fp);//获取文件长度，（得到文件位置指针当前位置相对于文件首的偏移字节数）
    rewind(fp);//将文件内部的指针重新指向一个流的开头
    //fclose(fp);

    char *pData= new char[lsize+1];
    memset(pData,0,lsize+1);//将内存空间都赋值为‘\0’

    int result=fread(pData,1,lsize,fp);//将pfile中内容读入pread指向内存中
    fclose(fp);
    strData = std::string(pData);

    if(pData)
    {
        delete[] pData;
        pData = NULL;
    }
    return strData;
}

bool FindandgetPose(string& strData,string& name,geometry_msgs::PoseStamped& grabPose)
{
    string strLine="";
    std::string oneLine="";
    
    // 以\n 分行
    std::vector<std::string> strList = StringSplit2(strData,   "\n");
    bool hasFind=false;
    int i=0;
    for(;i<strList.size();i++)
    {
        //去掉字符串头部空格，然后返回     
        oneLine = ClearHeadSpace(strList[i]);
        oneLine = replace_all_distinct(oneLine,"\r", "");
        //如果是空行，直接跳过
        if(oneLine.size()<1)
        {
            cout<<"this is empty line,continue"<<endl;
            continue;
        }
        //如果找到
        if(oneLine.size() >= name.size() && oneLine.find(name) != std::string::npos)
        {
            //如果找到，有这样地域名的一样，就以= 号分割，将后面的位置信息取出来
            std::vector<std::string> strList2 = StringSplit2(oneLine,  "="); 
            //以“，”分割，将后面的位置信息取出来,准备给pose
            std::vector<std::string> strList3 = StringSplit2(strList2[1],  ","); 
            
            //grabPose.point.x = atof(strList3[0].c_str()); 
            //grabPose.point.y = atof(strList3[1].c_str()); 
            //grabPose.point.z = atof(strList3[2].c_str());
            grabPose.pose.position.x = atof(strList3[0].c_str()); 
            grabPose.pose.position.y = atof(strList3[1].c_str()); 
            grabPose.pose.position.z = atof(strList3[2].c_str());
            hasFind =true;
            return hasFind; 
        }
    }
    //如果在文件中没有找到这一行
    if( !hasFind )
    {
        grabPose.pose.position.x = 0.0; 
        grabPose.pose.position.y = 0.0; 
        grabPose.pose.position.z = 0.0;
        return false;
    }
}

//获取抓取物体的位置
void targetPointCallback(const apriltag_ros::TargetAprilTag& data)
{
    geometry_msgs::PointStamped msg = data.pose;
    targetId=data.id;
    int target_height=-40.0;
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
        haveGetTarget=true;
        targetDobotPose.pose.position.x=(msg.point.x)*1000;
        targetDobotPose.pose.position.y=(msg.point.y)*1000;
        targetDobotPose.pose.position.z= target_height; //-40.0;
    }
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

    //抓取后，回到home 位置
    GoHome();
    sleep(2);

    return true;
}


bool getGrabPose(string& filePath,geometry_msgs::PoseStamped& grabPose)
{
    string strData=ReadFile(filePath);
    string name="抓取点";
    
    bool result=FindandgetPose(strData,name,grabPose);
    //如果没有从文件中，获取到抓取点位置坐标，则返回false
    if(!result)
    {
        cout<<"can not get grab pose, return false"<<endl;
        return false
    }
    else
    {
        return true;
    }
}

bool getReleaseObservePose(string& filePath,geometry_msgs::PoseStamped& ReleaseObservePose)
{
    string strData=ReadFile(filePath);
    string name="释放观察点";
    
    bool result=FindandgetPose(strData,name,ReleaseObservePose);
    //如果没有从文件中，获取到抓取点位置坐标，则返回false
    if(!result)
    {
        cout<<"can not get grab pose, return false"<<endl;
        return false
    }
    else
    {
        return true;
    }
}

bool getReleasePose(string& filePath,geometry_msgs::PoseStamped& ReleasePose)
{
    string strData=ReadFile(filePath);
    string name="上海";
    
    bool result=FindandgetPose(strData,name,ReleasePose);
    //如果没有从文件中，获取到抓取点位置坐标，则返回false
    if(!result)
    {
        cout<<"can not get grab pose, return false"<<endl;
        return false
    }
    else
    {
        return true;
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

bool navigationToPose(const geometry_msgs::PoseStamped& Pose)
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ac.waitForServer();
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    geometry_msgs::Quaternion quaternion;
    quaternion.x = Pose.pose.orientation.x ;
    quaternion.y = Pose.pose.orientation.y;
    quaternion.z = Pose.pose.orientation.z ;
    quaternion.w = Pose.pose.orientation.w ;
    goal.target_pose.pose.position.x = Pose.pose.position.x ;
    goal.target_pose.pose.position.y = Pose.pose.position.y ;
    goal.target_pose.pose.position.z = Pose.pose.position.z ;
    goal.target_pose.pose.orientation = quaternion ;
    
    ac.sendGoal(goal);
    //ROS_WARN("***navigation_ptp_timeout=%d",navigation_ptp_timeout); 
    bool finished_before_timeout = ac.waitForResult(ros::Duration(60));
    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_WARN("**pose1 Action finished: %s",state.toString().c_str());
        if(state.toString() == "SUCCEEDED")
        {
            return true;
        }
        else
        {
            ROS_WARN("navigation error, state =%s",state.toString().c_str());
            return false;
        }                       
    
    }
    else
    {
        ROS_WARN("timeout,navigation error");
        return false;
    }
}


void getStartCommonCallback(const std_msgs::Int32& msg)
{
    if(msg.data == 1)
    {
        ROS_WARN("******* START **********");
        
        //从文件中获取抓取位置，并导航到该位置
        string filePath="/home/eaibot/moveit_ws/src/moveit/visual_grab/config/race.txt";
        geometry_msgs::PoseStamped grabPose ;
        
        bool result=getGrabPose(filePath,grabPose);
        if(!result)
        {
            ROS_WARN("getGrabPose error,can not get grab pose");
            return ;
        }
        //默认这个点的坐标系是/map ,如果不是，需要进行转换
        //
        bool navResult=navigationToPose(grabPose);
        //如果导航到抓取点失败，直接返回
        if(!navResult)
        {
            return;
        }
        
        //发指令，提示开始获取识别码(目标)位置
        //获取二维码(目标)坐标位置
        if(targetIsVaild)
        {
            bool grabResult=GrabObject();
            if(!result)
	        {
	            //continue;
	            ROS_WARN("****can not grab object ");
	            return;
	        }
        }
        else
        {
            ROS_WARN("can not look target object");
            return ;
        }
        
        //从文件中获取释放观察点位置，并导航到该位置
        
        //发指令，提示获取释放点坐标位置
        geometry_msgs::PoseStamped ReleaseObservePose;
        bool rst=getReleaseObservePose(filePath,ReleaseObservePose);
        if(!rst)
        {
            ROS_WARN("can not get ReleaseObservePose from file");
            return ;
        }
        
        bool ropResult=navigationToPose(ReleaseObservePose);
        //如果导航到释放观察点失败，直接返回
        if(!ropResult)
        {
            return;
        }
        //正常比赛情况，在释放观察点，通过摄像头识别"上海" 等字眼和距离，得到相对应释放的位置
        
        //在此为了方便测试，直接通过文件中读取到"上海" 相对应的位置
        geometry_msgs::PoseStamped ReleasePose;
        rst=getReleasePose(filePath,ReleasePose);
        if(!rst)
        {
            ROS_WARN("can not get ReleasePose from file");
            return ;
        }
        
        //释放抓取的二维码
        releaseObject();
        
    }
    else if(msg.data == 0)
    {
        ROS_WANR("******** STOP **********");
    }
}

int main(int argc, char**argv)
{
    ros::init(argc,argv,"front_execute_grab_release");
    ros::NodeHandle n;
    ros::Rate loop(1);
    
    //获取开始执行指令
    ros::Subscriber offset_save_sub=n.subscribe("/get_start_common",1,getStartCommonCallback);
    
    ros::Subscriber target_point_sub=nh.subscribe("/target_to_dobot_point",1,targetPointCallback);
    
    loop.sleep();
    ros::spin();
    return 0;
}




