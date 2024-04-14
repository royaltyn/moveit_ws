#include <ros/ros.h>
#include "dobot/SetPTPCmd.h"
#include "dobot/GetPose.h"
#include "geometry_msgs/PointStamped.h"
#include <math.h>
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include <iostream>
#include "eaitool.h"

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/client.h>
#include "visual_grab/CameralToDobotConfig.h"
#include "std_msgs/Bool.h"

#include<fstream>
#include <cstring>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

using namespace std;

geometry_msgs::PointStamped dobot_endeffector_point;

//测量到摄像头投影中心离机械臂末端x为2.0cm,y为3.5cm
//double cameral_distance_x=0.02;
//double cameral_distance_y=0.025;
double cameral_distance_x;
double cameral_distance_y;

//double cameral_distance=sqrt(cameral_distance_x*cameral_distance_x+\
                             cameral_distance_y*cameral_distance_y \
                             );
double cameral_distance=0.035;                             
//double cameral_distance=0.02; //相机距离机械臂末端x 轴距离0.0532
double cameral_to_dobot_frame_point_x;
double cameral_to_dobot_frame_point_y;
double thea;
//tf::TransformBroadcaster cameral_to_dobot_broadcaster;
//geometry_msgs::TransformStamped cameral_to_dobot_transform;
//geometry_msgs::Quaternion cameral_to_dobot_quaternion;


void  GetPose(geometry_msgs::PointStamped& dobot_endeffector_point)
{
    ros::NodeHandle gh;
    ros::ServiceClient GetPoseClient;
    GetPoseClient = gh.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
    dobot::GetPose GetPoseSrv;
    if (GetPoseClient.call(GetPoseSrv)) 
    {
        //ROS_INFO("Result:%d", GetPoseSrv.response.result);
    } else 
    {
        ROS_ERROR("Failed to call JumpParams");
    }
    
    dobot_endeffector_point.point.x= GetPoseSrv.response.x/1000.0;
    dobot_endeffector_point.point.y= GetPoseSrv.response.y/1000.0;
    dobot_endeffector_point.point.z= GetPoseSrv.response.z/1000.0;
}

void callback(visual_grab::CameralToDobotConfig &config)
{
    cameral_distance_x=config.cameral_distance_x;
    cameral_distance_y=config.cameral_distance_y;
    ROS_WARN("after dynamic_reconfigure,cameral_distance_x,Y=(%lf,%lf)",cameral_distance_x,cameral_distance_y);
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

string ReadYamlFile(string& filePath)
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

string ChangeYamlFile(string& strData,map<string,double>& modifyParamList)
{
    string strLine="";
    std::string oneLine="";
    // 以\n 分行
    std::vector<std::string> strList = StringSplit2(strData,   "\n");
    map<string,double>::iterator map_it;

    //搜索每一行
    for(int i=0;i<strList.size();i++)
    {
        oneLine = strList[i];
        oneLine = replace_all_distinct(oneLine,"\r", "");
        if(oneLine[0] == '#')
        {
            strLine +=oneLine + "\r\n";
        }
        else
        {
            string  strSN1="";
            double value=0.0;
            //在一行中，搜索是否是需要修改的参数，如果是就修改，
            for(map_it=modifyParamList.begin();map_it !=modifyParamList.end();map_it++)
            {
                //cout<<map_it->first<<": "<<map_it->second<<endl;
                strSN1=map_it->first;
                value=map_it->second;
                if(oneLine.size() >= strSN1.size() && oneLine.find(strSN1) != std::string::npos)
                {
                    strLine += strSN1 + string(": ")+ to_string(value) + "\r\n";
                    break;
                }
            }
            //搜索完，都没法匹配到
            if(map_it ==modifyParamList.end())
            {
                strLine +=oneLine + "\r\n";
            }
        }
    }
    strList.clear();
    return strLine;
}

int writeToYamlFile(string& filePath,string& strData)
{
    const char * p8File = (char*)filePath.c_str();
    const char* pChangeData = strData.c_str();
    //把修改后的内容写到新的文件中
    int fd = open(p8File,  O_CREAT | O_WRONLY | O_TRUNC,0644);
    if(fd<0)
    {
       // LOGI("%s: vWriteDataToFile Create error",TAG);
        cout<<"in writeToYamlFile open file error"<<endl;
        return false;
    }
    int ret = write(fd, pChangeData, strData.size());
    close(fd);
    return ret;
}

void ChangeYaml(string& filePath,map<string,double> &modifyParamList)
{
    string strData=ReadYamlFile(filePath);
    string changeFile=ChangeYamlFile(strData,modifyParamList);
    int ret=writeToYamlFile(filePath,changeFile);
    if(!ret)
    {
        cout<<"cannot write to file ,open file error"<<endl;
    }
}

void offsetSaveCallback(const std_msgs::Bool &msg)
{
    if(msg.data == true)
    {
        //将动态参数写到yaml 文件中
        ROS_WARN("---getCurrentConfiguration, cameral_distance_x=%lf,cameral_distance_y=%lf",\
             cameral_distance_x,cameral_distance_y); 
        string file="/home/eaibot/moveit_ws/src/moveit/visual_grab/config/visual_grab.yaml";
        map<string,double> modifyParamList;
        modifyParamList["cameral_distance_x"]=cameral_distance_x;
        modifyParamList["cameral_distance_y"]=cameral_distance_y;
        ChangeYaml(file,modifyParamList);            
    }
}

int main(int argc, char**argv)
{
    ros::init(argc,argv,"cameral_to_dobot_tf");
    ros::NodeHandle n;
    
    bool isRegistered=bEAIHadAuthed();
    if(!isRegistered)
    {
        ROS_ERROR("robot is not registered, can not run visual grab");
        return -1;
    }
    
    dynamic_reconfigure::Server<visual_grab::CameralToDobotConfig> server;
    dynamic_reconfigure::Server<visual_grab::CameralToDobotConfig>::CallbackType f;

    f = boost::bind(&callback, _1);
    server.setCallback(f);
      
    n.param<double>("/cameral_distance_x", cameral_distance_x, 0.0);
    n.param<double>("/cameral_distance_y", cameral_distance_y, 0.0);
    //ROS_WARN("cameral_distance_x,Y=(%lf,%lf)",cameral_distance_x,cameral_distance_y);

    ros::Subscriber offset_save_sub=n.subscribe("/tf_offset_save",1,offsetSaveCallback);
    
    ros::Rate loop(3);
    tf::TransformBroadcaster cameral_to_dobot_broadcaster;
    tf::TransformBroadcaster dobot_end_to_dobot_broadcaster;
    tf::TransformBroadcaster cameral_to_dobot_end_broadcaster;
    
    geometry_msgs::TransformStamped dobot_end_to_dobot_transform;
    geometry_msgs::TransformStamped cameral_to_dobot_end_transform;
    geometry_msgs::TransformStamped cameral_to_dobot_transform;
    geometry_msgs::Quaternion cameral_to_dobot_quaternion;
    geometry_msgs::Quaternion dobot_end_to_dobot_quaternion;
    geometry_msgs::Quaternion cameral_to_dobot_end_quaternion;
    
    while(ros::ok())
    {
        //ROS_WARN("start get dobot pose");
        //获取机械臂末端坐标系
        GetPose(dobot_endeffector_point);
        //ROS_WARN("dobot pose=(%f,%f,%f)",dobot_endeffector_point.point.x,\
                 dobot_endeffector_point.point.y,dobot_endeffector_point.point.z);
                                                 
        double endeffector_x=fabs(dobot_endeffector_point.point.x);
        double endeffector_y=fabs(dobot_endeffector_point.point.y);
        if((endeffector_x ==0.0)&&(endeffector_y ==0.0))
        {
            ROS_WARN("x and y is ,this is error");
            return -1;
        }
 #if 1       
        //发布机械臂末端坐标系到dobot 坐标系的转换关系
        dobot_end_to_dobot_transform.header.stamp=ros::Time::now();
        dobot_end_to_dobot_transform.header.frame_id="dobot_base";
        dobot_end_to_dobot_transform.child_frame_id="dobot_end";
        dobot_end_to_dobot_transform.transform.translation.x=dobot_endeffector_point.point.x;
        dobot_end_to_dobot_transform.transform.translation.y=dobot_endeffector_point.point.y;
        dobot_end_to_dobot_transform.transform.translation.z=0.0;
        
        //计算机械臂旋转角度
        if(endeffector_x <0.00001 ) //x为0,机械臂没有旋转，或者已经旋转180度
        {
            if(dobot_endeffector_point.point.y > 0.0)
            {
                thea=1.57078;
            }
            else
            {
                thea=-1.57078;
            }
        }
        else
        {
            thea =atan2(dobot_endeffector_point.point.y,dobot_endeffector_point.point.x);
        }
        
        dobot_end_to_dobot_quaternion=tf::createQuaternionMsgFromRollPitchYaw(0.0,3.14159,thea-1.57078);
        dobot_end_to_dobot_transform.transform.rotation=dobot_end_to_dobot_quaternion;
        dobot_end_to_dobot_broadcaster.sendTransform(dobot_end_to_dobot_transform);                 
        
        //loop.sleep(); 
        
        //发布摄像头坐标系到机械臂末端坐标系的转换关系
        
        cameral_to_dobot_end_transform.header.stamp=ros::Time::now();
        cameral_to_dobot_end_transform.header.frame_id="dobot_end";
        cameral_to_dobot_end_transform.child_frame_id="cameral_base";
        
        cameral_to_dobot_end_transform.transform.translation.x=cameral_distance_x;
        cameral_to_dobot_end_transform.transform.translation.y=cameral_distance_y;
        
        
        cameral_to_dobot_end_transform.transform.translation.z=0.0;
        
        //cameral_to_dobot_end_quaternion=tf::createQuaternionMsgFromRollPitchYaw(0.0,3.14159,-1.57078);
        cameral_to_dobot_end_quaternion=tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
        cameral_to_dobot_end_transform.transform.rotation=cameral_to_dobot_end_quaternion;
        cameral_to_dobot_end_broadcaster.sendTransform(cameral_to_dobot_end_transform); 
        
        loop.sleep();
        ros::spinOnce();
        
#endif             
    }
    
   
    return 0;
}
