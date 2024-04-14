#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <unistd.h>
#include <move_to_grab/MoveToGrabMsg.h>
#include <move_to_grab/LoopMoveToGrabMsg.h>
#include <string>
#include <sstream>
#include <boost/thread.hpp>
#include <dynamic_reconfigure/server.h>
//#include "visual_grab/ExecuteMoveAndGrabConfig.h"

using namespace std;

ros::Publisher g_back_pub, grab_pub;

bool isLoop;
int navigation_ptp_time=0;
int navigation_ptp_timeout=0;

void poseCallback(const move_to_grab::MoveToGrabMsg& msg)
{
    ROS_WARN("in execute move an grab poseCallback");
    
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
 
    ac.waitForServer();
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    geometry_msgs::Quaternion quaternion;
    quaternion.x = msg.pose.pose.orientation.x ;
    quaternion.y = msg.pose.pose.orientation.y;
    quaternion.z = msg.pose.pose.orientation.z ;
    quaternion.w = msg.pose.pose.orientation.w ;
    goal.target_pose.pose.position.x = msg.pose.pose.position.x ;
    goal.target_pose.pose.position.y = msg.pose.pose.position.y ;
    goal.target_pose.pose.position.z = msg.pose.pose.position.z ;
    goal.target_pose.pose.orientation = quaternion ;

    ac.sendGoal(goal);
    //ros::spinOnce();
    //bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));
    ROS_WARN("***navigation_ptp_timeout=%d",navigation_ptp_timeout); 
    bool finished_before_timeout = ac.waitForResult(ros::Duration(navigation_ptp_timeout));

    if(finished_before_timeout)
    {
        
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_WARN("**pose1 Action finished: %s",state.toString().c_str());
        if(state.toString() == "SUCCEEDED")
        {
            sleep(3);
            
            ROS_WARN("start to suck");
            std_msgs::Int32 msg_int32;
            msg_int32.data=msg.isSuckup.data;
            grab_pub.publish(msg_int32);
            sleep(9);
        }
        else
        {
            if(isLoop)
            {
                isLoop=false;
            } 
            ROS_WARN("navigation error, state =%s",state.toString().c_str());
        }       
    }
    else
    {
        if(isLoop)
        {
            isLoop=false;
        }    
        ROS_INFO("pose1 Action did not finish before the time out.");
    }
}

void loopPoseCallback(const move_to_grab::LoopMoveToGrabMsg& msg)
{
    ROS_WARN("in loop Pose Callback,%d",msg.isLoop.data);
#if 0    
    move_to_grab::MoveToGrabMsg action1_msg=msg.action1;
    move_to_grab::MoveToGrabMsg action2_msg=msg.action2;
#endif
    move_to_grab::MoveToGrabMsg action1_msg;
    move_to_grab::MoveToGrabMsg action2_msg;
    if(msg.action1.isSuckup.data==true && msg.action2.isSuckup.data==false)
    {
        action1_msg=msg.action1;
        action2_msg=msg.action2;
    }
    else if(msg.action1.isSuckup.data==false && msg.action2.isSuckup.data==true)
    {
        action1_msg=msg.action2;
        action2_msg=msg.action1;
    }
    else
    {
        ROS_WARN("pose1 and pose2 dobot action is error,can not move loop and grab");
        return;
    }
    isLoop=true;
    int count=0;
    while(ros::ok() &&isLoop)
    {
        if(count%2 == 0)
        {
            sleep(3);
            ROS_WARN("in loop pose");
            //move_to_grab::MoveToGrabMsg action1_msg=msg.action1;
            action1_msg.isSuckup.data=true;
            ROS_WARN("in loop start action1");
            poseCallback(action1_msg);
            ROS_WARN("in loop finish action1");
            //sleep(17);
            ROS_WARN("***navigation_ptp_time=%d",navigation_ptp_time); 
            sleep(navigation_ptp_time);
            
            ROS_WARN("in loop start action2");
            //move_to_grab::MoveToGrabMsg action2_msg=msg.action2;
            action2_msg.isSuckup.data=false;
            ROS_WARN("in loop finish action2");
            poseCallback(action2_msg);
            //sleep(17);
            sleep(navigation_ptp_time);
            
        }
        else
        {
            sleep(3);
            ROS_WARN("in loop start action2");
            //move_to_grab::MoveToGrabMsg action2_msg=msg.action2;
            action2_msg.isSuckup.data=true;
            ROS_WARN("in loop finish action2");
            poseCallback(action2_msg);
            sleep(17);
            
            //move_to_grab::MoveToGrabMsg action1_msg=msg.action1;
            action1_msg.isSuckup.data=false;
            ROS_WARN("in loop start action1");
            poseCallback(action1_msg);
            ROS_WARN("in loop finish action1");
            sleep(17);
        }
        count++;
        ros::spinOnce();
    }
}

void pauseLoopPoseCallback(const move_to_grab::LoopMoveToGrabMsg& msg)
{
    ROS_WARN("in  pause loop Pose Callback,%d",msg.isLoop.data);
    isLoop=false;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ac.cancelAllGoals();
}

#if 0
void callback(visual_grab::ExecuteMoveAndGrabConfig &config)
 { 
    //nav_ptp_time
    //navigation_ptp_timeout
    ROS_WARN("***Reconfigure Request: %d %d", config.nav_ptp_time,config.navigation_ptp_timeout);
    nav_ptp_time=config.nav_ptp_time;
    navigation_ptp_timeout=config.navigation_ptp_timeout;
        
 }
#endif

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"test_move");
    ros::NodeHandle nh;
    ros::Rate loop(1);
    
    //dynamic_reconfigure::Server<visual_grab::ExecuteMoveAndGrabConfig> server;
    //dynamic_reconfigure::Server<visual_grab::ExecuteMoveAndGrabConfig>::CallbackType f;

    //f = boost::bind(&callback, _1);
   // server.setCallback(f);
    
    nh.param<int>("navigation_ptp_time", navigation_ptp_time, 18);
    nh.param<int>("navigation_ptp_timeout", navigation_ptp_timeout, 60);
    
    ros::Subscriber pose_sub=nh.subscribe("/move_to_grab/goal",1,poseCallback);
    ros::Subscriber loop_pose_sub=nh.subscribe("/loop_move_to_grab/goal",1,loopPoseCallback);
    ros::Subscriber pause_loop_pose_sub=nh.subscribe("/pause_loop_move_to_grab/goal",1,pauseLoopPoseCallback);
    g_back_pub =nh.advertise<std_msgs::String>("/check",5);
    
    grab_pub =nh.advertise<std_msgs::Int32>("/move_to_grab/grab",5);

    //为了让两个订阅器同时执行
    ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
    spinner.start();
    ros::waitForShutdown();
//    ros::spin();
    return 0;
}


