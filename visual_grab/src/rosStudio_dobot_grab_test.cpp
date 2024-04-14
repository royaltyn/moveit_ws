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

#include "visual_grab/RosStudioDobotGrabMsg.h"
#include "visual_grab/RosStudioDobotGrabResultMsg.h"
#include "geometry_msgs/PoseStamped.h"

#define KEYCODE_1 0x31
#define KEYCODE_2 0x32

int kfd = 0;
struct termios cooked, raw;

visual_grab::RosStudioDobotGrabMsg dobot_excute;
int count=0;

ros::Publisher dobot_excute_pub;

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
       
    
	    if(c == KEYCODE_1)
	    {
            ROS_INFO("1");
            dobot_excute.id.data=count;
            count++;
            
            dobot_excute.header.stamp.sec=ros::Time::now().toSec();
            dobot_excute.observerPose.pose.position.x= -35.2793;         //0.0 ; 
            dobot_excute.observerPose.pose.position.y= -231.0274;          //-256 ;
            dobot_excute.observerPose.pose.position.z= 128.2078;       //134 ;  
            
            dobot_excute.HomePose.pose.position.x= 12.6870;         //0.0 ;
            dobot_excute.HomePose.pose.position.y=234.4838;      //258 ;
            dobot_excute.HomePose.pose.position.z=132.9800;     //121 ;
            
            dobot_excute.robotReleasePose.pose.position.x= 3.5855;      //3.32064580917 ;
            dobot_excute.robotReleasePose.pose.position.y= 228.2501;        //228.527862549 ;
            dobot_excute.robotReleasePose.pose.position.z= -26.6536;    //-34.193069458 ;
            
            dobot_excute.target_height.data=-40; //-31.7086;      //-40.0;
            
            dobot_excute.dobot_action_type.data=true;
            dobot_excute.dobot_action1.data=true;
            dobot_excute.dobot_action2.data=true;
            dobot_excute.dobot_action3.data=true;
            dobot_excute.dobot_action4.data=true;
            dobot_excute.dobot_action5.data=true;
            
            dobot_excute_pub.publish(dobot_excute);
            ROS_WARN("publish RosStudioDobotGrabMsg message");
            //sleep(10);
            
	    }
	    else if(c == KEYCODE_2)
	    {
            ROS_INFO("2");
            dobot_excute.id.data=count;
            count++;
            dobot_excute.header.stamp.sec=ros::Time::now().toSec();
            dobot_excute.observerPose.pose.position.x=0.0 ;
            dobot_excute.observerPose.pose.position.y=-256 ;
            dobot_excute.observerPose.pose.position.z=134 ;  
            
            dobot_excute.HomePose.pose.position.x=0.0 ;
            dobot_excute.HomePose.pose.position.y=258 ;
            dobot_excute.HomePose.pose.position.z=121 ;
            
            dobot_excute.robotReleasePose.pose.position.x=3.32064580917 ;
            dobot_excute.robotReleasePose.pose.position.y=228.527862549 ;
            dobot_excute.robotReleasePose.pose.position.z=-34.193069458 ;
            
            dobot_excute.target_height.data= -40.0;
            
            dobot_excute.dobot_action_type.data=false;
            dobot_excute.dobot_action1.data=true;
            dobot_excute.dobot_action2.data=true;
            dobot_excute.dobot_action3.data=true;
            dobot_excute.dobot_action4.data=true;
            dobot_excute.dobot_action5.data=true;
            
            dobot_excute_pub.publish(dobot_excute);
            ROS_WARN("publish RosStudioDobotGrabMsg message");
	    }

    }

}

void dobotExcuteResultCallback(const visual_grab::RosStudioDobotGrabResultMsg& msg)
{
    ROS_WARN("result id=%d ,result data=[%d,%d]",msg.id.data,msg.dobot_excute_result[0].data,\
                                                 msg.dobot_excute_result[1].data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosStudio_dobot_grab_test");
    ros::NodeHandle n;   
    
    dobot_excute_pub=n.advertise<visual_grab::RosStudioDobotGrabMsg>("/RosStudioDobotGrab",5);
    ros::Subscriber dobot_excute_result_sub=n.subscribe("/RosStudioDobotGrabResult",5,dobotExcuteResultCallback);
    
    boost::thread t = boost::thread(boost::bind(&keyboardLoop, boost::ref(n)));
    

    ros::spin();

    t.interrupt();
    t.join();

    tcsetattr(kfd, TCSANOW, &cooked);
    return 0;
}
