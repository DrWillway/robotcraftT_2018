#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#define PI 3.1415
float x,y,th,v=0.05,w=0.5;
float dist_err,ang_err;
float max_v=0.06,max_w=0.6;
float dist_max=0.5,th_max=1.57;
int stp=0;
float delta_th;
//float goalsXY[][]={{0.5,0.5,};

void poseCallback(const geometry_msgs::Pose2D& pose_msg){
  x=pose_msg.x;
  y=pose_msg.y;
  th=pose_msg.theta;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "square_test");
  ros::NodeHandle n;
  //Subscribers
  ros::Subscriber pose_sub = n.subscribe("/pose", 1000, poseCallback);
  //Publishers
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

  ros::Rate loop_rate(10);
  while (ros::ok()){
    switch(stp){
        case 0:
            if(x<dist_max){
                dist_err = 1-(x/dist_max);  //(0.5-x)/0.5
                v=0.05*dist_err+0.005;
                w=0.0;
                ROS_INFO("inside 1 if x++");
            }
            else stp++;
            break;
        case 1:
            v=0.0;
            ROS_INFO("inside 1 if rotate");
            if(th<PI/2-0.01) w=0.1;
            else if(th>PI/2) w=-0.05;
            else stp++;
            break;
        case 2:
            if(y<dist_max){
                dist_err = 1-(y/dist_max);  //(0.5-x)/0.5
                v=0.05*dist_err+0.005;
                w=0.0;
                ROS_INFO("inside 2 if y++");
            }else stp++;
            break;
        case 3:
            v=0.0;
            ROS_INFO("inside 2 if rotate");
            if(th<PI-0.01 && th>0) w=0.1;
            else if(th<-PI && th<0) w=-0.05;
            else stp++;
            break;
        case 4:
            if(x>0){
                dist_err = (x/dist_max);  //(0.5-x)/0.5
                v=0.05*dist_err+0.005;
                w=0.0;
                ROS_INFO("inside 3 if x--");
            }
            else stp++;
            break;
        case 5:
            v=0.0;
            ROS_INFO("inside 3 if rotate");
            if(th<-(PI/2)-0.01) w=0.1;
            else if(th>-PI/2) w=-0.05;
            else stp++;
            break;
        case 6:
            if(y>0){
                dist_err = (y/dist_max);  //(0.5-x)/0.5
                v=0.05*dist_err+0.005;
                w=0.0;
                ROS_INFO("inside 4 if y--");
            }
            else stp++;
            break;
        case 7:
            v=0.0;
            ROS_INFO("inside 3 if rotate");
            if(th<0-0.01) w=0.1;
            else if(th>0) w=-0.05;
            else stp=0;
            break;
    }
   /* if(x<dist_max){
        dist_err = 1-(x/dist_max);  //(0.5-x)/0.5
        v=0.05*dist_err+0.005;
        w=0.0;
        ROS_INFO("inside 1 if x++");
    }
    else if(th<th_max){
        ang_err = 1-(th/th_max);
        v=0.0;
        w=0.3*ang_err+0.0001;
        ROS_INFO("inside 2 if rotate");
    }
    else if(y<dist_max && x<dist_max){
        dist_err = 1-(y/dist_max);  //(0.5-x)/0.5
        v=0.05*dist_err+0.005;
        w=0.0;
        ROS_INFO("inside 3 if y++");
    }
    else if(th<th_max+th_max){
        ang_err = 1-(th/(th_max+th_max));
        v=0.0;
        w=0.3*ang_err+0.0001;
        ROS_INFO("inside 4 if rotate");
    }
    else {
        v=0.0;
        w=0.0;
        ROS_INFO("inside else stop");
    }
    /*if(th<1.57){
        v=0.0;
        w=0.5;
    }
    v=0.0;
    w=0.0;
*/
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = v;
    cmd_vel.angular.z = w;

    //publish the message
    cmd_pub.publish(cmd_vel);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
