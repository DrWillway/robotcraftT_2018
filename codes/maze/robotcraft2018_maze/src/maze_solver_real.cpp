#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Range.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <string.h>


#define PI 3.1415
float x,y,th,v=0.0,w=0.0;
double obstacle_distance,left_distance,front_distance,right_distance;
float max_v=0.04,max_w=0.3;
int state = 0;
float min_dist = 0.09;
float linear_vel;
float angular_vel;
float spd_coef;


bool sensR(){
    if(right_distance < min_dist) return 1;
    else return 0;
}
bool sensL(){
    if(left_distance < min_dist) return 1;
    else return 0;
}
bool sensF(){
    if(front_distance < min_dist) return 1;
    else return 0;
}
void moveF(){
    v = max_v;
    //w = 0.0;
}
void moveTR(){
    //v = 0.0;
    w = max_w*-1;
    
}
void moveTL(){
    //v = 0.0;
    w = max_w;
}
int LOST(){
    if(!sensL() && !sensR() && !sensF()){
        w = 0.0;
	moveF();
    }
    else
        return 1;
    return 0;
}
int CCW(){
    if(sensL() || sensR() || sensF()){
        v = 0.0;
	moveTR();
    }
    else 
        return 2;
    return 1;
}
int WALL1(){
    if(!sensL()){
        moveF();
        moveTL();
    }
    else
        return 3;
    return 2;
}
int WALL2(){
    if(!sensR())
        if(sensL()){
            moveTR();
            moveF();
        }
        else
            return 2;
    else 
        return 1;
    return 3;
}

void poseCallback(const geometry_msgs::Pose2D& pose_msg){
  x=pose_msg.x;
  y=pose_msg.y;
  th=pose_msg.theta;
}

void distFCallback(const std_msgs::Float32& f_msg){
  front_distance=f_msg.data;
  //ROS_INFO("front distance to obstacle: %f", front_distance);
}
void distRCallback(const std_msgs::Float32& r_msg){
  right_distance=r_msg.data;
  //ROS_INFO("right distance to obstacle: %f", right_distance);
}
void distLCallback(const std_msgs::Float32& l_msg){
  left_distance=l_msg.data;
  //ROS_INFO("left distance to obstacle: %f", left_distance);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "maze_solver_real");
  ros::NodeHandle n;
  //Subscribers
  ros::Subscriber pose_sub = n.subscribe("/pose", 1000, poseCallback);
  ros::Subscriber front_distance_sub = n.subscribe("/front_distance", 1000, distFCallback);
  ros::Subscriber right_distance_sub = n.subscribe("/right_distance", 1000, distRCallback);
  ros::Subscriber left_distance_sub = n.subscribe("/left_distance", 1000, distLCallback);
  //Publishers
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

  ros::Rate loop_rate(5);
  geometry_msgs::Twist cmd_vel;

  while (ros::ok()){
    ROS_INFO("%d %d %d\t%i %f", int(left_distance*10),int(front_distance*10),int(right_distance*10),state,spd_coef);
    switch(state){
        case 0:
            ROS_INFO("LOST");
            state = LOST();
            break;
        case 1:
            ROS_INFO("CCW");
            state = CCW();
            break;
        case 2:
            ROS_INFO("WALL1");
            state = WALL1();
            break;    
        case 3:
            ROS_INFO("WALL2");
            state = WALL2();
            break;            
    }
    //spd_coef = (left_distance+front_distance+right_distance)/3*1.25;
    //v = max_v*spd _coef;
    //w = max_w*spd_coef;
    
    cmd_vel.linear.x = v;
    cmd_vel.angular.z = w;
    //publish the message
    cmd_pub.publish(cmd_vel);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
