// "reactive_navigation" node: subscribes laser data and publishes velocity commands

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>

//some global variables to "perceive" the world around the robot:
double obstacle_distance,left_distance,front_distance,right_distance;
bool robot_stopped;
double k_loops=0;
double k_loops1=0;
double k_right=0;
double k_right1=0;
bool rt_side=true;
bool rt_side1=true;
double k_loopsPos=0;
int rt_k = 2;
int r_num=0;

int state = 0;
float min_dist = 0.5;
float linear_vel = 0.4;
float angular_vel = 1.5;
geometry_msgs::Twist cmd_vel_msg;

//...

void laser_0_Callback(const sensor_msgs::LaserScan::ConstPtr& msg_front){
  if (!robot_stopped){  //print only when robot is moving...
    //For simplicity, let's save the distance of the closer obstacle to the robot:
    //obstacle_distance = *std::min_element (msg->ranges.begin(), msg->ranges.end());
    front_distance=msg_front->ranges[0];
    //ROS_INFO("front distance to obstacle: %f", front_distance);
  }  
}
void laser_1_Callback(const sensor_msgs::LaserScan::ConstPtr& msg_left){
  if (!robot_stopped){  //print only when robot is moving...
    //For simplicity, let's save the distance of the closer obstacle to the robot:
    //obstacle_distance = *std::min_element (msg->ranges.begin(), msg->ranges.end());
    left_distance=msg_left->ranges[0];
    //ROS_INFO("left distance to obstacle: %f", left_distance);
  }  
}
void laser_2_Callback(const sensor_msgs::LaserScan::ConstPtr& msg_right){
  if (!robot_stopped){  //print only when robot is moving...
    //For simplicity, let's save the distance of the closer obstacle to the robot:
    //obstacle_distance = *std::min_element (msg->ranges.begin(), msg->ranges.end());
    right_distance=msg_right->ranges[0];
    //ROS_INFO("right distance to obstacle: %f", right_distance);
  }  
}
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
    cmd_vel_msg.linear.x = linear_vel;
    //cmd_vel_msg.angular.z = 0.0;
}
void moveTR(){
    //cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = angular_vel*-1;
}
void moveTL(){
    //cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = angular_vel;
}
int LOST(){
    if(!sensL() && !sensR() && !sensF()){
        moveF();
        cmd_vel_msg.angular.z = 0.0;
    }
    else
        return 1;
    return 0;
}
int CCW(){
    if(sensL() || sensR() || sensF()){
        moveTR();
        cmd_vel_msg.linear.x = 0.0;
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


int main(int argc, char **argv){
    
  ros::init(argc, argv, "reactive_navigation");
  
  ros::NodeHandle n;

  //Publisher for /cmd_vel
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  //Subscriber for /base_scan
  ros::Subscriber laser_0_sub = n.subscribe("base_scan_0", 100, laser_0_Callback);
  ros::Subscriber laser_1_sub = n.subscribe("base_scan_1", 100, laser_1_Callback);
  ros::Subscriber laser_2_sub = n.subscribe("base_scan_2", 100, laser_2_Callback);

  ros::Rate loop_rate(10); //10 Hz
  
  //initializations:
  
  robot_stopped = false;
  left_distance = 1.0;
  front_distance = 1.0;
  right_distance = 1.0;
  //cmd_vel_msg.angular.z = 1.0;
  rt_side=true;
  k_right=0;
  while (ros::ok()){
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
    //publish velocity commands:
    cmd_vel_pub.publish(cmd_vel_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
