#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define FORWARD 0
#define LEFT 1
#define RIGHT 2
#define LOST 3
#define RIGHT_FAST 4

using namespace std;

bool sensR, sensL, sensF;
ros::Publisher cmd_vel_pub;

float obstacle_distance,obstacle_distanceR, obstacle_distanceL,obstacle_distanceF;

double x;
double y;
double th;

geometry_msgs::Pose2D pose_msg;
ros::Time current_time, last_time;

void poseCallback(const geometry_msgs::Pose2D& msg){
	x = msg.x;
    	y = msg.y;
	th = msg.theta;
}

void dist_rightCallback(const std_msgs::Float32& msg){
  obstacle_distanceR = msg.data;
}

void dist_leftCallback(const std_msgs::Float32& msg){
  obstacle_distanceL = msg.data;
}

void dist_frontCallback(const std_msgs::Float32& msg){
  obstacle_distanceF = msg.data;
}

void move (double linear_speed){
	geometry_msgs::Twist cmd_msg;
	cmd_msg.linear.x = linear_speed;
  	cmd_vel_pub.publish(cmd_msg);
}

void turn(double angular_speed){
	geometry_msgs::Twist cmd_msg;
	cmd_msg.angular.z = angular_speed;
        cmd_vel_pub.publish(cmd_msg);
}

float smallest(float x, float y, float z){
  return x < y ? (x < z ? x : z) : (y < z ? y : z);
}

/*
int updateState(){
	obstacle_distance = smallest(obstacle_distanceL,obstacle_distanceF,obstacle_distanceR);
  if(obstacle_distanceF<=0.08){
    	return RIGHT_FAST;
  }else if(obstacle_distanceL>0.12){
  	return LEFT;
  }else if(obstacle_distanceL<=0.12 && obstacle_distanceL > 0.08){
  	return FORWARD;
  }else if(obstacle_distanceL<0.08){
  	return RIGHT;
  }else return LOST;
}
*/

int updateState(){
  obstacle_distance = smallest(obstacle_distanceL,obstacle_distanceF,obstacle_distanceR);
  if(obstacle_distance>0.09){
  	return LOST;
  }else if(obstacle_distanceF<=0.07){
    return RIGHT_FAST;
  }else if(obstacle_distanceF<=0.09 || obstacle_distanceL < 0.07){
    return RIGHT;
  }
  else if(obstacle_distance<=0.07 && obstacle_distance>0.065){
    return LEFT;
  }else return LOST;
}

double previous_error = 0.0; 
double current_error;
double w = 0.2;
double wreal;
double sum_error = 0.0;
double period = 0.1;
double Kp = 0.07, Ki = 0.005, Kd = 0.005;
double gain = 0.0;
double diff_error;
double des_dist_left = 0.08;

void pid(){
  current_error = des_dist_left - obstacle_distanceL;

  sum_error += current_error;

  diff_error = current_error - previous_error;

  gain = Kp * current_error + Ki * sum_error * period + (Kd) * (diff_error / period);

  previous_error = current_error;

}

int main(int argc, char **argv){
    
  ros::init(argc, argv, "maze_solver");
  ros::NodeHandle n;
  ros::Rate loop_rate(10); //10 Hz
  
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber pose_sub = n.subscribe("pose", 1000, poseCallback);

  ros::Subscriber dist_front = n.subscribe("front_distance", 1000, dist_frontCallback);
  ros::Subscriber dist_right = n.subscribe("right_distance", 1000, dist_rightCallback);
  ros::Subscriber dist_left = n.subscribe("left_distance", 1000, dist_leftCallback);
  
  geometry_msgs::Twist cmd_vel_msg;

  int state;

  while (ros::ok()){
    pid();
    state = updateState();
    ROS_INFO("%d", state);
    ROS_INFO("%f", obstacle_distanceF);
	  switch(state){
			case FORWARD:
				ROS_INFO("FORWARD");
				cmd_vel_msg.linear.x = 0.04;
    				cmd_vel_msg.angular.z = 0.0;
				break;
			case RIGHT:
				ROS_INFO("RIGHT");
				cmd_vel_msg.linear.x = 0.01;
    				cmd_vel_msg.angular.z = -1*gain;
				break;
			case RIGHT_FAST:
				ROS_INFO("RIGHT_FAST");
				cmd_vel_msg.linear.x = 0.0;
    				cmd_vel_msg.angular.z = -1.5;
				break;
			case LEFT:
				ROS_INFO("LEFT");
				cmd_vel_msg.linear.x = 0.04;
       				cmd_vel_msg.angular.z = gain/*0.3*/;
				break;
			case LOST: 
				ROS_INFO("LOST");
				cmd_vel_msg.linear.x = 0.04;
        			cmd_vel_msg.angular.z = 0.3;
				break;
			default:
				ROS_ERROR("Invalid State");
	 }
      
	 cmd_vel_pub.publish(cmd_vel_msg);

	 ros::spinOnce();
   	 loop_rate.sleep();
 }
 return 0;
}













