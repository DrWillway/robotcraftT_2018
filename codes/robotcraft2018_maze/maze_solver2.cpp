#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#define FORWARD 0
#define LEFT 1
#define RIGHT 2
#define LOST 3
#define RIGHT_FAST 4

using namespace std;

bool sensR, sensL, sensF;
ros::Publisher cmd_vel_pub;

float obstacle_distance,obstacle_distanceR, obstacle_distanceL,obstacle_distanceF;

void sensR_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  obstacle_distanceR = *std::min_element (msg->ranges.begin(), msg->ranges.end());
}

void sensL_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  obstacle_distanceL = *std::min_element (msg->ranges.begin(), msg->ranges.end());
}

void sensF_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  obstacle_distanceF = *std::min_element (msg->ranges.begin(), msg->ranges.end());
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

int updateState(){
	obstacle_distance = smallest(obstacle_distanceL,obstacle_distanceF,obstacle_distanceR);
  if(obstacle_distance>0.7){
  	return LOST;
  }else if(obstacle_distanceF<=0.4){
    return RIGHT_FAST;
  }else if(obstacle_distanceF<=0.7 || obstacle_distanceL < 0.35){
    return RIGHT;
  }
  else if(obstacle_distance<=0.4 && obstacle_distance>0.3){
    return LEFT;
  }else return LOST;
}

double previous_error = 0.0; 
double current_error;
double w = 0.2;
double wreal;
double sum_error = 0.0;
double period = 0.1;
double Kp = 3.0, Ki = 0.01, Kd = 0.0;
double gain = 0.0;
double diff_error;
double des_dist_left = 0.3;

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
  geometry_msgs::Twist cmd_vel_msg;

  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  ros::Subscriber sensR_sub = n.subscribe("base_scan_2", 1000, sensR_Callback);
  ros::Subscriber sensL_sub = n.subscribe("base_scan_1", 1000, sensL_Callback);
  ros::Subscriber sensF_sub = n.subscribe("base_scan_0", 1000, sensF_Callback);
  
	int state;

  while (ros::ok()){
    pid();
    state = updateState();
		ROS_INFO("%d", state);
	  switch(state){
			case RIGHT:
				ROS_INFO("RIGHT");
				cmd_vel_msg.linear.x = 0.35;
    				cmd_vel_msg.angular.z = -2.5;
				break;
			case RIGHT_FAST:
				ROS_INFO("RIGHT_FAST");
				cmd_vel_msg.linear.x = 0.0;
    				cmd_vel_msg.angular.z = -2.5;
				break;
			case LEFT:
				ROS_INFO("LEFT");
				cmd_vel_msg.linear.x = 0.8;
       				cmd_vel_msg.angular.z = gain/*0.5*/;
				break;
			case LOST: 
				ROS_INFO("LOST");
				cmd_vel_msg.linear.x = 0.4;
        			cmd_vel_msg.angular.z = 1.3;
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













