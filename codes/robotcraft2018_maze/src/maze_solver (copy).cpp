#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#define LOST 3
#define FORWARD 0
#define LEFT 1
#define RIGHT 2

bool sensR, sensL, sensF;
ros::Publisher cmd_vel_pub;
float linear_speed = 0.3;
float angular_speed = 0.5;
float max_range = 0.3;

void sensR_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(*std::min_element (msg->ranges.begin(), msg->ranges.end()) < max_range){
	sensR = true;
  }else{
	sensR = false;
  }

}

void sensL_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(*std::min_element (msg->ranges.begin(), msg->ranges.end()) < max_range){
	sensL = true;
  }else{
	sensL = false;
  }

}
void sensF_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  if(*std::min_element (msg->ranges.begin(), msg->ranges.end()) < max_range){
	sensF = true;
  }else{
	sensF = false;
  }

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

int updateState(){
	if((sensL == false && sensF == false && sensR == false)){
		return LOST;
	}else if((sensL == false && sensF == false && sensR == true) || (sensL == false && sensF == true && sensR == false) || (sensL == true && sensF == false && sensR == false)){
		return FORWARD;
	}else if ((sensL == false && sensF == true && sensR == true) || (sensL == true && sensF == true && sensR == true)){
		return LEFT;
	}else if(sensL == true && sensF == true && sensR == false){
		return RIGHT;
	}else{
		return LEFT;
	}
}


int main(int argc, char **argv){
    
  ros::init(argc, argv, "maze_solver");
  ros::NodeHandle n;
  ros::Rate loop_rate(10); //10 Hz

  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  ros::Subscriber sensR_sub = n.subscribe("base_scan_2", 1000, sensR_Callback);
  ros::Subscriber sensL_sub = n.subscribe("base_scan_1", 1000, sensL_Callback);
  ros::Subscriber sensF_sub = n.subscribe("base_scan_0", 1000, sensF_Callback);

  int state = FORWARD;
  int count=0;
	
  while (ros::ok()){
		state = updateState();
		ROS_INFO("%d", state);
	 switch(state){
		case FORWARD:
			ROS_INFO("FORWARD");
			move(linear_speed);
							count = 0;
			break;
		
		case LEFT:
			ROS_INFO("LEFT");
			turn(angular_speed);
							count = 0;

			break;
		case RIGHT:
		  ROS_INFO("RIGHT");
			turn(-1 * angular_speed);
			count = 0;
			break;
		case LOST: 
			count ++;
			if(count > 50 ){
				if(!sensF && !sensR && !sensL){
					move(linear_speed);
				}
			}else{
							turn(angular_speed);
			move(linear_speed);
			} break;
		default:
			ROS_ERROR("Invalid State");
	 }

	 ros::spinOnce();
   loop_rate.sleep();
 }
 return 0;
}






*/











/*
int lost(){
	if(!sensR && !sensL){
		move(linear_speed);
	}else{
		return STATE_CCW;
	}
	return STATE_LOST;
}

int ccw(){
	if(sensR || sensL){
		turn(angular_speed);
	}else{
		return STATE_WALL1;
	}
	return STATE_CCW;
}

int wall1(){
	if(!sensR){
		move(linear_speed);
   		turn(angular_speed);
	}else{
		return STATE_WALL2;
	}
	return STATE_WALL1;
}

int wall2(){
	if(!sensL){
		if(sensR){
			turn(angular_speed);
			move(linear_speed);
		}else{
			return STATE_WALL1;
		}
	}else{
		return STATE_CCW;
	}
	return STATE_WALL2;
}*/
