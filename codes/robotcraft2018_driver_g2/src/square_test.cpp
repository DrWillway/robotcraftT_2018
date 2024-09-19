#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

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

void dist_frontCallback(const std_msgs::Float32& msg){
	if(msg.data < .15){
		ROS_WARN("Collision risk! The robot is %f meters of an obsctacle, on the front side", msg.data);
	}
}
void dist_rightCallback(const std_msgs::Float32& msg){
	if(msg.data < .15){
		ROS_WARN("Collision risk! The robot is %f meters of an obsctacle, on the right side", msg.data);
	}
}
void dist_leftCallback(const std_msgs::Float32& msg){
	if(msg.data < .15){
		ROS_WARN("Collision risk! The robot is %f meters of an obsctacle, on the left side", msg.data);
	}
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "square_test");
  ros::NodeHandle n;


  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber pose_sub = n.subscribe("pose", 1000, poseCallback);

  ros::Subscriber dist_front = n.subscribe("front_distance", 1000, dist_frontCallback);
  ros::Subscriber dist_right = n.subscribe("right_distance", 1000, dist_rightCallback);
  ros::Subscriber dist_left = n.subscribe("left_distance", 1000, dist_leftCallback);

  geometry_msgs::Twist msg;
  ros::Rate loop_rate(10);
  
  int count = 0;
  int test=0;
  float dist = 0.5;
  float rotate = 0.0; 
  while (ros::ok())
  {

	ROS_INFO("Count = %d",count);
	bool condition = false; 
    switch(count){
		case 0: if(x<dist){
				 condition = true;
				 if(th>0.01){
					rotate  = -1*th;
				 }else if(th<-0.01){
					rotate = -1*th;
				}else{
					rotate = 0;				
				}
			} break;
		case 1: if(y<dist){
			   		condition = true;
					if(th>1.58){
						rotate  = -0.01 ;
				 	}else if(th<1.56){
						rotate = 0.01;
					}else{
						rotate = 0;				
					} 
				}
			break;
		case 2: if(x>0){
					 condition = true;
					 if((th > 0 && th <3.13)){
					rotate  = 0.01;
				 	}else if(th < 0 && th > -3.13){
						rotate = -0.01;
					}else{
						rotate = 0;				
					}
		 		} break;
		case 3: if(y>0){ 
					condition = true;
					if(th<-1.58){
						rotate  = 0.01;
					 }else if(th > -1.56 && th <0){
						rotate = -0.01;
					}else{
						rotate = 0;				
					}
				} break;
	}
	bool condition_th = false; 
	switch(count){
		case 0: if(th<1.57) condition_th = true; break;
		case 1: if(th<3.14 && th >0) condition_th = true; break;
		case 2: if(th< -1.57) condition_th = true; break;
		case 3: if(th<0) condition_th = true; break;
	}
	if(condition) {
		ROS_INFO(" x = %f  Y = %f Rotate %f Z %f",x,y, rotate, th);
		test = 0;
		if(rotate >0){
			msg.angular.z = rotate;
		}else if (rotate <0){
			msg.angular.z = rotate;
		}else{
			msg.angular.z = 0.0;
		}
		msg.linear.x=0.03;
    }else{
	msg.linear.x=0.00;
	
	if(condition_th){
	  	msg.angular.z = 0.1;
		test = 0;
	    ROS_INFO("z = %f",th);
	}else{
		msg.angular.z=0.0;
		if(test == 0){
			count ++;
			test = 1;
		}
		if(count==4){
			count = 0;		
		}
	}
    }
    cmd_vel_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
