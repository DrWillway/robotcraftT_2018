#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <queue>

#define N 0
#define E 1
#define W 2
#define S 3

int current_orientation = 0;

double x;
double y;
double th;

double desired_x_move = 0.0;
double desired_y_move = 0.0;

int curr_x = 1;
int curr_y = 1;

double left_dist;
double right_dist;
double front_dist;
double dist = 0.5;

double square_dist = 0.1;

geometry_msgs::Pose2D pose_msg;
ros::Time current_time, last_time;


struct node{
	bool visited;
	int x;
	int y;
	Node adjacents[4];
};
typedef struct node Node;

void poseCallback(const geometry_msgs::Pose2D& msg){
	x = msg.x;
    	y = msg.y;
	th = msg.theta;
}

void dist_frontCallback(const std_msgs::Float32& msg){
	front_dist = msg.data;
}
void dist_rightCallback(const std_msgs::Float32& msg){
	right_dist = msg.data;
}
void dist_leftCallback(const std_msgs::Float32& msg){
	left_dist = msg.data;
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
  
  queue <Node> qnodes;
  Node first;
  first.visited = false;
  first.x=1;
  first.y=1;
  qnodes.push(first);
  geometry_msgs::Twist vel_msg;
  while (ros::ok())
  {
    if(!qnodes.isEmpty()){
	Node n = qnodes.pop();
	if(n.visited == false)
		n.visited = true;
		if(curr_y>n_y){
			//rotate to the south
			while(!(th> 3.10 || (th > 0 && th >-3.10)){
				vel_msg.angular.z = -0.5;
				vel_msg.linear.x=0.0;
				cmd_vel_pub.publish(vel_msg);
    				ros::spinOnce();
			}
			current_orientation = S;
			// move to the next square by defined distance
			double last_y = y;
			while(!(y > (last_y - square_dist))){
				vel_msg.angular.z = 0.0;
				vel_msg.linear.x=0.1;
				cmd_vel_pub.publish(vel_msg);
    				ros::spinOnce();
			}
			curr_y--;
		}else if(curr_y<n_y){
			// rotate to the north
			while(!(th> -0.10 ||th < 0.10){
				vel_msg.angular.z = -0.5;
				vel_msg.linear.x=0.0;
				cmd_vel_pub.publish(vel_msg);
    				ros::spinOnce();
			}
			current_orientation = N;
			// move 
			double last_y = y;
			while(!(y < (last_y + square_dist))){
				vel_msg.angular.z = 0.0;
				vel_msg.linear.x=0.1;
				cmd_vel_pub.publish(vel_msg);
    				ros::spinOnce();
			}
			curr_y++;
		}else if(curr_x>n_x){
			//rotate to the west
			while(!(th> 1.49 ||th < 1.62){
				vel_msg.angular.z = -0.5;
				vel_msg.linear.x=0.0;
				cmd_vel_pub.publish(vel_msg);
    				ros::spinOnce();
			}
			current_orientation = W;
			// move to the next square by defined distance
			double last_x = x;
			while(!(x > (last_x - square_dist))){
				vel_msg.angular.z = 0.0;
				vel_msg.linear.x=0.1;
				cmd_vel_pub.publish(vel_msg);
    				ros::spinOnce();
			}
			curr_x--;
		}else if(curr_x<n_x){
			// rotate to the east
			while(!(th> -1.62 ||th < -1.48){
				vel_msg.angular.z = -0.5;
				vel_msg.linear.x=0.0;
				cmd_vel_pub.publish(vel_msg);
    				ros::spinOnce();
			}
			current_orientation = E;
			// move 
			double last_x = x;
			while(!(x < (last_x + square_dist))){
				vel_msg.angular.z = 0.0;
				vel_msg.linear.x=0.1;
				cmd_vel_pub.publish(vel_msg);
    				ros::spinOnce();
			}
			curr_x++;
		}
		curr_x = n.x;
		curr_y = n.y;
		if(right_dist < dist){
			Node right;
			right.visited = false;
			if(current_orientation ==N){
				right.x = n.x + 1;
				right.y = n.y;			
			}else if(current_orientation ==W){
				right.y = n.y + 1;
				right.x = n.x;
			}else if(current_orientation ==E){
				right.y = n.y - 1;
				right.x = n.x;
			}else if(current_orientation ==S){
				right.y = n.y;
				right.x = n.x - 1;
			}
			qnodes.push(right);
		}
		if(left_dist < dist){
			Node right;
			right.visited = false;
			if(current_orientation ==N){
				right.x = n.x - 1;
				right.y = n.y;			
			}else if(current_orientation ==W){
				right.y = n.y - 1;
				right.x = n.x;
			}else if(current_orientation ==E){
				right.y = n.y + 1;
				right.x = n.x;
			}else if(current_orientation ==S){
				right.y = n.y;
				right.x = n.x + 1;
			}
			qnodes.push(right);
		}
		if(front_dist < dist){
			Node right;
			right.visited = false;
			if(current_orientation ==N){
				right.x = n.x;
				right.y = n.y+ 1;			
			}else if(current_orientation ==W){
				right.y = n.y;
				right.x = n.x - 1;
			}else if(current_orientation ==E){
				right.y = n.y;
				right.x = n.x + 1;
			}else if(current_orientation ==S){
				right.y = n.y - 1;
				right.x = n.x;
			}
			qnodes.push(right);
		}
	}
    }
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
