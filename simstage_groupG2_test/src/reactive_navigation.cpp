// "reactive_navigation" node: subscribes laser data and publishes velocity commands

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

double obstacle_distance,obstacle_distance_left,obstacle_distance_front,obstacle_distance_right;
bool front = true;
bool left = true;
bool right = true;
float x,z;
float count=0;
long count_front=0;


int spiral=1.57;
int count_sp=0;
float dist=0.7;
int change = 1;
int change2 = 1;


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  obstacle_distance = *std::min_element (msg->ranges.begin(), msg->ranges.end());
  obstacle_distance_right = *std::min_element (msg->ranges.begin(), msg->ranges.begin()+80);
  obstacle_distance_front = *std::min_element (msg->ranges.begin()+80, msg->ranges.begin()+160);
  obstacle_distance_left = *std::min_element (msg->ranges.begin()+160, msg->ranges.end());
}

void timerCallback(const ros::TimerEvent&)
{
  ROS_INFO("Lost");
  x = 0.8;
  z = 0.0;
}

int main(int argc, char **argv){
    
  ros::init(argc, argv, "reactive_navigation");
  
  ros::NodeHandle n;

  //Publisher for /cmd_vel
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  //Subscriber for /base_scan
  ros::Subscriber laser_sub = n.subscribe("base_scan", 100, laserCallback);

  ros::Rate loop_rate(10); //10 Hz

  //initializations:
  geometry_msgs::Twist cmd_vel_msg;
  bool side=true; //right wall
  bool sw=true;

  while (ros::ok()){

      ROS_INFO("%f",count);
      if(count<27){
            count+=0.1;
      }
      else if(count<(rand() % ((60 + 40) + 40))){
            count+=0.1;
            if(obstacle_distance>0.7){
                  ROS_INFO("Im one");
                  cmd_vel_msg.linear.x = 2.0;
                  if(side) cmd_vel_msg.angular.z = -0.5;
                  else cmd_vel_msg.angular.z = 0.5;
            }
      }
      else{
            count=0.0;
            side=!side;
      }
      if(obstacle_distance>2){
            ROS_INFO("Im two");
            cmd_vel_msg.linear.x = 2.0;
            if(side) cmd_vel_msg.angular.z = 0.2;
            else cmd_vel_msg.angular.z = -0.2;
      } 
      else if(obstacle_distance<=1.2 && obstacle_distance>0.7){
            ROS_INFO("Im three");
            cmd_vel_msg.linear.x = 2.0;
            if(side) cmd_vel_msg.angular.z = 1.2;
            else cmd_vel_msg.angular.z = -1.2;
      } 
      else if(obstacle_distance<=0.7){
            ROS_INFO("Im four");
            cmd_vel_msg.linear.x = 0.0;
            if(side) cmd_vel_msg.angular.z = -2.0;
            else cmd_vel_msg.angular.z = 2.0;
      }else ROS_INFO("Im five");

    //publish velocity commands:
    cmd_vel_pub.publish(cmd_vel_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

