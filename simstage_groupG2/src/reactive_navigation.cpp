// "reactive_navigation" node: subscribes laser data and publishes velocity commands

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

double obstacle_distance_left,obstacle_distance_front,obstacle_distance_right;
bool front = true;
bool left = true;
bool right = true;
float x,z;
long count=0;
long count_front=0;
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  obstacle_distance_right = *std::min_element (msg->ranges.begin(), msg->ranges.begin()+59);
  obstacle_distance_front = *std::min_element (msg->ranges.begin()+60, msg->ranges.begin()+119);
  obstacle_distance_left = *std::min_element (msg->ranges.begin()+120, msg->ranges.end());
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

  while (ros::ok()){
    
    //fill the "cmd_vel_msg" data according to some conditions (depending on laser data)
    
   /* if (obstacle_distance_front >= 1.5 && obstacle_distance_right >= 1.5 && obstacle_distance_left >= 1.5){
          ROS_INFO("go_forward lost");
          cmd_vel_msg.linear.x = 0.8;
          cmd_vel_msg.angular.z = 0.2;            
    }*/
 /*   if (obstacle_distance_left <= 1.0){
          left = true;
    } 
    if (obstacle_distance_right <= 1.0){
          right = true;
    }
    if (obstacle_distance_front >= 1.0 && (obstacle_distance_left > 1.0 && obstacle_distance_left < 1.0)||(obstacle_distance_right < 1.0 &&obstacle_distance_right > 0.5)){
          ROS_INFO("go_forward");
          cmd_vel_msg.linear.x = 0.8;
          cmd_vel_msg.angular.z = 0.0;            
    }else if (obstacle_distance_left >= 1.5){
          if(obstacle_distance_right >= 1.5 && obstacle_distance_front >= 1.5 && left == false) {
            ROS_INFO("go_forward in left");
            cmd_vel_msg.linear.x = 0.8;
            cmd_vel_msg.angular.z = 0.0;       
          }else{
            ROS_INFO("turn_left");
            cmd_vel_msg.linear.x = 0.2;
            cmd_vel_msg.angular.z = 1.57; 
          }   
          left = false;      
    }else if(obstacle_distance_right >= 1.5){
          if(obstacle_distance_left >= 1.5 && obstacle_distance_front >= 1.5 && right == false){
            ROS_INFO("go_forward in right");
            cmd_vel_msg.linear.x = 0.8;
            cmd_vel_msg.angular.z = 0.0;   
          }else{
          ROS_INFO("turn_right");
          cmd_vel_msg.linear.x = 0.1;
          cmd_vel_msg.angular.z = -1.57;   
          }   
          right = false;    
    }else {
          ROS_INFO("turn_around");
          cmd_vel_msg.linear.x = -0.2;
          cmd_vel_msg.angular.z = -3.14;
    } 
*/
      /*if (obstacle_distance_left >= 1.0){
            left = true;
      }
      if (obstacle_distance_right >= 1.0){
            right = true;
      }
      if (obstacle_distance_front >= 1.0 && (left || right)){
            ROS_INFO("go_forward");
            cmd_vel_msg.linear.x = 0.8;
            cmd_vel_msg.angular.z = 0.0;  
      }else if (obstacle_distance_left >= 1.0){
            left = false;
            ROS_INFO("turn_left");
            cmd_vel_msg.linear.x = 0.4;
            cmd_vel_msg.angular.z = 1.57;    
      }else if(obstacle_distance_right >= 1.0){
            right = false;
            ROS_INFO("turn_right");
            cmd_vel_msg.linear.x = 0.1;
            cmd_vel_msg.angular.z = -1.57;     
      }else{
          ROS_INFO("turn_around");
          cmd_vel_msg.linear.x = -0.5;
          cmd_vel_msg.angular.z = -3.14;
    }*/ 
      if(obstacle_distance_left > 1.0){
            count_front=0;
            count++;
            if(count < 50){
                  ROS_INFO("turn_left");
                  cmd_vel_msg.linear.x = 0.2;
                  cmd_vel_msg.angular.z = 1.57;
            }else{
                  ROS_INFO("go_forward after long turning");
                  cmd_vel_msg.linear.x = 0.8;
                  cmd_vel_msg.angular.z = 0.1;
            }
      }else if(obstacle_distance_front > 1.0){
            count=0;
            if(obstacle_distance_front > 1.0 && (obstacle_distance_right < 1.0 || obstacle_distance_left < 1.0) ){
                  if(count_front < 10){
                        ROS_INFO("go_forward");
                        cmd_vel_msg.linear.x = 0.8;
                        cmd_vel_msg.angular.z = 0.1;
                  }
                  else {
                        ROS_INFO("rotate after forward");
                        for(int i=0; i<30; i++){
                              cmd_vel_msg.linear.x = 0.0;
                              cmd_vel_msg.angular.z = 1.3;
                        }
                        ROS_INFO("go_forward");
                        cmd_vel_msg.linear.x = 0.8;
                        cmd_vel_msg.angular.z = 0.1;
                        count_front=0;
                  }
            }
      }else if(obstacle_distance_right > 1.0){
            count=0;
            count_front=0;
            ROS_INFO("turn_right");
            cmd_vel_msg.linear.x = 0.2;
            cmd_vel_msg.angular.z = -1.57;
      }else{
            ROS_INFO("turn_around");
            cmd_vel_msg.linear.x = -0.5;
            cmd_vel_msg.angular.z = -3.14;
      }
    //publish velocity commands:
    cmd_vel_pub.publish(cmd_vel_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

