#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Pose2D pose_msg;
ros::Time current_time, last_time;
std_msgs::UInt8MultiArray array_msg;

tf::TransformBroadcaster *odom_broadcaster_ptr;

double pre_x=0;
double pre_y=0;
double pre_th=0;
double vx;
double vy;
double vth;

ros::Publisher ir_front_pub, ir_left_pub, ir_right_pub, odom_pub, set_pose_pub, rgb_leds_pub, cmd_vel_pub;

void front_distanceCallback(const std_msgs::Float32& msg){
	sensor_msgs::Range rangeMsg;	
	rangeMsg.header.stamp = ros::Time::now();
	rangeMsg.header.frame_id = "front_ir";
	rangeMsg.radiation_type = 1,                     //0=ultrasonic, 1=IR
	rangeMsg.field_of_view = 0.034906585;
	rangeMsg.min_range = 0.1;
	rangeMsg.max_range = 0.8;
	rangeMsg.range = msg.data;
	
	ir_front_pub.publish(rangeMsg);

}
void left_distanceCallback(const std_msgs::Float32& msg){
	sensor_msgs::Range rangeMsg;	
	rangeMsg.header.stamp = ros::Time::now();
	rangeMsg.header.frame_id = "left_ir";
	rangeMsg.radiation_type = 1,                     //0=ultrasonic, 1=IR
	rangeMsg.field_of_view = 0.034906585;
	rangeMsg.min_range = 0.1;
	rangeMsg.max_range = 0.8;
	rangeMsg.range = msg.data;
	
	ir_left_pub.publish(rangeMsg);

}
void right_distanceCallback(const std_msgs::Float32& msg){
	sensor_msgs::Range rangeMsg;	
	rangeMsg.header.stamp = ros::Time::now();
	rangeMsg.header.frame_id = "right_ir";
	rangeMsg.radiation_type = 1,                     //0=ultrasonic, 1=IR
	rangeMsg.field_of_view = 0.034906585;
	rangeMsg.min_range = 0.1;
	rangeMsg.max_range = 0.8;
	rangeMsg.range = msg.data;
	
	ir_right_pub.publish(rangeMsg);

}
void poseCallback(const geometry_msgs::Pose2D& msg)
{
	
         current_time = ros::Time::now();

         geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(msg.theta);
	

	// ********************* FOR BROADCASTING ********************************************8///
	 geometry_msgs::TransformStamped odom_trans;
    	 odom_trans.header.stamp = current_time;
    	 odom_trans.header.frame_id = "odom";
    	 odom_trans.child_frame_id = "base_link";

         odom_trans.transform.translation.x = msg.x;
         odom_trans.transform.translation.y = msg.y;
         odom_trans.transform.translation.z = 0.0;
         odom_trans.transform.rotation = odom_quat;

         //send the transform
         odom_broadcaster_ptr->sendTransform(odom_trans);
	//****************************************************************************************8///

        //*************************** FOR PUBLIHSING **********************************************////
	 nav_msgs::Odometry odom;
         odom.header.stamp = current_time;
         odom.header.frame_id = "odom";
         //set the position
         odom.pose.pose.position.x = msg.x;
         odom.pose.pose.position.y = msg.y;
         odom.pose.pose.position.z = 0.0;
         odom.pose.pose.orientation = odom_quat;
     
	 double time_difference=(current_time.toSec() - last_time.toSec());
	 vx = ((double)msg.x-pre_x) / time_difference;
	 vy = ((double)msg.y-pre_y) / time_difference;
	 vth = ((double)msg.theta-pre_th) / time_difference;

         //set the velocity
         odom.child_frame_id = "base_link";
         odom.twist.twist.linear.x = vx;
         odom.twist.twist.linear.y = vy;
         odom.twist.twist.angular.z = vth;

     	 //updating the previos x y theta 
	 pre_x = msg.x;
	 pre_y = msg.y;
	 pre_th = msg.theta;
	 last_time = current_time;

         //publish the message
         odom_pub.publish(odom);
	//*****************************************************************************************/////////
}

void setLeds(/*int led0, int led1, int led2, int led3, int led4, int led5*/){
  array_msg.data.clear();
  array_msg.data.push_back(200);
  array_msg.data.push_back(0);
  array_msg.data.push_back(0);
  array_msg.data.push_back(0);
  array_msg.data.push_back(0);
  array_msg.data.push_back(200);
  rgb_leds_pub.publish(array_msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle n;

  tf::TransformBroadcaster odom_broadcaster;
  odom_broadcaster_ptr = &odom_broadcaster;


  ros::Subscriber pose_sub = n.subscribe("pose", 1000, poseCallback);

  ros::Subscriber ir_front_sensor_sub = n.subscribe("front_distance", 1000, front_distanceCallback);
  ros::Subscriber ir_left_sensor_sub = n.subscribe("left_distance", 1000, left_distanceCallback);
  ros::Subscriber ir_right_sensor_sub = n.subscribe("right_distance", 1000, right_distanceCallback);
  
  set_pose_pub = n.advertise<geometry_msgs::Pose2D>("set_pose",5);
  rgb_leds_pub = n.advertise<std_msgs::UInt8MultiArray>("rgb_leds",50);
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",5); //cmd_vel

  ir_front_pub = n.advertise<sensor_msgs::Range>("ir_front_sensor", 5);
  ir_left_pub = n.advertise<sensor_msgs::Range>("ir_left_sensor", 5);
  ir_right_pub = n.advertise<sensor_msgs::Range>("ir_right_sensor", 5);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50); 
  ros::Rate loop_rate(10);
  geometry_msgs::Twist msg;

 while (ros::ok())
  {
    last_time = ros::Time::now();
    setLeds();
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  //ros::spin();
  return 0;
}

