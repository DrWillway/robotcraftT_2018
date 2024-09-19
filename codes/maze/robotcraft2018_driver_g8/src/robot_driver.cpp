#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Range.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"
#include <sstream>
float x,y,th,distF,distL,distR;

void poseCallback(const geometry_msgs::Pose2D& pose_msg){
  x=pose_msg.x;
  y=pose_msg.y;
  th=pose_msg.theta;
}
void distFCallback(const std_msgs::Float32& dist_msg){
  distF=dist_msg.data;
}
void distRCallback(const std_msgs::Float32& dist_msg){
  distR=dist_msg.data;
}
void distLCallback(const std_msgs::Float32& dist_msg){
  distL=dist_msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle n;
  //Subscribers
  ros::Subscriber pose_sub = n.subscribe("/pose", 1000, poseCallback);
  ros::Subscriber front_distance_sub = n.subscribe("/front_distance", 1000, distFCallback);
  ros::Subscriber right_distance_sub = n.subscribe("/right_distance", 1000, distRCallback);
  ros::Subscriber left_distance_sub = n.subscribe("/left_distance", 1000, distLCallback);
  //Publishers
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
  ros::Publisher ir_front_sensor_pub = n.advertise<sensor_msgs::Range>("/ir_front_sensor", 5);
  ros::Publisher ir_right_sensor_pub = n.advertise<sensor_msgs::Range>("/ir_right_sensor", 5);
  ros::Publisher ir_left_sensor_pub = n.advertise<sensor_msgs::Range>("/ir_left_sensor", 5);

  tf::TransformBroadcaster odom_broadcaster;
  sensor_msgs::Range rangeFMsg;
  sensor_msgs::Range rangeLMsg;
  sensor_msgs::Range rangeRMsg;
  
  ros::Time current_time, last_time;

  ros::Rate loop_rate(10);
  while (ros::ok()){
    current_time = ros::Time::now();

    rangeFMsg.header.stamp = current_time;
    rangeFMsg.header.frame_id = "front_ir";
	rangeFMsg.radiation_type = 1;   //0 ultrasonic, 1 infrared
    rangeFMsg.field_of_view = 0.034906585;
	rangeFMsg.min_range = 0.1;
	rangeFMsg.max_range = 0.8;
	rangeFMsg.range = distF;

    rangeLMsg.header.stamp = current_time;
    rangeLMsg.header.frame_id = "left_ir";
	rangeLMsg.radiation_type = 1;   //0 ultrasonic, 1 infrared
    rangeLMsg.field_of_view = 0.034906585;
	rangeLMsg.min_range = 0.1;
	rangeLMsg.max_range = 0.8;
	rangeLMsg.range = distL;

    rangeRMsg.header.stamp = current_time;
    rangeRMsg.header.frame_id = "right_ir";
	rangeRMsg.radiation_type = 1;   //0 ultrasonic, 1 infrared
    rangeRMsg.field_of_view = 0.034906585;
	rangeRMsg.min_range = 0.1;
	rangeRMsg.max_range = 0.8;
	rangeRMsg.range = distR;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

/*    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
*/
    //publish the message
    odom_pub.publish(odom);
    ir_front_sensor_pub.publish(rangeFMsg);
    ir_left_sensor_pub.publish(rangeLMsg);
    ir_right_sensor_pub.publish(rangeRMsg);

    ros::spinOnce();
    loop_rate.sleep();
    last_time = ros::Time::now();
  }
  return 0;
}
