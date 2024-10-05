#include "geometry_msgs/Quaternion.h"
#include "my_rb1_ros/Rotate.h"
#include "ros/init.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <string>

float rb1_yaw;
float cycle_time = 1000;

ros::ServiceServer rotate_service;
ros::Publisher pub;
ros::Subscriber sub;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {

  geometry_msgs::Quaternion q = msg->pose.pose.orientation;

  float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);

  rb1_yaw = std::atan2(siny_cosp, cosy_cosp);
}

bool rotate_callback(my_rb1_ros::Rotate::Request &req,
                     my_rb1_ros::Rotate::Response &res) {

  ROS_INFO("Service Requested");

  ros::Rate loop_rate(cycle_time);
  geometry_msgs::Twist vel;
  float countTime = 0.0;
  float request_yaw = (req.degrees * M_PI / 180) + rb1_yaw;
  float delta_yaw = abs(request_yaw - rb1_yaw);

  if (req.degrees > 0) {
    vel.angular.z = 0.6;
  } else {
    vel.angular.z = -0.6;
  }

  float delta_time = (delta_yaw / abs(vel.angular.z)) * cycle_time;

  while (ros::ok()) {
    pub.publish(vel);

    countTime++;
    ros::spinOnce();
    loop_rate.sleep();

    if (delta_time <= countTime) {
      break;
    }
  }

  //  Stop Rotation
  vel.angular.z = 0.0;
  pub.publish(vel);

  res.result =
      "Successfully rotated RB1 " + std::to_string(req.degrees) + " degrees";

  ROS_INFO("Service Completed");
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "my_rb1_node");
  ros::NodeHandle nh;

  rotate_service = nh.advertiseService("/rotate_robot", rotate_callback);
  sub = nh.subscribe("odom", 1000, odom_callback);
  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ROS_INFO("Service Ready");

  ros::spin();

  return 0;
}