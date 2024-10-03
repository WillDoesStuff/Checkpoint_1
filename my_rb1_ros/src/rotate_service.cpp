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

float rotation_rate;
const float cycle_time = 5.0;

ros::ServiceServer rotate_service;
ros::Publisher pub;
ros::Subscriber sub;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
  rotation_rate = abs(msg->twist.twist.angular.z * (180 / M_PI)); // degree/sec
}

bool rotate_callback(my_rb1_ros::Rotate::Request &req,
                     my_rb1_ros::Rotate::Response &res) {

  ROS_INFO("Service Requested");

  ros::Rate loop_rate(cycle_time);
  geometry_msgs::Twist vel;

  float count_time = 0;
  float rotate_time = 0;

  if (req.degrees > 0) {
    vel.angular.z = 0.3;
  } else {
    vel.angular.z = -0.3;
  }

  // Start Rotation
  pub.publish(vel);

  while (ros::ok()) {
    rotate_time =
        (static_cast<float>(abs(req.degrees)) / rotation_rate) * cycle_time;
    count_time++;
    ros::spinOnce();
    loop_rate.sleep();

    if (rotate_time < count_time) {
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