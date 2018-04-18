#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include <termios.h>
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include "Mocap.hpp"
#include <memory>

using namespace std;

int main (int argc, char* argv[]) {

 
Mocap mocap(argc, argv);
Eigen::Vector3d retPos;
Eigen::Quaterniond retOrient;

ros::init(argc, argv, "mocap_ros2_1");
ros::NodeHandle n;
ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("optitrack", 1000);
ros::Rate loop_rate(200);

uint seq=0;
int count = 0;
  while (ros::ok())
  {
      mocap.getLatestPose(retPos, retOrient);
      geometry_msgs::Point point;
      point.x = retPos.x();
      point.y = retPos.y();
      point.z = retPos.z();
      
      geometry_msgs::Quaternion quat;
      quat.x = retOrient.x();
      quat.y = retOrient.y();
      quat.z = retOrient.z();
      quat.w = retOrient.w();
      
      geometry_msgs::Pose pose;
      pose.position = point;
      pose.orientation = quat;

      std_msgs::Header header;
      header.stamp = ros::Time::now();

      string frame_id; 
      seq++;
      header.seq = seq; 
      header.frame_id = "0";

      geometry_msgs::PoseStamped posestamped;
      posestamped.pose = pose;
      posestamped.header = header; 

      
     chatter_pub.publish(posestamped);
     ros::spinOnce();
     loop_rate.sleep();
     ++count;

  }
                     
}


