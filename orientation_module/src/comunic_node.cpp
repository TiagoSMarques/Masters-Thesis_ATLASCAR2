
// -----Includes -----
#include <algorithm>
#include <iostream>
#include <vector>
// Boost includes
#include <boost/shared_ptr.hpp>

// Ros includes
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>  //Messages on sreen

// Data types
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#define DEFAULT_TIME 0.03

using namespace ros;

using namespace std;

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

void printData(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  // float dstride0 = msg->layout.dim[0].stride;
  // float dstride1 = msg->layout.dim[1].stride;
  // float h = msg->layout.dim[0].size;
  // float w = msg->layout.dim[1].size;
  // ROS_INFO("Pitch =  %f", msg->data[0 + dstride1 * 0]);
  // ROS_INFO("Roll = %f", msg->data[0 + dstride1 * 1]);

  float pitch = msg->data[0];
  float roll = msg->data[1];
  float z_mean = msg->data[2];

  ROS_INFO("Pitch = %f", pitch);
  ROS_INFO("Roll = %f", roll);

  // Criar a transformação entre chassis e estrada
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  transform.setOrigin(tf::Vector3(0.0, 0.0, z_mean));
  transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, 0));

  // publicar a tranformada entre o chassis do carro e a estrada
  br.sendTransform(tf::StampedTransform(transform, Time::now(), "/ground", "/atc/vehicle/center_car_axis"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Rec_inclin");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("DadosInclin", 1000, printData);

  ros::spin();

  return 0;
}
