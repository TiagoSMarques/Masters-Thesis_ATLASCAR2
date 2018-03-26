
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

  // Receber e converter para radianos
  float pitch = msg->data[0] * 3.1415 / 180;
  float roll = msg->data[1] * 3.1415 / 180;
  // float z_mean = msg->data[2];

  // ROS_INFO("Pitch = %f", pitch);
  // ROS_INFO("Roll = %f", roll);

  // Criar a transformação entre chassis e estrada
  static tf::TransformBroadcaster br;
  tf::Transform transform;

  // aqui depois colocar zmean
  transform.setOrigin(tf::Vector3(0, 0, 0.24));
  // float pitch = -3.1415 / 10;
  // float roll = 0;

  transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, 0));

  // publicar a tranformada entre o chassis do carro e a estrada
  br.sendTransform(tf::StampedTransform(transform, Time::now(), "/ground", "/car_center"));
}

void drawMarkerLine(ros::NodeHandle n)
{
  tf::StampedTransform transform;
  tf::TransformListener listener;
  tf::Vector3 origin;

  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  try
  {
    listener.lookupTransform("ground", "moving_axis", ros::Time(10), transform);
    origin = transform.getOrigin();
    // // Marcador
    visualization_msgs::Marker marker;
    marker.header.frame_id = "moving_axis";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    // // marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = origin.x();
    marker.pose.position.y = origin.y();
    marker.pose.position.z = origin.z();
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.x = 1.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 10;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    ROS_INFO("Here!!!!!!!!!!!!!1");
    vis_pub.publish(marker);
  }

  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    // continue;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Rec_inclin");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("DadosInclin", 1000, printData);
  drawMarkerLine(n);

  ros::spin();

  return 0;
}
