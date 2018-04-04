
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

using namespace ros;

using namespace std;

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

class Comunication
{
public:
  Comunication()
  {
    sub = nh.subscribe("DadosInclin", 100, &Comunication::printData, this);
  }

  // void loop_function()
  // {
  // }

  void printData(const std_msgs::Float32MultiArray::ConstPtr& msg)
  {
    // Criar a transformação entre chassis e estrada
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    // Receber e converter para radianos (remover o 4.8)
    float pitch = (msg->data[0] + 4.8) * 3.1415 / 180;
    float roll = msg->data[1] * 3.1415 / 180;
    float z_mean = msg->data[2] / 1000;
    // pitch = -3.1415 / 10;
    // ROS_INFO("P: %f, R: %f, Z_m: %f", msg->data[0] + 4.8, msg->data[1], z_mean);
    // aqui depois colocar zmean
    transform.setOrigin(tf::Vector3(0, 0, z_mean));
    transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, 0));

    // publicar a tranformada entre o chassis do carro e a estrada
    br.sendTransform(tf::StampedTransform(transform, Time::now(), "ground", "car_center"));
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher vis_pub;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Comunication");
  // ros::NodeHandle nh;
  Comunication comunic;

  while (ros::ok())
  {
    // comunic.loop_function();
    ros::spinOnce();
  }

  return 0;
}
