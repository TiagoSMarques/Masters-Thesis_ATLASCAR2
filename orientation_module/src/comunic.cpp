
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

class Comunication
{
public:
  Comunication()
  {
    sub_ = nh_.subscribe("DadosInclin", 100, &Comunication::printData, this);
  }

  void printData(const std_msgs::Float32MultiArray::ConstPtr& msg)
  {
    // Criar a transformação entre chassis e estrada
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    // Receber e converter para radianos
    float pitch = msg->data[0] * 3.1415 / 180;
    float roll = msg->data[1] * 3.1415 / 180;
    // float z_mean = msg->data[2];
    pitch = -3.1415 / 10;
    // aqui depois colocar zmean
    transform.setOrigin(tf::Vector3(0, 0, 0.24));
    transform.setRotation(tf::createQuaternionFromRPY(roll, pitch, 0));

    // publicar a tranformada entre o chassis do carro e a estrada
    br.sendTransform(tf::StampedTransform(transform, Time::now(), "/ground", "/car_center"));
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
};

// void Comunication::loop_function()
// {
//   pub_.publish(msg);
//   loop_rate.sleep();
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Comunication");
  Comunication comunic;

  ros::spin();

  return 0;
}
