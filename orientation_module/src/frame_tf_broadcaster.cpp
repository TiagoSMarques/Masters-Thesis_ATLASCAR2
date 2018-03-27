// #include <ros/ros.h>
// #include <tf/transform_broadcaster.h>

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "ground_tf_broadcaster");
//   ros::NodeHandle node;

//   tf::TransformBroadcaster br;
//   tf::Transform transform;

//   ros::Rate rate(50);
//   while (node.ok())
//   {
//     // distância entre eixos do carro: 2550
//     // distância total do carro: 3475
//     transform.setOrigin(tf::Vector3(-(0.5 + 2.550 / 2), 0, -0.28));
//     transform.setRotation(tf::Quaternion(0, 0, 0, 1));
//     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "ground"));
//     rate.sleep();
//   }
//   return 0;
// };

#include <gps_common/GPSFix.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "std_msgs/String.h"
// Data types
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

float dist_tot = 0;
class GroundPosition

{
public:
  GroundPosition();
  void loop_function();

  void getVel(const gps_common::GPSFixConstPtr& msg)
  {
    // Velocidade em m/s
    spp = msg->speed;
    // Assumindo que vem 1 mensagem por 1/10 seg (1/9.3 aprox)
    // A distância percorrida é:
    dist = spp * (1 / 9.3);
    dist_tot = dist_tot + dist;

    // ROS_INFO("Vrumm: %f, dist: %f", spp, dist_tot);
  }

private:
  ros::NodeHandle n;
  ros::Rate loop_rate;
  ros::Subscriber sub;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  float spp;
  float dist;
};

GroundPosition::GroundPosition() : loop_rate(50)
{
  sub = n.subscribe("/gps", 1, &GroundPosition::getVel, this);
}

void GroundPosition::loop_function()
{
  transform.setOrigin(tf::Vector3(-(0.5 + 2.550 / 2), 0, -0.28));
  // transform.setOrigin(tf::Vector3(dist_tot, 0, -0.28));
  transform.setRotation(tf::Quaternion(0, 0, 0, 1));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "ground"));
  loop_rate.sleep();
}

// Main
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ground_tf_broadcaster");
  GroundPosition ground_pos;
  while (ros::ok())
  {
    ground_pos.loop_function();
    ros::spinOnce();
  }
  return 0;
}
