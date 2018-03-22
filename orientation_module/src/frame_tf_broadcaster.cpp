#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ground_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (node.ok())
  {
    // distância entre eixos do carro: 2550
    // distância total do carro: 3475
    transform.setOrigin(tf::Vector3(-(0.5 + 2.550 / 2), 0, -0.28));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/ground"));
    rate.sleep();
  }
  return 0;
};
