#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class topologyAnalyser
{
public:
  topologyAnalyser();
  void loop_function();

private:
  ros::NodeHandle n;
  ros::Subscriber sub_scan0;
  ros::Subscriber sub_scan1;
  ros::Subscriber sub_scan2;
  ros::Subscriber sub_scan3;

  void Callback_scan0(const sensor_msgs::LaserScan::ConstPtr& msg_0);
  void Callback_scan1(const sensor_msgs::LaserScan::ConstPtr& msg_1);
  void Callback_scan2(const sensor_msgs::LaserScan::ConstPtr& msg_2);
  void Callback_scan3(const sensor_msgs::LaserScan::ConstPtr& msg_3);

  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud2 cloud0, cloud1, cloud2, cloud3;
};

topologyAnalyser::topologyAnalyser()
{
  sub_scan0 = n.subscribe("scan0", 10, &topologyAnalyser::Callback_scan0, this);
  sub_scan1 = n.subscribe("scan1", 10, &topologyAnalyser::Callback_scan1, this);
  sub_scan2 = n.subscribe("scan2", 10, &topologyAnalyser::Callback_scan2, this);
  sub_scan3 = n.subscribe("/ld_rms/scan0", 10, &topologyAnalyser::Callback_scan3, this);
}

void topologyAnalyser::loop_function()
{
}

void topologyAnalyser::Callback_scan0(const sensor_msgs::LaserScan::ConstPtr& scan_0)
{
}

void topologyAnalyser::Callback_scan1(const sensor_msgs::LaserScan::ConstPtr& scan_1)
{
}

void topologyAnalyser::Callback_scan2(const sensor_msgs::LaserScan::ConstPtr& scan_2)
{
  float Nelem = scan_2->ranges.size();
  float res = scan_2->angle_increment;

  ROS_INFO("N points simul, resolucao: %f  -- %f", Nelem, res);
}

void topologyAnalyser::Callback_scan3(const sensor_msgs::LaserScan::ConstPtr& scan_3)
{
  float Nelem = scan_3->ranges.size();
  float res = scan_3->angle_increment;

  ROS_INFO("Number of points real, resolucao: %f -- %f", Nelem, res);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "topologyAnalyser");
  // ros::NodeHandle n;
  topologyAnalyser top_analyser;
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    top_analyser.loop_function();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
