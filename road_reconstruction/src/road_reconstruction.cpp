#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

class My_Filter
{
public:
  My_Filter();
  void scanCallback3(const sensor_msgs::LaserScan::ConstPtr& scan);
  void scanCallback2(const sensor_msgs::LaserScan::ConstPtr& scan);

private:
  ros::NodeHandle node_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener tfListener_;
  ros::Publisher point_cloud_publisher_;
  ros::Subscriber scan_sub3_;
  ros::Subscriber scan_sub2_;
  sensor_msgs::PointCloud2 cloud;
};

My_Filter::My_Filter()
{
  scan_sub3_ = node_.subscribe<sensor_msgs::LaserScan>("/ld_rms/scan3", 100, &My_Filter::scanCallback3, this);
  // scan_sub2_ = node_.subscribe<sensor_msgs::LaserScan>("/ld_rms/scan2", 100, &My_Filter::scanCallback2, this);
  point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>("/my_cloud_in", 100, false);
  // tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void My_Filter::scanCallback3(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // projector_.projectLaser(*scan, cloud);
  if (!tfListener_.waitForTransform(
          scan->header.frame_id, "map",
          scan->header.stamp + ros::Duration().fromSec(scan->ranges.size() * scan->time_increment), ros::Duration(1.0)))
  {
    return;
  }

  projector_.transformLaserScanToPointCloud("map", *scan, cloud, tfListener_);
  point_cloud_publisher_.publish(cloud);
}
// void My_Filter::scanCallback2(const sensor_msgs::LaserScan::ConstPtr& scan)
// {
//   // projector_.projectLaser(*scan, cloud);
//   if (!tfListener_.waitForTransform(
//           scan->header.frame_id, "map",
//           scan->header.stamp + ros::Duration().fromSec(scan->ranges.size() * scan->time_increment),
//           ros::Duration(1.0)))
//   {
//     return;
//   }

//   projector_.transformLaserScanToPointCloud("map", *scan, cloud, tfListener_);
//   point_cloud_publisher_.publish(cloud);
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_filter");
  My_Filter filter;

  ros::spin();

  return 0;
}
