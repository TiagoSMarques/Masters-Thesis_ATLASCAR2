#include <ros/ros.h>

#include <laser_assembler/AssembleScans2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <road_reconstruction/TutorialsConfig.h>

#include <sensor_msgs/PointCloud2.h>

using namespace laser_assembler;
// using namespace pcl;
double Raio;
int Viz;

void callback(road_reconstruction::TutorialsConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %f %d", config.raio, config.viz);
  Raio = config.raio;
  Viz = config.viz;
}

class RoadReconst
{
public:
  RoadReconst();
  void loop_function();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_cloud0, pub_cloud3, pub_cloudTotal;
  pcl::PointCloud<pcl::PointXYZ> CloudXYZ_LD0, CloudXYZ_LD3, CloudXYZ_Total;
  sensor_msgs::PointCloud2 CloudMsg_LD0, CloudMsg_LD3, CloudMsg_Total;

  dynamic_reconfigure::Server<road_reconstruction::TutorialsConfig> server;
  dynamic_reconfigure::Server<road_reconstruction::TutorialsConfig>::CallbackType f;

  void getCloudsFromSensors();
  void cleanCloud();
};

RoadReconst::RoadReconst()
{
  pub_cloud0 = nh_.advertise<sensor_msgs::PointCloud2>("cloud_minada0", 100);
  pub_cloud3 = nh_.advertise<sensor_msgs::PointCloud2>("cloud_minada3", 100);
  pub_cloudTotal = nh_.advertise<sensor_msgs::PointCloud2>("cloud_Total", 100);
}
void RoadReconst::loop_function()
{
  // Buscar os parametros

  getCloudsFromSensors();
  cleanCloud();
  // CloudXYZ_Total = (CloudXYZ_LD0 + CloudXYZ_LD3);
  pcl::toROSMsg(CloudXYZ_Total, CloudMsg_Total);
  pub_cloudTotal.publish(CloudMsg_Total);
}

void RoadReconst::getCloudsFromSensors()
{
  ros::service::waitForService("assemble_scans0");
  ros::ServiceClient client = nh_.serviceClient<AssembleScans2>("assemble_scans0");
  AssembleScans2 srv;
  srv.request.begin = ros::Time(0, 0);
  srv.request.end = ros::Time::now();

  if (client.call(srv))
  {
    // printf("Got cloud 0 with %lu points\n", srv.response.cloud.data.size());
    pub_cloud0.publish(srv.response.cloud);
    pcl::fromROSMsg(srv.response.cloud, CloudXYZ_LD0);
  }
  else
  {
    printf("Service call failed\n");
  }

  ros::service::waitForService("assemble_scans3");
  ros::ServiceClient client3 = nh_.serviceClient<AssembleScans2>("assemble_scans3");
  AssembleScans2 srv3;

  srv3.request.begin = ros::Time(0, 0);
  srv3.request.end = ros::Time::now();
  if (client3.call(srv3))
  {
    // printf("Got cloud 3 with %lu points\n", srv3.response.cloud.data.size());
    // pub_cloud3.publish(srv3.response.cloud);
    pcl::fromROSMsg(srv3.response.cloud, CloudXYZ_LD3);
  }
  else
    printf("Service call failed\n");
}

void RoadReconst::cleanCloud()
{
  tf::StampedTransform transformOdom;
  tf::TransformListener listener;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_clean(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromROSMsg(assembled_cloud_, *cloud_to_clean);
  *cloud_to_clean = (CloudXYZ_LD0 + CloudXYZ_LD3);

  // try
  // {
  //   listener.waitForTransform("map", "ground", ros::Time(0), ros::Duration(1.0));
  //   listener.lookupTransform("map", "ground", ros::Time(0), transformOdom);
  // }
  // catch (tf::TransformException& ex)
  // {
  //   ROS_ERROR("%s", ex.what());
  // }
  // // Localização da origem do ref ground
  // float Xo = transformOdom.getOrigin().x();
  // float Yo = transformOdom.getOrigin().y();
  // float Zo = transformOdom.getOrigin().z();

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());

  // // Condição para os limites da bounding box de representação da pointcloud
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
  //     new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, Xo - 50)));
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
  //     new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, Xo + 50)));

  // // build the filter
  // pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  // condrem.setCondition(range_cond);
  // condrem.setInputCloud(cloud_to_clean);
  // condrem.setKeepOrganized(true);
  // // apply filter
  // condrem.filter(*cloud_filtered);

  // Depois passar aqui um voxel filter para diminuir a densidade dos pontos
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredVox(new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(cloud_to_clean);
  vg.setLeafSize(0.1f, 0.1f, 0.1f);
  vg.filter(*cloud_filteredVox);

  // Filtro para selecionar os pontos das zonas mais densas

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredRad(new pcl::PointCloud<pcl::PointXYZ>);
  // build the filter

  if (cloud_filteredVox->size() != 0)
  {
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
    // ROS_WARN("Empty Cloud");
    outrem.setInputCloud(cloud_filteredVox);
    outrem.setRadiusSearch(Raio);
    outrem.setMinNeighborsInRadius(Viz);
    // apply filter
    outrem.filter(*cloud_filteredRad);
  }
  CloudXYZ_Total = *cloud_filteredRad;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RoadReconst");
  RoadReconst reconstruct;

  ros::Rate rate(50);
  while (ros::ok())
  {
    reconstruct.loop_function();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
