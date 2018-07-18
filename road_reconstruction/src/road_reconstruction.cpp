#include <ros/ros.h>

#include <laser_assembler/AssembleScans2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <road_reconstruction/TutorialsConfig.h>

#include <sensor_msgs/PointCloud2.h>

//  #include <novatel_gps_driver/novatel_gps.h>
#include <novatel_gps_msgs/Inspva.h>

//-----------------
#include <math.h>
#include <algorithm>  // std::max
#include <cmath>
#include <fstream>
#include <iostream>
//---------------------

using namespace laser_assembler;
// using namespace pcl;
double Raio, StDev;
int Viz, MeanK;
void callback(road_reconstruction::TutorialsConfig &config, uint32_t level)
{
  Raio = config.Radius;
  Viz = config.Viz;
  StDev = config.StDev;
  MeanK = config.MeanK;

  // ROS_INFO("Reconfigure Request: %f %d %f %d", config.raio, config.viz, StDev, MeanK);
}

class RoadReconst
{
public:
  // publicadores
  RoadReconst();

  // Filter the cloud based on recieved data
  void getVelocity(const novatel_gps_msgs::InspvaPtr &velMsg);
  void loop_function();

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_cloud0, pub_cloud3, pub_cloudTotal, pub_road_rec;
  pcl::PointCloud<pcl::PointXYZ> CloudXYZ_LD0, CloudXYZ_LD1, CloudXYZ_LD2, CloudXYZ_LD3, CloudXYZ_Total;
  sensor_msgs::PointCloud2 CloudMsg_LD0, CloudMsg_LD3, CloudMsg_Total, Cloud_Reconst;

  pcl::PointCloud<pcl::PointXYZ> Inter1, Inter2, InterM, RoadRec;

  dynamic_reconfigure::Server<road_reconstruction::TutorialsConfig> server;
  dynamic_reconfigure::Server<road_reconstruction::TutorialsConfig>::CallbackType f;

  // Subscriber for velocity data
  ros::Subscriber sub_getVel;

  // apagar-------------------
  // ros::Publisher pub_cloud_simple;
  // pcl::PointCloud<pcl::PointXYZ> CloudXYZ_Simple;
  // sensor_msgs::PointCloud2 CloudMsg_Simple;
  // apagar-------------------

  void getCloudsFromSensors();
  void cleanCloud();

  float RaioSpeed, VizSpeed;
};

void RoadReconst::getVelocity(const novatel_gps_msgs::InspvaPtr &velMsg)
{
  float N_vel, E_vel, U_vel;
  N_vel = velMsg->north_velocity;
  E_vel = velMsg->east_velocity;
  U_vel = velMsg->up_velocity;

  float carVelocity = sqrt(std::pow(N_vel, 2) + std::pow(E_vel, 2) + std::pow(U_vel, 2));

  // //---------Metodo Estatico --------------------
  // RaioSpeed = 0.19;
  // VizSpeed = 13;

  //---------Metodo dinamico 2 ------------------
  RaioSpeed = carVelocity / 50.0 + carVelocity / 350;
  float minLimit_Rad = 0.09;
  float minLimit_Viz = 6;
  // // limitar um raio minimo
  RaioSpeed = std::max(RaioSpeed, minLimit_Rad);

  VizSpeed = std::max((float)floor(carVelocity * 125.0 / 8.0 * 3.14 * std::pow(0.2, 2)), minLimit_Viz);

  //---------Metodo dinamico 1 ------------------
  // Equação deduzida pelo simulador para uma dist de acumulação de 4m para um raio de 0.2m
  // float minLimit_Viz = 6;
  // float maxLimit_Viz = 42;
  // RaioSpeed = 0.2;
  // VizSpeed = std::max((float)floor(4540 / carVelocity * std::pow(0.2, 2)), minLimit_Viz);
  // VizSpeed = std::min(VizSpeed, maxLimit_Viz);

  // ROS_INFO("Vels: %f %f %f and total %f", N_vel, E_vel, U_vel, carVelocity);
  ROS_INFO("Car_vel %f Filt Rad %f Viz %f", carVelocity, RaioSpeed, VizSpeed);
}

RoadReconst::RoadReconst()
{
  // Subscritores
  sub_getVel = nh_.subscribe("inspva", 10, &RoadReconst::getVelocity, this);

  // Publish clouds
  sensor_msgs::PointCloud2 CloudMsg_Simple;
  // pub_cloud0 = nh_.advertise<sensor_msgs::PointCloud2>("cloud_minada2", 100);
  // pub_cloud3 = nh_.advertise<sensor_msgs::PointCloud2>("cloud_minada3", 100);
  pub_road_rec = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("road_reconstruction", 1);
  pub_cloudTotal = nh_.advertise<sensor_msgs::PointCloud2>("cloud_Total", 100);
  // apagar-------------------------
  // pub_cloud_simple = nh_.advertise<sensor_msgs::PointCloud2>("cloud_simple", 100);
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
  //----------Assemble_0-------------------------------------
  ros::service::waitForService("assemble_scans0");
  ros::ServiceClient client_0 = nh_.serviceClient<AssembleScans2>("assemble_scans0");
  AssembleScans2 srv_0;
  srv_0.request.begin = ros::Time(0, 0);
  srv_0.request.end = ros::Time::now();
  if (client_0.call(srv_0))
  {
    // pub_cloud0.publish(srv.response.cloud);
    pcl::fromROSMsg(srv_0.response.cloud, CloudXYZ_LD0);
  }
  else
  {
    printf("Service call failed\n");
  }

  //----------Assemble_1-------------------------------------
  ros::service::waitForService("assemble_scans1");
  ros::ServiceClient client_1 = nh_.serviceClient<AssembleScans2>("assemble_scans1");
  AssembleScans2 srv_1;
  srv_1.request.begin = ros::Time(0, 0);
  srv_1.request.end = ros::Time::now();
  if (client_1.call(srv_1))
  {
    // pub_cloud0.publish(srv.response.cloud);
    pcl::fromROSMsg(srv_1.response.cloud, CloudXYZ_LD1);
  }
  else
  {
    printf("Service call failed\n");
  }

  //----------Assemble_2-------------------------------------

  ros::service::waitForService("assemble_scans2_");
  ros::ServiceClient client_2 = nh_.serviceClient<AssembleScans2>("assemble_scans2_");
  AssembleScans2 srv_2;
  srv_2.request.begin = ros::Time(0, 0);
  srv_2.request.end = ros::Time::now();
  if (client_2.call(srv_2))
  {
    // pub_cloud0.publish(srv_2.response.cloud);
    pcl::fromROSMsg(srv_2.response.cloud, CloudXYZ_LD2);
  }
  else
  {
    printf("Service call failed\n");
  }

  //----------Assemble_3-------------------------------------
  ros::service::waitForService("assemble_scans3");
  ros::ServiceClient client_3 = nh_.serviceClient<AssembleScans2>("assemble_scans3");
  AssembleScans2 srv_3;
  srv_3.request.begin = ros::Time(0, 0);
  srv_3.request.end = ros::Time::now();
  if (client_3.call(srv_3))
  {
    // pub_cloud3.publish(srv3.response.cloud);
    pcl::fromROSMsg(srv_3.response.cloud, CloudXYZ_LD3);
  }
  else
    printf("Service call failed\n");

  // Apagar isto depois -----------------------
  // ros::service::waitForService("assemble_single");
  // ros::ServiceClient client_simple = nh_.serviceClient<AssembleScans2>("assemble_single");
  // AssembleScans2 srv_simple;
  // srv_simple.request.begin = ros::Time(0, 0);
  // srv_simple.request.end = ros::Time::now();
  // if (client_simple.call(srv_simple))
  // {
  //   // printf("Got cloud 0 with %lu points\n", srv.response.cloud.data.size());
  //   pub_cloud_simple.publish(srv_simple.response.cloud);
  //   pcl::fromROSMsg(srv_simple.response.cloud, CloudXYZ_Simple);
  // }
  // else
  // {
  //   printf("Service call failed\n");
  // }
  //---------------------------------------------
  // std::ofstream myfile;
  // myfile.open("/home/tiago/catkin_ws_path/src/result_point.txt", std::ios::out | std::ios::app);

  // for (size_t i = 0; i < CloudXYZ_Simple.points.size(); i++)
  // {
  //   if (i == 51)  // 48 para o scan0; 41 para scan3
  //   {
  //     float S_x = CloudXYZ_Simple.points[i].x;
  //     float S_y = CloudXYZ_Simple.points[i].y;
  //     float S_z = CloudXYZ_Simple.points[i].z;
  //     myfile << S_x << "\t" << S_y << "\t" << S_z << '\n';
  //     ROS_INFO("Ponto %i: %f %f %f", i, S_x, S_y, S_z);
  //   }
  // }
  // myfile.close();
  // // ROS_INFO("Ponto: %f %f %f", Xpoint, Ypoint, Zpoint);

  // ROS_INFO("---------Nova Cloud-------------");
  // Apagar isto depois -----------------------

  //------------------Assemble all an publish---------------

  Inter1 = CloudXYZ_LD0 + CloudXYZ_LD1;  // 1-2 | first 2
  Inter2 = CloudXYZ_LD2 + CloudXYZ_LD3;  // 3-4 | last 2

  InterM = Inter1 + CloudXYZ_LD2;  // first 3 clouds
  RoadRec = Inter1 + Inter2;       // all clouds
  pcl::toROSMsg(InterM, Cloud_Reconst);
  pub_road_rec.publish(Cloud_Reconst);

  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(Inter1, minPt, maxPt);
  // ROS_INFO("Minim:%f  Maxim:%f", minPt.z, maxPt.z);
}

void RoadReconst::cleanCloud()
{
  tf::StampedTransform transformOdom;
  tf::TransformListener listener;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_clean(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromROSMsg(assembled_cloud_, *cloud_to_clean);
  *cloud_to_clean = RoadRec;

  // try
  // {
  //   listener.waitForTransform("map", "ground", ros::Time(0), ros::Duration(1.0));
  //   listener.lookupTransform("map", "ground", ros::Time(0), transformOdom);
  // }
  // catch (tf::TransformException &ex)
  // {
  //   ROS_ERROR("%s", ex.what());
  // }
  // // Localizaï¿½ï¿½o da origem do ref ground
  // float Xo = transformOdom.getOrigin().x();
  // float Yo = transformOdom.getOrigin().y();
  // float Zo = transformOdom.getOrigin().z();

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_border_filter(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());

  // // Condiï¿½ï¿½o para os limites da bounding box de representaï¿½ï¿½o da pointcloud
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
  //     new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, Xo - 2)));
  // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
  //     new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, Xo + 30)));

  // // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
  // //     new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, Yo - 10)));

  // // build the filter
  // pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  // condrem.setCondition(range_cond);
  // condrem.setInputCloud(cloud_to_clean);
  // condrem.setKeepOrganized(true);
  // // apply filter
  // condrem.filter(*cloud_border_filter);

  // Depois passar aqui um voxel filter para diminuir a densidade dos pontos
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredVox(new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(cloud_to_clean);
  vg.setLeafSize(0.06f, 0.06f, 0.06f);
  vg.filter(*cloud_filteredVox);

  // Filtro para selecionar os pontos das zonas mais densas

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredRad(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredStat(new pcl::PointCloud<pcl::PointXYZ>);
  // build the filter

  if (cloud_filteredVox->size() != 0)
  {
    // ROS_WARN("Empty Cloud");
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_filteredVox);
    outrem.setRadiusSearch(RaioSpeed);
    outrem.setMinNeighborsInRadius(VizSpeed);
    outrem.filter(*cloud_filteredRad);

    // if (cloud_filteredRad->size() != 0)
    // {
    //   // Create the filtering object
    //   pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    //   sor.setInputCloud(cloud_filteredRad);
    //   sor.setMeanK(MeanK);
    //   sor.setStddevMulThresh(StDev);
    //   // Outliers
    //   // sor.setNegative(true);
    //   sor.filter(*cloud_filteredStat);
    // }
  }

  CloudXYZ_Total = *cloud_filteredRad;
}

int main(int argc, char **argv)
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

//-------wrinting to file-------------
// std::ofstream myfile;
// myfile.open("/home/tiago/catkin_ws_path/src/result_point.txt", std::ios::out | std::ios::app);
// for (size_t i = 0; i < CloudXYZ_Simple.points.size(); i++)
// {
//   if (i == 41)
//   {
//     float S_x = CloudXYZ_Simple.points[i].x;
//     float S_y = CloudXYZ_Simple.points[i].y;
//     float S_z = CloudXYZ_Simple.points[i].z;
//     myfile << S_x << "\t" << S_y << "\t" << S_z << '\n';
//     // ROS_INFO("Ponto %i: %f %f %f", i, S_x, S_y, S_z);
//   }
// }
// myfile.close();
//-------write to file--------------

// try
// {
//   listener.waitForTransform("map", "ground", ros::Time(0), ros::Duration(1.0));
//   listener.lookupTransform("map", "ground", ros::Time(0), transformOdom);
// }
// catch (tf::TransformException& ex)
// {
//   ROS_ERROR("%s", ex.what());
// }
// // Localizaï¿½ï¿½o da origem do ref ground
// float Xo = transformOdom.getOrigin().x();
// float Yo = transformOdom.getOrigin().y();
// float Zo = transformOdom.getOrigin().z();

// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
// pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());

// // Condiï¿½ï¿½o para os limites da bounding box de representaï¿½ï¿½o da pointcloud
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
