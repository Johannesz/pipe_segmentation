#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/conversions.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

ros::Subscriber _cloudInput;
ros::Publisher  _modelCoefficient;

void callbackFindPipe(const PointCloud::ConstPtr& msg) {

   pcl::NormalEstimation<pcl::PointXYZHSV, pcl::Normal> ne_cy;
   pcl::search::KdTree<pcl::PointXYZHSV>::Ptr tree_cy(new pcl::search::KdTree<pcl::PointXYZHSV>());
   pcl::SACSegmentationFromNormals<pcl::PointXYZHSV, pcl::Normal> seg_cy;
   pcl::ExtractIndices<pcl::PointXYZHSV> extract;
   pcl::PassThrough<pcl::PointXYZHSV> pass;
   pcl::PCDWriter writer;


   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::Normal>::Ptr cloud_normals( new pcl::PointCloud<pcl::Normal>);
   pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
   pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);


   pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv (new pcl::PointCloud<pcl::PointXYZHSV>);
   pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_hsv_filtered (new pcl::PointCloud<pcl::PointXYZHSV>);
   pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
   pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
   pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZHSV>());
   pcl::ExtractIndices<pcl::Normal> extract_normals;

   *cloud = *msg;
   std::cerr << "Raw PointCloud has: " << cloud->points.size() << " data points." << std::endl;


   //convert RGB to HSV
   pcl::PointCloudXYZRGBtoXYZHSV(*cloud, *cloud_hsv);

   //send XYZ coordinates to HSV point cloud
   for (size_t i = 0; i < cloud->points.size (); ++i)
   {
      cloud_hsv->points[i].x = cloud->points[i].x;
      cloud_hsv->points[i].y = cloud->points[i].y;
      cloud_hsv->points[i].z = cloud->points[i].z;
   }


   //HSV pass through color filter
   pass.setInputCloud(cloud_hsv);
   pass.setFilterFieldName("s");
   pass.setFilterLimits(0.0, 0.2);
   pass.filter (*cloud_hsv_filtered);

   pass.setInputCloud(cloud_hsv_filtered);
   pass.setFilterFieldName("v");
   pass.setFilterLimits(0.80, 1.0);
   pass.filter (*cloud_hsv);

   // if possible replace by something better (the following two distance pass through filter
   pass.setInputCloud(cloud_hsv);
   pass.setFilterFieldName("y");
   pass.setFilterLimits(-0.25, 1.0);
   pass.filter (*cloud_hsv_filtered);

   pass.setInputCloud(cloud_hsv_filtered);
   pass.setFilterFieldName("z");
   pass.setFilterLimits(0.0, 1.9);
   pass.filter (*cloud_hsv);


   // Estimate point normals
   ne_cy.setSearchMethod(tree_cy);
   ne_cy.setInputCloud(cloud_hsv);
   ne_cy.setKSearch(50);
   ne_cy.compute(*cloud_normals);

   // Create the segmentation object for the planar model and set all the parameters
   seg_cy.setOptimizeCoefficients(true);
   seg_cy.setModelType(pcl::SACMODEL_NORMAL_PLANE);
   seg_cy.setNormalDistanceWeight(0.1);
   seg_cy.setMethodType(pcl::SAC_RANSAC);
   seg_cy.setMaxIterations(100);
   seg_cy.setDistanceThreshold(0.09);
   seg_cy.setInputCloud(cloud_hsv);
   seg_cy.setInputNormals(cloud_normals);
   // Obtain the plane inliers and coefficients
   seg_cy.segment(*inliers_plane, *coefficients_plane);
   std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

   // Extract and remove the planar inliers from the input cloud
   extract.setInputCloud(cloud_hsv);
   extract.setIndices(inliers_plane);
   extract.setNegative(true);
   extract.filter(*cloud_hsv_filtered);

   // write pointcloud after hsv to the disc for checking -- only for testing purpose
   std::cerr << "PointCloud after HSV filtering has: " << cloud_hsv_filtered->points.size() << " data points." << std::endl << std::endl;
   writer.write ("Cloud_after_HSV_filter.pcd", *cloud_hsv_filtered, false);

   // Estimate point normals
   ne_cy.setSearchMethod(tree_cy);
   ne_cy.setInputCloud(cloud_hsv_filtered);
   ne_cy.setKSearch(50);
   ne_cy.compute(*cloud_normals);

   // Schleife um Pipes zu Finden

   int i = 0;

   do {
      i++;

   // Create the segmentation object for cylinder segmentation and set all the parameters
   seg_cy.setOptimizeCoefficients(true);
   seg_cy.setModelType(pcl::SACMODEL_CYLINDER);
   seg_cy.setMethodType(pcl::SAC_RANSAC);
   seg_cy.setNormalDistanceWeight(0.1);
   seg_cy.setMaxIterations(1000);
   seg_cy.setDistanceThreshold(0.06);
   seg_cy.setRadiusLimits(0.01, 0.040);
   seg_cy.setInputCloud(cloud_hsv_filtered);
   seg_cy.setInputNormals(cloud_normals);

   // Obtain the cylinder inliers and coefficients
   seg_cy.segment(*inliers_cylinder, *coefficients_cylinder);
   std::cerr << i << " Cylinder coefficients: " << *coefficients_cylinder << std::endl << std::endl;

   // Publish Model Coefficients of cylinder
   pcl_msgs::ModelCoefficients ros_coefficients;
   pcl_conversions::fromPCL(*coefficients_cylinder, ros_coefficients);
   _modelCoefficient.publish (ros_coefficients);

   extract.setInputCloud(cloud_hsv_filtered);
   extract.setIndices(inliers_cylinder);
   extract.setNegative(false);
   extract.filter(*cloud_cylinder);


   if (cloud_cylinder->points.empty())
      std::cerr << i <<" Cylindrical component not found." << std::endl;
   else {
      std::cerr << i << "PointCloud representing following cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl << std::endl;
      writer.write("pipe_found.pcd", *cloud_cylinder, false);
         }

   // Extract the first found pipe from the cloud without a plane
   extract.setInputCloud(cloud_hsv_filtered);
   extract.setIndices(inliers_cylinder);
   extract.setNegative(true);
   extract.filter(*cloud_hsv_filtered);

   extract_normals.setNegative(true);
   extract_normals.setInputCloud(cloud_normals);
   extract_normals.setIndices(inliers_cylinder);
   extract_normals.filter(*cloud_normals);



   writer.write("was_noch_übrig_ist.pcd", *cloud_hsv_filtered, false);
   } while(i<5);




}

int main(int argc, char** argv) {
   ros::init(argc, argv, "sub_pcl");

   ros::NodeHandle nh;

   _cloudInput = nh.subscribe("cloud_pcd", 1, callbackFindPipe);

   _modelCoefficient = nh.advertise<pcl_msgs::ModelCoefficients> ("pipe_one", 1);


   ros::spin();
}
