#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iterator>

#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include "laser_geometry/laser_geometry.h"

using namespace std;

tf::TransformListener *tf_listener; 
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pcl_to_scan;
double kinect_orig[3];
const double PI = 3.14159265358979323846;

void pointcloud2_callbacks( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg )
{
    //double kinect_orig[3];
	pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(*cloud2_msg, cloud2);
    PointCloud* cloud (new PointCloud);
    PointCloud* cloud_local (new PointCloud);
    pcl::fromPCLPointCloud2(cloud2,*cloud_local);

    double z_min = 0.15;
    double z_max = 0.5;


    unsigned int num_readings = 640;
    double laser_frequency = 30;
    double ranges[num_readings];
    double intensities[num_readings];

    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "/camera_rgb_frame";
    scan.angle_min = -PI*57/360;
    scan.angle_max = PI*57/360;
    scan.angle_increment = 57*PI / ((num_readings-1)*180);
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 8;

    ros::Duration(0.07).sleep();
    while(!pcl_ros::transformPointCloud("/map", *cloud_local, *cloud, *tf_listener))
    {
        ros::Duration(0.01).sleep();
        for (int i = 1; i< cloud->width; i++)
        {
            double distance_scan_max = 0;
            for (int j = 1; j< cloud->height; j++)
            {
                if(cloud->at(i,j).z >= z_min && cloud->at(i,j).z <= z_max){
                  if(isnan(cloud->at(i,j).x)) continue;
                  double distance_scan = sqrt( pow(cloud->at(i,j).x, 2) + pow(cloud->at(i,j).y, 2));
                  if(distance_scan < distance_scan_max) scan.ranges[i - 1] = distance_scan;


                }
            }
        }
    }
    pcl_to_scan.publish(scan);
    delete cloud;
    delete cloud_local;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "poincloud2_to_laserscan");
	ros::NodeHandle nh;
    tf::StampedTransform transform;

	/*bool got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
        kinect_orig[0] = transform.getOrigin().x();
        kinect_orig[1] = transform.getOrigin().y();
        kinect_orig[2] = transform.getOrigin().z();
        //kinect_orig,second() = point3d(0, 0, transform.getRotation.z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        //ROS_WARN("Wait for tf: velodyne to map"); 
    } 
    ros::Duration(0.05).sleep();
    }*/

    ros::Subscriber kinect_sub;
    kinect_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, pointcloud2_callbacks);

    pcl_to_scan = nh.advertise<sensor_msgs::LaserScan>("kinect_PCL_scan", 1);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    nh.shutdown();          
    return 0;
}