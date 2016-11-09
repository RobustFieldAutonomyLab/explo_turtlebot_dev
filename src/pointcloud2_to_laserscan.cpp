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
//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

using namespace std;





tf::TransformListener *tf_listener; 
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pcl_to_scan;
const double PI = 3.14159265358979323846;
//double kinect_orig[3];

sensor_msgs::LaserScan scan;

double z_min = -0.248;
double z_max = 0.152;
unsigned int num_readings = 640;
//double laser_frequency = 1000 ;

void pointcloud2_callbacks( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg )
{

    //tf::StampedTransform transform;
    PointCloud* cloud (new PointCloud); 
    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(*cloud2_msg, cloud2);
    pcl::fromPCLPointCloud2(cloud2,*cloud);

    //scan.header = cloud2_msg->header;
    scan.header = cloud2_msg->header;
    scan.header.frame_id = "/camera_depth_frame"; 

    int t = 1;
    for (int i = 1; i < cloud->width; i = i+cloud->width/num_readings)
    {
        
        //bool all_nan = true;
        double distance_scan_min = 7.9;
        for (int j = 1; j < cloud->height; j=j+20)
        {
            //double cloud_x = cloud->at(i,j).x;
            //double cloud_y = cloud->at(i,j).y;
            //double cloud_z = cloud->at(i,j).z;
            //ROS_INFO("x %f, y %f, z %f", cloud_x, cloud_y, cloud_z);
            //all_nan = true;
            if(isnan(cloud->at(i,j).x)) continue;
            if(cloud->at(i,j).y >= z_min && cloud->at(i,j).y <= z_max){
                //all_nan = false;
                double distance_scan = sqrt( pow(cloud->at(i,j).x /*- kinect_orig[0]*/, 2) + pow(cloud->at(i,j).z /*-kinect_orig[1]*/ , 2));
                //ROS_INFO("distance %f", distance_scan);
                if(distance_scan < distance_scan_min){

                    distance_scan_min = distance_scan;

                }

                 
            }
            
        }
        //if(all_nan) distance_scan_min = 0;

        scan.ranges[num_readings - t] = distance_scan_min;
        scan.intensities[num_readings - t]=500;
        t++;
    }

    //sensor_msgs::LaserScan scan;
    //scan.header.stamp = ros::Time::now();
    //scan.header = cloud2_msg->header;
    //scan.header.frame_id = "/camera_depth_frame";
    //scan.header = cloud2_msg->header;
    /*scan.angle_min = -PI*57/360;
    scan.angle_max = PI*57/360;
    scan.angle_increment = 57*PI / ((num_readings-1)*180);
    scan.time_increment = 0; //(1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 5;*/

    pcl_to_scan.publish(scan);
    //++count;
    delete cloud;
    //delete cloud_local;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "poincloud2_to_laserscan");
    ros::NodeHandle nh;

    //tf::StampedTransform transform;
    //tf_listener = new tf::TransformListener();

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);

    scan.angle_min = -PI*57/360;
    scan.angle_max = PI*57/360;
    scan.angle_increment = 57*PI / ((num_readings-1)*180);
    scan.time_increment = 0; //(1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 5;


    ros::Subscriber kinect_sub;

    kinect_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, pointcloud2_callbacks);
    pcl_to_scan = nh.advertise<sensor_msgs::LaserScan>("kinect_PCL_scan", 1);

    while (ros::ok())
    {
        /*bool got_tf = false;
        while(!got_tf){
            try{
                //tf_listener->waitForTransform("/map", "/camera_rgb_frame", ros::Time(0), ros::Duration(10.0) );
                tf_listener->lookupTransform("/map", "/camera_depth_frame", ros::Time(0), transform);// need to change tf of kinect###############
                kinect_orig[0] = transform.getOrigin().x();
                kinect_orig[1] = transform.getOrigin().y();
                kinect_orig[2] = transform.getOrigin().z();
                //kinect_orig,second() = point3d(0, 0, transform.getRotation.z());
                got_tf = true;
               }
            catch (tf::TransformException ex) {
                ros::Duration(0.5).sleep();
                ROS_INFO("waiting");
                }
                //ros::Duration(0.05).sleep();
        }*/
        ros::spinOnce();
    }
    nh.shutdown();          
    return 0;
}
