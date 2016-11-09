#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iterator>

#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include "laser_geometry/laser_geometry.h"

using namespace std;

ros::Publisher filtered_scan;

void laerscan_callbacks(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    sensor_msgs::LaserScan scan_filter;

    scan_filter.ranges.resize(scan->ranges.size());
    scan_filter.intensities.resize(scan->ranges.size());


    scan_filter.header = scan->header;
    //scan_filter.header.stamp = ros::Time::now();
    scan_filter.angle_min = scan->angle_min;
    scan_filter.angle_max = scan->angle_max;
    scan_filter.angle_increment = scan->angle_increment;
    scan_filter.time_increment = scan->time_increment;
    scan_filter.range_min = 0.0;
    scan_filter.range_max = 8;
    scan_filter.intensities = scan->intensities;
    scan_filter.ranges = scan->ranges;

    for (int j = 0; j < scan->ranges.size(); j++)
    {


        //to verify is range[j] really nan for inf
        if(isnan(scan->ranges[j])){
            int R = 60;
            bool nan_verify = true;
            int n = 1;
            int m = 1;
            while(n <= R){
                if(isnan(scan->ranges[j+n])){
                    n++;
                }
                else{
                    nan_verify = false;
                    break;
                }
            }
            if(!nan_verify){
                while(m >= -R+n){
                    if(isnan(scan->ranges[j+m])){
                        m--;
                    }
                    else break;  
                }
                if(m < -R+n)  nan_verify = true;       
            }
            if(nan_verify)
            {

                scan_filter.ranges[j] = 8;
            
            }
        }

        /*if(j>R-1&&j<scan->ranges.size()-R){
        	for(int i = j-R; i <= j+R; i++){
        		if(!isnan(scan->ranges[j])){
        			nan_verify = false;
        		}
        	}
    	   if(nan_verify)
    	   {

    		scan_filter.ranges[j] = 8;
        
           }
        }*/

    }
    filtered_scan.publish(scan_filter);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laserscan_filter");
    ros::NodeHandle nh;

    ros::Subscriber laserscan_sub;
    laserscan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan_kinect", 1, laerscan_callbacks);// changed here##########

    
    filtered_scan = nh.advertise<sensor_msgs::LaserScan>("scan_filtered", 1);

    while (ros::ok())
    {
        ros::spin();
    }

    nh.shutdown();          
    return 0;
}
