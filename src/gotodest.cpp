#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iterator>
#include <ctime>
#include "std_msgs/Bool.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <octomap/octomap.h>
#include <geometry_msgs/Pose.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <octomap/octomap.h>

#include "navigation_utils.h"

using namespace std;
typedef octomap::point3d point3d;
tf::Quaternion Goal_heading;
point3d next_vp;

void Goal_callbacks(const geometry_msgs::Point Goal_point){

next_vp.x() = Goal_point.x;
next_vp.y() = Goal_point.y;
next_vp.z() = Goal_point.z;

}
void Pose_callbacks(const geometry_msgs::Vector3 Goal_pose){

Goal_heading.setRPY(Goal_pose.x, Goal_pose.y, Goal_pose.z);
Goal_heading.normalize();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gotodest");
    ros::NodeHandle n;
    std_msgs::Bool arrive;
    arrive.data = false;
    next_vp = point3d(0, 0, 0);
    Goal_heading.setRPY(0, 0, 0);
    Goal_heading.normalize();

    double x_diff ;//= 0;
    double y_diff ;//= 0;
    double z_diff ;//= 0;
    double angle_x_diff ;//= 0;
    double angle_y_diff ;//= 0;
    double angle_z_diff ;//= 0;

    double x_prior = 0;
    double y_prior = 0;
    double z_prior = 0;
    double angle_x_proir = 0;
    double angle_y_proir = 0;
    double angle_z_proir = 0;



    ros::Subscriber Goal_point_sub = n.subscribe<geometry_msgs::Point>("/Turtlebot_goal",1,Goal_callbacks);
    ros::Subscriber Goal_pose_sub = n.subscribe<geometry_msgs::Vector3>("/Turtlebot_heading",1,Pose_callbacks);
    ros::Publisher arrive_pub = n.advertise<std_msgs::Bool>("Arrive",1);

    while(ros::ok()){
    	ros::spinOnce();

        x_diff = abs(x_prior - next_vp.x());
        y_diff = abs(y_prior - next_vp.y());
        z_diff = abs(z_prior - next_vp.z());
        angle_x_diff = abs(angle_x_proir - Goal_heading.x());
        angle_y_diff = abs(angle_y_proir - Goal_heading.y());
        angle_z_diff = abs(angle_z_proir - Goal_heading.z());
        if(x_diff < 0.2||y_diff < 0.2||z_diff < 0.2){//||angle_x_diff < 0.2||angle_y_diff < 0.2||angle_z_diff < 0.2){
            continue;
        }
        //bool arrived = true;
        x_prior = next_vp.x();
        y_prior = next_vp.y();
        z_prior = next_vp.z();
        angle_x_proir = Goal_heading.x();
        angle_y_proir = Goal_heading.y();
        angle_z_proir = Goal_heading.z();
    	bool arrived= goToDest(next_vp, Goal_heading);
        arrive.data = arrived;
    	arrive_pub.publish( arrive );
    }

    //ros::spinOnce();

}
