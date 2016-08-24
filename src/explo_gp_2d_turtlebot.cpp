#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iterator>
#include <ctime>

#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"

#include <octomap/octomap.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include "navigation_utils.h"
#include <ros/callback_queue.h>
#include "gpregressor.h"
#include "covMaterniso3.h"


using namespace std;
// using namespace std::chrono;

typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
const double PI = 3.1415926;
const double free_prob = 0.3; // what is this?##########
const double octo_reso = 0.1; // what is this?##########

octomap::OcTree* cur_tree;  // include sensor date, sensor position and sensor_max_range
octomap::OcTree* cur_tree_2d;

bool octomap_flag = 0; // 0 : msg not received
bool kinect_flag = 0; // 0 : msg not received
tf::TransformListener *tf_listener; 
int octomap_seq = 0;
   
point3d position, laser_orig, velo_orig; // sensor positon

ofstream explo_log_file; //what's that? #########
std::string octomap_name_2d, octomap_name_3d;


struct SensorModel {
    double horizontal_fov;
    double vertical_fov;
    double angle_inc_hor;
    double angle_inc_vel;
    double width;
    double height;
    double max_range;
    vector<pair<double, double>> pitch_yaws;
    octomap::Pointcloud SensorRays;
    point3d InitialVector;

    SensorModel(double _width, double _height, double _horizontal_fov, double _vertical_fov, double _max_range)
            : width(_width), height(_height), horizontal_fov(_horizontal_fov), vertical_fov(_vertical_fov), max_range(_max_range) {
        angle_inc_hor = horizontal_fov / width;
        angle_inc_vel = vertical_fov / height;
        for(double j = -height / 2; j < height / 2; ++j) 
            for(double i = -width / 2; i < width / 2; ++i) {
                InitialVector = point3d(1.0, 0.0, 0.0);
                InitialVector.rotate_IP(0.0, j * angle_inc_vel, i * angle_inc_hor);
                SensorRays.push_back(InitialVector);
        }
    }
}; 

// Establish sensor kinect
SensorModel Kinect_360(64, 48, 2*PI*57/360, 2*PI*43/360, 5);//maybe need to change

//entropy Input: octree   Output:volume
double get_free_volume(const octomap::OcTree *octree) {
    double volume = 0;
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        if(!octree->isNodeOccupied(*n))
            volume += pow(n.getSize(), 3);
    }
    return volume;
}

// Input: octree, position, direction. Output: hits
octomap::Pointcloud cast_sensor_rays(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &direction) {
    octomap::Pointcloud hits;

    octomap::Pointcloud SensorRays_copy;
    SensorRays_copy.push_back(Kinect_360.SensorRays);
    SensorRays_copy.rotate(0.0,0.0,direction.z());
    point3d end;
    // #pragma omp parallel for
    for(int i = 0; i < SensorRays_copy.size(); i++) {
        if(octree->castRay(position, SensorRays_copy.getPoint(i), end, true, Kinect_360.max_range)) {
            
        } else {
            end = SensorRays_copy.getPoint(i) * Kinect_360.max_range;
            end += position;
            hits.push_back(end);
        }
    }
    return hits;
}

vector<vector<point3d>> generate_frontier_points(const octomap::OcTree *octree) {

    vector<vector<point3d>> frontier_lines;
    vector<point3d> frontier_points;
    octomap::OcTreeNode *n_cur_frontier;
    bool frontier_true; // whether or not a frontier point
    bool belong_old;//whether or not belong to old group
    double distance;
    double R1 = 0.4; //group length
    //double x_frontier;
    //double y_frontier;
    //double z_frontier;

    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n)
    {
        frontier_true = false;
        unsigned long int num_free = 0; //number of free cube around frontier, for filtering out fake frontier

      if(!cur_tree_2d->isNodeOccupied(*n))
        {
         double x_cur = n.getX();
         double y_cur = n.getY();
         double z_cur = n.getZ();
         //if there are unknown around the cube, the cube is frontier
         for (double x_cur_buf = x_cur - 0.1; x_cur_buf < x_cur + 0.15; x_cur_buf += octo_reso)
             for (double y_cur_buf = y_cur - 0.1; y_cur_buf < y_cur + 0.15; y_cur_buf += octo_reso)
            {
                n_cur_frontier = cur_tree_2d->search(point3d(x_cur_buf, y_cur_buf, z_cur));
                if(!n_cur_frontier)
                {
                    frontier_true = true;
                    continue;            
                }
                /*else if (!cur_tree_2d->isNodeOccupied(n_cur_frontier))
                {
                    num_free++;

                }*/

            }
            if(frontier_true)// && num_free >5 )
            {

                double x_frontier = x_cur;
                double y_frontier = y_cur;
                double z_frontier = z_cur;

                // divede frontier points into groups
                if(frontier_lines.size() < 1)
                {
                    frontier_points.resize(1);
                    frontier_points[0] = point3d(x_frontier,y_frontier,z_frontier);
                    frontier_lines.push_back(frontier_points);
                    frontier_points.clear();
                }
                else
                {
                    bool belong_old = false;            

                    for(vector<vector<point3d>>::size_type u = 0; u < frontier_lines.size(); u++){
                            distance = sqrt(pow(frontier_lines[u][0].x()-x_frontier, 2)+pow(frontier_lines[u][0].y()-y_frontier, 2)) ;
                            if(distance < R1){
                               frontier_lines[u].push_back(point3d(x_frontier, y_frontier, z_frontier));
                               belong_old = true;
                               break;
                            }
                    }
                    if(!belong_old){
                               frontier_points.resize(1);
                               frontier_points[0] = point3d(x_frontier, y_frontier, z_frontier);
                               frontier_lines.push_back(frontier_points);
                               frontier_points.clear();
                    }                              
                }

            } 
        }
        
    }
    return frontier_lines;
}








//generate candidates for moving. Input sensor_orig and initial_yaw, Output candidates
//senor_orig: locationg of sensor.   initial_yaw: yaw direction of sensor
vector<pair<point3d, point3d>> generate_candidates(vector<vector<point3d>> frontier_lines, point3d sensor_orig ) {
    double R = 0.6;   // Robot step, in meters.
    double n = 6;
    octomap::OcTreeNode *n_cur;
    octomap::OcTreeNode *n_cur_3d;
    vector<pair<point3d, point3d>> candidates;
    double z = sensor_orig.z();
    double x, y;
    double yaw;
    double distance_can;

        for(vector<vector<point3d>>::size_type u = 0; u < frontier_lines.size(); u++){

            double initial_yaw = atan2(frontier_lines[u][0].y()-sensor_orig.y(),  frontier_lines[u][0].x()-sensor_orig.x() ); //yaw is from sensor_orig to center of a frontier group

            for(double yaw = initial_yaw; yaw < initial_yaw+2*PI; yaw += PI*2 / n){ 
                // get candidates around the center of a frontier group
                x = frontier_lines[u][0].x() - R * cos(yaw);
                y = frontier_lines[u][0].y() - R * sin(yaw);

                // for every candidate goal, check surroundings
                bool candidate_valid = true;

                distance_can =sqrt(pow(x - sensor_orig.x(),2) + pow(y - sensor_orig.y(),2));
                if(distance_can < 0.25){
                  candidate_valid = false;// delete candidates close to sensor_orig
                  
                 }
                else{
                    for(vector<vector<point3d>>::size_type n = 0; n < frontier_lines.size(); n++)
                        for(vector<point3d>::size_type m = 0; m < frontier_lines[n].size(); m++){
                            distance_can = sqrt(pow(x - frontier_lines[n][m].x(),2) + pow(y - frontier_lines[n][m].y(),2));
                            if(distance_can < 0.25){
                                candidate_valid = false;//delete candidates close to frontier
                                goto candidates_go;
                            }
                    }
                
                    
                    n_cur = cur_tree_2d->search(point3d(x, y, z));
                    n_cur_3d = cur_tree->search(point3d(x, y, z));


                    if(!n_cur) {
                            candidate_valid = false;//delete candidates which are unknown cubes
                            }
                    else{
                            for (double x_buf = x - 0.3; x_buf < x + 0.3; x_buf += octo_reso) 
                                for (double y_buf = y - 0.3; y_buf < y + 0.3; y_buf += octo_reso)
                                    //for (double z_buf = z - 0.2; z_buf < z + 0.2; z_buf += octo_reso/2)
                            {
                                n_cur = cur_tree_2d->search(point3d(x_buf, y_buf, z));
                                if(!n_cur) {
                                    continue;
                                }                                 
                                if(cur_tree_2d->isNodeOccupied(n_cur)) {
                                       candidate_valid = false;//delete candidates which have occupied cube around
                                }  
                            }
                        }
                    }
                if(candidate_valid){
                    for (double x_buf = x - 0.3; x_buf < x + 0.3; x_buf += octo_reso) 
                                for (double y_buf = y - 0.3; y_buf < y + 0.3; y_buf += octo_reso)
                                    for (double z_buf = 0.2; z_buf <0.4; z_buf += octo_reso)
                            {
                                n_cur_3d = cur_tree->search(point3d(x_buf, y_buf, z_buf));
                                if(!n_cur_3d) {
                                    continue;
                                }                                 
                                if(cur_tree->isNodeOccupied(n_cur_3d)) {
                                       candidate_valid = false;//delete candidates which have ccupied cubes around in 3D area
                                }  
                            }
                }

                candidates_go:

                if (candidate_valid)
                {
                    candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, yaw)));
                }
                else{
                    //ROS_WARN("Part of Candidtae(%f, %f, %f) occupied or unknow", x, y, z);
                }
            }
            
        }
    return candidates;
}

//generate gp pose
vector<pair<point3d, point3d>> generate_testing(vector<vector<point3d>> frontier_lines, point3d sensor_orig ) {
    double R = 0.6;   // Robot step, in meters.
    double n = 24;
    octomap::OcTreeNode *n_cur;
    octomap::OcTreeNode *n_cur_3d;
    vector<pair<point3d, point3d>> candidates;
    double z = sensor_orig.z();
    double x, y;
    double yaw;
    double distance_can;

        for(vector<vector<point3d>>::size_type u = 0; u < frontier_lines.size(); u++){

            double initial_yaw = atan2(frontier_lines[u][0].y()-sensor_orig.y(),  frontier_lines[u][0].x()-sensor_orig.x() );

            for(double yaw = initial_yaw; yaw < initial_yaw+2*PI; yaw += PI*2 / n){
                x = frontier_lines[u][0].x() - R * cos(yaw);
                y = frontier_lines[u][0].y() - R * sin(yaw);

                // for every candidate goal, check surroundings
                bool candidate_valid = true;

                distance_can =sqrt(pow(x - sensor_orig.x(),2) + pow(y - sensor_orig.y(),2));
               if(distance_can < 0.25){
                  candidate_valid = false;
                  
                 }
           else{
                for(vector<vector<point3d>>::size_type n = 0; n < frontier_lines.size(); n++)
                    for(vector<point3d>::size_type m = 0; m < frontier_lines[n].size(); m++){
                        distance_can = sqrt(pow(x - frontier_lines[n][m].x(),2) + pow(y - frontier_lines[n][m].y(),2));
                        if(distance_can < 0.25){
                            candidate_valid = false;
                            goto candidates_go;
                        }
                    }
                
                    
                n_cur = cur_tree_2d->search(point3d(x, y, z));
                n_cur_3d = cur_tree->search(point3d(x, y, z));


                if(!n_cur) {
                        candidate_valid = false;
                        }
                else{
                            for (double x_buf = x - 0.3; x_buf < x + 0.3; x_buf += octo_reso) 
                                for (double y_buf = y - 0.3; y_buf < y + 0.3; y_buf += octo_reso)
                                    //for (double z_buf = z - 0.2; z_buf < z + 0.2; z_buf += octo_reso/2)
                            {
                                n_cur = cur_tree_2d->search(point3d(x_buf, y_buf, z));
                                if(!n_cur) {
                                    continue;
                                }                                 
                                if(cur_tree_2d->isNodeOccupied(n_cur)) {
                                       candidate_valid = false;
                                }  
                            }
                    }
                }
                if(candidate_valid){
                    for (double x_buf = x - 0.3; x_buf < x + 0.3; x_buf += octo_reso) 
                                for (double y_buf = y - 0.3; y_buf < y + 0.3; y_buf += octo_reso)
                                    for (double z_buf = 0.2; z_buf <0.4; z_buf += octo_reso)
                            {
                                n_cur_3d = cur_tree->search(point3d(x_buf, y_buf, z_buf));
                                if(!n_cur_3d) {
                                    continue;
                                }                                 
                                if(cur_tree->isNodeOccupied(n_cur_3d)) {
                                       candidate_valid = false;
                                }  
                            }
                }

                candidates_go:

                if (candidate_valid)
                {
                    candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, yaw)));
                }
                else{
                    //ROS_WARN("Part of Candidtae(%f, %f, %f) occupied or unknow", x, y, z);
                }
            }
            
        }
    return candidates;
}


// Calculate Mutual Information. Input: octree, sensor_orig, hits, before
double calc_MI(const octomap::OcTree *octree, const point3d &sensor_orig, const octomap::Pointcloud &hits, const double before) {
    auto octree_copy = new octomap::OcTree(*octree);

    octree_copy->insertPointCloud(hits, sensor_orig, Kinect_360.max_range, true, true);
    double after = get_free_volume(octree_copy);
    delete octree_copy;
    return after - before;
}

void kinect_callbacks( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg ) {
    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(*cloud2_msg, cloud2);
    PointCloud* cloud (new PointCloud);
    PointCloud* cloud_local (new PointCloud);
    pcl::fromPCLPointCloud2(cloud2,*cloud_local);
    octomap::Pointcloud hits;

    ros::Duration(0.07).sleep();
    while(!pcl_ros::transformPointCloud("/map", *cloud_local, *cloud, *tf_listener))
    {
        ros::Duration(0.01).sleep();
    }
    // Insert points into octomap one by one...
    for (int i = 1; i< cloud->width; i++)
    {
        for (int j = 1; j< cloud->height; j++)
        {
            if(isnan(cloud->at(i,j).x)) continue;
            if(cloud->at(i,j).z < -1.0)    continue;  
            hits.push_back(point3d(cloud->at(i,j).x, cloud->at(i,j).y, cloud->at(i,j).z));
        }
    }

    cur_tree->insertPointCloud(hits, velo_orig, Kinect_360.max_range);
    ROS_INFO("Entropy(3d map) : %f", get_free_volume(cur_tree));

    cur_tree->write(octomap_name_3d);
    delete cloud;
    delete cloud_local;
}

void hokuyo_callbacks( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg )
{
    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(*cloud2_msg, cloud2);
    PointCloud* cloud (new PointCloud);
    PointCloud* cloud_local (new PointCloud);
    pcl::fromPCLPointCloud2(cloud2,*cloud_local);
    octomap::Pointcloud hits;

    ros::Duration(0.07).sleep();
    while(!pcl_ros::transformPointCloud("/map", *cloud_local, *cloud, *tf_listener))
    {
        ros::Duration(0.01).sleep();
    }

    // Insert points into octomap one by one...
    for (int j = 1; j< cloud->width; j++)
    {
        // if(isnan(cloud->at(j).x)) continue;
        hits.push_back(point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z));
    }
    cur_tree_2d->insertPointCloud(hits, laser_orig, 5.6);
    ROS_INFO("Entropy(2d map) : %f", get_free_volume(cur_tree_2d));
    cur_tree_2d->write(octomap_name_2d);
    delete cloud;
    delete cloud_local;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "explo_sam_2d_turtlebot");
    ros::NodeHandle nh;

    // Initialize time
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,80,"Trajectory_gp_%R_%S_%m%d_DA.txt",timeinfo);
    std::string logfilename(buffer);
    std::cout << logfilename << endl;
    strftime(buffer,80,"octomap_gp_2d_%R_%S_%m%d_DA.ot",timeinfo);
    octomap_name_2d = buffer;
    strftime(buffer,80,"octomap_gp_3d_%R_%S_%m%d_DA.ot",timeinfo);
    octomap_name_3d = buffer;


    ros::Subscriber kinect_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, kinect_callbacks);// need to change##########
    ros::Subscriber hokuyo_sub = nh.subscribe<sensor_msgs::PointCloud2>("/hokuyo_points", 1, hokuyo_callbacks);// need to change#############
    ros::Publisher GoalMarker_pub = nh.advertise<visualization_msgs::Marker>( "Goal_Marker", 1 );
    ros::Publisher JackalMarker_pub = nh.advertise<visualization_msgs::Marker>( "Jackal_Marker", 1 );
    ros::Publisher Candidates_pub = nh.advertise<visualization_msgs::MarkerArray>("Candidate_MIs", 1);
    ros::Publisher Octomap_marker_pub = nh.advertise<visualization_msgs::Marker>("Occupied_MarkerArray", 1);
    ros::Publisher Octomap_marker_3d_pub = nh.advertise<visualization_msgs::Marker>("Occupied_MarkerArray_3d", 1);
    ros::Publisher Frontier_points_pub = nh.advertise<visualization_msgs::Marker>("Frontier_points", 1);//changed here#######
    ros::Publisher Free_marker_pub = nh.advertise<visualization_msgs::Marker>("Free_MarkerArray", 1);
    ros::Publisher Free_marker_3d_pub = nh.advertise<visualization_msgs::Marker>("Free_MarkerArray_3d", 1);
    ros::Publisher pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

    tf_listener = new tf::TransformListener();
    tf::StampedTransform transform;
    tf::Quaternion Goal_heading; // robot's heading direction

    visualization_msgs::MarkerArray CandidatesMarker_array;
    visualization_msgs::Marker OctomapOccupied_cubelist;
    visualization_msgs::Marker OctomapOccupied_cubelist_3d;
    visualization_msgs::Marker Frontier_points_cubelist;
    visualization_msgs::Marker Free_cubelist;
    visualization_msgs::Marker Free_cubelist_3d;
    geometry_msgs::Twist twist_cmd;



    
    ros::Time now_marker = ros::Time::now();
   
    double R_velo, P_velo, Y_velo;

    // Initialize parameters 
    int max_idx = 0;

    position = point3d(0, 0, 0.0);
    point3d Sensor_PrincipalAxis(1, 0, 0);
    octomap::OcTreeNode *n;
    octomap::OcTree new_tree(octo_reso);
    octomap::OcTree new_tree_2d(octo_reso);
    cur_tree = &new_tree;
    cur_tree_2d = &new_tree_2d;
    point3d next_vp;

    bool got_tf = false;
    bool arrived;

    //Update the pose of velodyne from predefined tf.
    /*got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/base_link", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        tf::Matrix3x3(transform.getRotation()).getRPY(R_velo, P_velo, Y_velo);
        Sensor_PrincipalAxis.rotate_IP(R_velo, P_velo, Y_velo);
        ROS_INFO("Current Kinect heading: vector(%2.2f, %2.2f, %2.2f) -  RPY(%3.1f, %3.1f, %3.1f).", Sensor_PrincipalAxis.x(), Sensor_PrincipalAxis.y(), Sensor_PrincipalAxis.z(), R_velo/PI*180.0, P_velo/PI*180.0, Y_velo/PI*180.0);
        got_tf = true;
        }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: initial pose of Kinect"); 
        ros::Duration(0.05).sleep();
        } 
    }   */
    
     //Rotate Sensor Model based on Velodyn Pose
    //Kinect_360.SensorRays.rotate(R_velo, P_velo, Y_velo);
    
    // Update the initial location of the robot
    for(int o =0; o < 6; o++){
        // Update the pose of the robot
        got_tf = false;
        while(!got_tf){
        try{
            tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
            velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            got_tf = true;
        }
        catch (tf::TransformException ex) {
            ROS_WARN("Wait for tf: velodyne to map"); 
        } 
        ros::Duration(0.05).sleep();
        }

        got_tf = false;
        while(!got_tf){
        try{
            tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
            laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            got_tf = true;
        }
        catch (tf::TransformException ex) {
            ROS_WARN("Wait for tf: laser to map"); 
        } 
        ros::Duration(0.05).sleep();
        }

        // Take a Scan
        ros::spinOnce();

        // Rotate another 60 degrees
        twist_cmd.linear.x = twist_cmd.linear.y = twist_cmd.angular.z = 0;
        ros::Time start_turn = ros::Time::now();

        ROS_WARN("Rotate 60 degrees");
        while (ros::Time::now() - start_turn < ros::Duration(2.6)){ // turning duration - second
        twist_cmd.angular.z = 0.6; // turning speed
        // turning angle = turning speed * turning duration / 3.14 * 180
        pub_twist.publish(twist_cmd);
        ros::Duration(0.05).sleep();
        }
        // stop
        twist_cmd.angular.z = 0;
        pub_twist.publish(twist_cmd);

    }

    /*got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect##############
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: Kinect to map"); 
    } 
    ros::Duration(0.05).sleep();
    }

    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
        laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: laser to map"); 
    } 
    ros::Duration(0.05).sleep();
    }
    ROS_INFO("Initial  Position : %3.2f, %3.2f, %3.2f - Yaw : %3.1f ", laser_orig.x(), laser_orig.y(), laser_orig.z(), transform.getRotation().getAngle()*PI/180);
    // Take a Initial Scan
    ros::spinOnce();Here we are just making a temporary directory to record data and then running rosbag record with the option -a, indicating that all published topics should be accumulated in a bag file.

    // Rotate ~60 degrees 
    point3d next_vp(laser_orig.x(), laser_orig.y(),laser_orig.z());
    Goal_heading.setRPY(0.0, 0.0, transform.getRotation().getAngle()-0.5233); // why 0.5233?###
    Goal_heading.normalize();
    bool arrived = goToDest(laser_orig, Goal_heading);

    // Update the pose of the robot
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: velodyne to map"); 
    } 
    ros::Duration(0.05).sleep();
    }

    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
        laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: laser to map"); 
    } 
    ros::Duration(0.05).sleep();
    }

    // Take a Second Scan
    ros::spinOnce();

    // Rotate another 60 degrees
    Goal_heading.setRPY(0.0, 0.0, transform.getRotation().getAngle()-0.5233);
    arrived = goToDest(laser_orig, Goal_heading);

    // Update the pose of the robot
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: velodyne to map"); 
    } 
    ros::Duration(0.05).sleep();
    }

    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
        laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: laser to map"); 
    } 
    ros::Duration(0.05).sleep();
    }

    // Take a Third Scan
    ros::spinOnce();

    // Rotate another 60 degrees
    Goal_heading.setRPY(0.0, 0.0, transform.getRotation().getAngle()-0.5233);
    arrived = goToDest(laser_orig, Goal_heading);*/

    // Update the pose of the robot
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: velodyne to map"); 
    } 
    ros::Duration(0.05).sleep();
    }

    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
        laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: laser to map"); 
    } 
    ros::Duration(0.05).sleep();
    }
    // Take a Fourth Scan
    ros::spinOnce();

    double train_time, test_time;

    // steps robot taken, counter
    int robot_step_counter = 0;

    while (ros::ok())
    {
            start_over:
            
            vector<vector<point3d>> frontier_lines=generate_frontier_points( cur_tree_2d );
            /*unsigned long int t = 0;
            
            visualization_msgs::Marker Frontier_points_cubelist[frontier_lines.size()];
            //try if really need to resize#########################################################
            //for(vector<vector<point3d>>::size_type n = 0; n < frontier_lines.size(); n++) {
                //Frontier_points_cubelist[n].points.resize(frontier_lines[n].size());
            //}
            
            
            for(vector<vector<point3d>>::size_type n = 0; n < frontier_lines.size(); n++) {
                //o = o+frontier_lines[n].size();
                Frontier_points_cubelist[n].points.resize(frontier_lines[n].size());

                //Frontier_points_cubelist.points.resize(frontier_lines[n].size());
                //ROS_INFO("frontier points %ld", o);
                now_marker = ros::Time::now();
                Frontier_points_cubelist[n].header.frame_id = "map";
                Frontier_points_cubelist[n].header.stamp = now_marker;
                Frontier_points_cubelist[n].ns = "frontier_points_array";
                Frontier_points_cubelist[n].id = 0;
                Frontier_points_cubelist[n].type = visualization_msgs::Marker::CUBE_LIST;
                Frontier_points_cubelist[n].action = visualization_msgs::Marker::ADD;
                Frontier_points_cubelist[n].scale.x = octo_reso;
                Frontier_points_cubelist[n].scale.y = octo_reso;
                Frontier_points_cubelist[n].scale.z = octo_reso;
                Frontier_points_cubelist[n].color.a = 1.0;
                Frontier_points_cubelist[n].color.r = (double)255/255;
                Frontier_points_cubelist[n].color.g = 0;
                Frontier_points_cubelist[n].color.b = (double)6*n/255;
                Frontier_points_cubelist[n].lifetime = ros::Duration();
                
                //int l = 0;
                geometry_msgs::Point q;
                for(vector<point3d>::size_type m = 0; m < frontier_lines[n].size(); m++){

                    q.x = frontier_lines[n][m].x();
                    q.y = frontier_lines[n][m].y();
                    q.z = frontier_lines[n][m].z()+octo_reso;
                    Frontier_points_cubelist[n].points.push_back(q);   

                }
                t++;
                Frontier_points_pub.publish(Frontier_points_cubelist[n]); //publish frontier_points############
                Frontier_points_cubelist[n].points.clear(); 
            }
            
            ROS_INFO("Publishing %ld frontier_lines", t);*/
            
            //int delete_points = 0;
            
            //frontier_lines.clear();//in the next line
            unsigned long int o = 0;
            for(vector<vector<point3d>>::size_type e = 0; e < frontier_lines.size(); e++) {
                o = o+frontier_lines[e].size();
            }

            Frontier_points_cubelist.points.resize(o);
            ROS_INFO("frontier points %ld", o);
            now_marker = ros::Time::now();
            Frontier_points_cubelist.header.frame_id = "map";
            Frontier_points_cubelist.header.stamp = now_marker;
            Frontier_points_cubelist.ns = "frontier_points_array";
            Frontier_points_cubelist.id = 0;
            Frontier_points_cubelist.type = visualization_msgs::Marker::CUBE_LIST;
            Frontier_points_cubelist.action = visualization_msgs::Marker::ADD;
            Frontier_points_cubelist.scale.x = octo_reso;
            Frontier_points_cubelist.scale.y = octo_reso;
            Frontier_points_cubelist.scale.z = octo_reso;
            Frontier_points_cubelist.color.a = 1.0;
            Frontier_points_cubelist.color.r = (double)255/255;
            Frontier_points_cubelist.color.g = 0;
            Frontier_points_cubelist.color.b = (double)0/255;
            Frontier_points_cubelist.lifetime = ros::Duration();

            unsigned long int t = 0;
            int l = 0;
            geometry_msgs::Point q;
            for(vector<vector<point3d>>::size_type n = 0; n < frontier_lines.size(); n++) { 
                for(vector<point3d>::size_type m = 0; m < frontier_lines[n].size(); m++){


                   q.x = frontier_lines[n][m].x();
                   q.y = frontier_lines[n][m].y();
                   q.z = frontier_lines[n][m].z()+octo_reso;
                   Frontier_points_cubelist.points.push_back(q); 
                   
                }
                t++;
            }
            ROS_INFO("Publishing %ld frontier_lines", t);
            
            Frontier_points_pub.publish(Frontier_points_cubelist); //publish frontier_points
            Frontier_points_cubelist.points.clear();            


        // Generate Candidates
        vector<pair<point3d, point3d>> candidates = generate_candidates(frontier_lines, laser_orig); 
        // Generate Testing poses
        vector<pair<point3d, point3d>> gp_test_poses = generate_testing(frontier_lines, laser_orig);

        frontier_lines.clear();

        while(candidates.size() <= 1)
        {
            // Get the current heading
            got_tf = false;
            while(!got_tf){
            try{
                tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
                laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                got_tf = true;
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Wait for tf: laser to map"); 
            } 
            ros::Duration(0.05).sleep();
            }
            // Rotate another 60 degrees
            twist_cmd.linear.x = twist_cmd.linear.y = twist_cmd.angular.z = 0;
            ros::Time start_turn = ros::Time::now();

            ROS_WARN("Rotate 60 degrees");
            while (ros::Time::now() - start_turn < ros::Duration(2.6)){ // turning duration - second
            twist_cmd.angular.z = 0.4; // turning speed
            // turning angle = turning speed * turning duration / 3.14 * 180
            pub_twist.publish(twist_cmd);
            ros::Duration(0.05).sleep();
            }
            // stop
            twist_cmd.angular.z = 0;
            pub_twist.publish(twist_cmd);
            vector<pair<point3d, point3d>> candidates = generate_candidates(frontier_lines, laser_orig);
        }
        
        ROS_INFO("%lu candidates generated.", candidates.size());
        vector<double> MIs(candidates.size());
        double before = get_free_volume(cur_tree);
        max_idx = 0;

        unsigned int p = 0;

        // for every candidate...
        double Secs_CastRay, Secs_InsertRay, Secs_tmp;  //What are those? ####
        Secs_InsertRay = 0;
        Secs_CastRay = 0;

        #pragma omp parallel for
        for(int i = 0; i < candidates.size(); i++) 
        {   
            //max_order[i] = i;
            auto c = candidates[i];
            // Evaluate Mutual Information
            Secs_tmp = ros::Time::now().toSec();
            Sensor_PrincipalAxis.rotate_IP(c.second.roll(), c.second.pitch(), c.second.yaw() );
            octomap::Pointcloud hits = cast_sensor_rays(cur_tree, c.first, Sensor_PrincipalAxis);  // what are those?#####
            Secs_CastRay += ros::Time::now().toSec() - Secs_tmp;
            Secs_tmp = ros::Time::now().toSec();
            MIs[i] = calc_MI(cur_tree, c.first, hits, before)/sqrt(pow(c.first.x()-laser_orig.x(), 2)+pow(c.first.y() - laser_orig.y(), 2));
            Secs_InsertRay += ros::Time::now().toSec() - Secs_tmp;
        }

        //Initialize gp regression
        GPRegressor g(100, 2, 0.01);// what's this?
        MatrixXf gp_train_x(candidates.size(), 3), gp_train_label(candidates.size(), 1), gp_test_x(gp_test_poses.size(), 3);

        for (int i=0; i< candidates.size(); i++){
            gp_train_x(i,0) = candidates[i].first.x();
            gp_train_x(i,1) = candidates[i].first.y();
            gp_train_x(i,2) = candidates[i].second.z();
            gp_train_label(i) = MIs[i];
        }

        for (int i=0; i< gp_test_poses.size(); i++){
            gp_test_x(i,0) = gp_test_poses[i].first.x();
            gp_test_x(i,1) = gp_test_poses[i].first.y();
            gp_test_x(i,2) = gp_test_poses[i].second.z();
        }

        // Perform GP regression
        MatrixXf m, s2;
        train_time = ros::Time::now().toSec();
        g.train(gp_train_x, gp_train_label);
        train_time = ros::Time::now().toSec() - train_time;

        test_time = ros::Time::now().toSec();
        g.test(gp_test_x, m, s2);
        test_time = ros::Time::now().toSec() - test_time;
        ROS_INFO("GP: Train(%zd) took %f | Test(%zd) took %f", candidates.size(), train_time, gp_test_poses.size(), test_time);        
        for(int i = 0; i < gp_test_poses.size(); i++)
            m(i) = m(i)/sqrt(sqrt(pow(gp_test_x(i,0)-laser_orig.x(), 2)+pow(gp_test_x(i,1) - laser_orig.y(), 2)));
          
        // ###########################
        long int max_order[gp_test_poses.size()];

        for(int j=0; j<gp_test_poses.size(); j++)
           {
            p=0;
           for(int n=0; n<gp_test_poses.size(); n++)
              {
            
              if (m(j) > m(n))
                 {
                 p++;
                 }
              }
           max_order[p] = j;
           }
        
        p = gp_test_poses.size()-1;
        loop:
        max_idx = max_order[p];
 //ROS_INFO("here!!!!");
 
        next_vp = point3d(gp_test_x(max_idx,0),gp_test_x(max_idx,1),candidates[0].first.z());
        Goal_heading.setRPY(0.0, 0.0, gp_test_x(max_idx,2));
        Goal_heading.normalize();
        ROS_INFO("Estimated Max MI : %f , @ %3.2f,  %3.2f,  %3.2f", m(max_idx), next_vp.x(), next_vp.y(), next_vp.z() );
        ROS_INFO("CastRay Time: %2.3f Secs. InsertRay Time: %2.3f Secs.", Secs_CastRay, Secs_InsertRay);

        // Publish the candidates as marker array in rviz
        tf::Quaternion MI_heading;
        MI_heading.setRPY(0.0, -PI/2, 0.0);
        MI_heading.normalize();
        
        CandidatesMarker_array.markers.resize(gp_test_poses.size());
        for (int i = 0; i < gp_test_poses.size(); i++)
        {
            CandidatesMarker_array.markers[i].header.frame_id = "map";
            CandidatesMarker_array.markers[i].header.stamp = ros::Time::now();
            CandidatesMarker_array.markers[i].ns = "candidates";
            CandidatesMarker_array.markers[i].id = i;
            CandidatesMarker_array.markers[i].type = visualization_msgs::Marker::ARROW;
            CandidatesMarker_array.markers[i].action = visualization_msgs::Marker::ADD;
            CandidatesMarker_array.markers[i].pose.position.x = gp_test_x(i,0);
            CandidatesMarker_array.markers[i].pose.position.y = gp_test_x(i,1);
            CandidatesMarker_array.markers[i].pose.position.z = candidates[0].first.z();
            CandidatesMarker_array.markers[i].pose.orientation.x = MI_heading.x();
            CandidatesMarker_array.markers[i].pose.orientation.y = MI_heading.y();
            CandidatesMarker_array.markers[i].pose.orientation.z = MI_heading.z();
            CandidatesMarker_array.markers[i].pose.orientation.w = MI_heading.w();
            CandidatesMarker_array.markers[i].scale.x = (double)m(i)/m(max_idx);
            CandidatesMarker_array.markers[i].scale.y = 0.05;
            CandidatesMarker_array.markers[i].scale.z = 0.05;
            CandidatesMarker_array.markers[i].color.a = (double)m(i)/m(max_idx);
            CandidatesMarker_array.markers[i].color.r = 0.0;
            CandidatesMarker_array.markers[i].color.g = 1.0;
            CandidatesMarker_array.markers[i].color.b = 0.0;
        }
        Candidates_pub.publish(CandidatesMarker_array); //publish candidates##########
        CandidatesMarker_array.markers.clear();
        //candidates.clear();

        // Publish the goal as a Marker in rviz
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "goal_marker";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = next_vp.x();
        marker.pose.position.y = next_vp.y();
        marker.pose.position.z = 1.0;
        marker.pose.orientation.x = Goal_heading.x();
        marker.pose.orientation.y = Goal_heading.y();
        marker.pose.orientation.z = Goal_heading.z();
        marker.pose.orientation.w = Goal_heading.w();
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        GoalMarker_pub.publish( marker ); //publish goal##########

        // Send the Robot 
        Goal_heading.setRPY(0.0, 0.0, gp_test_x(max_idx,2));
        arrived = goToDest(next_vp, Goal_heading);

        if(arrived)
        {
            // Update the initial location of the robot
            got_tf = false;
            while(!got_tf){
            try{
                tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
                velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                got_tf = true;
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Wait for tf: velodyne to map"); 
            } 
            ros::Duration(0.05).sleep();
            }

            got_tf = false;
            while(!got_tf){
            try{
                tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
                laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                got_tf = true;
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Wait for tf: laser to map"); 
            } 
            ros::Duration(0.05).sleep();
            }

            // Update Octomap
            ros::spinOnce();
            ROS_INFO("Succeed, new Map Free Volume: %f", get_free_volume(cur_tree));
            robot_step_counter++;

            // Prepare the header for occupied array
            now_marker = ros::Time::now();
            OctomapOccupied_cubelist.header.frame_id = "map";
            OctomapOccupied_cubelist.header.stamp = now_marker;
            OctomapOccupied_cubelist.ns = "octomap_occupied_array";
            OctomapOccupied_cubelist.id = 0;
            OctomapOccupied_cubelist.type = visualization_msgs::Marker::CUBE_LIST;
            OctomapOccupied_cubelist.action = visualization_msgs::Marker::ADD;
            OctomapOccupied_cubelist.scale.x = octo_reso;
            OctomapOccupied_cubelist.scale.y = octo_reso;
            OctomapOccupied_cubelist.scale.z = octo_reso;
            OctomapOccupied_cubelist.color.a = 0.5;
            OctomapOccupied_cubelist.color.r = (double)19/255;
            OctomapOccupied_cubelist.color.g = (double)121/255;
            OctomapOccupied_cubelist.color.b = (double)156/255;

            unsigned long int j = 0;
            geometry_msgs::Point p;
            for(octomap::OcTree::leaf_iterator n = cur_tree_2d->begin_leafs(cur_tree_2d->getTreeDepth()); n != cur_tree_2d->end_leafs(); ++n) { // changed there#######
                if(!cur_tree_2d->isNodeOccupied(*n)) continue;
                p.x = n.getX();
                p.y = n.getY();
                p.z = n.getZ();
                OctomapOccupied_cubelist.points.push_back(p); 
                j++;
            }
            ROS_INFO("Publishing %ld occupied cells", j);
            Octomap_marker_pub.publish(OctomapOccupied_cubelist); //publish octomap############

              // Prepare the header for 2d occupied array
              now_marker = ros::Time::now();
              OctomapOccupied_cubelist_3d.header.frame_id = "map";
              OctomapOccupied_cubelist_3d.header.stamp = now_marker;
              OctomapOccupied_cubelist_3d.ns = "octomap_occupied_array";
              OctomapOccupied_cubelist_3d.id = 0;
              OctomapOccupied_cubelist_3d.type = visualization_msgs::Marker::CUBE_LIST;
              OctomapOccupied_cubelist_3d.action = visualization_msgs::Marker::ADD;
              OctomapOccupied_cubelist_3d.scale.x = octo_reso;
              OctomapOccupied_cubelist_3d.scale.y = octo_reso;
              OctomapOccupied_cubelist_3d.scale.z = octo_reso;
              OctomapOccupied_cubelist_3d.color.a = 0.5;
              OctomapOccupied_cubelist_3d.color.r = (double)19/255;
              OctomapOccupied_cubelist_3d.color.g = (double)121/255;
              OctomapOccupied_cubelist_3d.color.b = (double)156/255;
  
              j = 0;
              for(octomap::OcTree::leaf_iterator n = cur_tree->begin_leafs(cur_tree->getTreeDepth()); n != cur_tree->end_leafs(); ++n) {
                  if(!cur_tree->isNodeOccupied(*n)) continue;
                  p.x = n.getX();
                  p.y = n.getY();
                  p.z = n.getZ();
                  OctomapOccupied_cubelist.points.push_back(p); 
                  j++;
              }
              ROS_INFO("Publishing %ld 3D occupied cells", j);
              Octomap_marker_3d_pub.publish(OctomapOccupied_cubelist_3d); //publish octomap############

            // Prepare the header for free array
            now_marker = ros::Time::now();
            Free_cubelist.header.frame_id = "map";
            Free_cubelist.header.stamp = now_marker;
            Free_cubelist.ns = "octomap_free_array";
            Free_cubelist.id = 0;
            Free_cubelist.type = visualization_msgs::Marker::CUBE_LIST;
            Free_cubelist.action = visualization_msgs::Marker::ADD;
            Free_cubelist.scale.x = octo_reso;
            Free_cubelist.scale.y = octo_reso;
            Free_cubelist.scale.z = octo_reso;
            Free_cubelist.color.a = 0.3;
            Free_cubelist.color.r = (double)102/255;
            Free_cubelist.color.g = (double)255/255;
            Free_cubelist.color.b = (double)102/255;

            j = 0;
            
            for(octomap::OcTree::leaf_iterator n = cur_tree_2d->begin_leafs(cur_tree_2d->getTreeDepth()); n != cur_tree_2d->end_leafs(); ++n) { // changed there#######
                if(cur_tree_2d->isNodeOccupied(*n)) continue;
                p.x = n.getX();
                p.y = n.getY();
                p.z = n.getZ();
                Free_cubelist.points.push_back(p); 
                j++;
            }
            ROS_INFO("Publishing %ld free cells", j);
            Free_marker_pub.publish(Free_cubelist); //publish octomap############
            //unsigned long int g;

           // Prepare the header for 3D free array
            now_marker = ros::Time::now();
            Free_cubelist_3d.header.frame_id = "map";
            Free_cubelist_3d.header.stamp = now_marker;
            Free_cubelist_3d.ns = "octomap_free_array";
            Free_cubelist_3d.id = 0;
            Free_cubelist_3d.type = visualization_msgs::Marker::CUBE_LIST;
            Free_cubelist_3d.action = visualization_msgs::Marker::ADD;
            Free_cubelist_3d.scale.x = octo_reso;
            Free_cubelist_3d.scale.y = octo_reso;
            Free_cubelist_3d.scale.z = octo_reso;
            Free_cubelist_3d.color.a = 0.3;
            Free_cubelist_3d.color.r = (double)102/255;
            Free_cubelist_3d.color.g = (double)255/255;
            Free_cubelist_3d.color.b = (double)102/255;

            j = 0;
            //geometry_msgs::Point p;
            for(octomap::OcTree::leaf_iterator n = cur_tree->begin_leafs(cur_tree_2d->getTreeDepth()); n != cur_tree->end_leafs(); ++n) { // changed there#######
                if(cur_tree->isNodeOccupied(*n)) continue;
                p.x = n.getX();
                p.y = n.getY();
                p.z = n.getZ();
                Free_cubelist_3d.points.push_back(p); 
                j++;
            }
            ROS_INFO("Publishing %ld free cells", j);
            Free_marker_3d_pub.publish(Free_cubelist_3d); //publish octomap############

            // Send out results to file.
            explo_log_file.open(logfilename, std::ofstream::out | std::ofstream::app);
            explo_log_file << "DA Step: " << robot_step_counter << "  | Current Entropy: " << get_free_volume(cur_tree) << endl;
            explo_log_file.close();

        }
        else
        {
            ROS_ERROR("Failed to navigate to goal");
            /*/ if max MI candidates can't be arrived, try second max MI candidates

            // Get the current heading
            got_tf = false;
            while(!got_tf){
            try{
                tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
                laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                got_tf = true;
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Wait for tf: laser to map");
            }
            ros::Duration(0.05).sleep();
            }
            // Rotate negative along Yaw for 30 deg to look for more open areas
            Goal_heading.setRPY(0.0, 0.0, transform.getRotation().getAngle() - PI/6);
            Goal_heading.normalize();
            arrived = goToDest(laser_orig, Goal_heading); // order to move_base
            ros::spinOnce();*/
            

            /*if(p == 0){
               goto start_over;
            }*/
            p=p-10;
            if(p < 0){
               continue;
            }

            goto loop;
        }
    }
    nh.shutdown();          
    return 0;
}
