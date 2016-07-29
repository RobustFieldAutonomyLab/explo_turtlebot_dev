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

#include <octomap/octomap.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
// #include <tf/transform_broadcaster.h>
#include "navigation_utils.h"
// #include "laser_geometry/laser_geometry.h"
#include <ros/callback_queue.h>
#include "gpregressor.h"
#include "covMaterniso3.h"


using namespace std;
using namespace std::chrono;
using namespace Eigen;


typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
const double PI = 3.1415926;
const double free_prob = 0.3;
const double octo_reso = 0.1;

octomap::OcTree* cur_tree;
octomap::OcTree* cur_tree_2d;

bool octomap_flag = 0; // 0 : msg not received
bool kinect_flag = 0; // 0 : msg not received
tf::TransformListener *tf_listener; 
int octomap_seq = 0;
   
point3d position, laser_orig, velo_orig;
ofstream explo_log_file;
std::string octomap_name_2d, octomap_name_3d;

// ros::Publisher Map_pcl_pub;
// ros::Publisher Free_pcl_pub;
// ros::Publisher VScan_pcl_pub;
// ros::Publisher octomap_pub;

// PointCloud::Ptr map_pcl (new PointCloud);
// PointCloud::Ptr free_pcl (new PointCloud);
// PointCloud::Ptr vsn_pcl (new PointCloud);

struct SensorModel {
    double horizontal_fov;
    double vertical_fov;
    double angle_inc_hor;
    double angle_inc_vel;
    double width;
    double height;
    double max_range;
    vector<pair<double, double>> pitch_yaws;

    SensorModel(double _width, double _height, double _horizontal_fov, double _vertical_fov, double _max_range)
            : width(_width), height(_height), horizontal_fov(_horizontal_fov), vertical_fov(_vertical_fov), max_range(_max_range) {
        angle_inc_hor = horizontal_fov / width;
        angle_inc_vel = vertical_fov / height;
        for(double j = -height / 2; j < height / 2; ++j) 
            for(double i = -width / 2; i < width / 2; ++i) {
                pitch_yaws.push_back(make_pair(j * angle_inc_vel, i * angle_inc_hor));
                pitch_yaws.push_back(make_pair(j * angle_inc_vel, i * angle_inc_hor));
        }
    }
}; 

// SensorModel Velodyne_puck(3600, 16, 2*PI, PI/6, 100.0 );
SensorModel Velodyne_puck(360, 16, 2*PI, 0.5236, 20.0);


double get_free_volume(const octomap::OcTree *octree) {
    double volume = 0;
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        if(!octree->isNodeOccupied(*n))
            volume += pow(n.getSize(), 3);
    }
    return volume;
}



vector<point3d> cast_sensor_rays(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &direction) {
    vector<point3d> hits;
    point3d direction_copy(direction.normalized());
    point3d end;
    // octomap::OcTreeNode *n;

    // #pragma omp parallel for
    for(auto p_y : Velodyne_puck.pitch_yaws) {
        double pitch = p_y.first;
        double yaw = p_y.second;

        direction_copy.rotate_IP(0.0, pitch, yaw);
        if(octree->castRay(position, direction_copy, end, true, Velodyne_puck.max_range)) {
            hits.push_back(end);
        } else {
            direction_copy *= Velodyne_puck.max_range;
            direction_copy += position;
            // n = octree->search(direction_copy);
            // if (!n)                                 continue;
            // if (n->getOccupancy() < free_prob )     continue;
            hits.push_back(direction_copy);
        }
    }
    return hits;
}

vector<pair<point3d, point3d>> generate_candidates(point3d sensor_orig, double initial_yaw) {
    double R = 0.5;   // Robot step, in meters.
    double n = 3;
    int counter = 0;
    octomap::OcTreeNode *n_cur;

    vector<pair<point3d, point3d>> candidates;
    double z = sensor_orig.z();                // fixed 
    double pitch = 0;                           // fixed
    double x, y;

    // for(z = sensor_orig.z() - 1; z <= sensor_orig.z() + 1; z += 1)
        for(double yaw = initial_yaw-PI/2; yaw < initial_yaw+PI/2; yaw += PI / (2*n) ) {
            x = sensor_orig.x() + R * cos(yaw);
            y = sensor_orig.y() + R * sin(yaw);
            n_cur = cur_tree_2d->search(point3d(x,y,z));
            if(!n_cur)                                  continue;
            if(n_cur->getOccupancy() > free_prob)       continue;
            candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0, pitch, yaw)));
            counter++;
        }
    return candidates;
}


vector<pair<point3d, point3d>> generate_testing(point3d sensor_orig, double initial_yaw) {
    double R = 0.5;   // Robot step, in meters.
    double n = 10;
    int counter = 0;
    octomap::OcTreeNode *n_cur;

    vector<pair<point3d, point3d>> candidates;
    double z = sensor_orig.z();                // fixed 
    double pitch = 0;                           // fixed
    double x, y;

    // for(z = sensor_orig.z() - 1; z <= sensor_orig.z() + 1; z += 1)
        for(double yaw = initial_yaw-PI/2; yaw < initial_yaw+PI/2; yaw += PI / (2*n) ) {
            x = sensor_orig.x() + R * cos(yaw);
            y = sensor_orig.y() + R * sin(yaw);
            n_cur = cur_tree_2d->search(point3d(x,y,z));
            if(!n_cur)                                  continue;
            if(n_cur->getOccupancy() > free_prob)       continue;
            candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0, pitch, yaw)));
            counter++;
        }
    return candidates;
}

double calc_MI(const octomap::OcTree *octree, const point3d &sensor_orig, const vector<point3d> &hits, const double before) {
    auto octree_copy = new octomap::OcTree(*octree);

    // #pragma omp parallel for
    for(const auto h : hits) {
        octree_copy->insertRay(sensor_orig, h, Velodyne_puck.max_range);
    }
    octree_copy->updateInnerOccupancy();
    double after = get_free_volume(octree_copy);
    delete octree_copy;
    return after - before;
}

void RPY2Quaternion(double roll, double pitch, double yaw, double *x, double *y, double *z, double *w) {
    double cr2, cp2, cy2, sr2, sp2, sy2;
    cr2 = cos(roll*0.5);
    cp2 = cos(pitch*0.5);
    cy2 = cos(yaw*0.5);

    sr2 = -sin(roll*0.5);
    sp2 = -sin(pitch*0.5);
    sy2 = sin(yaw*0.5);

    *w = cr2*cp2*cy2 + sr2*sp2*sy2;
    *x = sr2*cp2*cy2 - cr2*sp2*sy2;
    *y = cr2*sp2*cy2 + sr2*cp2*sy2;
    *z = cr2*cp2*sy2 - sr2*sp2*cy2;
}


void velodyne_callbacks( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg ) {
    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(*cloud2_msg, cloud2);
    PointCloud* cloud (new PointCloud);
    PointCloud* cloud_local (new PointCloud);
    pcl::fromPCLPointCloud2(cloud2,*cloud_local);
    octomap_msgs::Octomap cur_tree_msg;

    ros::Duration(0.07).sleep();
    while(!pcl_ros::transformPointCloud("/map", *cloud_local, *cloud, *tf_listener))
    {
        ros::Duration(0.01).sleep();
    }

    // Insert points into octomap one by one...
    for (int j = 1; j< cloud->width; j++)
    {
        if(isnan(cloud->at(j).x)) continue;
        cur_tree->insertRay(point3d( velo_orig.x(),velo_orig.y(),velo_orig.z()), 
            point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z), Velodyne_puck.max_range);
    }
    cur_tree->updateInnerOccupancy();
    // Insert point cloud into octomap.
    // cur_tree->insertPointCloud(*cloud,velo_orig);

    // bool res = octomap_msgs::fullMapToMsg(*cur_tree, cur_tree_msg);
    // if(res)
    //     ROS_INFO("octomap msg got it!");
    // else
    //     ROS_ERROR("Failed to convert octomap to msg...");
    // cur_tree_msg.header.seq = octomap_seq++;
    // cur_tree_msg.header.frame_id = "/map";
    // cur_tree_msg.header.stamp = cloud2_msg->header.stamp;
    // cur_tree_msg.resolution = octo_reso;
    // cur_tree_msg.binary = true;
    // cur_tree_msg.id = "OcTree";
    
    ROS_INFO("Entropy(3d map) : %f", get_free_volume(cur_tree));

    // cur_tree->writeBinary("Octomap_DA.bt");
    cur_tree->write(octomap_name_3d);
    // octomap_pub.publish(cur_tree_msg);
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

    ros::Duration(0.07).sleep();
    while(!pcl_ros::transformPointCloud("/map", *cloud_local, *cloud, *tf_listener))
    {
        ros::Duration(0.01).sleep();
    }

    // Insert points into octomap one by one...
    for (int j = 1; j< cloud->width; j++)
    {
        if(isnan(cloud->at(j).x)) continue;
        // cur_tree->insertRay(point3d( laser_orig.x(),laser_orig.y(),laser_orig.z()), 
        //     point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z), Velodyne_puck.max_range);
        cur_tree_2d->insertRay(point3d( laser_orig.x(),laser_orig.y(),laser_orig.z()), 
            point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z), 20.0);
    }
    cur_tree_2d->updateInnerOccupancy();
    ROS_INFO("Entropy(2d map) : %f", get_free_volume(cur_tree_2d));
    
    cur_tree_2d->write(octomap_name_2d);
    delete cloud;
    delete cloud_local;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "explo_octomap_da_2d");
    ros::NodeHandle nh;
    // ros::Subscriber octomap_sub;
    // octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, octomap_callback);

    // Initialize time
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,80,"%R_%M%S_%m%d_GP.txt",timeinfo);
    std::string logfilename(buffer);
    std::cout << logfilename << endl;
    strftime(buffer,80,"octomap_2d_%R_%M%S_%m%d_GP.ot",timeinfo);
    octomap_name_2d = buffer;
    strftime(buffer,80,"octomap_3d_%R_%M%S_%m%d_GP.ot",timeinfo);
    octomap_name_3d = buffer;    

    ros::Subscriber velodyne_sub = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, velodyne_callbacks);
    ros::Subscriber hokuyo_sub = nh.subscribe<sensor_msgs::PointCloud2>("/hokuyo_points", 1, hokuyo_callbacks);
    ros::Publisher GoalMarker_pub = nh.advertise<visualization_msgs::Marker>( "Goal_Marker", 1 );
    ros::Publisher JackalMarker_pub = nh.advertise<visualization_msgs::Marker>( "Jackal_Marker", 1 );
    ros::Publisher Candidates_pub = nh.advertise<visualization_msgs::MarkerArray>("Candidate_MIs", 1);
    ros::Publisher Octomap_marker_pub = nh.advertise<visualization_msgs::Marker>("Occupied_MarkerArray", 1);

    // octomap_pub = nh.advertise<octomap_msgs::Octomap>( "Octomap_realtime", 1);

    tf_listener = new tf::TransformListener();
    tf::StampedTransform transform;
    tf::Quaternion Goal_heading;

    visualization_msgs::MarkerArray CandidatesMarker_array;
    visualization_msgs::Marker OctomapOccupied_cubelist;
    
    // Map_pcl_pub = nh.advertise<PointCloud>("Current_Map", 1);
    // VScan_pcl_pub = nh.advertise<PointCloud>("virtual_Scans", 1);
    // Free_pcl_pub = nh.advertise<PointCloud>("Free_points", 1);

    // map_pcl->header.frame_id = "/velodyne";
    // map_pcl->height = 1;
    // map_pcl->width = 0;

    // free_pcl->header.frame_id = "/map";
    // free_pcl->height = 1;
    // free_pcl->width = 0;

    // vsn_pcl->header.frame_id = "/map";
    // vsn_pcl->height = 1;
    // vsn_pcl->width = 0;
   
    double R_velo, P_velo, Y_velo;
    // RPY2Quaternion(0, 0, 1, &qx, &qy, &qz, &qw);

    // Initialize parameters 
    // ros::Rate r(10); // 1 hz
    int max_idx = 0;

    position = point3d(0, 0, 0.0);
    point3d next_vp;
    // point3d dif_pos = point3d(0,0,0);
    point3d Sensor_PrincipalAxis(1, 0, 0);
    octomap::OcTreeNode *n;
    octomap::OcTree new_tree(octo_reso);
    octomap::OcTree new_tree_2d(octo_reso);
    cur_tree = &new_tree;
    cur_tree_2d = &new_tree_2d;

    bool got_tf = false;

    // Update the pose of velodyne from predefined tf.
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/base_link", "/velodyne", ros::Time(0), transform);
        velo_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        tf::Matrix3x3(transform.getRotation()).getRPY(R_velo, P_velo, Y_velo);
        Sensor_PrincipalAxis.rotate_IP(R_velo, P_velo, Y_velo);
        ROS_INFO("Current Velodyne heading: vector(%2.2f, %2.2f, %2.2f) -  RPY(%3.1f, %3.1f, %3.1f).", Sensor_PrincipalAxis.x(), Sensor_PrincipalAxis.y(), Sensor_PrincipalAxis.z(), R_velo/PI*180.0, P_velo/PI*180.0, Y_velo/PI*180.0);
        got_tf = true;
        }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: initial pose of Velodyne"); 
        ros::Duration(0.05).sleep();
        } 
    }   

    // Update the initial location of the robot and take the initial scan
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/velodyne", ros::Time(0), transform);
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

    ROS_INFO("Initial  Position : %3.2f, %3.2f, %3.2f - Yaw : %3.1f ", laser_orig.x(), laser_orig.y(), laser_orig.z(), transform.getRotation().getAngle()*PI/180);

    Goal_heading.setRPY(0.0, 0.0, 0.5);
    Goal_heading.normalize();
    bool arrived = goToDest(laser_orig, Goal_heading);

    // Update the current location of the robot and take the second scan
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/velodyne", ros::Time(0), transform);
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

    Goal_heading.setRPY(0.0, 0.0, 1.0);
    Goal_heading.normalize();
    arrived = goToDest(laser_orig, Goal_heading);

    // Update the location of the robot
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/velodyne", ros::Time(0), transform);
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

    double train_time, test_time;
    
    // steps robot taken, counter
    int robot_step_counter = 0;

    while (ros::ok())
    {
        // Generate Training poses
        vector<pair<point3d, point3d>> candidates = generate_candidates(laser_orig, transform.getRotation().getAngle());
        // Generate Testing poses
        vector<pair<point3d, point3d>> gp_test_poses = generate_testing(laser_orig, transform.getRotation().getAngle());

        vector<double> MIs(candidates.size());
        double before = get_free_volume(cur_tree);
        max_idx = 0;

        // for every candidate...
        #pragma omp parallel for
        for(int i = 0; i < candidates.size(); i++) 
        {
            auto c = candidates[i];
            // Evaluate Mutual Information
            Sensor_PrincipalAxis.rotate_IP(c.second.roll(), c.second.pitch(), c.second.yaw() );
            vector<point3d> hits = cast_sensor_rays(cur_tree, c.first, Sensor_PrincipalAxis);
            MIs[i] = calc_MI(cur_tree, c.first, hits, before);
        }

        // Initialize gp regression
        GPRegressor g(100, 2, 0.01);
        MatrixXf gp_train_x(candidates.size(), 2), gp_train_label(candidates.size(), 1), gp_test_x(gp_test_poses.size(), 2);

        for (int i=0; i< candidates.size(); i++){
            gp_train_x(i,0) = candidates[i].first.x();
            gp_train_x(i,1) = candidates[i].first.y();
            gp_train_label(i) = MIs[i];
        }

        for (int i=0; i< gp_test_poses.size(); i++){
            gp_test_x(i,0) = gp_test_poses[i].first.x();
            gp_test_x(i,1) = gp_test_poses[i].first.y();
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

        // Pick the Best from regression
        for (int i = 0; i < gp_test_poses.size(); i++){
            if (m(i) > m(max_idx))
            {
                max_idx = i;
            }
        }

        // Send the robot to the best action picked by GP 
        next_vp = point3d(gp_test_x(max_idx,0),gp_test_x(max_idx,1),candidates[0].first.z());
        Goal_heading.setRPY(0.0, 0.0, candidates[max_idx].second.yaw());
        Goal_heading.normalize();
        ROS_INFO("Estimated Max MI : %f , @ %3.2f,  %3.2f,  %3.2f", m(max_idx), next_vp.x(), next_vp.y(), next_vp.z() );


        // Publish the MIs of candidate actions as marker array in rviz
        tf::Quaternion MI_heading;
        MI_heading.setRPY(0.0, -PI/2, 0.0);
        MI_heading.normalize();
        
        CandidatesMarker_array.markers.resize(gp_test_poses.size());
        ros::Time now_marker = ros::Time::now();
        for (int i = 0; i < gp_test_poses.size(); i++)
        {
            CandidatesMarker_array.markers[i].header.frame_id = "map";
            CandidatesMarker_array.markers[i].header.stamp = now_marker;
            CandidatesMarker_array.markers[i].ns = "candidates";
            CandidatesMarker_array.markers[i].id = i;
            CandidatesMarker_array.markers[i].type = visualization_msgs::Marker::ARROW;
            CandidatesMarker_array.markers[i].action = visualization_msgs::Marker::ADD;
            CandidatesMarker_array.markers[i].pose.position.x = gp_test_x(i,0);
            CandidatesMarker_array.markers[i].pose.position.y = gp_test_x(i,1);
            CandidatesMarker_array.markers[i].pose.position.z = 0.0;
            CandidatesMarker_array.markers[i].pose.orientation.x = MI_heading.x();
            CandidatesMarker_array.markers[i].pose.orientation.y = MI_heading.y();
            CandidatesMarker_array.markers[i].pose.orientation.z = MI_heading.z();
            CandidatesMarker_array.markers[i].pose.orientation.w = MI_heading.w();
            CandidatesMarker_array.markers[i].scale.x = m(i)/m(max_idx) + 0.01;
            CandidatesMarker_array.markers[i].scale.y = 0.05;
            CandidatesMarker_array.markers[i].scale.z = 0.05;
            CandidatesMarker_array.markers[i].color.a = m(i)/m(max_idx) + 0.01;
            CandidatesMarker_array.markers[i].color.r = 0.0;
            CandidatesMarker_array.markers[i].color.g = 1.0;
            CandidatesMarker_array.markers[i].color.b = 0.0;
        }
        Candidates_pub.publish(CandidatesMarker_array);
        candidates.clear();

        // Publish the goal as a Marker in rviz
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now_marker;
        marker.ns = "goal_marker";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = next_vp.x();
        marker.pose.position.y = next_vp.y();
        marker.pose.position.z = next_vp.z();
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
        GoalMarker_pub.publish( marker );

        // Send the Robot 
        arrived = goToDest(next_vp, Goal_heading);

        if(arrived)
        {
            // Update the initial location of the robot
            got_tf = false;
            while(!got_tf){
            try{
                tf_listener->lookupTransform("/map", "/velodyne", ros::Time(0), transform);
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

            unsigned long num_occupied_cells = 0;
            double testX;
            for(octomap::OcTree::leaf_iterator n = cur_tree->begin_leafs(cur_tree->getTreeDepth()); n != cur_tree->end_leafs(); ++n) {
                if(cur_tree->isNodeOccupied(*n)){
                    num_occupied_cells++;
                }
            }            
            
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
            OctomapOccupied_cubelist.color.a = 1.0;
            OctomapOccupied_cubelist.color.r = (double)19/255;
            OctomapOccupied_cubelist.color.g = (double)121/255;
            OctomapOccupied_cubelist.color.b = (double)156/255;

            unsigned long int j = 0;
            geometry_msgs::Point p;
            for(octomap::OcTree::leaf_iterator n = cur_tree->begin_leafs(cur_tree->getTreeDepth()); n != cur_tree->end_leafs(); ++n) {
                if(!cur_tree->isNodeOccupied(*n)) continue;
                p.x = n.getX();
                p.y = n.getY();
                p.z = n.getZ();
                OctomapOccupied_cubelist.points.push_back(p); 
                j++;
            }
            ROS_INFO("Publishing %ld occupied cells", j);
            Octomap_marker_pub.publish(OctomapOccupied_cubelist);

            OctomapOccupied_cubelist.points.clear();
            gp_test_poses.clear();

            // Send out results to file.
            explo_log_file.open(logfilename, std::ofstream::out | std::ofstream::app);
            explo_log_file << "GP Step: " << robot_step_counter << "  | Current Entropy: " << get_free_volume(cur_tree) << endl;
            explo_log_file.close();

        }
        else
        {
            ROS_ERROR("Failed to navigate to goal");
        }
        // r.sleep();
    }
    nh.shutdown();          
    return 0;
}
