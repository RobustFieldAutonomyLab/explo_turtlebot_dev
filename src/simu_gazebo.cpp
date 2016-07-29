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
#include <geometry_msgs/Pose.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <tf/transform_broadcaster.h>
#include "navigation_utils.h"
#include "move_base.h"


using namespace std;
using namespace std::chrono;


typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
const double PI = 3.1415926;
const double free_prob = 0.3;
const double z_sensor = 0.2972;
octomap::OcTree *tree;
octomap::OcTree* cur_tree;
octomap::OcTree *tree_kinect;
bool octomap_flag = 0; // 0 : msg not received
bool kinect_flag = 0; // 0 : msg not received
tf::TransformListener *tf_listener; 
   
point3d position;
point3d orientation;

ros::Publisher Map_pcl_pub;
ros::Publisher Free_pcl_pub;
ros::Publisher VScan_pcl_pub;

PointCloud::Ptr map_pcl (new PointCloud);
PointCloud::Ptr free_pcl (new PointCloud);
PointCloud::Ptr vsn_pcl (new PointCloud);

struct Kinect {
    double horizontal_fov;
    double angle_inc;
    double width;
    double height;
    double max_range;
    vector<pair<double, double>> pitch_yaws;

    Kinect(double _width, double _height, double _horizontal_fov, double _max_range)
            : width(_width), height(_height), horizontal_fov(_horizontal_fov), max_range(_max_range) {
        angle_inc = horizontal_fov / width;
        for(double i = -width / 2; i < width / 2; ++i) {
            for(double j = -height / 2; j < height / 2; ++j) {
                pitch_yaws.push_back(make_pair(j * angle_inc, i * angle_inc));
            }
        }
    }
}; 
Kinect InitialScan(1000, 1000, 6.0, 15.0);
Kinect kinect(640, 40, 3.047198, 15.0);

double get_free_volume(const octomap::OcTree *octree) {
    double volume = 0;
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        if(!octree->isNodeOccupied(*n))
            volume += pow(n.getSize(), 3);
    }
    return volume;
}

void get_free_points(const octomap::OcTree *octree, PointCloud::Ptr pclPtr) {
    
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        if(!octree->isNodeOccupied(*n))
        {
            pclPtr->points.push_back(pcl::PointXYZ(n.getX()/5.0, n.getY()/5.0, n.getZ()/5.0));
            pclPtr->width++;
        }

    }
    return;
}

vector<point3d> cast_init_rays(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &direction) {
    vector<point3d> hits;
    octomap::OcTreeNode *n;

    for(auto p_y : InitialScan.pitch_yaws) {
        double pitch = p_y.first;
        double yaw = p_y.second;
        point3d direction_copy(direction.normalized());
        point3d end;
        direction_copy.rotate_IP(0.0, pitch, yaw);
        if(octree->castRay(position, direction_copy, end, true, InitialScan.max_range)) {
            hits.push_back(end);
        } else {
            direction_copy *= InitialScan.max_range;
            direction_copy += position;
            n = octree->search(direction_copy);
            if (!n)
                continue;
            if (n->getOccupancy() < free_prob )
                continue;
            hits.push_back(direction_copy);
        }
    }        
    return hits;
}


vector<point3d> cast_kinect_rays(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &direction) {
    vector<point3d> hits;
    octomap::OcTreeNode *n;
    // cout << "d_x : " << direction.normalized().x() << " d_y : " << direction.normalized().y() << " d_z : " 
    //   << direction.normalized().z() << endl;
    for(auto p_y : kinect.pitch_yaws) {
        double pitch = p_y.first;
        double yaw = p_y.second;
        point3d direction_copy(direction.normalized());
        point3d end;
        direction_copy.rotate_IP(0.0, pitch, yaw);
        if(octree->castRay(position, direction_copy, end, true, kinect.max_range)) {
            hits.push_back(end);
        } else {
            direction_copy *= kinect.max_range;
            direction_copy += position;
            n = octree->search(direction_copy);
            if (!n)
                continue;
            if (n->getOccupancy() < free_prob )
                continue;
            hits.push_back(direction_copy);
        }
    }
    return hits;
}

vector<pair<point3d, point3d>> generate_candidates(point3d position) {
    double R = 0.25;   // Robot step, in meters.
    double n = 5;

    vector<pair<point3d, point3d>> candidates;
    double z = position.z();                // fixed 
    double pitch = 0;     // fixed
    double x, y;

    // for(z = position.z() - 1; z <= position.z() + 1; z += 1)
        for(double yaw = 0; yaw < 2 * PI; yaw += PI / n) {
            x = position.x() + R * cos(yaw);
            y = position.y() + R * sin(yaw);
            candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0, pitch, yaw)));
        }
    return candidates;
}

double calc_MI(const octomap::OcTree *octree, const point3d &position, const vector<point3d> &hits, const double before) {
    auto octree_copy = new octomap::OcTree(*octree);
    // octomap::OcTree *octree_copy = dynamic_cast<octomap::OcTree *>(octree);
    // cout << "th iter " << endl;
    for(const auto h : hits) {
        octree_copy->insertRay(position, h, kinect.max_range);
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

void octomap_callback(const octomap_msgs::Octomap::ConstPtr &octomap_msg) {
        octomap::OcTree *octomap_load = dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*octomap_msg));
        octomap_load->setResolution(0.2);
        // octomap::OcTree tree(0.2);
        // octomap::OcTree *octomap_curr = &tree;
        // auto octree_copy = new octomap::OcTree(*octomap_load);
        tree = octomap_load;
        octomap_flag = true;
        cout << "msg got !" << endl;
}

void kinect_callback( const sensor_msgs::PointCloud2ConstPtr& cloud2_msg ) {
    // kinect_flag = 1;
    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(*cloud2_msg, cloud2);
    PointCloud* cloud (new PointCloud);
    PointCloud* cloud_local (new PointCloud);
    pcl::fromPCLPointCloud2(cloud2,*cloud_local);

    pcl_ros::transformPointCloud("/map", *cloud_local, *cloud, *tf_listener);

    // update the position to center of the sensor
    // position(position.x(),) = z_sensor
    map_pcl->clear();
    map_pcl->header.frame_id = "/base_laser_link";
    long nan_points = 0;
    // for (int i = 1; i <= cloud->height; i++)
        for (int j = 1; j< cloud->width; j++)
        {
            // cout << cloud->at(j) << endl;
            if(isnan(cloud->at(j).x))
            {
                continue;
                nan_points++;
            }
                
            cur_tree->insertRay(point3d( position.x(),position.y(),z_sensor), 
                point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z), kinect.max_range);
            // cout << "pts : " << point3d(cloud->at(j).x, cloud->at(j).y, cloud->at(j).z) << endl;
            // add the point into map
            map_pcl->points.push_back(pcl::PointXYZ(cloud_local->at(j).x, cloud_local->at(j).y, cloud_local->at(j).z));
            map_pcl->width++;
        }
        map_pcl->height = 1;
    
    cout << "height : " << cloud->height << "  , width :" << cloud->width << " resulting map free volume" << get_free_volume(cur_tree) << endl;
    cout << "Nan Points in this kinect scan : " << nan_points << " out of " << cloud->width << endl;
    // delete octree_copy;
    map_pcl->header.stamp = ros::Time::now().toNSec() / 1e3;
    Map_pcl_pub.publish(map_pcl);
    cout << "Updating current map in pcl (points): " << map_pcl->width << endl;
    delete cloud;
    delete cloud_local;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Info_Exploration_Octomap");
    ros::NodeHandle nh;
    // Initial sub topic
    ros::Subscriber octomap_sub;
    ros::Subscriber kinect_sub;
    // ros::Subscriber octomap_sub;
    // octomap_sub = nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 10, octomap_callback);
    kinect_sub = nh.subscribe<sensor_msgs::PointCloud2>("/scan_pc2", 1, kinect_callback);
    // kinect_sub = nh.subscribe<sensor_msgs::PointCloud2>("/scan_pc2", 1, kinect_callback);
    
    Map_pcl_pub = nh.advertise<PointCloud>("Current_Map", 1);
    VScan_pcl_pub = nh.advertise<PointCloud>("virtual_Scans", 1);
    Free_pcl_pub = nh.advertise<PointCloud>("Free_points", 1);

    tf_listener = new tf::TransformListener();
    tf::StampedTransform transform;

    map_pcl->header.frame_id = "/base_laser_link";
    map_pcl->height = 1;
    map_pcl->width = 0;

    free_pcl->header.frame_id = "/map";
    free_pcl->height = 1;
    free_pcl->width = 0;

    vsn_pcl->header.frame_id = "/map";
    vsn_pcl->height = 1;
    vsn_pcl->width = 0;
   
    // Initialize parameters 
    ros::Rate r(1); // 10 hz
    int max_idx = 0;

    position = point3d(0, 0, 0.0);
    point3d dif_pos = point3d(0,0,0);
    point3d eu2dr(1, 0, 0);
    octomap::OcTreeNode *n;
    octomap::OcTree tree_new(0.1);
    cur_tree = &tree_new;

    // Initial Scan
    double goalx, goaly, goalz, goalw;
    MoveBase move(nh);

    // Get the current localization from tf
    try{
    ros::Time now = ros::Time::now();
    tf_listener->waitForTransform("/map", "/base_link",
                              now, ros::Duration(3.0));
    tf_listener->lookupTransform("/map", "/base_link",
                             now, transform);

    // tf_listener->lookupTransform("/map", "/base_link",  
    // ros::Time(0), transform);
    position = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    cout << "Current Robot Position : " << position << " yaw : " << transform.getRotation().w() << endl;
    }
    catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    }
    cout << "Initial  Position : " << position << endl;

    // RPY2Quaternion(0, 0, 1, &qx, &qy, &qz, &qw);
    goalx = position.x();
    goaly = position.y();
    goalz = position.z();

    for (int ini_i = 0; ini_i < 6; ini_i++)
    {
        try{
        ros::Time now = ros::Time::now();
        tf_listener->waitForTransform("/map", "/base_link",
                                  now, ros::Duration(3.0));
        tf_listener->lookupTransform("/map", "/base_link",
                                 now, transform);
        // tf_listener->lookupTransform("/map", "/base_link",  
        // ros::Time(0), transform);
        position = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        // cout << "Current Robot Position : " << position << " yaw : " << transform.getRotation().w() << endl;
        }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        }
        // move.move_to(position.x, position.y, position.z, 1.0);
        goalw = ini_i;
        move.move_to(goalx, goaly, goalz, goalw);
        ros::Duration(1.5).sleep(); // sleep for half a second
        cout << "make initial observation.. " << ini_i << "." << endl;
        ros::spinOnce();
    }


    double before = get_free_volume(cur_tree);
    cout << "CurMap  Entropy : " << get_free_volume(cur_tree) << endl;

    while (ros::ok())
    {
        // Generate Candidates
        vector<pair<point3d, point3d>> candidates = generate_candidates(position);
        vector<double> MIs(candidates.size());
        max_idx = 0;

        // Calculate Mutual Information
        for(int i = 0; i < candidates.size(); ++i) 
        {
            auto c = candidates[i];
            n = cur_tree->search(c.first);

            if (!n)     continue;
            if(n->getOccupancy() > free_prob)       continue;

            high_resolution_clock::time_point t1 = high_resolution_clock::now();
            eu2dr.rotate_IP(c.second.roll(), c.second.pitch(), c.second.yaw() );
            vector<point3d> hits = cast_kinect_rays(cur_tree, c.first, eu2dr);
            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
            cout << "Time on ray cast: " << duration << endl;

            t1 = high_resolution_clock::now();
            MIs[i] = calc_MI(cur_tree, c.first, hits, before);
            t2 = high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
            cout << "Candidate : " << c.first << "  **  MI: " << MIs[i] << endl;

            // Pick the Best Candidate
            if (MIs[i] > MIs[max_idx])
            {
                max_idx = i;
                vsn_pcl->clear();
                for (auto h : hits) {
                    vsn_pcl->points.push_back(pcl::PointXYZ(h.x()/5.0, h.y()/5.0, h.z()/5.0));
                    vsn_pcl->width++;
                }
                VScan_pcl_pub.publish(vsn_pcl);
            }

        }

        // Send the Robot 
        cout << "Sending the Goal : " << candidates[max_idx].first << " , Yaw : " << candidates[max_idx].second.yaw() << endl;
        // RPY2Quaternion(0, 0, candidates[max_idx].second.yaw(), &qx, &qy, &qz, &qw);

        // dif_pos = point3d(candidates[max_idx].first.x()-position.x(), 
        //     candidates[max_idx].first.y()-position.y(), 
        //     candidates[max_idx].first.z()-position.z()  );
        // move.move_to(dif_pos.x, dif_pos.y, dif_pos.z, candidates[max_idx].second.yaw());
        goalx = candidates[max_idx].first.x();
        goaly = candidates[max_idx].first.y();
        goalz = candidates[max_idx].first.z();
        goalw = candidates[max_idx].second.yaw();
        move.move_to(goalx, goaly, goalz, goalw);

        // Get the current localization from tf
        try{
        ros::Time now = ros::Time::now();
        tf_listener->waitForTransform("/map", "/base_link",
                                  now, ros::Duration(3.0));
        tf_listener->lookupTransform("/map", "/base_link",
                                 now, transform);
        // tf_listener->lookupTransform("/map", "/base_link",  
        // ros::Time(0), transform);
        position = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        cout << "Update current Position : " << position << endl;
        }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        }


        // cout << "Current Robot Location : " << position << endl;
        
        if(1)
          {
            cout << "We made one step forward" << endl;
            // Updating octomap with kinect call back.
            // RPY2Quaternion(0, 0, 1, &qx, &qy, &qz, &qw);
            for (int ini_i = 0; ini_i < 6; ini_i++)
            {
            try{
            ros::Time now = ros::Time::now();
            tf_listener->waitForTransform("/map", "/base_link",
                                      now, ros::Duration(3.0));
            tf_listener->lookupTransform("/map", "/base_link",
                                     now, transform);
            // tf_listener->lookupTransform("/map", "/base_link",  
            // ros::Time(0), transform);
            position = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            // cout << "Current Robot Position : " << position << " yaw : " << transform.getRotation().w() << endl;
            }
            catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            }
            // bool arrived = goToDest(position, qx, qy, qz, qw);
            cout << "make more observation.. " << ini_i << "." << endl;
            ros::spinOnce();
            }
                ros::spinOnce();
            // position = candidates[max_idx].first;
          }
        else
        {
            cout << "We didn't make it... starting over..." << endl;
        }
          
        cout << "CurMap  Entropy after movement : " << get_free_volume(cur_tree) << endl;
        r.sleep();
    }
    nh.shutdown();          
    return 0;
}
