#include <iostream>
#include <fstream>
#include <algorithm>
#include <iterator>

#include <ros/ros.h>

#include <isam/isam.h>

using namespace std;
using namespace isam;
using namespace Eigen;

int main(int argc, char **argv) {
    ros::init(argc, argv, "isam_ros");
    ros::NodeHandle nh;
  // instance of the main class that manages and optimizes the pose graph
  Slam slam;

  // locally remember poses
  vector<Pose2d_Node*> pose_nodes;

  Noise noise3 = Information(100. * eye(3));
  Noise noise2 = Information(100. * eye(2));

  // create a first pose (a node)
  Pose2d_Node* new_pose_node = new Pose2d_Node();
  // add it to the graph
  slam.add_node(new_pose_node);
  // also remember it locally
  pose_nodes.push_back(new_pose_node);

  // create a prior measurement (a factor)
  Pose2d origin(0., 0., 0.);
  Pose2d_Factor* prior = new Pose2d_Factor(pose_nodes[0], origin, noise3);
  // add it to the graph
  slam.add_factor(prior);

  for (int i=1; i<4; i++) {
    // next pose
    Pose2d_Node* new_pose_node = new Pose2d_Node();
    slam.add_node(new_pose_node);
    pose_nodes.push_back(new_pose_node);

    // connect to previous with odometry measurement
    Pose2d odometry(1.0, 0., 0.); // x,y,theta
    Pose2d_Pose2d_Factor* constraint = new Pose2d_Pose2d_Factor(pose_nodes[i-1], pose_nodes[i], odometry, noise3);
    slam.add_factor(constraint);
  }

  // create a landmark
  Point2d_Node* new_point_node = new Point2d_Node();
  slam.add_node(new_point_node);

  // create a pose and the landmark by a measurement
  Point2d measure(5., 3.); // x,y
  Pose2d_Point2d_Factor* measurement =
    new Pose2d_Point2d_Factor(pose_nodes[1], new_point_node, measure, noise2);
  slam.add_factor(measurement);

  // optimize the graph
  slam.batch_optimization();

  // accessing the current estimate of a specific pose
  cout << "Pose 1: " << pose_nodes[1]->value() << endl;
  cout << "Pose 2: " << pose_nodes[2]->value() << endl;

  // printing the complete graph
  cout << endl << "Full graph:" << endl;
  slam.write(cout);

  return 0;
}
