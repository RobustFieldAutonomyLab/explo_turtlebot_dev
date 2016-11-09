#include "navigation_utils.h"
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <octomap/octomap.h>


 
bool goToDest(point3d go_posi, tf::Quaternion q) {

  // make an action client that spins up a thread
  MoveBaseClient ac("move_base", true);

  // cancel previous goals
  ac.cancelAllGoals();

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  // goal.target_pose.header.frame_id = "base_footprint";
    goal.target_pose.header.frame_id = "map";

  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = go_posi.x();
  goal.target_pose.pose.position.y = go_posi.y();
  goal.target_pose.pose.position.z = go_posi.z();
  // goal.target_pose.pose.position.x = cur_posi.x();
  // goal.target_pose.pose.position.y = cur_posi.y();
  // goal.target_pose.pose.position.z = cur_posi.z();
  goal.target_pose.pose.orientation.x = q.x();
  goal.target_pose.pose.orientation.y = q.y();
  goal.target_pose.pose.orientation.z = q.z();
  goal.target_pose.pose.orientation.w = q.w();

  ROS_INFO("Sending goal to (%3.2f, %3.2f, %3.2f)", go_posi.x(), go_posi.y(), go_posi.z());
  ac.sendGoal(goal);

  // while(ros::ok())
  ac.waitForResult(ros::Duration(30.0));

  // Returns true iff we reached the goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    return true;
  else
    return false;
  }


/*int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_utils");
  // point3d cur_posi(0,0,0);

  // double qx, qy, qz, qw;
  // RPY2Quaternion(0, 0, 1, &qx, &qy, &qz, &qw);

  // point3d posi1(1.0,0.0,0);
  // bool arrived = goToDest(posi1, qx, qy, qz, qw);

  // posi1 = point3d(0.0,0.0,0);
  // RPY2Quaternion(0, 0, -1, &qx, &qy, &qz, &qw);
  //  arrived = goToDest(posi1, qx, qy, qz, qw);

  //  //   go_posi = point3d(0.2,0.1,0);
  //  // arrived = goToDest(go_posi, 0, 0, 0, 1.0);

  // if(arrived)
  //   ROS_INFO("We moved forward one meter!");
  // else
  //   ROS_INFO("We could not move forward one meter for some reason");

  return 0;
}*/
