#include "exploration.h"
#include "navigation_utils.h"
#include "gpregressor.h"
#include "covMaterniso3.h"
#include "frontier.h"
#include "candidates.h"

using namespace std;

void frontier_callbacks(const visualization_msgs::Marker frontiers){

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "explo_sam_2d_turtlebot");
	ros::NodeHandle nh;

	ros::Subscriber frontier_sub = nh.subscribe("Frontier_points", 1, frontier_callbacks);

	ros::spin();

	return 0;
}