#ifndef KMEAN_H
#define KMEAN_H

#include "exploration.h"

using namespace cv;
using namespace std;

Mat kmean_explo(const vector<pair<point3d, point3d>> candidates, const int clustercount, const int count, const int max_iter, double epsilon,const int attempts, const int flags){

	//vector<Point2f> points(candidates.size());

	Mat labels(candidates.size(), 1, CV_32FC2), centers, points(candidates.size(), 2, CV_32FC2);
        ROS_INFO("here 1!");

	for(int i = 0; i< candidates.size() ; i++){
                ROS_INFO("here 2!");
		points.at<double>(i,0) = candidates[i].first.x();
		points.at<double>(i,1) = candidates[i].first.y();
                ROS_INFO("here 3!");
        }
        ROS_INFO("here 4!");
	kmeans(points, clustercount, labels, TermCriteria(count, max_iter, epsilon), attempts, flags, centers);
        ROS_INFO("here 5!");
    return labels;

}
#endif