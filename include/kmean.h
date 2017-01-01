#ifndef KMEAN_H
#define KMEAN_H

#include "exploration.h"

using namespace cv;
using namespace std;

Mat kmean_explo(const vector<pair<point3d, point3d>> candidates, const int clustercount, const int count, const int max_iter, double epsilon,const int attempts, const int flags){

	vector<Point2f> points(candidates.size());
	Mat labels, centers;

	for(int i = 0; i< candidates.size() ; i++)
		points[i] = Point2f((float)(candidates[i].first.x()), (float)(candidates[i].first.y()));

	kmeans(points, clustercount, labels, TermCriteria(count, max_iter, epsilon), attempts, flags, centers);

    return labels;

}
#endif