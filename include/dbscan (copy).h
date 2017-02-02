#ifndef DBSCAN_H
#define DBSCAN_H

#include "exploration.h"

using namespace std;

/*vector<pair<point3d, point3d>> dbscan(vector<pair<point3d, point3d>> candidates, float eps, int minPts)
{
	vector<KeyPoint> *keypoints;
	keypoints->resize(candidates.size());
	for(i = 0; i < candidates.size(); i++){
		keypoints->at(i).pt.x = candidates[i].first.x();
		keypoints->at(i).pt.y = candidates[i].first.y();
	}

	vector<vector<KeyPoint>> clusters = DBSCAN_keypoints(keypoints, eps, minPts);


}*/

vector<int> regionQuery(vector<pair<point3d, point3d>> keypoints, pair<point3d, point3d> keypoint, float eps)
{
	float dist;
	vector<int> retKeys;
	for(int i = 0; i< keypoints.size(); i++)
	{
	    dist = sqrt(pow((keypoint.first.x() - keypoints[i].first.x()),2)+pow((keypoint.first.y() - keypoints[i].first.y()),2));
	    if(dist <= eps && dist != 0.0f)
	    {
	        retKeys.push_back(i);
	    }
	}
	return retKeys;
}

vector<vector<pair<point3d, point3d>>> DBSCAN_keypoints(vector<pair<point3d, point3d>> keypoints, float eps, int minPts)
{
	vector<vector<pair<point3d, point3d>>> clusters;
	vector<bool> clustered;
	vector<int> noise;
	vector<bool> visited;
	vector<int> neighborPts;
	vector<int> neighborPts_;
	int c;

	int noKeys = keypoints.size();

	//init clustered and visited
	for(int k = 0; k < noKeys; k++)
	{
	    clustered.push_back(false);
	    visited.push_back(false);
	}

	//C =0;
	c = 0;
	clusters.push_back(vector<pair<point3d, point3d>>()); //will stay empty?

	//for each unvisted point P in dataset keypoints
	for(int i = 0; i < noKeys; i++)
	{
	    if(!visited[i])
	    {
	        //Mark P as visited
	        visited[i] = true;
	        neighborPts = regionQuery(keypoints,keypoints[i],eps);
	        if(neighborPts.size() < minPts)
	            //Mark P as Noise
	            noise.push_back(i);
	        else
	        {
	            clusters.push_back(vector<pair<point3d, point3d>>());
	            c++;
	            //expand cluster
	            // add P to cluster c
	            clusters[c].push_back(keypoints[i]);
	            //for each point P' in neighborPts
	            for(int j = 0; j < neighborPts.size(); j++)
	            {
	                //if P' is not visited
	                if(!visited[neighborPts[j]])
	                {
	                    //Mark P' as visited
	                    visited[neighborPts[j]] = true;
	                    neighborPts_ = regionQuery(keypoints,keypoints[neighborPts[j]],eps);
	                    if(neighborPts_.size() >= minPts)
	                    {
	                        neighborPts.insert(neighborPts.end(),neighborPts_.begin(),neighborPts_.end());
	                    }
	                }
	                // if P' is not yet a member of any cluster
	                // add P' to cluster c
	                if(!clustered[neighborPts[j]])
	                    clusters[c].push_back(keypoints[neighborPts[j]]);
	            }
	        }

	    }
	}
	return clusters;
}

#endif
