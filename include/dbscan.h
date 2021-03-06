#ifndef DBSCAN_H
#define DBSCAN_H

#include "exploration.h"

using namespace std;

vector<int> regionQuery(vector<pair<point3d, point3d>> keypoints, int keypoint, float eps)
{
	float dist;
	vector<int> retKeys;
	for(int i = 0; i< keypoints.size(); i++)
	{
	    dist = sqrt(pow((keypoints[keypoint].first.x() - keypoints[i].first.x()),2)+pow((keypoints[keypoint].first.y() - keypoints[i].first.y()),2));
	    if(dist <= eps && i != keypoint)
	    {
	        retKeys.push_back(i);
	    }
	}
	return retKeys;
}

pair<vector<int>, int> DBSCAN_keypoints(vector<pair<point3d, point3d>> keypoints, float eps, int minPts)
{
	vector<int> clusters_index;
	vector<bool> clustered;
	//vector<int> noise;
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
	    clusters_index.push_back(0);
	}

	
	c = 1;
	//clusters.push_back(vector<pair<point3d, point3d>>()); //will stay empty?

	//for each unvisted point P in dataset keypoints
	for(int i = 0; i < noKeys; i++)
	{
	    if(!visited[i])
	    {
	        //Mark P as visited
	        visited[i] = true;
	        neighborPts = regionQuery(keypoints,i,eps);
	        if(neighborPts.size() < minPts)
	            //Mark P as Noise
	            clusters_index[i] = 0;
	        else
	        {
				clusters_index[i] = c;
	            clustered[i] = true; 
	            //for each point P' in neighborPts
	            for(int j = 0; j < neighborPts.size(); j++)
	            {
	                //if P' is not visited
	                if(!visited[neighborPts[j]])
	                {
	                    //Mark P' as visited
	                    visited[neighborPts[j]] = true;
	                    neighborPts_ = regionQuery(keypoints,neighborPts[j],eps);
	                    if(neighborPts_.size() >= minPts)
	                    {
	                        neighborPts.insert(neighborPts.end(),neighborPts_.begin(),neighborPts_.end());
	                    }
	                    else
	                    {
	                    	clusters_index[neighborPts[j]] = 0;
	                    }
	                }
	                // if P' is not yet a member of any cluster
	                // add P' to cluster c
	                if(!clustered[neighborPts[j]]){
	                    clusters_index[neighborPts[j]] = c;
	                    clustered[neighborPts[j]] = true;
	                }
	            }
            	c++;	            
	        }

	    }
	}
	return make_pair( clusters_index, c );
}

#endif
