#ifndef GENERATE_CANDIDATES_H
#define GENERATE_CANDIDATES_H


#include "exploration.h"

using namespace cv;
using namespace std;

//generate candidates for moving. Input sensor_orig and initial_yaw, Output candidates
//senor_orig: locationg of sensor.   initial_yaw: yaw direction of sensor
vector<pair<point3d, point3d>> generate_candidates(vector<vector<point3d>> frontier_groups, point3d sensor_orig, const double R2_min, double R_interv, const double R2_max, int n_angle) {
    double R2;        // choose candidtates in the distance of R2 to the center of frontier group
    double R3 = 0.1;       //minial distance of candidates to other frontiers

    octomap::OcTreeNode *n_cur_3d;
    vector<pair<point3d, point3d>> candidates;
    vector<pair<point3d, point3d>> candidates_temp;
    vector<pair<point3d, point3d>> candidates_temp2;
    vector<double> dist;
    double z = sensor_orig.z();
    double x, y;
    double yaw;
    double distance_can;
    for(R2 = R2_min; R2 <= R2_max; R2 = R2 + R_interv){

        for(vector<vector<point3d>>::size_type u = 0; u < frontier_groups.size(); u++) {
            if(frontier_groups[u].size() > 8){
                for(double yaw = 0; yaw < 2*PI; yaw += PI*2 / n_angle){ 
                    x = frontier_groups[u][0].x() - R2 * cos(yaw);
                    y = frontier_groups[u][0].y() - R2 * sin(yaw);

                    bool candidate_valid = true;
                    n_cur_3d = cur_tree->search(point3d(x, y, z));


                    if (!n_cur_3d) {
                        candidate_valid = false;
                        continue;
                    }

                    if(sqrt(pow(x - sensor_orig.x(),2) + pow(y - sensor_orig.y(),2)) < 0.25){
                      candidate_valid = false;// delete candidates close to sensor_orig
                      continue;
                    }

                    else{

                        // check candidate to other frontiers;
                        for(vector<vector<point3d>>::size_type n = 0; n < frontier_groups.size(); n++)
                            for(vector<point3d>::size_type m = 0; m < frontier_groups[n].size(); m++){
                                distance_can = sqrt(pow(x - frontier_groups[n][m].x(),2) + pow(y - frontier_groups[n][m].y(),2));
                                if(distance_can < R3){
                                    candidate_valid = false;        //delete candidates close to frontier
                                    continue;
                                }
                        }
                    
                        // volumn check
                        for (double x_buf = x - 0.3; x_buf < x + 0.3; x_buf += octo_reso) 
                            for (double y_buf = y - 0.3; y_buf < y + 0.3; y_buf += octo_reso)
                                for (double z_buf = sensor_orig.z()-0.2; z_buf <sensor_orig.z()+0.2; z_buf += octo_reso)
                                {
                                    n_cur_3d = cur_tree->search(point3d(x_buf, y_buf, z_buf));
                                    if(!n_cur_3d)       continue;
                                    else if (cur_tree->isNodeOccupied(n_cur_3d)){
                                    candidate_valid = false;//delete candidates which have ccupied cubes around in 3D area
                                    continue;
                                    }  
                                }

                    }

                    if (candidate_valid)
                    {
                        dist.push_back(sqrt(pow(x - sensor_orig.x(), 2) + pow(y - sensor_orig.y(), 2)));
                        candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, yaw)));
                    }
                }
            }
        }

    }
    return candidates;
}

#endif
