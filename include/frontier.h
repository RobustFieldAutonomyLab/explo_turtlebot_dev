#ifndef GENERATE_FRONTIER_H
#define GENERATE_FRONTIER_H

#include "exploration.h"

using namespace cv;
using namespace std;

/*vector<vector<point3d>> generate_frontier_points(const octomap::OcTree *octree) {

    vector<vector<point3d>> frontier_lines;
    vector<point3d> frontier_points;
    octomap::OcTreeNode *n_cur_frontier;
    bool frontier_true; // whether or not a frontier point
    bool belong_old;//whether or not belong to old group
    double distance;
    double R1 = 1; //group length


    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n)
    {
        frontier_true = false;
        unsigned long int num_free = 0; //number of free cube around frontier, for filtering out fake frontier

      if(!octree->isNodeOccupied(*n))
        {
         double x_cur = n.getX();
         double y_cur = n.getY();
         double z_cur = n.getZ();
         //if there are unknown around the cube, the cube is frontier
         for (double x_cur_buf = x_cur - octo_reso; x_cur_buf < x_cur + octo_reso; x_cur_buf += octo_reso)
             for (double y_cur_buf = y_cur - octo_reso; y_cur_buf < y_cur + octo_reso; y_cur_buf += octo_reso)
            {
                n_cur_frontier = octree->search(point3d(x_cur_buf, y_cur_buf, z_cur));
                if(!n_cur_frontier)
                {
                    frontier_true = true;
                    continue;            
                }
                else if (!octree->isNodeOccupied(n_cur_frontier))
                {
                    num_free++;

                }

            }
            if(frontier_true)// && num_free >5 )//filter out fake frontier by counting free
            {

                double x_frontier = x_cur;
                double y_frontier = y_cur;
                double z_frontier = z_cur;

                // divede frontier points into groups
                if(frontier_lines.size() < 1)
                {
                    frontier_points.resize(1);
                    frontier_points[0] = point3d(x_frontier,y_frontier,z_frontier);
                    frontier_lines.push_back(frontier_points);
                    frontier_points.clear();
                }
                else
                {
                    bool belong_old = false;            

                    for(vector<vector<point3d>>::size_type u = 0; u < frontier_lines.size(); u++){
                            distance = sqrt(pow(frontier_lines[u][0].x()-x_frontier, 2)+pow(frontier_lines[u][0].y()-y_frontier, 2)) ;
                            if(distance < R1){
                               frontier_lines[u].push_back(point3d(x_frontier, y_frontier, z_frontier));
                               belong_old = true;
                               break;
                            }
                    }
                    if(!belong_old){
                               frontier_points.resize(1);
                               frontier_points[0] = point3d(x_frontier, y_frontier, z_frontier);
                               frontier_lines.push_back(frontier_points);
                               frontier_points.clear();
                    }                              
                }

            } 
        }
        
    }
    return frontier_lines;
}*/



vector<vector<point3d>> generate_frontier_points_3d(const octomap::OcTree *octree, const double z, const double lower, const double upper) {

    vector<vector<point3d>> frontier_lines;
    vector<point3d> frontier_points;
    octomap::OcTreeNode *n_cur_frontier;
    bool frontier_true; // whether or not a frontier point
    bool belong_old;//whether or not belong to old group
    double distance;
    double R1 = 0.5; //group length
    //double x_frontier;
    //double y_frontier;
    //double z_frontier;

    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n)
    {
        frontier_true = false;
        unsigned long int num_free = 0; //number of free cube around frontier, for filtering out fake frontier

      if(!octree->isNodeOccupied(*n))
        {
            if(n.getZ() >= z+lower&&n.getZ() <= z+upper)// frontier on specific height
            {
                double x_cur = n.getX();
                double y_cur = n.getY();
                double z_cur = n.getZ();
                //if there are unknown around the cube, the cube is frontier


                    for (double x_cur_buf = x_cur - octo_reso; x_cur_buf < x_cur + octo_reso; x_cur_buf += octo_reso)
                       for (double y_cur_buf = y_cur - octo_reso; y_cur_buf < y_cur + octo_reso; y_cur_buf += octo_reso)
                       //for (double z_cur_buf = z_cur - 0.1; z_cur_buf < z_cur + 0.15; z_cur_buf += octo_reso)
                        {
                           n_cur_frontier = octree->search(point3d(x_cur_buf, y_cur_buf, z_cur));
                           if(!n_cur_frontier)
                            {
                              frontier_true = true;
                              continue;            
                            }
                            else if (!octree->isNodeOccupied(n_cur_frontier))
                            {
                               num_free++;
                            }     
                        }
                if(frontier_true)// && num_free >5 )
                {

                    double x_frontier = x_cur;
                    double y_frontier = y_cur;
                    double z_frontier = z_cur;

                    // divede frontier points into groups
                    if(frontier_lines.size() < 1)
                    {
                        frontier_points.resize(1);
                        frontier_points[0] = point3d(x_frontier,y_frontier,z_frontier);
                        frontier_lines.push_back(frontier_points);
                        frontier_points.clear();
                    }
                    else
                    {
                        bool belong_old = false;
                        bool repet = false;           

                        for(vector<vector<point3d>>::size_type u = 0; u < frontier_lines.size(); u++){
                                distance = sqrt(pow(frontier_lines[u][0].x()-x_frontier, 2)+pow(frontier_lines[u][0].y()-y_frontier, 2)) ;
                                if(!repet){
                                  for(vector<point3d>::size_type v = 0; v < frontier_lines[u].size(); v++){
                                    double dist_2 = sqrt(pow(frontier_lines[u][v].x()-x_frontier, 2)+pow(frontier_lines[u][v].y()-y_frontier, 2));
                                    if(dist_2 < 0.6*octo_reso) {
                                      repet = true;
                                      break;
                                    }
                                  }
                                }

                                if(distance < R1&&!repet){
                                    frontier_lines[u].push_back(point3d(x_frontier, y_frontier, z_frontier));
                                    belong_old = true;
                                    break;
                                }
                        }
                        if(!belong_old&&!repet){
                                   frontier_points.resize(1);
                                   frontier_points[0] = point3d(x_frontier, y_frontier, z_frontier);
                                   frontier_lines.push_back(frontier_points);
                                   frontier_points.clear();
                        }                              
                    }

                } 
            }
        }
        
    }
    return frontier_lines;
}

#endif
