#ifndef GENERATE_FRONTIER_H
#define GENERATE_FRONTIER_H

#include "exploration.h"
#include "Eigen/Dense"
#include "Eigen/Eigenvalues"

using namespace cv;
using namespace std;

vector<vector<point3d>> frontier_median_filter(const vector<vector<point3d>> frontier_lines){

  vector<vector<point3d>> frontier_lines_new;
  vector<double> frontier_distance;
  vector<int> frontier_index;
  frontier_lines_new.resize(frontier_lines.size());
  frontier_distance.resize(frontier_lines.size());
  double distance_frontier_temp;
  vector<int> index_temp;
  //ROS_INFO("here 1");
  for(int i = 0; i < frontier_lines.size(); i++){
    frontier_lines_new[i].resize(frontier_lines[i].size());       
    for(int j = 0; j < frontier_lines[i].size(); j++){
      frontier_distance.resize(1);
      frontier_index.resize(1);
      frontier_distance[0] = 0;
      frontier_index[0] = j;
      //ROS_INFO("here 2"); 
      for(int m = 0; m < frontier_lines[i].size(); m++){
        if(j != m){
          distance_frontier_temp = sqrt(pow(frontier_lines[i][j].x() - frontier_lines[i][m].x(), 2) + pow(frontier_lines[i][j].y() - frontier_lines[i][m].y(), 2) + pow(frontier_lines[i][j].z() - frontier_lines[i][m].z(), 2));
          //ROS_INFO("here 3");
          if(distance_frontier_temp < 0.25){
            frontier_distance.push_back(distance_frontier_temp);
            //ROS_INFO("here 4");
            frontier_index.push_back(m);
            //ROS_INFO("here 5");
          }
        }
      }
      index_temp = sort_index(frontier_distance);
      //ROS_INFO("here 6");
      if(index_temp.size()%2 == 0){
        frontier_lines_new[i][j] = frontier_lines[i][frontier_index[index_temp[index_temp.size()/2]]];
        //ROS_INFO("here 7");
      }
      else{
        frontier_lines_new[i][j] = point3d((frontier_lines[i][frontier_index[index_temp[(index_temp.size()+1)/2]]].x() + frontier_lines[i][frontier_index[index_temp[(index_temp.size()-1)/2]]].x())/2,
          (frontier_lines[i][frontier_index[index_temp[(index_temp.size()+1)/2]]].y() + frontier_lines[i][frontier_index[index_temp[(index_temp.size()-1)/2]]].y())/2,
          (frontier_lines[i][frontier_index[index_temp[(index_temp.size()+1)/2]]].z() + frontier_lines[i][frontier_index[index_temp[(index_temp.size()-1)/2]]].z())/2 );
          //ROS_INFO("here 8");
      }
    }
  }
  return frontier_lines_new;
}

vector<point3d> generate_fnormal(const vector<vector<point3d>> frontier_lines){
  MatrixXd frontiers;
  MatrixXd centered;
  MatrixXd cov_f;
  double scalar;
  vector<point3d> normals(frontier_lines.size());

  for(int i = 0; i < frontier_lines.size(); i++){

    frontiers.resize(3, frontier_lines[i].size());
    for(int j = 0; j < frontier_lines[i].size(); j++){
      frontiers(0, j) = frontier_lines[i][j].x();
      frontiers(1, j) = frontier_lines[i][j].y();
      frontiers(2, j) = frontier_lines[i][j].z();
    }
    centered = frontiers.colwise() - frontiers.rowwise().mean();
    cov_f = (centered*centered.adjoint())/double(centered.rows() - 1);
    SelfAdjointEigenSolver<MatrixXd> es(cov_f);
    VectorXd v = es.eigenvectors().col(0);
    scalar = 0.5/sqrt(pow(v(0, 0),2) + pow(v(1, 0),2) +pow(v(2, 0),2));
    normals[i].x() = v(0, 0)*scalar;
    normals[i].y() = v(1, 0)*scalar;
    normals[i].z() = v(2, 0)*scalar;
  }
  return normals;
}

vector<vector<point3d>> generate_frontier_points_3d(const octomap::OcTree *octree, const double z, const double lower, const double upper) {

    vector<vector<point3d>> frontier_lines;
    vector<point3d> frontier_points;
    octomap::OcTreeNode *n_cur_frontier;
    bool frontier_true; // whether or not a frontier point
    bool belong_old;//whether or not belong to old group
    double distance;
    double R1 = 0.4; //group length
    //double x_frontier;
    //double y_frontier;
    //double z_frontier;

    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n)
    {
        frontier_true = false;
        unsigned long int num_free = 0;//number of free cube around frontier, for filtering out fake frontier
        unsigned long int num_occupied = 0;

      if(!octree->isNodeOccupied(*n))
        {
            if(n.getZ() >= z+lower&&n.getZ() <= z+upper)// frontier on specific height
            {
                double x_cur = n.getX();
                double y_cur = n.getY();
                double z_cur = n.getZ();
                //if there are unknown around the cube, the cube is frontier
                //n_cur_frontier = octree->search(point3d(x_cur, y_cur, z_cur+octo_reso));
                //if(!n_cur_frontier) continue;

                for (double x_cur_buf = x_cur - 2*octo_reso; x_cur_buf < x_cur + 2*octo_reso; x_cur_buf += octo_reso)
                   for (double y_cur_buf = y_cur - 2*octo_reso; y_cur_buf < y_cur + 2*octo_reso; y_cur_buf += octo_reso)
			                 for (double z_cur_buf = z_cur - 2*octo_reso; z_cur_buf < z_cur + 2*octo_reso; z_cur_buf += octo_reso){
                     	 n_cur_frontier = octree->search(point3d(x_cur_buf, y_cur_buf, z_cur_buf));
                     	 if(!n_cur_frontier)
                     	  {
                        	frontier_true = true;
                        	continue;            
                      	}
                   	 /* for (double z_cur_buf = z_cur - 0.1; z_cur_buf < z_cur + 0.15; z_cur_buf += octo_reso)
                      {
                         n_cur_frontier = octree->search(point3d(x_cur, y_cur, z_cur_buf));
                         if(!n_cur_frontier)
                          {
                            frontier_true = false;
                            continue;            
                          }
                          else if (!octree->isNodeOccupied(n_cur_frontier))
                          {
                             num_free++;
                          }
                          else if (octree->isNodeOccupied(n_cur_frontier)){
                             num_occupied++;
                          }     
                      }*/
                    }
                if(frontier_true)// && num_occupied < 100)
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
                                distance = sqrt(pow(frontier_lines[u][0].x()-x_frontier, 2)+pow(frontier_lines[u][0].y()-y_frontier, 2)+pow(frontier_lines[u][0].z()-z_frontier, 2)) ;
                                if(!repet){
                                  for(vector<point3d>::size_type v = 0; v < frontier_lines[u].size(); v++){
                                    double dist_2 = sqrt(pow(frontier_lines[u][v].x()-x_frontier, 2)+pow(frontier_lines[u][v].y()-y_frontier, 2));
                                    if(dist_2 < 0.6*octo_reso) {
                                      repet = true;
                                      break;
                                    }
                                  }
                                }

                                if(distance < R1){//&&!repet){
                                    frontier_lines[u].push_back(point3d(x_frontier, y_frontier, z_frontier));
                                    belong_old = true;
                                    break;
                                }
                        }
                        if(!belong_old){//&&!repet){
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
