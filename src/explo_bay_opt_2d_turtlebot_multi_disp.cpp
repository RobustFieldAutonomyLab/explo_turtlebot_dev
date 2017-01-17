//Related headers:
#include "exploration.h"
#include "navigation_utils.h"
#include "gpregressor.h"
#include "covMaterniso3.h"
#include "frontier.h"
#include "candidates_multi.h"
#include "kmean.h"
using namespace std;
// using namespace std::chrono;

int main(int argc, char **argv) {

    ros::init(argc, argv, "explo_sam_2d_turtlebot");
    ros::NodeHandle nh;

    ros::Time time_init = ros::Time::now();

    // Initialize time
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    if(BayOpt)  strftime(buffer,80,"Trajectory_gp_BO_%R_%S_%m%d_DA.txt",timeinfo);
    else  strftime(buffer,80,"Trajectory_DA_%R_%S_%m%d_DA.txt",timeinfo);
    std::string logfilename(buffer);
    std::cout << logfilename << endl;
    strftime(buffer,80,"octomap_gp_2d_%R_%S_%m%d_DA.ot",timeinfo);
    octomap_name_2d = buffer;
    strftime(buffer,80,"octomap_gp_3d_%R_%S_%m%d_DA.ot",timeinfo);
    octomap_name_3d = buffer;


    ros::Subscriber kinect_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, kinect_callbacks);
    //ros::Subscriber hokuyo_sub = nh.subscribe<sensor_msgs::PointCloud2>("/kinect_points", 1, kinect_2d_callbacks);
    ros::Publisher GoalMarker_pub = nh.advertise<visualization_msgs::Marker>( "Goal_Marker", 1 );
    ros::Publisher JackalMarker_pub = nh.advertise<visualization_msgs::Marker>( "Jackal_Marker", 1 );
    ros::Publisher Candidates_pub = nh.advertise<visualization_msgs::MarkerArray>("Candidate_MIs", 1);
    ros::Publisher Frontier_points_pub = nh.advertise<visualization_msgs::Marker>("Frontier_points", 1);
    ros::Publisher Frontier_points_3d_pub = nh.advertise<visualization_msgs::Marker>("Frontier_points_3d", 1);
    ros::Publisher pub_twist = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    ros::Publisher Octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap_3d",1);

    ros::Publisher MI_marker_pub = nh.advertise<visualization_msgs::Marker>("MI_MarkerArray", 1);


    tf_listener = new tf::TransformListener();
    tf::StampedTransform transform;
    tf::Quaternion Goal_heading; // robot's heading direction

    visualization_msgs::MarkerArray CandidatesMarker_array;
    visualization_msgs::Marker OctomapOccupied_cubelist;
    visualization_msgs::Marker OctomapOccupied_cubelist_3d;
    visualization_msgs::Marker Frontier_points_cubelist;
    visualization_msgs::Marker Frontier_points_cubelist_3d;
    visualization_msgs::MarkerArray MI_cubelist;
    visualization_msgs::Marker Free_cubelist_3d;
    geometry_msgs::Twist twist_cmd;



    
    ros::Time now_marker = ros::Time::now();
   
    double R_velo, P_velo, Y_velo;

    // Initialize parameters 
    int max_idx = 0;

    position = point3d(0, 0, 0.0);
    point3d Sensor_PrincipalAxis(1, 0, 0);
    octomap::OcTreeNode *n;
    octomap::OcTree new_tree(octo_reso);
    octomap::OcTree new_tree_2d(octo_reso);
    cur_tree = &new_tree;
    //cur_tree_2d = &new_tree_2d;
    point3d next_vp;

    bool got_tf = false;
    bool arrived;
    
    // Update the initial location of the robot
    for(int o =0; o < 6; o++){
        // Update the pose of the robot
        got_tf = false
;
        while(!got_tf){
        try{
            tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
            kinect_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            got_tf = true;
        }
        catch (tf::TransformException ex) {
            ROS_WARN("Wait for tf: velodyne to map"); 
        } 
        ros::Duration(0.05).sleep();
        }

        got_tf = false;
        while(!got_tf){
        try{
            tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
            laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            got_tf = true;
        }
        catch (tf::TransformException ex) {
            ROS_WARN("Wait for tf: laser to map"); 
        } 
        ros::Duration(0.05).sleep();
        }

        // Take a Scan
        ros::spinOnce();

        // Rotate another 60 degrees
        twist_cmd.linear.x = twist_cmd.linear.y = twist_cmd.angular.z = 0;
        ros::Time start_turn = ros::Time::now();

        ROS_WARN("Rotate 60 degrees");
        while (ros::Time::now() - start_turn < ros::Duration(2.6)){ // turning duration - second
        twist_cmd.angular.z = 0.6; // turning speed
        // turning angle = turning speed * turning duration / 3.14 * 180
        pub_twist.publish(twist_cmd);
        ros::Duration(0.05).sleep();
        }
        // stop
        twist_cmd.angular.z = 0;
        pub_twist.publish(twist_cmd);

    }

   

    // Update the pose of the robot
    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
        kinect_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: velodyne to map"); 
    } 
    ros::Duration(0.05).sleep();
    }

    got_tf = false;
    while(!got_tf){
    try{
        tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
        laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        got_tf = true;
    }
    catch (tf::TransformException ex) {
        ROS_WARN("Wait for tf: laser to map"); 
    } 
    ros::Duration(0.05).sleep();
    }
   
    // Take a Fourth Scan
    ros::spinOnce();
    

    double train_time, test_time;

    // steps robot taken, counter
    int robot_step_counter = 1;

    ros::Time time_now = ros::Time::now();
    double time_past = time_now.toSec() - time_init.toSec(); 
                // Send out results to file.
    explo_log_file.open(logfilename, std::ofstream::out | std::ofstream::app);
    explo_log_file << robot_step_counter <<"," << get_free_volume(cur_tree) << "," << time_past <<","<< 0 << endl;
    explo_log_file.close();

    while (ros::ok())
    {
            start_over:
            vector<vector<point3d>> frontier_lines;
            vector<pair<point3d, point3d>> candidates;
            vector<pair<point3d, point3d>> candidates_init;
            double entropy = get_free_volume(cur_tree);
            ROS_INFO("entropy_frontie %f",entropy);
            int level = 0;
            if(entropy < 1000){
                level = 1;
                frontier_lines = generate_frontier_points_3d( cur_tree, kinect_orig.z(), -3*octo_reso, 3*octo_reso);

                if(!BayOpt) candidates_init = generate_candidates(frontier_lines, kinect_orig, 0.1, 0.4, 2, 10);
                else candidates_init = generate_candidates(frontier_lines, kinect_orig, 0.1, 0.4, 2, 10);
                
                //int n = frontier_lines.size();
                //ROS_INFO("frontier_num %d", n); 
            }

            else if(entropy < 2000){
                level = 2;
                frontier_lines = generate_frontier_points_3d( cur_tree, 1.5,octo_reso,octo_reso );
                candidates_init = generate_candidates(frontier_lines, kinect_orig, 3.9, 0.1, 3.9, 5); 
            }

            else{
                ROS_INFO("Exploration is done");
                while(1);
            }
            
           
            unsigned long int o = 0;
            for(vector<vector<point3d>>::size_type e = 0; e < frontier_lines.size(); e++) {
                o = o+frontier_lines[e].size();
            }

            Frontier_points_cubelist.points.resize(o);
            ROS_INFO("frontier points %ld", o);
            now_marker = ros::Time::now();
            Frontier_points_cubelist.header.frame_id = "map";
            Frontier_points_cubelist.header.stamp = now_marker;
            Frontier_points_cubelist.ns = "frontier_points_array";
            Frontier_points_cubelist.id = 0;
            Frontier_points_cubelist.type = visualization_msgs::Marker::CUBE_LIST;
            Frontier_points_cubelist.action = visualization_msgs::Marker::ADD;
            Frontier_points_cubelist.scale.x = octo_reso;
            Frontier_points_cubelist.scale.y = octo_reso;
            Frontier_points_cubelist.scale.z = octo_reso;
            Frontier_points_cubelist.color.a = 1.0;
            Frontier_points_cubelist.color.r = (double)255/255;
            Frontier_points_cubelist.color.g = 0;
            Frontier_points_cubelist.color.b = (double)0/255;
            Frontier_points_cubelist.lifetime = ros::Duration();

            unsigned long int t = 0;
            //int l = 0;
            geometry_msgs::Point q;
            for(vector<vector<point3d>>::size_type n = 0; n < frontier_lines.size(); n++) { 
                for(vector<point3d>::size_type m = 0; m < frontier_lines[n].size(); m++){
                   q.x = frontier_lines[n][m].x();
                   q.y = frontier_lines[n][m].y();
                   q.z = frontier_lines[n][m].z()+octo_reso;
                   Frontier_points_cubelist.points.push_back(q); 
                   
                }
                t++;
            }
            ROS_INFO("Publishing %ld frontier_lines", t);
            
            Frontier_points_pub.publish(Frontier_points_cubelist); //publish frontier_points
            Frontier_points_cubelist.points.clear();



            vector<vector<point3d>> frontier_lines_3d=generate_frontier_points_3d( cur_tree, 1.5,octo_reso,octo_reso );

            o = 0;
            for(vector<vector<point3d>>::size_type e = 0; e < frontier_lines_3d.size(); e++) {
                o = o+frontier_lines_3d[e].size();
            }

            Frontier_points_cubelist_3d.points.resize(o);
            ROS_INFO("frontier points 3d %ld", o);
            now_marker = ros::Time::now();
            Frontier_points_cubelist_3d.header.frame_id = "map";
            Frontier_points_cubelist_3d.header.stamp = now_marker;
            Frontier_points_cubelist_3d.ns = "frontier_points_array_3d";
            Frontier_points_cubelist_3d.id = 0;
            Frontier_points_cubelist_3d.type = visualization_msgs::Marker::CUBE_LIST;
            Frontier_points_cubelist_3d.action = visualization_msgs::Marker::ADD;
            Frontier_points_cubelist_3d.scale.x = octo_reso;
            Frontier_points_cubelist_3d.scale.y = octo_reso;
            Frontier_points_cubelist_3d.scale.z = octo_reso;
            Frontier_points_cubelist_3d.color.a = 0.3;
            Frontier_points_cubelist_3d.color.r = (double)255/255;
            Frontier_points_cubelist_3d.color.g = 0;
            Frontier_points_cubelist_3d.color.b = (double)255/255;
            Frontier_points_cubelist_3d.lifetime = ros::Duration();

            t = 0;
            //l = 0;
            geometry_msgs::Point q2;
            for(vector<vector<point3d>>::size_type n = 0; n < frontier_lines_3d.size(); n++) { 
                for(vector<point3d>::size_type m = 0; m < frontier_lines_3d[n].size(); m++){


                   q2.x = frontier_lines_3d[n][m].x();
                   q2.y = frontier_lines_3d[n][m].y();
                   q2.z = frontier_lines_3d[n][m].z()+octo_reso;
                   Frontier_points_cubelist_3d.points.push_back(q2); 
                   
                }
                t++;
            }
            ROS_INFO("Publishing %ld frontier_lines_3d", t);
            
            Frontier_points_3d_pub.publish(Frontier_points_cubelist_3d); //publish frontier_points
            Frontier_points_cubelist_3d.points.clear();





    
        // Generate Candidates
        //vector<pair<point3d, point3d>> candidates = generate_candidates(frontier_lines, kinect_orig, 0.3, 1); 
        // Generate Testing poses
        ROS_INFO("%lu candidates generated.", candidates_init.size());

        // Generate Testing poses
        //vector<pair<point3d, point3d>> gp_test_poses = generate_testing(frontier_lines, kinect_orig);


        //frontier_lines.clear();

        while(candidates_init.size() < 1)
        {
            // Get the current heading
            got_tf = false;
            while(!got_tf){
            try{
                tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);
                kinect_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                got_tf = true;
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Wait for tf: kinect to map"); 
            } 
            ros::Duration(0.05).sleep();
            }
            // Rotate another 60 degrees
            twist_cmd.linear.x = twist_cmd.linear.y = twist_cmd.angular.z = 0;
            ros::Time start_turn = ros::Time::now();

            ROS_WARN("Rotate 60 degrees");
            while (ros::Time::now() - start_turn < ros::Duration(2.6)){ // turning duration - second
            twist_cmd.angular.z = 0.4; // turning speed
            // turning angle = turning speed * turning duration / 3.14 * 180
            pub_twist.publish(twist_cmd);
            ros::Duration(0.05).sleep();
            }
            // stop
            twist_cmd.angular.z = 0;
            pub_twist.publish(twist_cmd);
            if(!BayOpt) candidates_init = generate_candidates(frontier_lines, kinect_orig, 0.5, 0.25, 2, 5);
            if(BayOpt) candidates_init = generate_candidates(frontier_lines, kinect_orig, 0.5, 0.25, 2, 5);
        }
        
        vector<double> MIs_init(candidates_init.size());
        vector<double> MIs;
        //vector<double> MIs_temp(candidates.size());
        double before = get_free_volume(cur_tree);
        max_idx = 0;

        unsigned int p = 0;

        // for every candidate...
        double Secs_CastRay, Secs_InsertRay, Secs_tmp;  //What are those? ####
        Secs_InsertRay = 0;
        Secs_CastRay = 0;

        #pragma omp parallel for
        for(int i = 0; i < candidates_init.size(); i++) 
        {   
            //max_order[i] = i;
            auto c = candidates_init[i];
            // Evaluate Mutual Information
            Secs_tmp = ros::Time::now().toSec();
            Sensor_PrincipalAxis.rotate_IP(c.second.roll(), c.second.pitch(), c.second.yaw() );
            octomap::Pointcloud hits = cast_sensor_rays(cur_tree, c.first, Sensor_PrincipalAxis);  // what are those?#####
            Secs_CastRay += ros::Time::now().toSec() - Secs_tmp;
            Secs_tmp = ros::Time::now().toSec();
            //MIs_temp[i] = calc_MI(cur_tree, c.first, hits, before);
            if(level == 1) MIs_init[i] = calc_MI(cur_tree, c.first, hits, before);//pow(pow(c.first.x()-kinect_orig.x(), 2)+pow(c.first.y() - kinect_orig.y(), 2), 0.5);
            else if(level == 2) MIs_init[i] = calc_MI(cur_tree, c.first, hits, before);//pow(pow(c.first.x()-kinect_orig.x(), 2)+pow(c.first.y() - kinect_orig.y(), 2), 0.5);
            
            Secs_InsertRay += ros::Time::now().toSec() - Secs_tmp;
        }



        // Bayesian Opt
        
            
        // Generate Testing poses
        vector<pair<point3d, point3d>> gp_test_poses;
        if(level == 1) gp_test_poses = generate_candidates(frontier_lines, kinect_orig, 0.1, 0.2, 2, 40);
        else if(level == 2) gp_test_poses = generate_candidates(frontier_lines, kinect_orig, 3.9, 0.2, 3.9, 20);

        //Initialize gp regression
        GPRegressor g(100, 2, 0.01);// what's this?
        MatrixXf gp_train_x(candidates_init.size(), 3), gp_train_label(candidates_init.size(), 1), gp_test_x(gp_test_poses.size(), 3);

        for (int i=0; i< candidates_init.size(); i++){
            gp_train_x(i,0) = candidates_init[i].first.x();
            gp_train_x(i,1) = candidates_init[i].first.y();
            gp_train_x(i,2) = candidates_init[i].second.z();
            gp_train_label(i) = MIs_init[i];
        }

        for (int i=0; i< gp_test_poses.size(); i++){
            gp_test_x(i,0) = gp_test_poses[i].first.x();
            gp_test_x(i,1) = gp_test_poses[i].first.y();
            gp_test_x(i,2) = gp_test_poses[i].second.z();
        }

        // Perform GP regression
        MatrixXf m, s2;
        train_time = ros::Time::now().toSec();
        g.train(gp_train_x, gp_train_label);
        train_time = ros::Time::now().toSec() - train_time;

        test_time = ros::Time::now().toSec();
        g.test(gp_test_x, m, s2);
        test_time = ros::Time::now().toSec() - test_time;
        ROS_INFO("GP: Train(%zd) took %f | Test(%zd) took %f", candidates_init.size(), train_time, gp_test_poses.size(), test_time);
        candidates.resize(gp_test_poses.size());
        MIs.resize(gp_test_poses.size());        
        for(int i = 0; i < gp_test_poses.size(); i++){
            MIs[i] = m(i);
            candidates[i] = gp_test_poses[i];
        }

        frontier_lines.clear();
        unsigned long int size_c = candidates.size();
        unsigned long int size_M = MIs.size();
        ROS_INFO("candidates size %ld MIS size %ld ", size_c, size_M);

            /*for(int i = 0; i < candidates.size(); i++) {
                MIs[i]=MIs[i]/pow(pow(candidates[i].first.x()-kinect_orig.x(), 2)+pow(candidates[i].first.y() - kinect_orig.y(), 2), 1.5);
            }*/
        

        
        //Clustering candidates by kmean

        int K = 5;//how many robots
        int count = 1;
        int max_iter = 10;
        int epsilon = 2;
        int attempt = 10;
        int flag = 1.0;

        Mat labels = kmean_explo(candidates, K, count, max_iter, epsilon, attempt, flag);
        
        // ###########################
        long int max_order[candidates.size()];
        //long int equ_order[candidates.size()];
        /*for(int j=0; j<candidates.size(); j++){
           equ_order[j] = 0;
           }*/

        for(int j=0; j<candidates.size(); j++)
        {
            p=-1;
           for(int m=0; m<=j; m++){
            
              if (MIs[j] > MIs[m])
                 {
                    p++;
                 }
            }
            for(int m=j; m<candidates.size(); m++){
              if (MIs[j] >= MIs[m])
                 {
                    p++;
                 }

            }
           max_order[p] = j;
        }
        
        p = candidates.size()-1;
        max_idx = max_order[p];
        loop:
        //max_idx = max_order[p];
 
        next_vp = point3d(candidates[max_order[p]].first.x(),candidates[max_order[p]].first.y(),candidates[max_order[p]].first.z());
        Goal_heading.setRPY(0.0, 0.0, candidates[max_order[p]].second.yaw());
        Goal_heading.normalize();
        ROS_INFO("Estimated Max MI : %f , @ %3.2f,  %3.2f,  %3.2f", MIs[max_order[p]], next_vp.x(), next_vp.y(), next_vp.z() );
        ROS_INFO("CastRay Time: %2.3f Secs. InsertRay Time: %2.3f Secs.", Secs_CastRay, Secs_InsertRay);

        // Publish the candidates as marker array in rviz
        tf::Quaternion MI_heading;
        MI_heading.setRPY(0.0, -PI/2, 0.0);
        MI_heading.normalize();
        
        CandidatesMarker_array.markers.resize(candidates.size());
        for (int i = 0; i < candidates.size(); i++)
        {
            CandidatesMarker_array.markers[i].header.frame_id = "map";
            CandidatesMarker_array.markers[i].header.stamp = ros::Time::now();
            CandidatesMarker_array.markers[i].ns = "candidates";
            CandidatesMarker_array.markers[i].id = i;
            CandidatesMarker_array.markers[i].type = visualization_msgs::Marker::ARROW;
            CandidatesMarker_array.markers[i].action = visualization_msgs::Marker::ADD;
            CandidatesMarker_array.markers[i].pose.position.x = candidates[i].first.x();
            CandidatesMarker_array.markers[i].pose.position.y = candidates[i].first.y();
            CandidatesMarker_array.markers[i].pose.position.z = candidates[i].first.z();
            CandidatesMarker_array.markers[i].pose.orientation.x = MI_heading.x();
            CandidatesMarker_array.markers[i].pose.orientation.y = MI_heading.y();
            CandidatesMarker_array.markers[i].pose.orientation.z = MI_heading.z();
            CandidatesMarker_array.markers[i].pose.orientation.w = MI_heading.w();
            CandidatesMarker_array.markers[i].scale.x = (double)MIs[i]/MIs[max_idx];
            CandidatesMarker_array.markers[i].scale.y = 0.05;
            CandidatesMarker_array.markers[i].scale.z = 0.05;
            CandidatesMarker_array.markers[i].color.a = 1;
            CandidatesMarker_array.markers[i].color.r = (double)MIs[i]/MIs[max_idx];
            CandidatesMarker_array.markers[i].color.g = 1-(double)MIs[i]/MIs[max_idx];
            CandidatesMarker_array.markers[i].color.b = 0;
        }
        Candidates_pub.publish(CandidatesMarker_array); //publish candidates##########
        CandidatesMarker_array.markers.clear();
        

        //MI distribution
        //discrete candidates
        /*octomap::Pointcloud hits_candid;
        vector<point3d> candidates_discrete;
        ROS_INFO("1");
        for(int i = 0; i < candidates.size(); i++){
            hits_candid.push_back(point3d(candidates[i].first.x(), candidates[i].first.y(), candidates[i].first.z()));
        }
        ROS_INFO("2");
        cur_tree_2d->insertPointCloud(hits_candid, kinect_orig, Kinect_360.max_range);
        ROS_INFO("3");
        for(octomap::OcTree::leaf_iterator n = cur_tree_2d->begin_leafs(cur_tree_2d->getTreeDepth()); n != cur_tree_2d->end_leafs(); ++n){
            candidates_discrete.push_back(point3d(n.getX(), n.getY(), n.getZ()));
        }
        ROS_INFO("4");
        cur_tree_2d->clear();

        //Initialize gp regression
        GPRegressor g_disc(100, 2, 0.01);// what's this?
        MatrixXf gp_train_x_disc(candidates_init.size(), 3), gp_train_label_disc(candidates_init.size(), 1), gp_test_x_disc(candidates_discrete.size(), 3);
        ROS_INFO("5");
        for (int i=0; i< candidates_init.size(); i++){
            gp_train_x_disc(i,0) = candidates_init[i].first.x();
            gp_train_x_disc(i,1) = candidates_init[i].first.y();
            gp_train_x_disc(i,2) = candidates_init[i].second.z();
            gp_train_label_disc(i) = MIs_init[i];
        }
        ROS_INFO("6");
        for (int i=0; i< candidates_discrete.size(); i++){
            gp_test_x_disc(i,0) = candidates_discrete[i].x();
            gp_test_x_disc(i,1) = candidates_discrete[i].y();
            gp_test_x_disc(i,2) = candidates_discrete[i].z();
        }
        ROS_INFO("7");
        // Perform GP regression
        MatrixXf m_disc, s2_disc;
        //train_time = ros::Time::now().toSec();
        g_disc.train(gp_train_x_disc, gp_train_label_disc);
        //train_time = ros::Time::now().toSec() - train_time;

        //test_time = ros::Time::now().toSec();
        g_disc.test(gp_test_x_disc, m_disc, s2_disc);


        ROS_INFO("8");

        MI_cubelist.markers.resize(candidates_discrete.size());
        for(int i = 0; i < candidates_discrete.size(); i++){
            MI_cubelist.markers[i].header.frame_id = "map";
            MI_cubelist.markers[i].header.stamp = ros::Time::now();
            MI_cubelist.markers[i].ns = "MI_array";
            MI_cubelist.markers[i].id = i;
            MI_cubelist.markers[i].type = visualization_msgs::Marker::ARROW;
            MI_cubelist.markers[i].action = visualization_msgs::Marker::ADD;
            MI_cubelist.markers[i].pose.position.x = candidates_discrete[i].x();
            MI_cubelist.markers[i].pose.position.y = candidates_discrete[i].y();
            MI_cubelist.markers[i].pose.position.z = candidates_discrete[i].z();
            MI_cubelist.markers[i].pose.orientation.x = MI_heading.x();
            MI_cubelist.markers[i].pose.orientation.y = MI_heading.y();
            MI_cubelist.markers[i].pose.orientation.z = MI_heading.z();
            MI_cubelist.markers[i].pose.orientation.w = MI_heading.w();
            MI_cubelist.markers[i].scale.x = (double)m_disc(i)/MIs[max_idx];
            MI_cubelist.markers[i].scale.y = 0.05;
            MI_cubelist.markers[i].scale.z = 0.05;
            MI_cubelist.markers[i].color.a = 1;
            MI_cubelist.markers[i].color.r = (double)m_disc(i)/MIs[max_idx];
            MI_cubelist.markers[i].color.g = 0;
            MI_cubelist.markers[i].color.b = 1-(double)m_disc(i)/MIs[max_idx];
        }
        MI_marker_pub.publish(MI_cubelist); //publish octomap############
        MI_cubelist.markers.clear();*/
        candidates.clear();
        candidates_init.clear();
        //candidates_discrete.clear();

        // Publish the goal as a Marker in rviz
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "goal_marker";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = next_vp.x();
        marker.pose.position.y = next_vp.y();
        marker.pose.position.z = 1.0;
        marker.pose.orientation.x = Goal_heading.x();
        marker.pose.orientation.y = Goal_heading.y();
        marker.pose.orientation.z = Goal_heading.z();
        marker.pose.orientation.w = Goal_heading.w();
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        GoalMarker_pub.publish( marker ); //publish goal##########

        // Send the Robot 
        Goal_heading.setRPY(0.0, 0.0, candidates[max_order[p]].second.yaw());
        arrived = goToDest(next_vp, Goal_heading);

        if(arrived)
        {
            // Update the initial location of the robot
            got_tf = false;
            while(!got_tf){
            try{
                tf_listener->lookupTransform("/map", "/camera_rgb_frame", ros::Time(0), transform);// need to change tf of kinect###############
                kinect_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                got_tf = true;
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Wait for tf: velodyne to map"); 
            } 
            ros::Duration(0.05).sleep();
            }

            got_tf = false;
            while(!got_tf){
            try{
                tf_listener->lookupTransform("/map", "/laser", ros::Time(0), transform);
                laser_orig = point3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
                got_tf = true;
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Wait for tf: laser to map"); 
            } 
            ros::Duration(0.05).sleep();
            }

            // Update Octomap
            ros::spinOnce();
            ROS_INFO("Succeed, new Map Free Volume: %f", get_free_volume(cur_tree));
            robot_step_counter++;

            // prepare octomap msg
            octomap_msgs::binaryMapToMsg(*cur_tree, msg_octomap);
            msg_octomap.binary = 1;
            msg_octomap.id = 1;
            msg_octomap.resolution = octo_reso;
            msg_octomap.header.frame_id = "/map";
            msg_octomap.header.stamp = ros::Time::now();
            Octomap_pub.publish(msg_octomap);

            
            ros::Time time_now = ros::Time::now();
            double time_past = time_now.toSec() - time_init.toSec(); 
            // Send out results to file.
            explo_log_file.open(logfilename, std::ofstream::out | std::ofstream::app);
            explo_log_file << robot_step_counter <<"," << get_free_volume(cur_tree) << "," << time_past <<","<< candidates.size() << endl;
            explo_log_file.close();

        }
        else
        { 
            ROS_ERROR("Failed to navigate to goal");
            p--;
            if(p < 0){
               continue;
            }

            goto loop;
        }
    }
    nh.shutdown();          
    return 0;
}
