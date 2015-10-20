#include <tp_rrt_planner/planner.hpp>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::nav;
using namespace mrpt::maps;
using namespace std;

Planner::Planner(const std::string filename) {    
    // Parameters/Config:
    planner.loadConfig( mrpt::utils::CConfigFile(filename) );

    planner.params.maxLength = 3.0; 
    planner.params.minDistanceBetweenNewNodes = 0.10;
    planner.params.minAngBetweenNewNodes = mrpt::utils::DEG2RAD(20);
    planner.params.goalBias = 0.05;

    // Logging:
    planner.params.save_3d_log_freq = 500; //0; // save some iterations for debugging, make it 0 for no debugging

    // End criteria:
    planner.end_criteria.acceptedDistToTarget = 0.10;
    planner.end_criteria.acceptedAngToTarget  = DEG2RAD(180); // 180d=Any orientation is ok
    planner.end_criteria.maxComputationTime = 0.08;
    planner.end_criteria.minComputationTime = 0.5; // 0=accept first found acceptable solution

    //Thresholds:
    linear_deviation_threshold = 0.20;
    angular_deviation_threshold = mrpt::utils::DEG2RAD(20);

    // Init planner:
    planner.initialize();

    //Extracting the PTGs:
    PTG_list = planner.getPTGs();

    //Initialising other params:
    debug = false;
    winptr = NULL;
    iters = 0;
}

Planner::~Planner() {
    //Clean the pointers
    if (winptr!=NULL) {
        delete winptr;
    }
}

void Planner::setDebug(bool val) {
    debug = val;
}

Velocity2D Planner::planWith(mrpt::math::TPose2D current_pose,
                        mrpt::math::TPose2D target_pose,
                        mrpt::maps::CSimplePointsMap lidar_map) {

    //If it has reached the next node (lt current threshold) -> replan from that position
    //if not
    //If path already there and closest distance from one on the edge is less than a threshold
    //try to follow that path
    // Start & goal:
    planner_input.start_pose = current_pose;
    planner_input.goal_pose  = target_pose;

    // Obstacles:
    planner_input.obstacles_points = lidar_map;
    mrpt::math::TPoint3D bbox_min,bbox_max;
    planner_input.obstacles_points.boundingBox(bbox_min,bbox_max);
    // Convert gridmap -> obstacle points:
    //gridmap.getAsPointCloud( planner_input.obstacles_points );

    // Workspace bounding box:
    planner_input.world_bbox_min = mrpt::math::TPoint2D(bbox_min.x,bbox_min.y);
    planner_input.world_bbox_max = mrpt::math::TPoint2D(bbox_max.x,bbox_max.y);

    Velocity2D cmd_vel(0,0,0);

    if(debug == true) {
    // Show results in a GUI and keep improving:
    #if MRPT_HAS_WXWIDGETS      //----------------------------------------------------------------
        if (winptr == NULL) {
            winptr = new mrpt::gui::CDisplayWindow3D("Navigation Debug",1024,800);
        }
    }
    #else
    }
        for (size_t i=0;i<1;i++)
    #endif
        {
            mrpt::utils::TNodeID target_node = planner_result.best_goal_node_id;
            mrpt::nav::TMoveTreeSE2_TP::path_t final_path;
            
            //TODO: replace code below this with a switch case
            //If there is a path already
            if (planner_result.success == true) {
                //Check if the target changed
                float target_fluctuation = getTargetDeviation();
                printf("                Target fluctuation : %f\n", target_fluctuation );
                if (target_fluctuation > planner.end_criteria.acceptedDistToTarget * 4) {
                    planner_result = PlannerRRT_SE2_TPS::TPlannerResult();
                    planner_result.success = false;
                }
            }

            if (planner_result.success == true) {
                //Gives the node ids of all the nodes from root to target
                planner_result.move_tree.backtrackPath(target_node, final_path);

                mrpt::nav::TMoveTreeSE2_TP::path_t::iterator edgeit;
                edgeit = final_path.begin();
                edgeit++;
                mrpt::nav::TMoveEdgeSE2_TP *initial_edge_ptr = edgeit->edge_to_parent;

                //Check for reaching the end of trajectory
                mrpt::math::TPose2D end_state = initial_edge_ptr->end_state;
                float node_end_dist = sqrt((current_pose.x-end_state.x)*(current_pose.x-end_state.x)+(current_pose.y-end_state.y)*(current_pose.y-end_state.y));
                printf("                Distance to trajectory end : %f\n", node_end_dist );
                if ( node_end_dist < 0.10) {
                    planner_result = PlannerRRT_SE2_TPS::TPlannerResult();
                    planner_result.success = false;
                }
            }

            if (planner_result.success == true) {
                //Gives the node ids of all the nodes from root to target
                planner_result.move_tree.backtrackPath(target_node, final_path);

                mrpt::nav::TMoveTreeSE2_TP::path_t::iterator edgeit;
                edgeit = final_path.begin();
                edgeit++;
                mrpt::nav::TMoveEdgeSE2_TP *initial_edge_ptr = edgeit->edge_to_parent;

                // pass the edge and current pose
                // return the velocity
                bool hasDeviated = false;

                //Velocity production
                Velocity2D calc_vel = generateVelfromTrajectory(initial_edge_ptr, current_pose, hasDeviated);
                if (!hasDeviated) {
                    cmd_vel = calc_vel;
                } else {
                    planner_result = PlannerRRT_SE2_TPS::TPlannerResult();
                    planner_result.success = false;
                    cmd_vel = Velocity2D(0,0,0);
                }
            }

            //Incase there is no path, keep planning
            if (planner_result.success == false) {
                planner.solve(planner_input, planner_result);
                if (planner_result.success == false) { 
                    iters++;
                    if (iters % MAX_ITERS == 0) {
                        iters = 0;
                        //Every max iters assure that in case there is no path, 0 velocity is returned
                        planner_result = PlannerRRT_SE2_TPS::TPlannerResult();
                        planner_result.success = false;
                    }
                }
            }

            //cout << "Found goal_distance: " << planner_result.goal_distance << endl;
            //cout << "Found path_cost: " << planner_result.path_cost << endl;
            //cout << "Acceptable goal nodes: " << planner_result.acceptable_goal_node_ids.size() << endl;
    
    #if MRPT_HAS_WXWIDGETS
            // Show result in a GUI:
            mrpt::opengl::COpenGLScenePtr & scene = winptr->get3DSceneAndLock();
    
            scene->clear();
        
            PlannerRRT_SE2_TPS::TRenderPlannedPathOptions render_opts;
            render_opts.highlight_path_to_node_id = planner_result.best_goal_node_id;
    
            planner.renderMoveTree(*scene, planner_input, planner_result,render_opts );
    
            winptr->unlockAccess3DScene();
            winptr->repaint();
            //winptr->waitForKey();
    #endif
            //Return the velocity
            return cmd_vel;
        }
}

Velocity2D Planner::generateVelfromTrajectory(mrpt::nav::TMoveEdgeSE2_TP * edge_ptr,
                                        mrpt::math::TPose2D current_pose, bool& hasDeviated) {
    hasDeviated = false;

    mrpt::nav::CParameterizedTrajectoryGeneratorPtr ptg_gen_ptr;
    ptg_gen_ptr = PTG_list[edge_ptr->ptg_index];

    int trajectory_index = edge_ptr->ptg_K;
    int total_trajectory_pts = ptg_gen_ptr->getPointsCountInCPath_k(trajectory_index);

    //Finding the nearest configuration on the trajectory by applying metric on the C-Space
    int flag_point_index = -1;  //The index of the point on the trajectory
    float min_metric = 100000;
    mrpt::math::TPose2D temp_pose;
    for (int i = 0; i < total_trajectory_pts; i++) {
        mrpt::math::TPose2D temp_on_trajectory( ptg_gen_ptr->GetCPathPoint_x(trajectory_index, i),
                                                ptg_gen_ptr->GetCPathPoint_y(trajectory_index, i),
                                                ptg_gen_ptr->GetCPathPoint_phi(trajectory_index, i));

        //Evaluating the metric at each point
        float mod = getMetric(temp_on_trajectory, current_pose);

        //Storing the value of the index
        if (mod < min_metric) {
            min_metric = mod;
            flag_point_index = i;
            temp_pose = temp_on_trajectory;
        }
    }

    if (flag_point_index != -1) {
        //Checking if bot has deviated. I have used threshold values. Metric can also be used
        float distance = sqrt((current_pose.x-temp_pose.x)*(current_pose.x-temp_pose.x) + (current_pose.y-temp_pose.y)*(current_pose.y-temp_pose.y));

        if (    distance > linear_deviation_threshold ||
                (current_pose.phi - temp_pose.phi) > angular_deviation_threshold ) {
            hasDeviated = true;
            return Velocity2D(0,0,0);
        } else {
            hasDeviated = false;
        }
    } else {
        //Handle exception
        hasDeviated = true;
    }

    //Workspace I have assumed as my own 2d space
    // int trajectory_index_result;
    // float trajectory_d_result;
    // bool found = ptg_gen_ptr->inverseMap_WS2TP( current_pose.x,
    //                                             current_pose.y,
    //                                             trajectory_index_result,
    //                                             trajectory_d_result
    //                                             );
    // if ( trajectory_index_result == trajectory_index && found) {
    //     hasDeviated = false;
    // } else {
    //     hasDeviated = true;
    //     return Velocity2D(0,0,0);
    // }

    float lin_vel, ang_vel;
    lin_vel = ptg_gen_ptr->GetCPathPoint_v(trajectory_index, flag_point_index);
    ang_vel = ptg_gen_ptr->GetCPathPoint_w(trajectory_index, flag_point_index);

    //KalmanFilter can be applied before this
    return Velocity2D(lin_vel, 0, ang_vel);
}
