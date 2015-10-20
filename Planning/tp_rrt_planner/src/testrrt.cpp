/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/nav.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/system/filesystem.h> // directoryExists(), ...
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/random.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <tp_rrt_planner/velocity2d.hpp>
#include <tp_rrt_planner/debug_parameters.hpp>
#include <tp_rrt_planner/planner.hpp>

#include <iostream>
#include <cmath>

#include "ros/ros.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::nav;
using namespace mrpt::maps;
using namespace std;

KalmanFilterL kf;

Velocity2D generateVelocityTest(mrpt::nav::TListPTGPtr& PTG, mrpt::nav::TMoveTreeSE2_TP::NODE_TYPE& node, DEBUG_PARAMETERS& dp) {
    
    float lvelocity = 0.0f;
    float avelocity = 0.0f;

    // get command velocity from seeds
    // float min_v = 1000.0;
    // float min_w = 1000;
    // for(size_t i = 0; i < 3; ++i) {
    //     auto ptg = PTG[i];
    //     ptg->directionToMotionCommand(node.edge_to_parent->ptg_K, lvelocity, avelocity);
    //     if(lvelocity < min_v && lvelocity >= 0) {
    //         min_v = lvelocity;
    //         min_w = avelocity;

    //     }
    //     // if(std::fabs(avelocity) < std::fabs(min_w)) {
    //     //     min_w = avelocity;
    //     // }
    // }

    float random = 0.0f;
    PTG[1]->directionToMotionCommand(node.edge_to_parent->ptg_K, lvelocity, random);
    PTG[1]->directionToMotionCommand(node.edge_to_parent->ptg_K, random, avelocity);

    std::clog<<"PTG V : "<<lvelocity<<" W : "<<avelocity<<"\n";

    double linearvelocity=lvelocity;
    double angularvelocity= avelocity;
    const double radiusofcurvature = PTG[3]->getMax_V() / PTG[3]->getMax_W();

    double reduction = std::min(1.0, PTG[1]->getMax_V() / sqrt(mrpt::utils::square(linearvelocity) +
        mrpt::utils::square(radiusofcurvature * angularvelocity)));
    if (reduction < 0.5) {
        reduction = 0.5;
        linearvelocity *= reduction;
        angularvelocity *= reduction;
    }

    // Assure maximum velocitys:
    if (std::fabs(linearvelocity) > PTG[1]->getMax_V()*0.8) {
        const double norm = std::fabs(PTG[1]->getMax_V()*0.8 / linearvelocity);
        linearvelocity *= norm;
        angularvelocity *= norm;
    }

    if (std::fabs(angularvelocity) > PTG[1]->getMax_W()) {
        const double norm = std::fabs((double) (PTG[1]->getMax_W()) / angularvelocity);
        angularvelocity *= norm;
        linearvelocity *= norm;

    }


    const double meanexecutionperiod = 100.0f; // Should take the feedback from controller
    //First order low-pass filter
    const double KSPEEDFILTER_TAU = 0.2;

    // const double alfa = meanexecutionperiod / (meanexecutionperiod + KSPEEDFILTER_TAU);
    // linearvelocity = alfa * linearvelocity + (1 - alfa) * last_cmd_v;
    // angularvelocity = alfa * angularvelocity + (1 - alfa) * last_cmd_w;

    angularvelocity = mrpt::math::wrapToPi(angularvelocity);

    // last_cmd_v = linearvelocity;
    // last_cmd_w = angularvelocity;
    if(true) {
        dp.original_vel = Velocity2D(linearvelocity, 0.0, angularvelocity);
    }
    std::pair<float, float> velocity = kf.update(float(linearvelocity), float(angularvelocity));
    return Velocity2D(double(velocity.first), 0.0, double(velocity.second));
}
// Load example grid maps
//string   mySimpleMap( MRPT_EXAMPLES_BASE_DIRECTORY + string("../share/mrpt/datasets/malaga-cs-fac-building.simplemap.gz") );
//string   myCfgFileName( MRPT_EXAMPLES_BASE_DIRECTORY + string("../share/mrpt/config_files/navigation-ptgs/ptrrt_config_example1.ini") );

// ------------------------------------------------------
//              TestRRT1
// ------------------------------------------------------
void TestRRT1(std::string cfgfilename, std::string mapfile)
{
    //mrpt::random::Randomize();

    // Load the gridmap:
    CSimpleMap simplemap;
    
    ASSERT_FILE_EXISTS_(mapfile);

    cout << "Loading map...";
    CFileGZInputStream(mapfile) >> simplemap;
    cout << "Done! Number of sensory frames: " << simplemap.size() << endl;

    // Set planner params:
    // ------------------------------
    mrpt::nav::PlannerRRT_SE2_TPS  planner;

    // Parameters:
    planner.loadConfig( mrpt::utils::CConfigFile(cfgfilename) );

    planner.params.goalBias = 0.05;
    planner.params.maxLength = 4.0; 
    planner.params.minDistanceBetweenNewNodes = 0.10;
    planner.params.minAngBetweenNewNodes = mrpt::utils::DEG2RAD(20);
    planner.params.ptg_verbose = true;

    // Logging:
    planner.params.save_3d_log_freq = 0; //500; // save some iterations for debugging

    // End criteria:
    planner.end_criteria.acceptedDistToTarget = 0.25;
    planner.end_criteria.acceptedAngToTarget  = DEG2RAD(180); // 180d=Any orientation is ok
    planner.end_criteria.maxComputationTime = 0.08;
    planner.end_criteria.minComputationTime = 0; // 0=accept first found acceptable solution

    // Init planner:
    // ------------------------------
    planner.initialize();

    static mrpt::nav::TListPTGPtr PTG = planner.getPTGs();

    // Set up planning problem:
    // ------------------------------
    PlannerRRT_SE2_TPS::TPlannerResult planner_result;
    PlannerRRT_SE2_TPS::TPlannerInput planner_input;

    // Start & goal:
    planner_input.start_pose = mrpt::math::TPose2D(0,0,0.3);
    planner_input.goal_pose  = mrpt::math::TPose2D(20, 5, 0);

    // Obstacles:
    planner_input.obstacles_points.loadFromSimpleMap( simplemap );
    mrpt::math::TPoint3D bbox_min,bbox_max;
    planner_input.obstacles_points.boundingBox(bbox_min,bbox_max);
    // Convert gridmap -> obstacle points:
    //gridmap.getAsPointCloud( planner_input.obstacles_points );

    // Workspace bounding box:
    planner_input.world_bbox_min = mrpt::math::TPoint2D(bbox_min.x,bbox_min.y);
    planner_input.world_bbox_max = mrpt::math::TPoint2D(bbox_max.x,bbox_max.y);

    size_t iters=0;
    // Show results in a GUI and keep improving:
    float angle = 0.0, radius = 3.0;
#if MRPT_HAS_WXWIDGETS
    mrpt::gui::CDisplayWindow3D  win("Result",1024,800);
    while (win.isOpen())
#else
    for (size_t i=0;i<1;i++)
#endif
    {
        // Refine solution or start over:
        bool refine_solution = (iters++ % 3 != 0);

        // Start from scratch: 
        if (!refine_solution)
            planner_result = PlannerRRT_SE2_TPS::TPlannerResult();

        // Do path planning:
        planner.solve( planner_input, planner_result);

        cout << "Found goal_distance: " << planner_result.goal_distance << endl;
        cout << "Found path_cost: " << planner_result.path_cost << endl;
        cout << "Acceptable goal nodes: " << planner_result.acceptable_goal_node_ids.size() << endl;
        string status;
        cout << "Success: " << ( status = planner_result.success ? "True" : "False" ) << endl;

#if MRPT_HAS_WXWIDGETS
        // Show result in a GUI:
        mrpt::opengl::COpenGLScenePtr & scene = win.get3DSceneAndLock();

        scene->clear();
    
        PlannerRRT_SE2_TPS::TRenderPlannedPathOptions render_opts;
        render_opts.highlight_path_to_node_id = planner_result.best_goal_node_id;

        planner.renderMoveTree(*scene, planner_input, planner_result,render_opts );

        win.unlockAccess3DScene();
        win.repaint();

        mrpt::math::TPose2D trg_state(0, 0, 0);
        mrpt::poses::CPose2D prev_state;

        DEBUG_PARAMETERS dp;
        // get best node from solution
        if (planner_result.success == true) {
            mrpt::nav::TMoveTreeSE2_TP::NODE_TYPE node = (planner_result.move_tree.getAllNodes().end() -1)->second;

        if(true) {
            const mrpt::nav::TMoveTreeSE2_TP::node_map_t& lstNodes = planner_result.move_tree.getAllNodes();
            std::vector<TPoint2D> trg_pts;
            for (mrpt::nav::TMoveTreeSE2_TP::node_map_t::const_iterator itNode = lstNodes.begin(); itNode != lstNodes.end(); ++itNode) {
                node = itNode->second;
                if (node.parent_id != INVALID_NODEID) {
                    prev_state = lstNodes.find(node.parent_id)->second.state;
                }
                trg_state = node.state;
                trg_pts.push_back(TPoint2D(trg_state.x, trg_state.y));
            }
            dp.trg_pts_debug = trg_pts;
            dp.node = node;
            dp.PTG = PTG;
        }

        Velocity2D vel = generateVelocityTest(PTG, node, dp);
        planner_input.start_pose.x += vel.x()*cos(vel.phi());
        planner_input.start_pose.y += vel.x()*sin(vel.phi());
        planner_input.start_pose.phi -= vel.phi();
        //angle -= 0.1;
        //mrpt::math::TPose2D insPoint(planner_input.start_pose.x+radius*cos(angle),planner_input.start_pose.y+radius*sin(angle),0);
        //planner_input.obstacles_points.insertPoint(-insPoint.y, insPoint.x);
        //win.waitForKey();
    }
#endif
    }
    

}

int main(int argc, char **argv)
{
    std::string node_name = "tp_rrt_test";
    ros::init(argc, argv, node_name);

    ros::NodeHandle n;

    std::string configfile, mapfile;
    n.getParam(std::string("/") + node_name + std::string("/config_file"), configfile);
    n.getParam(std::string("/") + node_name + std::string("/map_file"), mapfile);

    try
    {
        TestRRT1(configfile, mapfile);
        return 0;
    } catch (exception &e)
    {
        cout << "MRPT exception caught: " << e.what() << endl;
        return -1;
    }
    catch (...)
    {
        printf("Another exception!!");
        return -1;
    }
}


