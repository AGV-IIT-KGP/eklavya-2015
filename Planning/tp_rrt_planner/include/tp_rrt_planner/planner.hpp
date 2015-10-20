#ifndef __TP_RRT_PLANNER_HPP__
#define __TP_RRT_PLANNER_HPP__

/*! MRPT includes
 */
#include <mrpt/nav.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/random.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/nav/planners/TMoveTree.h>

/*! Project includes
 */
#include <tp_rrt_planner/velocity2d.hpp>
#include <tp_rrt_planner/debug_parameters.hpp>
#include <tp_rrt_planner/kalmanfilterl.hpp>

/*! Only for debugging
 */
#include <cstdio>

/*! Maximum iterations which it should continue upon failure before
 *  starting afresh with a new planner_result.
 */
static const size_t MAX_ITERS = 5;

/*! http://www.isa.uma.es/personal/jafma/papers/tp-space_iros06.pdf Page:4
 */
static const float ROBOT_R = 1.2; 

/*! The class for planner abstracting the actual algorithm used inside
 */
class Planner {
public:
    /*! Contructor
     *  Takes the filename of the config file for the PTGs
     */
    Planner(const std::string filename);

    /*! Destructor
     */
    ~Planner();

    /*! Set this to true to enable debug output
     */
    void setDebug(bool val);

    /*! The entry function that needs to be called for every update of
     *  velocity command.
     */
    Velocity2D planWith(mrpt::math::TPose2D current_pose,
                        mrpt::math::TPose2D target_pose,
                        mrpt::maps::CSimplePointsMap lidar_map);

    float linear_deviation_threshold;   /*! The linear distance above which the bot is considered deviated */
    float angular_deviation_threshold;  /*! The angle in radians above which the bot is considered deviated */ 

private:
    mrpt::nav::PlannerRRT_SE2_TPS  planner; /*! The Trajectory Planner object. Multiple planners can be introduced in this class */

    mrpt::nav::PlannerRRT_SE2_TPS::TPlannerResult planner_result;   /*! The result for the trajectory planner */
    mrpt::nav::PlannerRRT_SE2_TPS::TPlannerInput planner_input;     /*! The input for the trajectory planner */
    mrpt::nav::TListPTGPtr PTG_list;    /*! The list of ParamterisedTrajectoryGenerators */
    
    bool debug; /*! The debug flag */

    mrpt::gui::CDisplayWindow3D* winptr;    /* The output windows */
    size_t iters;   /* The number of iters that keeps track of how many times to plan before renewing */
    KalmanFilterL kf;   /* The Kalman Filter for smoothing of velocity */

    /*! Method that given a trajectory and current pose calculates the velocity
     *  of the vehicle. If the vehicle has deviated, it returns by the
     *  paramter hasDeviated
     */
    Velocity2D generateVelfromTrajectory(mrpt::nav::TMoveEdgeSE2_TP * edge_ptr,
                                        mrpt::math::TPose2D current_pose,
                                        bool& hasDeviated);

    /*! http://www.isa.uma.es/personal/jafma/papers/tp-space_iros06.pdf Page:4
     */
    inline float getMetric(mrpt::math::TPose2D a, mrpt::math::TPose2D b) {
        return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+ROBOT_R*ROBOT_R*(a.phi-b.phi)*(a.phi-b.phi);
    }

    /*! Gives the deviation of the target from the end pose
     *  of the the path calculated last time.
     */
    inline float getTargetDeviation() {
        mrpt::utils::TNodeID target_node_id = planner_result.best_goal_node_id;
        mrpt::nav::TMoveTreeSE2_TP::node_map_t node_map = planner_result.move_tree.getAllNodes();
        mrpt::nav::TMoveTreeSE2_TP::NODE_TYPE arrival_node_obj = node_map[target_node_id];
        
        mrpt::math::TPose2D arrival_pose = (arrival_node_obj.edge_to_parent)->end_state;
        mrpt::math::TPose2D target_pose = planner_input.goal_pose;
        return sqrt((target_pose.x-arrival_pose.x)*(target_pose.x-arrival_pose.x)+(target_pose.y-arrival_pose.y)*(target_pose.y-arrival_pose.y));
    }
};

#endif //ifndef __TP_RRT_PLANNER_HPP__