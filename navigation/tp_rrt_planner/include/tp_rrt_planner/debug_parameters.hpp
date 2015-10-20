

#ifndef __PLANNER__DEBUG_PARAMETERS__
#define __PLANNER__DEBUG_PARAMETERS__

#include <vector>

#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/nav/planners/TMoveTree.h>
#include <tp_rrt_planner/velocity2d.hpp>

using mrpt::math::TPoint2D;
using mrpt::math::TPose2D;
using mrpt::math::TPolygon2D;
using std::vector;

class DEBUG_PARAMETERS {
public:
	TPolygon2D original_map_debug;
	TPolygon2D map_scan_debug;
	TPose2D map_center_debug;
	TPose2D goal_map_intersection_half_debug;
	TPose2D final_goal_debug;
	TPose2D rejected_goal;
	vector<TPoint2D> plot_pts_debug;
	vector<TPose2D> goal_pts_debug;
	vector<TPoint2D> trg_pts_debug;
	mrpt::nav::TMoveTreeSE2_TP::NODE_TYPE node;
	mrpt::nav::TListPTGPtr PTG;
	Velocity2D original_vel;
};

#endif /* defined(__PLANNER__DEBUG_PARAMETERS__) */