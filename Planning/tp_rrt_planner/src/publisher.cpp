#include <iostream>

#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/random.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt_bridge/laser_scan.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_datatypes.h"

#include <mrpt/slam/CICP.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDF.h>

#include <tp_rrt_planner/planner.hpp>

//LIDAR offset values
static const float LIDAR_X_OFFSET = 0.10; //In forward direction
static const float LIDAR_Y_OFFSET = 0.0;  //In left direction
static const float LIDAR_Z_OFFSET = 0.0;  //In upward direction

//Maps for the input
mrpt::obs::CObservation2DRangeScan lidar_data_map;
mrpt::maps::CSimplePointsMap lidar_map;
mrpt::maps::COccupancyGridMap2D icp_map;

//ICP object
mrpt::slam::CICP ICP;

//Current and Target Pose
mrpt::math::TPose2D curr_pose_icp;
mrpt::math::TPose2D target_pose_icp;
mrpt::math::TPose2D current_pose;
mrpt::math::TPose2D target_pose;

//Lidar position wrt to the baselink
const mrpt::poses::CPose3D lidar_wrt_baselink_pose(LIDAR_X_OFFSET, LIDAR_Y_OFFSET, LIDAR_Z_OFFSET);

void lidar_data_subscriber_callback(const sensor_msgs::LaserScan& laser_msg) {

  float runningTime;
  mrpt::slam::CICP::TReturnInfo info;

  mrpt_bridge::convert(laser_msg, lidar_wrt_baselink_pose, lidar_data_map);

  mrpt::poses::CPose2D curr_cpose2d = mrpt::poses::CPose2D(current_pose.x,current_pose.y,current_pose.phi);
  mrpt::poses::CPose3D curr_cpose3d = mrpt::poses::CPose3D(curr_cpose2d);
  //lidar_map.insertObservation(&lidar_data_map, &curr_cpose3d);
  icp_map.insertObservation(&lidar_data_map, &curr_cpose3d);


  // if (!icp_map.isEmpty()) {
  //   //ICP.options.ICP_algorithm = mrpt::slam::icpLevenbergMarquardt;
  //   ICP.options.ICP_algorithm = mrpt::slam::icpClassic;

  //   ICP.options.maxIterations           = 20;
  //   ICP.options.thresholdAng            = mrpt::utils::DEG2RAD(10.0f);
  //   ICP.options.thresholdDist           = 2.0f;
  //   ICP.options.ALFA                    = 0.5f;
  //   ICP.options.smallestThresholdDist   = 0.05f;
  //   ICP.options.doRANSAC = false;

  //   //ICP.options.dumpToConsole();

  //   mrpt::poses::CPose2D initialGuess(curr_pose_icp);

  //   mrpt::poses::CPosePDFPtr pdf = ICP.Align(&icp_map, &lidar_map, initialGuess);
  //   mrpt::poses::CPose2D icpEstimateMean = pdf->getMeanVal();
  //   curr_pose_icp = mrpt::math::TPose2D(icpEstimateMean);

  //   mrpt::poses::CPose3D estimatedPose(icpEstimateMean);
  //   icp_map.insertObservation(&lidar_data_map, &estimatedPose);
  // } else {
  //   icp_map.insertObservation(&lidar_data_map, &curr_cpose3d);
  //   curr_pose_icp = mrpt::math::TPose2D(curr_cpose2d);
  // }

}

void current_pose_callback(const nav_msgs::Odometry& msg) {
  current_pose = mrpt::math::TPose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, tf::getYaw(msg.pose.pose.orientation));
}

void target_pose_callback(const geometry_msgs::PoseStamped& msg) {
  target_pose = mrpt::math::TPose2D(msg.pose.position.x, msg.pose.position.y, tf::getYaw(msg.pose.orientation));
}

int main(int argc, char** argv){
    std::string node_name = "tp_rrt_node"; //launchfile-> <node .... name="tp_rrt_planner" />
    ros::init(argc, argv, node_name);

    ros::NodeHandle n;

    ros::Subscriber lidar_subscriber = n.subscribe("scan", 10, lidar_data_subscriber_callback);
    ros::Subscriber current_pose_subscriber = n.subscribe("odometry/filtered", 10, current_pose_callback);
    ros::Subscriber target_pose_subscriber = n.subscribe("lane_navigator/proposed_target", 10, target_pose_callback);

    ros::Publisher cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Rate loop_rate(10);


    //Debugging params
    bool DEBUG;
    std::string CfgFileName;

    //bool: true if nose following is on, false if not (default: true)
    bool nose_following = true;
    double lookahead;

    //If true uses icp
    bool useSLAM;

    n.getParam(std::string("/") + node_name + std::string("/config_file"), CfgFileName);
    n.getParam(std::string("/") + node_name + std::string("/debug"), DEBUG);
    n.getParam(std::string("/") + node_name + std::string("/nose_following"), nose_following);
    n.getParam(std::string("/") + node_name + std::string("/lookahead"), lookahead);
    n.getParam(std::string("/") + node_name + std::string("/use_slam"), useSLAM);

    std::cout<<"Reading configs"<<std::endl;
    Planner planner(CfgFileName);
    planner.setDebug(DEBUG);
    std::cout<<"Read configs"<<std::endl;
    std::cout.flush();

    ros::spinOnce();

    Velocity2D velocity;
    geometry_msgs::Twist cmd_vel;

    while(ros::ok()) {
        // if (!useSLAM) {
        //     //If we do not want to use SLAM, we just create a new map everytime we plan the path
        //     icp_map = mrpt::maps::COccupancyGridMap2D();
        // }
        ros::spinOnce();
        mrpt::maps::CSimplePointsMap pcmap;
        icp_map.getAsPointCloud(pcmap);
        if (nose_following) {
            target_pose_icp = mrpt::math::TPose2D(curr_pose_icp.x + lookahead, curr_pose_icp.y, curr_pose_icp.phi);
            velocity = planner.planWith(curr_pose_icp, target_pose_icp, pcmap);
        } else {
            velocity = planner.planWith(current_pose, target_pose, pcmap);
        }
        cmd_vel.linear.x = velocity.x();
        cmd_vel.linear.y = velocity.y();
        cmd_vel.angular.z = velocity.phi();
        cmd_vel_publisher.publish(cmd_vel);
        printf("                Cmd Vel  X: %f PHI: %f\n\n", cmd_vel.linear.x, cmd_vel.angular.z );
        loop_rate.sleep();
        icp_map.clear();
    }

    return 0;
}
