/***********************************************************************
Changes made by: Anurag A. Bharadwaj
Date: 19th May 

This node publishes the Final output to the high level planner in PoseStamped form
in the odom frame.

*************************************************************************/
#include <waypoint_selector.hpp>

typedef long double precnum_t;



bool WaypointSelector::readWaypoints(std::ifstream& waypoints, std::vector<std::pair<sensor_msgs::NavSatFix, bool> >& gps_waypoints, int& num_of_waypoints, std::string filename) {
    std::pair < sensor_msgs::NavSatFix, bool> target;
    std::string line;
    int count = 0;
    int temp_no_waypoints = -1;
    waypoints.open(filename.c_str(), std::ios::in);
    int i = 0;
    while (waypoints.good()) {
        if (i == 0 && waypoints.eof()) {
            waypoints.close();
            return false;
        }

        if (!waypoints.eof()) {
            while (std::getline(waypoints, line)) {
                if (line[0] != '#') {//checks if there is any commented line
                    std::istringstream iss(line);
                    if (i == 0) {//checks if an integer is given in the beginning
                        int num;
                        iss >> num;
                        temp_no_waypoints = num;
                        i++;
                        continue;
                    }
                    std::string lat, lon;
                    while (count < temp_no_waypoints) {//if waypoints are less than the number written exits
                        while (std::getline(iss, lat, ' ') && std::getline(iss, lon, ' ')) {//checks if both lat and lon are given
                            target.first.latitude = strtod(lat.c_str(), NULL);
                            //std::cerr<<target.first.latitude<<"YEEEEEEE\n";
                            target.first.longitude = strtod(lon.c_str(), NULL);
                            //std::cerr<<target.first.longitude<<"YOOOOOOOOOO\n";
                            target.first.altitude = altitude_preset;
                            target.second = false;
                            gps_waypoints.push_back(target);
                            count++;
                        }
                        break;
                    }
                }
            }
            num_of_waypoints = count;
            return true;
        }
    }
    return false;
}

geometry_msgs::Pose2D WaypointSelector::interpret(sensor_msgs::NavSatFix current_fix_, sensor_msgs::NavSatFix target_fix_) {
    const precnum_t a = 6378137L; // Semi-major axis of the Earth (meters)
    const precnum_t b = 6356752.3142L; // Semi-minor axis:
    const precnum_t ae = acos(b / a); // eccentricity:
    const precnum_t cos2_ae_earth = cos(ae) * cos(ae); // The cos^2 of the angular eccentricity of the Earth: // 0.993305619995739L;
    const precnum_t sin2_ae_earth = sin(ae) * sin(ae); // The sin^2 of the angular eccentricity of the Earth: // 0.006694380004261L;

    precnum_t lon = (precnum_t(current_fix_.longitude) * M_PI / 180);
    precnum_t lat = (precnum_t(current_fix_.latitude) * M_PI / 180);
    precnum_t N = a / std::sqrt(1 - sin2_ae_earth * std::pow(sin(lat), 2));

    geometry_msgs::Pose enu_relative_target;
    enu_relative_target.position.x = (N + current_fix_.altitude) * cos(lat) * cos(lon);
    enu_relative_target.position.y = (N + current_fix_.altitude) * cos(lat) * sin(lon);
    enu_relative_target.position.z = (cos2_ae_earth * N + current_fix_.altitude) * sin(lat);

    lon = (precnum_t(target_fix_.longitude) * M_PI / 180);
    lat = (precnum_t(target_fix_.latitude) * M_PI / 180);
    N = a / std::sqrt(1 - sin2_ae_earth * std::pow(sin(lat), 2));

    geometry_msgs::Pose temp;
    temp.position.x = (N + target_fix_.altitude) * cos(lat) * cos(lon);
    temp.position.y = (N + target_fix_.altitude) * cos(lat) * sin(lon);
    temp.position.z = (cos2_ae_earth * N + target_fix_.altitude) * sin(lat);

    const double clat = cos((current_fix_.latitude * M_PI / 180)), slat = sin((current_fix_.latitude * M_PI / 180));
    const double clon = cos((current_fix_.longitude * M_PI / 180)), slon = sin((current_fix_.longitude * M_PI / 180));

    temp.position.x -= enu_relative_target.position.x;
    temp.position.y -= enu_relative_target.position.y;
    temp.position.z -= enu_relative_target.position.z;

    enu_relative_target.position.x = -slon * temp.position.x + clon * temp.position.y;
    enu_relative_target.position.y = -clon * slat * temp.position.x - slon * slat * temp.position.y + clat * temp.position.z;
    enu_relative_target.position.z = clon * clat * temp.position.x + slon * clat * temp.position.y + slat * temp.position.z;

    geometry_msgs::Pose2D enu_relative_target_2D;
    enu_relative_target_2D.x = enu_relative_target.position.x;
    enu_relative_target_2D.y = enu_relative_target.position.y;

    return enu_relative_target_2D;
}


geometry_msgs::Pose2D WaypointSelector::get_Pose2D_odom(sensor_msgs::NavSatFix current_fix_, sensor_msgs::NavSatFix target_fix_)
{

    tf::StampedTransform transform;

    geometry_msgs::Pose2D target_ENU;
    geometry_msgs::Pose2D target_base_link;
    geometry_msgs::Pose2D target_odom;

   // std::cout << "yaw_north = " << current_yaw_north_ << std::endl;
    //std::cout << "yaw_odom = " << current_yaw_odom_ << std::endl;
    double yaw_radian_base_link = (current_yaw_north_ + 90 - current_yaw_odom_) * (PI / 180) ;
   // std::cout << "yaw = " << current_yaw_ << std::endl;
   // std::cout << "latitude = " << target_fix_.latitude << " longitude = " << target_fix_.longitude << std::endl;


    target_ENU = interpret(current_fix_, target_fix_);

    //std::cerr<<target_ENU_.x<<"ENU x \n";

    target_odom.x = target_ENU.x * cos(yaw_radian_base_link) + target_ENU.y * sin(yaw_radian_base_link);
    target_odom.y = target_ENU.y * cos(yaw_radian_base_link) - target_ENU.x * sin(yaw_radian_base_link);

    /*try {
        listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    tf::Vector3 vector_base_link = tf::Vector3(target_base_link.x , target_base_link.y , 0.0);
    tf::Vector3 vector_odom = transform(vector_base_link);
    target_odom.x = vector_odom.getX();
    target_odom.y = vector_odom.getY();
    target_odom.theta = 0;*/

    target_odom.x += current_ekf_position_.pose.pose.position.x;
    target_odom.y += current_ekf_position_.pose.pose.position.y;
    target_odom.theta = 0;
    
    return target_odom;

}


WaypointSelector::~WaypointSelector(){

}

double WaypointSelector::getMod(geometry_msgs::Point p1, geometry_msgs::Pose2D p2) {
    return std::sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}

double WaypointSelector::getModgps(const sensor_msgs::NavSatFix a, const sensor_msgs::NavSatFix b) {
    /*double a_utmX = 0, b_utmX = 0, a_utmY = 0, b_utmY = 0;
    std::string a_utmzone, b_utmzone;
    tf::Pose a_utm_pose, b_utm_pose;*/

    //gps_common::LLtoUTM(a.latitude, a.longitude, a_utmY, a_utmX, a_utmzone);
    //gps_common::LLtoUTM(b.latitude, b.longitude, b_utmY, b_utmX, b_utmzone);
    geometry_msgs::Pose2D target_odom;
    //geometry_msgs::Pose2D target_odom_b;
    target_odom= get_Pose2D_odom(a, b);

   // target_odom_b = get_Pose2D_odom(current_gps_position_, b);


    //std::cout << "position of target: " << target_odom.x << ", " << target_odom.y << std::endl;
    //std::cout << "bot position: " << current_ekf_position_.pose.pose.position.x << ", " << current_ekf_position_.pose.pose.position.y << std::endl << std::endl;
    //std::cout << "position_b : " << target_odom_b.x << ", " << target_odom_b.y << std::endl;

    return std::sqrt((target_odom.x - current_ekf_position_.pose.pose.position.x)*(target_odom.x - current_ekf_position_.pose.pose.position.x) + (target_odom.y - current_ekf_position_.pose.pose.position.y)*(target_odom.y -current_ekf_position_.pose.pose.position.y));
    //return std::sqrt((a_utmX - b_utmX)*(a_utmX - b_utmX)+(a_utmY - b_utmY)*(a_utmY - b_utmY));
    //a_utm_pose.setOrigin(tf::Vector3(a_utmX, a_utmY, a.latitude));
    //b_utm_pose.setOrigin(tf::Vector3(b_utmX, b_utmY, b.latitude));
}

geometry_msgs::Pose2D WaypointSelector::getPose2DfromGPS(sensor_msgs::NavSatFix target) {
    tf::StampedTransform transform;

    double utmX, utmY;
    std::string utmzone;
    gps_common::LLtoUTM(target.latitude, target.longitude, utmY, utmX, utmzone);

    tf::Pose utmPose;
    utmPose.setOrigin(tf::Vector3(utmX, utmY, target.altitude));

    geometry_msgs::Pose2D target_pose;

    //bool gotTransform = false;
    try {
        listener.waitForTransform("/odom", "/utm", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/odom", "/utm", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    // while(!gotTransform){
    //     try{
    //         listener.lookupTransform("/odom", "/utm", ros::Time(0), transform);
    //         target_pose.setOrigin(transform.getOrigin());
    //         target_pose.setRotation(transform.getRotation());
    //         gotTransform = true;
    //     } catch (tf::TransformException &ex){
    //         ROS_ERROR("%s",ex.what());
    //         ros::Duration(1.0).sleep();
    //         continue;
    //     }
    // }
    tf::Vector3 vector_utm = tf::Vector3(utmX, utmY, target.altitude);
    tf::Vector3 vector_odom = transform(vector_utm);
    target_pose.x = vector_odom.getX();
    target_pose.y = vector_odom.getY();
    target_pose.theta = 0;

    return target_pose;

}

void WaypointSelector::set_current_gps_position(const sensor_msgs::NavSatFixConstPtr& msg) {
    bool hasGps_ = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&
               !std::isnan(msg->altitude) &&
               !std::isnan(msg->latitude) &&
               !std::isnan(msg->longitude));

    if(hasGps_)
    {
        double utmX = 0;
        double utmY = 0;
        gps_common::LLtoUTM(msg->latitude, msg->longitude, utmY, utmX, utmZone_);
        latestUtmPose_.setOrigin(tf::Vector3(utmX, utmY, msg->altitude));

        current_gps_position_ = *msg;
        subscription_started_gps = true; //makes sure we have subscribed at least once before we process the data
    }
}

void WaypointSelector::set_current_ekf_position(nav_msgs::Odometry subscribed_fix)
{
    current_ekf_position_ = subscribed_fix;
    subscription_started_odom = true;

    tf::Quaternion q(subscribed_fix.pose.pose.orientation.x, subscribed_fix.pose.pose.orientation.y, subscribed_fix.pose.pose.orientation.z, subscribed_fix.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_yaw_odom_ = yaw*(180/M_PI);
    //odom1_publisher.publish(current_ekf_position_);
}

void WaypointSelector::set_current_yaw_north(sensor_msgs::Imu current_imu)
{

    tf::Quaternion q(current_imu.orientation.x, current_imu.orientation.y, current_imu.orientation.z, current_imu.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    //current_yaw_north_ = current_yaw_north.data;
    current_yaw_north_ = yaw*(180/M_PI);

}

/*void WaypointSelector::set_current_yaw_odom(std_msgs::Float64 current_yaw_odom)
{
    current_yaw_odom_ = current_yaw_odom.data;
}*/



std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator WaypointSelector::selectNearestWaypoint() {
    int flagged_index = -1;
    double min_distance;
    for (std::vector <std::pair<sensor_msgs::NavSatFix, bool > >::iterator it = gps_waypoints_.begin(); it != gps_waypoints_.end(); ++it) {
        if (!it->second) {
            if (flagged_index == -1) {
                flagged_index = it - gps_waypoints_.begin();
                min_distance = getModgps(current_gps_position_, it->first);
            }
            if (getModgps(current_gps_position_, it->first) < min_distance) {
                flagged_index = it - gps_waypoints_.begin();
                min_distance = getModgps(current_gps_position_, it->first);
            }
        }
    }
    if (flagged_index == -1) {
        return gps_waypoints_.end();
    }

    return gps_waypoints_.begin() + flagged_index;
}

std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator WaypointSelector::selectNextWaypointInSequence() {
    int flagged_index = -1;
    if (last_waypoint_ == gps_waypoints_.end()) {
        return gps_waypoints_.begin();
    }

    unsigned int index = (last_waypoint_ - gps_waypoints_.begin() + 1) % gps_waypoints_.size();
    for (; index != (last_waypoint_ - gps_waypoints_.begin()); index = (index + 1) % gps_waypoints_.size()) {
        if (!gps_waypoints_.at(index).second) {
            flagged_index = index;
            break;
        }
    }
    if (flagged_index == -1) {
        if (!gps_waypoints_.at(index).second) {
            flagged_index = index;
        }
    }
    if (flagged_index == -1) {
        return gps_waypoints_.end();
    }

    return gps_waypoints_.begin() + flagged_index;
}

bool WaypointSelector::reachedCurrentWaypoint(std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator target_ptr) {
    double error = getModgps(current_gps_position_, target_ptr->first);
    //std::cout<< " current gps " << current_gps_position_ << "\n";
    std::cout << "Error: " << error << std::endl;

    if (error < proximity_ || planner_status_ == "TARGET REACHED") {
        target_ptr->second = true;
        last_waypoint_ = target_ptr;
        num_visited_waypoints_++;
        if (num_visited_waypoints_ == gps_waypoints_.size()) {
            inside_no_mans_land_ = false;
        } else {
            inside_no_mans_land_ = true;
        }
        return true;
    }
    return false;
}

void WaypointSelector::set_planner_status(std_msgs::String status) {
    planner_status_ = status.data;
}

WaypointSelector::WaypointSelector(std::string file, int strategy, ros::NodeHandle& node_handle) {

    if (!readWaypoints(waypoints_, gps_waypoints_, num_of_waypoints_, file)) {
        std::cout << "exiting";
        exit(1);
    }

    planner_status_subscriber = node_handle.subscribe("local_planner/status", buffer_size, &WaypointSelector::set_planner_status, this);
    fix_subscriber = node_handle.subscribe("gps/filtered", buffer_size, &WaypointSelector::set_current_gps_position, this);
    yaw_subscriber_north = node_handle.subscribe("/vn_ins/imu", buffer_size, &WaypointSelector::set_current_yaw_north, this);
    //yaw_subscriber_odom = node_handle.subscribe("/robot_localization/yaw_filtered", buffer_size, &WaypointSelector::set_current_yaw_odom, this);
    odom_subscriber = node_handle.subscribe("odometry/filtered", buffer_size,  &WaypointSelector::set_current_ekf_position, this);
    next_waypoint_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("waypoint_navigator/proposed_target", buffer_size);
    nml_flag_publisher = node_handle.advertise<std_msgs::Bool>("waypoint_selector/nml_flag", buffer_size);
    //odom1_publisher = node_handle.advertise<nav_msgs::Odometry>("odom1",buffer_size);

    strategy_ = strategy;
    last_waypoint_ = gps_waypoints_.end();
    inside_no_mans_land_ = false;
    num_visited_waypoints_ = 0;
    subscription_started_gps = false;
    subscription_started_odom = false;
   // std::exit(0);
}

geometry_msgs::Pose2D WaypointSelector::findTarget() {
    switch (strategy_) {
        case sequential_selector:
            if (num_visited_waypoints_ == 0) {
                current_target_ptr = selectNearestWaypoint();
                if (!reachedCurrentWaypoint(current_target_ptr)) {
                    return get_Pose2D_odom(current_gps_position_ , current_target_ptr->first);
                }
            } else {
                current_target_ptr = selectNextWaypointInSequence();
                if (current_target_ptr == gps_waypoints_.end()) {
                    for (std::vector < std::pair < sensor_msgs::NavSatFix, bool> >::iterator it = gps_waypoints_.begin(); it != gps_waypoints_.end(); it++) {
                        it->second = false;
                    }
                    current_target_ptr = selectNextWaypointInSequence();
                }
                if (!reachedCurrentWaypoint(current_target_ptr)) {
                    return get_Pose2D_odom(current_gps_position_ , current_target_ptr->first);
                }
            }
            break;
        case greedy_selector:
            current_target_ptr = selectNearestWaypoint();
            if (current_target_ptr == gps_waypoints_.end()) {
                {
                    return get_Pose2D_odom(current_gps_position_ , current_gps_position_);
                }
            }
            if (!reachedCurrentWaypoint(current_target_ptr)) {
                return get_Pose2D_odom(current_gps_position_ , current_target_ptr->first);
            } else {
                current_target_ptr = selectNearestWaypoint();
                return get_Pose2D_odom(current_gps_position_ , current_target_ptr->first);
            }
            break;
    }
}

geometry_msgs::PoseStamped WaypointSelector::convert_Pose2D_to_PoseStamped(geometry_msgs::Pose2D pose2d){
    geometry_msgs::PoseStamped pose_stamp;

    pose_stamp.pose.position.x = pose2d.x;
    pose_stamp.pose.position.y = pose2d.y;
    pose_stamp.pose.position.z = 0;
    pose_stamp.header.frame_id = "odom";
    pose_stamp.header.stamp = ros::Time(0);
    tf::Quaternion frame_quat;
    frame_quat=tf::createQuaternionFromYaw(pose2d.theta);

    pose_stamp.pose.orientation.x=frame_quat.x();
    pose_stamp.pose.orientation.y=frame_quat.y();
    pose_stamp.pose.orientation.z=frame_quat.z();
    pose_stamp.pose.orientation.w=frame_quat.w();

    return pose_stamp;
}

bool WaypointSelector::isInsideNoMansLand() {
    return inside_no_mans_land_;
}

//void WaypointSelector::convert_gps_to_odom()
//{
//    for(std::vector < std::pair <sensor_msgs::NavSatFix, bool> >::iterator it = gps_waypoints_.begin(); it != gps_waypoints_.end(); ++it){
//        geometry_msgs::Pose2D bot_relative_2D;
//        std::pair<geometry_msgs::Pose2D, bool> odom_target_2D;
//
//        bot_relative_2D = interpret(current_gps_position_, it->first);
//        odom_target_2D.first.x = bot_relative_2D.x + current_odom_position_.pose.pose.position.x;
//        odom_target_2D.first.y = bot_relative_2D.y + current_odom_position_.pose.pose.position.y;
//        odom_target_2D.first.theta = 0;
//        odom_target_2D.second = false;
//        odom_waypoints_.push_back(odom_target_2D);
//    }
//}
