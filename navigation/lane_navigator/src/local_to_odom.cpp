// Author: Chinmoy Samant
// Date: 20th May 2015
// This function converts the targets proposed by the lane navigator from 
// Pose2D to PoseStamped

#include <local_to_odom.hpp>
int step_move = -700;
geometry_msgs::Pose2D Local_To_Odom::get_odom_target() {
    geometry_msgs::Pose2D odom_target ;
    float yaw = atan2(2.0*(odom_data.pose.pose.orientation.y*odom_data.pose.pose.orientation.z + odom_data.pose.pose.orientation.w*odom_data.pose.pose.orientation.x), odom_data.pose.pose.orientation.w*odom_data.pose.pose.orientation.w - odom_data.pose.pose.orientation.x*odom_data.pose.pose.orientation.x - odom_data.pose.pose.orientation.y*odom_data.pose.pose.orientation.y + odom_data.pose.pose.orientation.z*odom_data.pose.pose.orientation.z);
    odom_target.x = local_target.x + odom_data.pose.pose.position.x;
    odom_target.y = local_target.y + odom_data.pose.pose.position.y;
    odom_target.theta = local_target.theta + yaw;
    return odom_target;
    }

Local_To_Odom::Local_To_Odom()
{
	ros::NodeHandle node_handle;
	pub_point = node_handle.advertise<geometry_msgs::PoseStamped>("/lane_navigator/proposed_target", 50);
    lanes_subscriber = node_handle.subscribe("/data_fuser1/map", 1, &Local_To_Odom::setTarget, this);
    odom_subscriber = node_handle.subscribe("/odom", 1000, &Local_To_Odom::setOdom, this);  
}

void Local_To_Odom::setTarget(const sensor_msgs::ImageConstPtr msg)
{
	if (debug){ROS_INFO("Listened for the %d time\n", count++);
    cv::namedWindow("listening", CV_WINDOW_AUTOSIZE);}
    cv::Mat img;
    //geometry_msgs::Pose2D msge;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    // img= cv_bridge.imgMsgToCv(msge, "mono8");
    // cv::cvtColor(msg.image,img,CV_BGR2GRAY);
    img = cv_ptr->image;
    local_target = findTarget(img);
    if(debug)
    {
       cv::waitKey(33);
       ROS_INFO("%f %f %f ", local_target.x, local_target.y, local_target.theta);
    }
}

void Local_To_Odom::setOdom(nav_msgs::Odometry subscribed_fix)
{
	odom_data = subscribed_fix;
}

geometry_msgs::Pose2D Local_To_Odom::findTarget(cv::Mat img) {
    cv::Mat cdst, mdst;
    mdst = img - img;
    std::vector<cv::Vec4i>lines;
    cv::Point center_point;
    center_point.x = 0, center_point.y = 0;
    double center_angle = 0.0;
    cdst = img;
    cv::Canny(img, cdst, 25, 75, 3);
    cv::HoughLinesP(cdst, lines, 1, CV_PI / 180, 25, 15, 5);
    for (int i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        cv::line(mdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 3, CV_AA);
    }

    for (int i = 0; i < lines.size(); i++) {
        cv::Vec4i p = lines[i];
        center_point.x += (p[0] + p[2]) / 2;
        center_point.y += (p[1] + p[3]) / 2;
        if ((p[1] - p[3]) != 0) {
            double theta = std::atan2((p[1] - p[3]), (p[0] - p[2]));
            if (theta < 0)
                theta = 3.14 + theta;
            center_angle += (theta);
        }
    }
    if (debug)std::cout << "Total Angle" << center_angle << std::endl;
    if (lines.size() != 0) {
        center_angle = center_angle / lines.size();
        center_point.x = center_point.x / lines.size();
        center_point.y = center_point.y / lines.size();
        if (debug)std::cout << center_point.x << " " << center_point.y << std::endl;
    }
    else
    {
	center_angle= 1.57;
	center_point.x= bot_x;
	center_point.y= bot_y;
    }  
    double m = tan(center_angle);
    if (m == HUGE_VAL) {
        m = 10000;
    }
    cv::Point leftCenter(0, 0), rightCenter(0, 0);
    double leftSlope = 0.0, rightSlope = 0.0, leftCount = 0, rightCount = 0;
    for (int i = 0; i < lines.size(); i++) {
        cv::Vec4i p = lines[i];
        cv::Point midPoint = cv::Point((p[0] + p[2]) / 2, (p[1] + p[3]) / 2);
        double L11 = (0 - center_point.y) / m - (0 - center_point.x);
        double L22 = (midPoint.y - center_point.y) / m - (midPoint.x - center_point.x + 0.0);

        if (L11 * L22 > 0) {
            leftCenter.x += midPoint.x;
            leftCenter.y += midPoint.y;
            if ((lines[i][0] - lines[i][2]) != 0) {
                leftSlope += -(lines[i][1] - lines[i][3]) / (lines[i][0] - lines[i][2]);
            }
            leftCount++;
        } 
        else {
            rightCenter.x += midPoint.x;
            rightCenter.y += midPoint.y;
            if ((lines[i][0] - lines[i][2]) != 0) {
                rightSlope += -(lines[i][1] - lines[i][3]) / (lines[i][0] - lines[i][2]);
            }
            rightCount++;
        }
    }

    leftCenter.x /= leftCount; //Load when count==0
    leftCenter.y /= leftCount;
    leftSlope /= leftCount;

    rightCenter.x /= rightCount;
    rightCenter.y /= rightCount;
    cv::Point target, proj;
    rightSlope /= rightCount;
    if ((center_point.x - leftCenter.x) < 50 || (leftCenter.x - center_point.x) < 50) {
        center_point.x += 150; //Target must not lie on the lane
        target.x = 150;
    }
    if ((rightCenter.x - center_point.x) < 50 || (center_point.x - rightCenter.x) < 50) {
        center_point.x = center_point.x - 150;
        target.x -= 150;
    }

    proj.x = (bot_x + m * (bot_y - center_point.y) + m * m * center_point.x) / (1 + m * m); // Verified
    proj.y = (center_point.y + m * (bot_x - center_point.x) + m * m * bot_y) / (1 + m * m); // Verify it
    target.x += proj.x + cos(center_angle) * step_move;
    target.y = proj.y + sin(center_angle) * step_move;
    cv::Point point2;
    point2.x = center_point.x - 100;
    point2.y = center_point.y - m * 100;
    if (debug)
    {
        std::cout << "target.x: " << target.x << " target.y: " << target.y << std::endl;
        std::cout << "proj.x: " << proj.x << " " << "proj.y: " << proj.y << std::endl;
        //cv::line(mdst, proj, target, cv::Scalar(150),2,8);
        cv::line(mdst, cv::Point(bot_x, bot_y), target, cv::Scalar(255), 2, 8);
        cv::namedWindow("Center_path", cv::WINDOW_AUTOSIZE);
        cv::imshow("Center_path", mdst);
        // cv::waitKey(0);
    }
    center_angle = -1 * center_angle * 180 / CV_PI;
    geometry_msgs::Pose2D target_pose;
    target_pose.x = target.x;
    target_pose.y = (-1 * target.y + origin.y);
    target_pose.theta = center_angle;
    return target_pose;
}

geometry_msgs::PoseStamped Local_To_Odom::convert_Pose2D_to_PoseStamped(geometry_msgs::Pose2D pose2d){
    geometry_msgs::PoseStamped pose_stamp;

    pose_stamp.pose.position.x = pose2d.x;
    pose_stamp.pose.position.y = pose2d.y;
    pose_stamp.pose.position.z = 0;
    pose_stamp.header.frame_id = "odom";
    tf::Quaternion frame_quat;
    frame_quat=tf::createQuaternionFromYaw(pose2d.theta);

    pose_stamp.pose.orientation.x=frame_quat.x();
    pose_stamp.pose.orientation.y=frame_quat.y();
    pose_stamp.pose.orientation.z=frame_quat.z();
    pose_stamp.pose.orientation.w=frame_quat.w();

    return pose_stamp;
}


