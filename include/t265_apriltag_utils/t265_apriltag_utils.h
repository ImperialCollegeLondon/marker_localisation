#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <future>
#include <math.h>
#include <vector>
#include <algorithm>



#include <sensor_msgs/CameraInfo.h>
#include <ros/ros.h>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/homography.h>

rs2_intrinsics get_rs2_int_from_camera_info(const sensor_msgs::CameraInfoConstPtr& caminf);
rs2_extrinsics get_rs2_ext();