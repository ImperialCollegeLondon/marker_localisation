#pragma once

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
#include <tf/tf.h>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/homography.h>

rs2_intrinsics get_rs2_int_from_camera_info(const sensor_msgs::CameraInfoConstPtr& caminf);
rs2_extrinsics get_rs2_ext();

void homography_compute2(const double c[4][4], matd_t* H);

typedef rs2_extrinsics transformation;

static transformation to_transform(const double R[9], const double t[3]);
static transformation to_transform(const rs2_quaternion& q, const rs2_vector& t);
static transformation operator*(const transformation& a, const transformation& b);
std::string print(const transformation &tf);
tf::Quaternion to_quaternion(const float (&rot)[9]);

class apriltag_manager {
public:
    apriltag_manager(const rs2_intrinsics& _intr,  const rs2_extrinsics _extr_b2f, double tagsize);
    ~apriltag_manager();
    
    struct apriltag_array_t {
        std::shared_ptr<zarray_t>                     det;
        std::vector<std::shared_ptr<apriltag_pose_t>> pose_raw;       //tag pose from library
        std::vector<transformation>                   pose_in_camera; //tag pose in camera coordinate
        std::vector<transformation>                   pose_in_world;  //tag pose in world coordinate
        
        apriltag_detection_t* get(int t) const { apriltag_detection_t* ptr; zarray_get(det.get(), t, &ptr); return ptr; };
        int get_id(int t) const { return get(t)->id; };
        int size() const { return pose_in_camera.size(); };
    };
    
    static void apriltag_pose_destroy(apriltag_pose_t* p){ matd_destroy(p->R); matd_destroy(p->t); delete p;};
    
    apriltag_array_t detect(unsigned char* gray, const rs2_pose* camera_pose) const;
    
protected:
    apriltag_family_t        *tf;
    apriltag_detector_t      *td;
    apriltag_detection_info_t info;
    rs2_intrinsics            intr;
    transformation            tf_body_to_fisheye;
    
    void compute_tag_pose_in_world(apriltag_array_t& tags, const rs2_pose& camera_world_pose) const;
    
    static void undistort(apriltag_detection_t& src, const rs2_intrinsics& intr);
    
    static void deproject(double pt[2], const rs2_intrinsics& intr, const double px[2]);
};
