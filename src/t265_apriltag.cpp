#include <iostream>
#include <vector>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <t265_apriltag_utils/t265_apriltag_utils.h>

using namespace std;

class Detector
{
private:
    std::string parent_frame;
    std::string child_frame;
    int frame_id = 0;

    apriltag_manager tag_manager;

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;

    tf::TransformBroadcaster br;
    tf::Transform latest_tf;

public:
    Detector(ros::NodeHandle nh_, rs2_intrinsics rsint, rs2_extrinsics rsext, double marker_size): it(nh_), tag_manager(rsint, rsext, marker_size)
    {
        std::string img_topic;
        ros::param::param<std::string>("~image", img_topic, "/t265/fisheye1/image_raw");
        ros::param::param<std::string>("~parent", this->parent_frame, "t265_fisheye1_optical_frame");
        ros::param::param<std::string>("~child", this->child_frame, "t265_tag");

        sub = this->it.subscribe(img_topic, 100, &Detector::imgCallback, this);
        
    }

    void imgCallback(const sensor_msgs::Image::ConstPtr &img)
    {
        // We copy the image because of the const - is there a better way?
        unsigned char* myimg = new unsigned char[img->data.size()];
        std::copy(img->data.begin(), img->data.end(), myimg);

        if(this->frame_id%6 == 0){
            std::async(std::launch::async, std::bind([this](unsigned char* img, int fn)
            {
                auto tags = this->tag_manager.detect(img, NULL);

                if(tags.pose_in_camera.size() == 0) {
                    ROS_INFO("no Apriltag detections");
                }
                else {
                    auto tag = tags.pose_in_camera[0];

                    tf::Transform transform;
                    transform.setOrigin(tf::Vector3(tag.translation[0], tag.translation[1], tag.translation[2]));
                    tf::Quaternion q = to_quaternion(tag.rotation);
                    transform.setRotation(q);

                    this->latest_tf = transform;
                }

                // We focus on the 1st marker (there should only be one), but here is the code to iterate
                // for(int t=0; t<tags.pose_in_camera.size(); ++t){
                //     // ROS_INFO("Pose in camera: %s", print(tags.pose_in_camera[t]));
                //     ROS_INFO_STREAM("camera " << print((transformation)tags.pose_in_camera[t]) << std::endl);
                //     // ROS_INFO("Pose in world: ");
                // } 
            },
            myimg, this->frame_id));

        }

        delete [] myimg;
        this->frame_id += 1;
    }

    void run()
    {
        ros::Rate loop_rate(30);
        while (ros::ok()){
        
            if(this->frame_id>0){
                br.sendTransform(tf::StampedTransform(this->latest_tf, ros::Time::now(), this->parent_frame, this->child_frame));
            }
            ros::spinOnce();
        }
       
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "t265_april_detect");
    ros::NodeHandle nh;

    std::string camera_info_topic;
    ros::param::param<std::string>("~camera_info", camera_info_topic, "/t265/fisheye1/camera_info");
    auto info_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic, nh);
    rs2_intrinsics rsint = get_rs2_int_from_camera_info(info_msg);
    rs2_extrinsics rsext = get_rs2_ext();

    double marker_size;
    ros::param::param<double>("~marker_size", marker_size, 0.161);

    Detector d(nh, rsint, rsext, marker_size);
    d.run();
    return 0;
}