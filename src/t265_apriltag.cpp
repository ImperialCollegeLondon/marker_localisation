#include <iostream>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

using namespace std;

class Detector
{
private:
    std::string parent_frame;
    std::string child_frame;
    bool img_received = false;

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;

    tf::TransformBroadcaster br;
    tf::Transform latest_tf;

public:
    Detector(): nh(ros::NodeHandle("~")), it(nh)
    {
        std::string img_topic;
        this->nh.param<std::string>("image", img_topic, "/t265/fisheye1/image_raw");
        this->nh.param<std::string>("parent", this->parent_frame, "t265_fisheye1_optical_frame");
        this->nh.param<std::string>("child", this->child_frame, "t265_tag");

        this->sub = this->it.subscribe(img_topic, 100, &Detector::imgCallback, this);

    }

    void imgCallback(const sensor_msgs::Image::ConstPtr &img)
    {
        // DO DETECTION
        ROS_INFO("Image received");

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(1.0, 0.0, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);

        this->latest_tf = transform;
        this->img_received = true;
    }

    void run()
    {
        ros::Rate loop_rate(30);
        while (ros::ok()){
        
            if(this->img_received){
                br.sendTransform(tf::StampedTransform(this->latest_tf, ros::Time::now(), this->parent_frame, this->child_frame));
            }
            ros::spinOnce();
        }
       
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "t265_april_detect");
    Detector d;
    d.run();
    return 0;
}