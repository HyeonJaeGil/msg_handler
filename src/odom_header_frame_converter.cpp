#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using std::cout; using std::endl;

template <typename T>
class odomHeaderFrameConverter{
    
private:
    bool broadcast_tf;
    std::string in_topic;
    std::string out_topic;
    std::string parent_frame;
    std::string child_frame;
    
public:

    odomHeaderFrameConverter();
    odomHeaderFrameConverter(std::string in_topic, std::string out_topic, 
                        std::string parent_frame, std::string child_frame);
    // ~odomHeaderFrameConverter();

    void topic_cb(const T& in_msg);

    ros::Subscriber topic_sub;
    ros::Publisher topic_pub;
    tf2_ros::TransformBroadcaster br;


};

template <typename T>
odomHeaderFrameConverter<T>::odomHeaderFrameConverter()
{
    ros::NodeHandle nh_("~");
    if (!nh_.getParam("broadcast_tf", broadcast_tf))
        broadcast_tf = true;
    nh_.getParam("in_topic", in_topic);
    nh_.getParam("out_topic", out_topic);
    nh_.getParam("parent_frame", parent_frame);
    nh_.getParam("child_frame", child_frame);
    topic_sub = nh_.subscribe(this->in_topic, 100, 
                    &odomHeaderFrameConverter::topic_cb, this);
    topic_pub = nh_.advertise<T>(out_topic, 1);
}

template <typename T>
odomHeaderFrameConverter<T>::odomHeaderFrameConverter
                (std::string in_topic, std::string out_topic, 
                std::string parent_frame, std::string child_frame)
{
    ros::NodeHandle nh_("~");
    topic_sub = nh_.subscribe(in_topic, 1, 
                    &odomHeaderFrameConverter::topic_cb, this);
    topic_pub = nh_.advertise<T>(out_topic, 1);
    this->parent_frame = parent_frame;
    this->child_frame = child_frame;
}


template <typename T>
void odomHeaderFrameConverter<T>::topic_cb(const T& in_msg)
{
    T out_msg(std::move(in_msg));
    // out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = parent_frame;
    out_msg.child_frame_id = child_frame;
    topic_pub.publish(out_msg);

    if(broadcast_tf)
    {
        geometry_msgs::TransformStamped transformStamped;
        // transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.stamp = in_msg.header.stamp;
        transformStamped.header.frame_id = parent_frame;
        transformStamped.child_frame_id = child_frame;
        transformStamped.transform.translation.x = out_msg.pose.pose.position.x;
        transformStamped.transform.translation.y = out_msg.pose.pose.position.y;
        transformStamped.transform.translation.z = out_msg.pose.pose.position.z;
        transformStamped.transform.rotation.x = out_msg.pose.pose.orientation.x;
        transformStamped.transform.rotation.y = out_msg.pose.pose.orientation.y;
        transformStamped.transform.rotation.z = out_msg.pose.pose.orientation.z;
        transformStamped.transform.rotation.w = out_msg.pose.pose.orientation.w;
        br.sendTransform(transformStamped);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_header_frame_converter");
    odomHeaderFrameConverter<nav_msgs::Odometry> odometry_restamper
                    ("/camera/odom/sample", "/odom", "odom", "base_link");
                    // ("/odometry/filtered_new", "/odom", "odom", "base_link");
    
    ROS_INFO("Starting odom header frame converter node ...");
    ros::spin();
    return 0;
}