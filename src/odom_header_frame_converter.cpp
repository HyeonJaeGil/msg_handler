#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using std::cout; using std::endl;

class odomHeaderFrameConverter{
    
private:
    bool broadcast_tf;
    bool new_timestamp;
    std::string in_topic;
    std::string out_topic;
    std::string parent_frame;
    std::string child_frame;
    
public:

    odomHeaderFrameConverter(ros::NodeHandle* nodehandle);
    odomHeaderFrameConverter(ros::NodeHandle* nodehandle,
                        std::string in_topic, std::string out_topic, 
                        std::string parent_frame, std::string child_frame);
    // ~odomHeaderFrameConverter();

    void topic_cb(const nav_msgs::Odometry& in_msg);

    ros::Subscriber topic_sub;
    ros::Publisher topic_pub;
    tf2_ros::TransformBroadcaster br;


};
odomHeaderFrameConverter::odomHeaderFrameConverter(ros::NodeHandle* nodehandle)
{
    ros::NodeHandle nh_("~");
    if (!nh_.getParam("broadcast_tf", broadcast_tf))
        broadcast_tf = true;    
    if (!nh_.getParam("new_timestamp", new_timestamp))
        new_timestamp = false;
    if (nh_.getParam("in_topic", in_topic))
        in_topic = "/camera/odom/sample";
    if (nh_.getParam("out_topic", out_topic))
        out_topic = "/odom";
    if (nh_.getParam("parent_frame", parent_frame))
        parent_frame = "odom";
    if (nh_.getParam("child_frame", child_frame))
        child_frame = "base_link";
    topic_sub = nh_.subscribe(this->in_topic, 100, 
                    &odomHeaderFrameConverter::topic_cb, this);
    topic_pub = nh_.advertise<nav_msgs::Odometry>(out_topic, 1);
}

odomHeaderFrameConverter::odomHeaderFrameConverter(ros::NodeHandle* nodehandle,
                std::string in_topic, std::string out_topic, 
                std::string parent_frame, std::string child_frame)
{
    ros::NodeHandle nh_("~");

    if (!nh_.getParam("broadcast_tf", broadcast_tf))
        broadcast_tf = true;    
    if (!nh_.getParam("new_timestamp", new_timestamp))
        new_timestamp = false;
    topic_sub = nh_.subscribe(in_topic, 1, 
                    &odomHeaderFrameConverter::topic_cb, this);
    topic_pub = nh_.advertise<nav_msgs::Odometry>(out_topic, 1);
    this->parent_frame = parent_frame;
    this->child_frame = child_frame;
}


void odomHeaderFrameConverter::topic_cb(const nav_msgs::Odometry& in_msg)
{
    nav_msgs::Odometry out_msg(std::move(in_msg));
    // out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = parent_frame;
    out_msg.child_frame_id = child_frame;
    topic_pub.publish(out_msg);

    if(broadcast_tf)
    {
        geometry_msgs::TransformStamped transformStamped;
        if(new_timestamp)
            transformStamped.header.stamp = ros::Time::now();
        else
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
    ros::NodeHandle nh("~");
    // odomHeaderFrameConverter ohfc_wheel
    //                 (&nh, "/odometry/filtered_new", "/odom", "odom", "wheel_center");
    // odomHeaderFrameConverter ohfc_camera
    //                 (&nh, "/camera/odom/sample_new", "/odom", "odom", "wheel_center");
    odomHeaderFrameConverter ohfc_lio
                    (&nh, "/lio_sam/mapping/odometry", "/odom", "odom", "wheel_center");

    ROS_INFO("Starting odom header frame converter node ...");
    ros::spin();
    return 0;
}