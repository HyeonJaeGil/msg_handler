#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

using std::cout; using std::endl;

class odomInitializer{
    
private:
    bool broadcast_tf;
    std::string in_topic;
    std::string out_topic;
    ros::Subscriber topic_sub;
    ros::Publisher topic_pub;
    
public:

    odomInitializer();
    odomInitializer(std::string in_topic);
    // ~odomInitializer();

    void topic_cb(nav_msgs::OdometryConstPtr odom_in);
};

odomInitializer::odomInitializer()
{
    ros::NodeHandle nh_("~");
    topic_sub = nh_.subscribe(this->in_topic, 100, 
                    &odomInitializer::topic_cb, this);
    topic_pub = nh_.advertise<nav_msgs::Odometry>(out_topic, 1);
}

odomInitializer::odomInitializer(std::string in_topic)
    :in_topic(in_topic), out_topic(in_topic+"_new")
{
    ros::NodeHandle nh_("~");
    topic_sub = nh_.subscribe(in_topic, 100, 
                    &odomInitializer::topic_cb, this);
    topic_pub = nh_.advertise<nav_msgs::Odometry>(out_topic, 1);
}

void odomInitializer::topic_cb(nav_msgs::OdometryConstPtr odom_in)
{

    static tf2::Transform initial_pose(
        tf2::Quaternion(odom_in->pose.pose.orientation.x,
                        odom_in->pose.pose.orientation.y,
                        odom_in->pose.pose.orientation.z,
                        odom_in->pose.pose.orientation.w),
        tf2::Vector3(odom_in->pose.pose.position.x,
                        odom_in->pose.pose.position.y,
                        odom_in->pose.pose.position.z));

    tf2::Transform transform_in, transform_out;
    tf2::fromMsg(odom_in->pose.pose, transform_in);
    transform_out.mult(initial_pose.inverse(), transform_in);

    nav_msgs::Odometry odom_out = std::move(*odom_in);
    tf2::toMsg(transform_out, odom_out.pose.pose);

    topic_pub.publish(odom_out);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_initializer");
    odomInitializer odom_initializer("/odometry/filtered");

    ROS_INFO("Starting odom initializer node ...");
    ros::spin();
    return 0;
}