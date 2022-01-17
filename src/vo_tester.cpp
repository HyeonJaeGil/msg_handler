#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

using std::cout; using std::endl;

class voTester{

private:
    std::string odom_topic;
    std::string pose_topic;
    ros::Subscriber odom_sub;
    ros::Subscriber pose_sub;
    ros::Publisher topic_pub;
    geometry_msgs::Pose tracking_vo;
    geometry_msgs::Pose tracking_pose;
    tf2::Transform initial_pose;

public:
    voTester();

    void vo_cb(nav_msgs::OdometryConstPtr vo_in);
    void pose_cb(const geometry_msgs::PoseStamped& pose_in);
    void setInitialPose(const geometry_msgs::Pose& pose);

};

voTester::voTester()
{
    ros::NodeHandle nh_("~");
    odom_sub = nh_.subscribe(this->odom_topic, 100, 
                    &voTester::vo_cb, this);
    pose_sub = nh_.subscribe(this->pose_topic, 100, 
                    &voTester::pose_cb, this);
    // topic_pub = nh_.advertise<nav_msgs::Odometry>(out_topic, 1);
}

void voTester::setInitialPose(const geometry_msgs::Pose& pose){

    tf2::fromMsg(pose, this->initial_pose);

}
void voTester::vo_cb(nav_msgs::OdometryConstPtr vo_in)
{
    setInitialPose(vo_in->pose.pose);

    tf2::Transform transform_in, transform_out;
    tf2::fromMsg(vo_in->pose.pose, transform_in);
    transform_out.mult(initial_pose.inverse(), transform_in);
    tf2::toMsg(transform_out, this->tracking_vo);

}

void voTester::pose_cb(const geometry_msgs::PoseStamped& pose_in)
{
    setInitialPose(pose_in.pose);

    tf2::Transform transform_in, transform_out;
    tf2::fromMsg(pose_in.pose, transform_in);
    transform_out.mult(initial_pose.inverse(), transform_in);
    tf2::toMsg(transform_out, this->tracking_pose);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vo_tester");
    voTester vo_tester();

    ROS_INFO("Starting vo_tester node ...");
    ros::spin();
    return 0;
}