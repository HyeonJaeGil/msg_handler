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
    std::string initial_pose_topic;
    ros::Subscriber odom_sub;
    ros::Subscriber pose_sub;
    ros::Publisher topic_pub;
    geometry_msgs::Pose tracking_vo;
    geometry_msgs::Pose tracking_pose;
    geometry_msgs::Pose local_vo;
    geometry_msgs::Pose local_pose;
    tf2::Transform initial_pose;
    // ros::Timer timer;

public:
    voTester();

    void vo_cb(nav_msgs::OdometryConstPtr vo_in);
    void pose_cb(const geometry_msgs::PoseStamped& pose_in);
    void timer_cb(const ros::TimerEvent&);
    void setInitialPose(const geometry_msgs::Pose& pose);
    bool isValidPose();


};

voTester::voTester()
{
    ros::NodeHandle nh_("~");
    odom_sub = nh_.subscribe(this->odom_topic, 100, 
                    &voTester::vo_cb, this);
    pose_sub = nh_.subscribe(this->pose_topic, 100, 
                    &voTester::pose_cb, this);
    topic_pub = nh_.advertise<nav_msgs::Odometry>(initial_pose_topic, 1);
    // timer = nh_.createTimer(ros::Duration(1.0), &voTester::timer_cb);

}

void voTester::setInitialPose(const geometry_msgs::Pose& pose){

    tf2::fromMsg(pose, this->initial_pose);

}
void voTester::vo_cb(nav_msgs::OdometryConstPtr vo_in)
{
    // setInitialPose(vo_in->pose.pose);
    setInitialPose(tracking_pose);

    tf2::Transform transform_in, transform_out;
    tf2::fromMsg(vo_in->pose.pose, transform_in);
    transform_out.mult(initial_pose.inverse(), transform_in);
    tf2::toMsg(transform_out, this->local_vo);

    // tf2::Transform transform_in, transform_out;
    tf2::fromMsg(vo_in->pose.pose, transform_in);
    transform_out.mult(transform_in, initial_pose);
    tf2::toMsg(transform_out, this->tracking_vo);


}

void voTester::pose_cb(const geometry_msgs::PoseStamped& pose_in)
{
    // setInitialPose(pose_in.pose);
    setInitialPose(tracking_vo);

    tf2::Transform transform_in, transform_out;
    tf2::fromMsg(pose_in.pose, transform_in);
    transform_out.mult(initial_pose.inverse(), transform_in);
    tf2::toMsg(transform_out, this->local_pose);

    // tf2::Transform transform_in, transform_out;
    tf2::fromMsg(pose_in.pose, transform_in);
    transform_out.mult(transform_in, initial_pose);
    tf2::toMsg(transform_out, this->tracking_pose);

}

bool voTester::isValidPose()
{
    double l2_distance  = this->local_vo.position.x - this->local_pose.position.x
                        + this->local_vo.position.y - this->local_pose.position.y
                        + this->local_vo.position.z - this->local_pose.position.z;

    double angluar_diff;

    if(l2_distance < 1.0)
        return true;
    else   
        return false;
}

void voTester::timer_cb(const ros::TimerEvent&)
{
    if(this->isValidPose())
        setInitialPose(tracking_pose);
    else
        setInitialPose(tracking_vo);

}

void timer_cb(const ros::TimerEvent&)
{
    ROS_INFO("timer triggered...");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vo_tester");
    ros::NodeHandle nh("~");
    voTester vo_tester();

    ros::Timer timer = nh.createTimer(ros::Duration(2), timer_cb);
    ROS_INFO("Starting vo_tester node ...");
    ros::spin();
    return 0;
}