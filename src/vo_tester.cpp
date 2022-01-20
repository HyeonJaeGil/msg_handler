#include <ros/ros.h>
#include <iostream>
#include<cmath>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>

using std::cout; using std::endl;

class voTester{

private:
    std::string velocity_topic;
    std::string odom_topic;
    std::string wheel_topic;
    std::string pose_topic;
    std::string initial_pose_topic;
    
    ros::Subscriber velocity_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber wheel_sub;
    ros::Subscriber pose_sub;
    ros::Publisher topic_pub;

    ros::Publisher markers_pub;


    double x_vel;
    double theta_vel;

    geometry_msgs::Pose tracking_vo;
    geometry_msgs::Pose tracking_wheel;
    geometry_msgs::Pose tracking_pose;

    geometry_msgs::Pose local_vo;
    geometry_msgs::Pose local_wheel;
    geometry_msgs::Pose local_pose;

    tf2::Transform initial_vo;
    tf2::Transform initial_wheel;
    tf2::Transform initial_pose;
    ros::Timer timer;
    ros::NodeHandle nh_;

public:
    voTester(ros::NodeHandle* nh);

    void velocity_cb(const geometry_msgs::Twist& vel_in);
    void vo_cb(nav_msgs::OdometryConstPtr vo_in);
    void wheel_cb(nav_msgs::OdometryConstPtr wheel_in);
    void pose_cb(const geometry_msgs::PoseStamped& pose_in);
    void timer_cb(const ros::TimerEvent&);
    void setInitialPose();
    void setInitialPose(const geometry_msgs::Pose& pose);
    bool isValidPose();
    void printState();
    void publishState(geometry_msgs::Pose pose, std_msgs::Header header);
};

voTester::voTester(ros::NodeHandle* nh)
    :nh_(*nh)
{
    velocity_topic = "/cmd_vel";
    odom_topic = "/camera/odom/sample";
    wheel_topic = "/odometry/filtered";
    pose_topic = "/tracked_pose";
    initial_pose_topic = "/initialpose";
    velocity_sub = nh_.subscribe(this->velocity_topic, 100, 
                    &voTester::velocity_cb, this);
    odom_sub = nh_.subscribe(this->odom_topic, 100, 
                    &voTester::vo_cb, this);
    wheel_sub = nh_.subscribe(this->wheel_topic, 100, 
                    &voTester::wheel_cb, this);
    pose_sub = nh_.subscribe(this->pose_topic, 100, 
                    &voTester::pose_cb, this);
    timer = nh_.createTimer(ros::Duration(1.0), boost::bind(&voTester::timer_cb, this, _1));
    
    topic_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(initial_pose_topic, 1);

    markers_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);


}


void voTester::setInitialPose(){
    tf2::fromMsg(this->tracking_vo, this->initial_vo);
    tf2::fromMsg(this->tracking_wheel, this->initial_wheel);
    tf2::fromMsg(this->tracking_pose, this->initial_pose);

}


void voTester::setInitialPose(const geometry_msgs::Pose& pose){

    tf2::fromMsg(pose, this->initial_pose);

}

void voTester::velocity_cb(const geometry_msgs::Twist& vel_in)
{
    this->x_vel = vel_in.linear.x;
    this->theta_vel = vel_in.angular.z;
}


void voTester::vo_cb(nav_msgs::OdometryConstPtr vo_in)
{
    tf2::Transform transform_in, transform_out;
    tf2::fromMsg(vo_in->pose.pose, transform_in);
    transform_out.mult(initial_vo.inverse(), transform_in);
    tf2::toMsg(transform_out, this->local_vo);

    this->tracking_vo = vo_in->pose.pose;
    // publishState(tracking_vo, vo_in->header);
}


void voTester::wheel_cb(nav_msgs::OdometryConstPtr wheel_in)
{
    tf2::Transform transform_in, transform_out;
    tf2::fromMsg(wheel_in->pose.pose, transform_in);
    transform_out.mult(initial_wheel.inverse(), transform_in);
    tf2::toMsg(transform_out, this->local_wheel);

    this->tracking_wheel = wheel_in->pose.pose;
    publishState(tracking_wheel, wheel_in->header);
}


void voTester::pose_cb(const geometry_msgs::PoseStamped& pose_in)
{
    tf2::Transform transform_in, transform_out;
    tf2::fromMsg(pose_in.pose, transform_in);
    transform_out.mult(initial_pose.inverse(), transform_in);
    tf2::toMsg(transform_out, this->local_pose);

    this->tracking_pose = pose_in.pose;
}


double getLinearDist(
    const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
{
    double linear_displacement = 
        sqrt(pose1.position.x - pose2.position.x
            + pose1.position.y - pose2.position.y
            + pose1.position.z - pose2.position.z);
    return linear_displacement;
}

double getAngularDist(
    const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
{
    double angular_displacement = 
        2*acos(pose1.orientation.w) - 2*acos(pose2.orientation.w); 
    return angular_displacement;
}

bool voTester::isValidPose()
{
    if(getLinearDist(local_pose, local_vo) < this->x_vel /2)
    {
        ROS_INFO("Strange cartographer Localization, follow visual odometry");

    } 
    getLinearDist(local_pose, local_wheel);
    getLinearDist(local_wheel, local_vo);

    return true;

}


void voTester::printState()
{
    tf2::Quaternion q;
    double angle;
    tf2::fromMsg(local_vo.orientation, q);
    angle = q.getAngle() * 180 / M_PI;
    ROS_INFO("local vo: (%f, %f, %f)", local_vo.position.x, local_vo.position.y, angle);

    tf2::fromMsg(local_wheel.orientation, q);
    angle = q.getAngle() * 180 / M_PI;
    ROS_INFO("local wheel: (%f, %f, %f)", local_wheel.position.x, local_wheel.position.y, angle);

    tf2::fromMsg(local_pose.orientation, q);
    angle = q.getAngle() * 180 / M_PI;
    ROS_INFO("local pose: (%f, %f, %f)", local_pose.position.x, local_pose.position.y, angle);
}


void voTester::publishState(geometry_msgs::Pose pose, std_msgs::Header header)
{
    visualization_msgs::Marker m;
    m.header.stamp = header.stamp;
    m.header.frame_id = header.frame_id;
    // m.ns = "LEGS";
    // m.id = id_num++;
    m.type = m.SPHERE;
    m.pose.position.x = pose.position.x;
    m.pose.position.y = pose.position.y;
    m.pose.position.z = 0.2;
    m.lifetime = ros::Duration(5.0);
    m.scale.x = 0.13;
    m.scale.y = 0.13;
    m.scale.z = 0.13;
    m.color.a = 1;
    m.color.r = 0;
    m.color.g = 0;
    m.color.b = 1;
    this->markers_pub.publish(m);
}


void voTester::timer_cb(const ros::TimerEvent&)
{
    ROS_INFO("timer triggered...");

    setInitialPose();
    printState();

    if(!isValidPose())
    {
        // geometry_msgs::Pose new_pose;
        // tf2::Transform tf2_tracking_pose, tf2_local_vo;
        // tf2::fromMsg(this->tracking_pose, tf2_tracking_pose);
        // tf2::fromMsg(this->local_vo, tf2_local_vo);
        // initial_pose.mult(tf2_local_vo, tf2_tracking_pose);
        // tf2::toMsg(initial_pose, new_pose);
        
        geometry_msgs::Pose new_pose;
        tf2::Transform tf2_local_vo;
        tf2::fromMsg(this->local_vo, tf2_local_vo);
        initial_pose.mult(tf2_local_vo, initial_pose);
        tf2::toMsg(initial_pose, new_pose);

        double angle = 2*acos(tracking_pose.orientation.w);
        double new_angle = 2*acos(new_pose.orientation.w);

        ROS_INFO("tracked pose: (%f, %f, %f)",tracking_pose.position.x, tracking_pose.position.y, angle);
        ROS_INFO("initialize pose: (%f, %f, %f)",new_pose.position.x, new_pose.position.x, new_angle);
        geometry_msgs::PoseWithCovarianceStamped initialpose;
        initialpose.header.stamp = ros::Time::now();
        initialpose.header.frame_id = "map";
        initialpose.pose.pose = new_pose;
        topic_pub.publish(initialpose);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "vo_tester");
    ros::NodeHandle nh("~");
    voTester vo_tester(&nh);

    ROS_INFO("Starting vo_tester node ...");
    ros::spin();
    return 0;
}