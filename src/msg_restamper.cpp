#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <std_msgs/Header.h>
#include <string>
#include <functional>
using std::cout; using std::endl;


template <typename T>
class msgRestamper{
    
private:
    std::string in_topic;
    std::string out_topic;
    std::string imu_topic;
    std::string scan_topic;
    std::string pc2_topic;
    std::string odom_topic;
    bool subsribe_imu ;
    bool subsribe_scan;
    bool subsribe_pc2 ;
    bool subsribe_odom;

    ros::NodeHandle nh_;

    
public:

    msgRestamper(ros::NodeHandle* nodehandle);
    msgRestamper(std::string in_topic, std::string out_topic);
    // ~msgRestamper();

    void topic_cb(const T& in_msg);

    ros::Subscriber topic_sub;
    ros::Publisher topic_pub;

};

template <typename T>
msgRestamper<T>::msgRestamper(ros::NodeHandle* nodehandle)
    :nh_(*nodehandle)
{
    // nh_.getParam("in_topic", in_topic);
    // nh_.getParam("out_topic", out_topic);

    if(nh_.getParam("imu_topic", imu_topic)){
        // topic_sub = nh_.subscribe(this->imu_topic, 1, &msgRestamper<sensor_msgs::Imu>::topic_cb, this);
        in_topic = imu_topic;
        out_topic = imu_topic + "_restamped";
        ROS_INFO("HI");
    }
    else if(nh_.getParam("scan_topic", scan_topic)){
        // topic_sub = nh_.subscribe(this->scan_topic, 1, &msgRestamper<sensor_msgs::LaserScan>::topic_cb, this);
        in_topic = scan_topic;
        out_topic = scan_topic + "_restamped";
    }
    else if(nh_.getParam("pc2_topic", pc2_topic)){
        // topic_sub = nh_.subscribe(this->pc2_topic, 1, &msgRestamper<sensor_msgs::PointCloud2>::topic_cb, this);
        in_topic = pc2_topic;
        out_topic = pc2_topic + "_restamped";
    }
    else if(nh_.getParam("odom_topic", odom_topic)){
        // topic_sub = nh_.subscribe(this->odom_topic, 1, &msgRestamper<nav_msgs::Odometry>::topic_cb, this);
        in_topic = odom_topic;
        out_topic = odom_topic + "_restamped";
    }
    ROS_INFO("%s", in_topic.c_str());
    ROS_INFO("%s", out_topic.c_str());
    topic_sub = nh_.subscribe(in_topic, 1, &msgRestamper<T>::topic_cb, this);
    topic_pub = nh_.advertise<T>(out_topic, 1);
}

template <typename T>
msgRestamper<T>::msgRestamper(std::string in_topic, std::string out_topic)
{
    ros::NodeHandle nh_("~");
    topic_sub = nh_.subscribe(in_topic, 1, &msgRestamper::topic_cb, this);
    topic_pub = nh_.advertise<T>(out_topic, 1);
}


template <typename T>
void msgRestamper<T>::topic_cb(const T& in_msg)
{
    T out_msg(std::move(in_msg));
    // T out_msg = in_msg;
    out_msg.header.stamp = ros::Time::now();
    topic_pub.publish(out_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "msg_restamper");

    ros::NodeHandle nh("~");
    // std::string class_name;
    // nh.getParam("class", class_name);
    // msgRestamper<sensor_msgs::Imu> imu_restamper("/imu/data", "/imu/data_restamped");
    // msgRestamper<sensor_msgs::LaserScan> scan_restamper("/scan", "/scan_restamped");
    // msgRestamper<nav_msgs::Odometry> odom_restamper("/camera/odom/sample", "/camera/odom/sample_restamped");
    // if(nh.getParam("subscribe_imu")){
    bool subscribe_imu ;
    bool subscribe_scan;
    bool subscribe_pc2 ;
    bool subscribe_odom; 

    nh.getParam("subscribe_imu", subscribe_imu );
    nh.getParam("subscribe_scan", subscribe_scan);
    nh.getParam("subscribe_pc2", subscribe_pc2 );
    nh.getParam("subscribe_odom", subscribe_odom);

    if(subscribe_pc2){
        msgRestamper<sensor_msgs::PointCloud2> pointcloud_restamper(&nh);
        ROS_INFO("start pc2 restamper...");
    }
    if(subscribe_imu){
        msgRestamper<sensor_msgs::Imu> imu_restamper(&nh);
        ROS_INFO("start imu restamper...");
    }
    if(subscribe_scan){
        msgRestamper<sensor_msgs::LaserScan> scan_restamper(&nh);
        ROS_INFO("start scan restamper...");
    }
    if(subscribe_odom){
        msgRestamper<nav_msgs::Odometry> odom_restamper(&nh);
        ROS_INFO("start odom restamper...");
    }

    ros::param::del("subscribe_pc2");
    ros::param::del("subscribe_imu");
    ros::param::del("subscribe_scan");
    ros::param::del("subscribe_odom");
    ros::spin();
    return 0;
}