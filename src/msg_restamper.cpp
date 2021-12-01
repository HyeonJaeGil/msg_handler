#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <std_msgs/Header.h>
#include <string>

using std::cout; using std::endl;


template <typename T>
class msgRestamper{
    
private:
    std_msgs::Header header_;
    ros::Time time_;
    std::string in_topic;
    std::string out_topic;
    
public:

    msgRestamper();
    // ~msgRestamper();

    void topic_cb(const T& in_msg);

    ros::Subscriber topic_sub;
    ros::Publisher topic_pub;

};

template <typename T>
msgRestamper<T>::msgRestamper()
{
    ros::NodeHandle nh_("~");
    nh_.getParam("in_topic", in_topic);
    nh_.getParam("out_topic", out_topic);
    topic_sub = nh_.subscribe(this->in_topic, 1, &msgRestamper::topic_cb, this);
    topic_pub = nh_.advertise<T>(out_topic, 1);
}

template <typename T>
void msgRestamper<T>::topic_cb(const T& in_msg)
{
    topic_pub.publish(in_msg);
}

template <>
void msgRestamper<sensor_msgs::Imu>::topic_cb(const sensor_msgs::Imu& in_msg)
{
    //get time
    auto now = ros::Time::now();
    sensor_msgs::Imu out_msg(std::move(in_msg));
    out_msg.header.stamp = ros::Time::now();
    // out_msg.header.frame_id = in_msg.header.frame_id;
    topic_pub.publish(out_msg);

}

template <>
void msgRestamper<sensor_msgs::PointCloud2>::topic_cb(const sensor_msgs::PointCloud2& in_msg)
{
    //get time
    auto now = ros::Time::now();
    // sensor_msgs::PointCloud2 out_msg;
    sensor_msgs::PointCloud2 out_msg(std::move(in_msg));
    out_msg.header.stamp = ros::Time::now();
    // out_msg.header.frame_id = in_msg.header.frame_id;
    topic_pub.publish(out_msg);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "msg_restamper");

    // ros::NodeHandle nh;
    // std::string class_name;
    // nh.getParam("class", class_name);
    msgRestamper<sensor_msgs::PointCloud2> pointcloud_restamper;
    // msgRestamper<sensor_msgs::Imu> imu_restamper;
    ros::spin();
    return 0;
}