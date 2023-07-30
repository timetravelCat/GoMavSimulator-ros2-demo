#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2/LinearMath/Transform.h>

class VehicleSensorSubscriber : public rclcpp::Node
{
public:
    VehicleSensorSubscriber();

private:
    tf2::Transform _transform;
    sensor_msgs::msg::CameraInfo _depth_info;

    void on_pose_subscribed(const geometry_msgs::msg::PoseStamped &msg);
    void on_range_subscribed(const sensor_msgs::msg::Range& msg);
    void on_lidar_subscribed(const sensor_msgs::msg::PointCloud2& msg);
    void on_depth_subscribed(const sensor_msgs::msg::Image& msg);
    void on_depth_info_subscribed(const sensor_msgs::msg::CameraInfo& msg);

    // Vehicle Pose Subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _sub_pose_stamped;

    // Vehicle Sensor Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr _sub_range;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_lidar;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _sub_depth;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _sub_depth_info;

    // Vehicle Sensor Remapped by Pose Publisher
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _pub_range;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_lidar;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_depth;
};