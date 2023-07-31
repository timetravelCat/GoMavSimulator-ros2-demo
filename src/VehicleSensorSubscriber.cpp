#include <GoMavSimulator-ros2-demo/VehicleSensorSubscriber.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

VehicleSensorSubscriber::VehicleSensorSubscriber() : Node("VehicleSensorSubscriber")
{
    _sub_pose_stamped = this->create_subscription<geometry_msgs::msg::PoseStamped>("vehicle_pose", rclcpp::QoS{1}, std::bind(&VehicleSensorSubscriber::on_pose_subscribed, this, std::placeholders::_1));
    _sub_range = this->create_subscription<sensor_msgs::msg::Range>("vehicle/range", rclcpp::QoS{1}, std::bind(&VehicleSensorSubscriber::on_range_subscribed, this, std::placeholders::_1));
    _sub_lidar = this->create_subscription<sensor_msgs::msg::PointCloud2>("vehicle/lidar", rclcpp::QoS{1}, std::bind(&VehicleSensorSubscriber::on_lidar_subscribed, this, std::placeholders::_1));
    _sub_depth = this->create_subscription<sensor_msgs::msg::Image>("vehicle/depth", rclcpp::QoS{1}, std::bind(&VehicleSensorSubscriber::on_depth_subscribed, this, std::placeholders::_1));
    _sub_depth_info = this->create_subscription<sensor_msgs::msg::CameraInfo>("vehicle/depth_info", rclcpp::QoS{1}, std::bind(&VehicleSensorSubscriber::on_depth_info_subscribed, this, std::placeholders::_1));

    _pub_range = this->create_publisher<geometry_msgs::msg::PointStamped>("vehicle/range_result", rclcpp::QoS{1});
    _pub_lidar = this->create_publisher<sensor_msgs::msg::PointCloud2>("vehicle/lidar_result", rclcpp::QoS{1});
    _pub_depth = this->create_publisher<sensor_msgs::msg::PointCloud2>("vehicle/depth_result", rclcpp::QoS{1});
}

void VehicleSensorSubscriber::on_pose_subscribed(const geometry_msgs::msg::PoseStamped &msg)
{
    tf2::fromMsg(msg.pose, _transform);
}

void VehicleSensorSubscriber::on_range_subscribed(const sensor_msgs::msg::Range &msg)
{
    geometry_msgs::msg::PointStamped res = geometry_msgs::msg::PointStamped();
    res.header = msg.header;
    // Assumed Range Sensor faces front. if faces down, change to {0.0, 0.0, -msg.range}.
    tf2::toMsg(_transform * tf2::Vector3{msg.range, 0.0, 0.0}, res.point);
    _pub_range->publish(res);
}

void VehicleSensorSubscriber::on_lidar_subscribed(const sensor_msgs::msg::PointCloud2 &msg)
{
    sensor_msgs::msg::PointCloud2 res = sensor_msgs::msg::PointCloud2();
    res.header = msg.header;
    res.width = msg.width;
    res.height = msg.height;

    sensor_msgs::PointCloud2Modifier modifier{res};
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(res.width * res.height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x{msg, "x"};
    sensor_msgs::PointCloud2ConstIterator<float> iter_y{msg, "y"};
    sensor_msgs::PointCloud2ConstIterator<float> iter_z{msg, "z"};
    sensor_msgs::PointCloud2Iterator<float> iter_res_x{res, "x"};
    sensor_msgs::PointCloud2Iterator<float> iter_res_y{res, "y"};
    sensor_msgs::PointCloud2Iterator<float> iter_res_z{res, "z"};

    for (int i = 0; i < msg.width * msg.height; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_res_x, ++iter_res_y, ++iter_res_z)
    {
        tf2::Vector3 pt{*iter_x, *iter_y, *iter_z};
        pt = _transform * pt;
        *iter_res_x = pt.x();
        *iter_res_y = pt.y();
        *iter_res_z = pt.z();
    }

    _pub_lidar->publish(res);
}

void VehicleSensorSubscriber::on_depth_subscribed(const sensor_msgs::msg::Image &msg)
{
    sensor_msgs::msg::PointCloud2 res = sensor_msgs::msg::PointCloud2();
    res.header = msg.header;
    res.width = msg.width * msg.height;
    res.height = 1;

    sensor_msgs::PointCloud2Modifier modifier{res};
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(res.width * res.height);

    const float *depth = reinterpret_cast<const float *>(msg.data.data());
    sensor_msgs::PointCloud2Iterator<float> iter_x{res, "x"};
    sensor_msgs::PointCloud2Iterator<float> iter_y{res, "y"};
    sensor_msgs::PointCloud2Iterator<float> iter_z{res, "z"};

    for (int i = 0; i < msg.height; ++i)
    {
        for (int j = 0; j < msg.width; j++, ++iter_x, ++iter_y, ++iter_z, ++depth)
        {
            tf2::Vector3 pt;
            pt.setX(*depth);
            pt.setY(*depth * (_depth_info.k[2] - j) / _depth_info.k[0]);
            pt.setZ(*depth * (_depth_info.k[5] - i) / _depth_info.k[4]);
            pt = _transform * pt;
            *iter_x = pt.x();
            *iter_y = pt.y();
            *iter_z = pt.z();
        }
    }

    _pub_depth->publish(res);
}

void VehicleSensorSubscriber::on_depth_info_subscribed(const sensor_msgs::msg::CameraInfo &msg)
{
    _depth_info = msg;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleSensorSubscriber>());
    rclcpp::shutdown();
    return 0;
}