// File: src/laser_to_pointcloud.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

class LaserToPointCloudNode : public rclcpp::Node
{
public:
  LaserToPointCloudNode()
      : Node("laser_to_pointcloud_node"), projector_()
  {
    // Subscriber to LaserScan on /scan
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(10),
        std::bind(&LaserToPointCloudNode::scan_callback, this, std::placeholders::_1));
    // Publisher for PointCloud2
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    // 1. Project LaserScan into a PointCloud2 in the LIDAR ('laser') frame
    sensor_msgs::msg::PointCloud2 cloud_msg;
    projector_.projectLaser(*scan_msg, cloud_msg);

    // 2. Simulate a tilt: rotate points around Y-axis by tilt_angle
    //    (you can make tilt_angle vary over time or set it constant)
    double tilt_angle = 0.1; // radians (static example)
    // Example for oscillation: tilt_angle = 0.2 * sin(this->now().seconds());

    // Convert ROS cloud to PCL for easy transformation
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(cloud_msg, pcl_cloud);

    // Create an Eigen transform for rotation about Y axis
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(tilt_angle, Eigen::Vector3f::UnitY()));

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_out;
    pcl::transformPointCloud(pcl_cloud, pcl_cloud_out, transform);

    // Convert back to ROS PointCloud2
    sensor_msgs::msg::PointCloud2 cloud_out;
    pcl::toROSMsg(pcl_cloud_out, cloud_out);
    cloud_out.header.frame_id = "base_link"; // now in base_link frame after tilt
    cloud_out.header.stamp = scan_msg->header.stamp;
    cloud_pub_->publish(cloud_out);
  }

  laser_geometry::LaserProjection projector_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserToPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}
