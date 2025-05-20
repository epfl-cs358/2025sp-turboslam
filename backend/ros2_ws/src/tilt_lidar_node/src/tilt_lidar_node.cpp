#include <memory>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "laser_geometry/laser_geometry.hpp"

using std::placeholders::_1;

class TiltLaserNode : public rclcpp::Node {
public:
  TiltLaserNode(const rclcpp::NodeOptions & options)
  : Node("tilt_laser_node", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Topics
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);
    scan_sub_       = create_subscription<sensor_msgs::msg::LaserScan>(
                        "/scan", 10, std::bind(&TiltLaserNode::scanCallback, this, _1));
    servo_sub_      = create_subscription<std_msgs::msg::Int32>(
                        "/lidar_servo_angle", 10, std::bind(&TiltLaserNode::servoCallback, this, _1));

    // 1) Inject static mount: servo_frame → laser_frame
    geometry_msgs::msg::TransformStamped mount_tf;
    mount_tf.header.frame_id    = "servo_frame";
    mount_tf.child_frame_id     = "laser_frame";
    mount_tf.header.stamp       = rclcpp::Time(0);  // static
    mount_tf.transform.translation.x = 0.10;
    mount_tf.transform.translation.y = 0.0;
    mount_tf.transform.translation.z = 0.05;
    tf2::Quaternion q_mount;
    q_mount.setRPY(0,0,0);
    mount_tf.transform.rotation.x = q_mount.x();
    mount_tf.transform.rotation.y = q_mount.y();
    mount_tf.transform.rotation.z = q_mount.z();
    mount_tf.transform.rotation.w = q_mount.w();

    tf_buffer_.setTransform(mount_tf, "static_mount", true);
    RCLCPP_INFO(get_logger(),"Injected static mount servo_frame→laser_frame");

    // 2) Inject initial servo tilt = 0°: world → servo_frame
    geometry_msgs::msg::TransformStamped init_servo;
    init_servo.header.frame_id    = "world";
    init_servo.child_frame_id     = "servo_frame";
    init_servo.header.stamp       = rclcpp::Time(0);
    init_servo.transform.translation.x = 0.0;
    init_servo.transform.translation.y = 0.0;
    init_servo.transform.translation.z = 0.0;
    tf2::Quaternion q0;
    q0.setRPY(0,0,0);
    init_servo.transform.rotation.x = q0.x();
    init_servo.transform.rotation.y = q0.y();
    init_servo.transform.rotation.z = q0.z();
    init_servo.transform.rotation.w = q0.w();

    tf_buffer_.setTransform(init_servo, "static_servo", true);
    RCLCPP_INFO(get_logger(),"Injected static init world→servo_frame @0");

    current_servo_angle_deg_ = 0;
    RCLCPP_INFO(get_logger(),"TiltLaserNode started (servo=0°)");
  }

private:
  void servoCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    current_servo_angle_deg_ = msg->data;
    // we do *not* broadcast here; scans will inject exactly the times they need
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    // start/end times
    rclcpp::Time t_start(scan_msg->header.stamp);
    double dur = scan_msg->time_increment>0
                 ? scan_msg->time_increment * scan_msg->ranges.size()
                 : scan_msg->scan_time;
    rclcpp::Time t_end = t_start + rclcpp::Duration::from_seconds(dur);

    RCLCPP_INFO(get_logger(),"scan@%.3f dur=%.3f→[%.3f]",
                t_start.seconds(), dur, t_end.seconds());

    // inject servo_frame→laser_frame already in buffer (static)
    // inject world→servo_frame at t_start and t_end:
    injectServo(t_start);
    injectServo(t_end);

    // see if we can now chain world→laser_frame at t_end
    if (!tf_buffer_.canTransform("world","laser_frame",t_end,
                                 rclcpp::Duration::from_seconds(0.0)))
    {
      RCLCPP_WARN(get_logger(),
                  "TF chain not ready at %.3f → skipping", t_end.seconds());
      return;
    }

    // high‐fidelity projection
    sensor_msgs::msg::PointCloud2 cloud2;
    try {
      projector_.transformLaserScanToPointCloud(
        "world", *scan_msg, cloud2, tf_buffer_);
      RCLCPP_INFO(get_logger(),"projected %zu points",cloud2.width*cloud2.height);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(),"projection failed: %s", ex.what());
      return;
    }

    // publish with original stamp
    cloud2.header.frame_id = "world";
    cloud2.header.stamp    = scan_msg->header.stamp;
    pointcloud_pub_->publish(cloud2);
    RCLCPP_INFO(get_logger(),"published PointCloud2 @ %.3f",t_start.seconds());
  }

  void injectServo(const rclcpp::Time & stamp) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id    = "world";
    tf_msg.child_frame_id     = "servo_frame";
    tf_msg.header.stamp       = stamp;
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;
    double a = current_servo_angle_deg_ * M_PI/180.0;
    tf2::Quaternion q;
    q.setRPY(0, a, 0);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_buffer_.setTransform(tf_msg, "servo", false);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr  servo_sub_;

  tf2_ros::Buffer           tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  laser_geometry::LaserProjection projector_;
  int current_servo_angle_deg_{0};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<TiltLaserNode>(opts);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


// #include <memory>
// #include <cmath>
// #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/int32.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"

// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2_ros/transform_broadcaster.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"

// #include "laser_geometry/laser_geometry.hpp"

// using std::placeholders::_1;

// class TiltLaserNode : public rclcpp::Node {
// public:
//   TiltLaserNode(const rclcpp::NodeOptions & options)
//   : Node("tilt_laser_node", options)
//   {
//     // Publisher & subscribers
//     pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);
//     scan_sub_       = create_subscription<sensor_msgs::msg::LaserScan>(
//                         "/scan", 10, std::bind(&TiltLaserNode::scanCallback, this, _1));
//     servo_sub_      = create_subscription<std_msgs::msg::Int32>(
//                         "/lidar_servo_angle", 10, std::bind(&TiltLaserNode::servoCallback, this, _1));

//     // TF broadcaster & listener
//     tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
//     tf_buffer_      = std::make_unique<tf2_ros::Buffer>(get_clock());
//     tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//     // Start with 0° tilt
//     current_servo_angle_deg_ = 0;
//     RCLCPP_INFO(get_logger(), "TiltLaserNode started (servo=0°)");
//   }

// private:
//   void servoCallback(const std_msgs::msg::Int32::SharedPtr msg) {
//     // Just update the angle; actual TFs will be stamped in scanCallback
//     current_servo_angle_deg_ = msg->data;
//   }

//   // Broadcast world→servo_frame at given time
//   void publishServoTF(const rclcpp::Time & stamp) {
//     geometry_msgs::msg::TransformStamped tf_msg;
//     tf_msg.header.stamp    = stamp;
//     tf_msg.header.frame_id = "world";
//     tf_msg.child_frame_id  = "servo_frame";
//     tf_msg.transform.translation.x = 0.0;
//     tf_msg.transform.translation.y = 0.0;
//     tf_msg.transform.translation.z = 0.0;
//     double a = current_servo_angle_deg_ * M_PI / 180.0;
//     tf2::Quaternion q;
//     q.setRPY(0.0, a, 0.0);
//     tf_msg.transform.rotation.x = q.x();
//     tf_msg.transform.rotation.y = q.y();
//     tf_msg.transform.rotation.z = q.z();
//     tf_msg.transform.rotation.w = q.w();
//     tf_broadcaster_->sendTransform(tf_msg);
//   }

//   // Broadcast servo_frame→laser_frame at given time (static mount)
//   void publishMountTF(const rclcpp::Time & stamp) {
//     geometry_msgs::msg::TransformStamped tf_msg;
//     tf_msg.header.stamp    = stamp;
//     tf_msg.header.frame_id = "servo_frame";
//     tf_msg.child_frame_id  = "laser_frame";
//     tf_msg.transform.translation.x = 0.10;  // your physical offset
//     tf_msg.transform.translation.y = 0.0;
//     tf_msg.transform.translation.z = 0.05;
//     tf2::Quaternion qi;
//     qi.setRPY(0.0, 0.0, 0.0);
//     tf_msg.transform.rotation.x = qi.x();
//     tf_msg.transform.rotation.y = qi.y();
//     tf_msg.transform.rotation.z = qi.z();
//     tf_msg.transform.rotation.w = qi.w();
//     tf_broadcaster_->sendTransform(tf_msg);
//   }

//   void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
//     // 1) Compute the start and end times of this scan
//     rclcpp::Time t_start(scan_msg->header.stamp);
//     double scan_duration = scan_msg->time_increment > 0.0
//       ? scan_msg->time_increment * scan_msg->ranges.size()
//       : scan_msg->scan_time;
//     rclcpp::Duration span = rclcpp::Duration::from_seconds(scan_duration);
//     rclcpp::Time t_end = t_start + span;

//     RCLCPP_INFO(get_logger(),
//       "scan@%.3f dur=%.3f → [%.3f]",
//       t_start.seconds(), scan_duration, t_end.seconds());

//     // 2) Inject TFs stamped precisely at t_start and t_end
//     publishServoTF(t_start);
//     publishMountTF(t_start);
//     publishServoTF(t_end);
//     publishMountTF(t_end);

//     // 3) Wait/check that the transform chain exists at t_end
//     if (!tf_buffer_->canTransform(
//           "world", "laser_frame", t_end, rclcpp::Duration::from_seconds(0.0)))
//     {
//       RCLCPP_WARN(get_logger(),
//         "TF not ready at %.3f, skipping", t_end.seconds());
//       return;
//     }

//     // 4) Perform the high-fidelity projection
//     sensor_msgs::msg::PointCloud2 cloud2;
//     try {
//       projector_.transformLaserScanToPointCloud(
//         "world", *scan_msg, cloud2, *tf_buffer_);
//       RCLCPP_INFO(get_logger(),
//                   "projected %u points", cloud2.width * cloud2.height);
//     } catch (tf2::TransformException & ex) {
//       RCLCPP_ERROR(get_logger(),
//                    "projection failed: %s", ex.what());
//       return;
//     }

//     // 5) Publish
//     cloud2.header.frame_id = "world";
//     cloud2.header.stamp    = scan_msg->header.stamp;
//     pointcloud_pub_->publish(cloud2);
//     RCLCPP_INFO(get_logger(),
//                 "published PointCloud2 @ %.3f", t_start.seconds());
//   }

//   // Members
//   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr servo_sub_;

//   std::unique_ptr<tf2_ros::TransformBroadcaster>        tf_broadcaster_;
//   std::unique_ptr<tf2_ros::Buffer>                      tf_buffer_;
//   std::shared_ptr<tf2_ros::TransformListener>           tf_listener_;

//   laser_geometry::LaserProjection projector_;
//   int current_servo_angle_deg_{0};
// };

// int main(int argc, char ** argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<TiltLaserNode>(rclcpp::NodeOptions());
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

/////////////////////////////////////////////////////////////

// #include <memory>
// #include <cmath>
// #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/int32.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"

// #include "tf2/LinearMath/Quaternion.h"
// #include "tf2_ros/transform_broadcaster.h"
// #include "tf2_ros/static_transform_broadcaster.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"

// #include "laser_geometry/laser_geometry.hpp"

// using std::placeholders::_1;

// class TiltLaserNode : public rclcpp::Node {
// public:
//   TiltLaserNode()
//   : Node("tilt_laser_node")
//   {
//     // Create publishers and subscribers
//     pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);
//     scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//         "/scan", 10, std::bind(&TiltLaserNode::scanCallback, this, _1));
//     servo_sub_ = this->create_subscription<std_msgs::msg::Int32>(
//         "/lidar_servo_angle", 10, std::bind(&TiltLaserNode::servoCallback, this, _1));

//     // Initialize TF broadcasters (dynamic and static)
//     tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
//     tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

//     // Set up TF listener (to receive transforms into our buffer)
//     tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//     // Publish the static transform from servo_frame to laser_frame (mounting offset)
//     geometry_msgs::msg::TransformStamped static_tf;
//     static_tf.header.stamp = this->get_clock()->now();
//     // static_tf.header.stamp = rclcpp::Time(0); // TODO: DEBUG
//     static_tf.header.frame_id = "servo_frame";
//     static_tf.child_frame_id  = "laser_frame";
//     // Example fixed offset: LiDAR is 0.1 m ahead of the servo and 0.05 m upward
//     static_tf.transform.translation.x = 0.10;
//     static_tf.transform.translation.y = 0.0;
//     static_tf.transform.translation.z = 0.05;
//     // Orientation: assume LiDAR is oriented same as servo (no additional rotation)
//     tf2::Quaternion q_identity;
//     q_identity.setRPY(0, 0, 0);
//     static_tf.transform.rotation.x = q_identity.x();
//     static_tf.transform.rotation.y = q_identity.y();
//     static_tf.transform.rotation.z = q_identity.z();
//     static_tf.transform.rotation.w = q_identity.w();
//     // Broadcast the static transform (latched, so this will be available permanently)
//     tf_static_broadcaster_->sendTransform(static_tf);

//     // If needed, initialize an initial servo angle (assume 0 degrees) transform
//     current_servo_angle_deg_ = 0;
//     publishServoTransform(current_servo_angle_deg_);
//     RCLCPP_INFO(this->get_logger(), "TiltLaserNode started (initial servo angle %d deg).", current_servo_angle_deg_);
//   }

// private:
//   void servoCallback(const std_msgs::msg::Int32::SharedPtr msg)
//   {
//     // Update current servo angle and broadcast a new transform
//     current_servo_angle_deg_ = msg->data;
//     publishServoTransform(current_servo_angle_deg_);
//   }

//   void publishServoTransform(int angle_deg)
//   {
//     // Prepare the TransformStamped for world -> servo_frame
//     geometry_msgs::msg::TransformStamped tf_msg;
//     tf_msg.header.stamp = this->get_clock()->now();    // current time
//     tf_msg.header.frame_id = "world";
//     tf_msg.child_frame_id  = "servo_frame";
//     // Set translation (assuming servo pivot is at world origin for simplicity)
//     tf_msg.transform.translation.x = 0.0;
//     tf_msg.transform.translation.y = 0.0;
//     tf_msg.transform.translation.z = 0.0;
//     // Compute rotation: tilt about Y-axis by angle (in radians)
//     double angle_rad = angle_deg * M_PI / 180.0;
//     tf2::Quaternion q;
//     q.setRPY(0.0, angle_rad, 0.0);  // roll=0, pitch=angle_rad (Y-axis), yaw=0
//     tf_msg.transform.rotation.x = q.x();
//     tf_msg.transform.rotation.y = q.y();
//     tf_msg.transform.rotation.z = q.z();
//     tf_msg.transform.rotation.w = q.w();
//     // Broadcast the dynamic transform
//     tf_broadcaster_->sendTransform(tf_msg);
//   }

//   void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
//   {
//     // Frame of incoming laser scan (should be "laser_frame")
//     std::string laser_frame = scan_msg->header.frame_id;
//     // Time of first measurement in scan
//     rclcpp::Time start_time = scan_msg->header.stamp;
//     // Compute time of last measurement in the scan
//     // last_time = start_time + scan.time_increment * (number of readings)
//     double scan_duration = scan_msg->time_increment * scan_msg->ranges.size();
//     rclcpp::Duration scan_time_span = rclcpp::Duration::from_seconds(scan_duration);
//     rclcpp::Time end_time = start_time + scan_time_span;

//     // Wait (up to 0.5s) for the transform at the end of the scan to be available
//     const double timeout_seconds = 0.5;
//     bool can_transform = tf_buffer_->canTransform(
//         "world",  // target frame
//         laser_frame,   // source frame
//         end_time,      // time at end of scan
//         rclcpp::Duration::from_seconds(timeout_seconds) );
//     if (!can_transform) {
//       RCLCPP_WARN(this->get_logger(),
//                   "No transform available from %s to world at scan time (t=%.2f sec). Skipping point cloud.",
//                   laser_frame.c_str(), end_time.seconds());
//       return;
//     }

//     // Use laser_geometry to project the LaserScan into a PointCloud2 in world
//     sensor_msgs::msg::PointCloud2 cloud_out;
//     try {
//       projector_.transformLaserScanToPointCloud("world", *scan_msg, cloud_out, *tf_buffer_);
//     } catch (const tf2::TransformException &ex) {
//       RCLCPP_ERROR(this->get_logger(), "Error transforming LaserScan to point cloud: %s", ex.what());
//       return;
//     }
//     // Optionally, we can set the timestamp of the output cloud (e.g., to the scan's end time or start time)
//     cloud_out.header.stamp = scan_msg->header.stamp;  // use start of scan time
//     cloud_out.header.frame_id = "world";

//     // Publish the point cloud
//     RCLCPP_INFO(this->get_logger(), "Publishing point cloud");
//     pointcloud_pub_->publish(cloud_out);
//   }

//   // Subscribers and publisher
//   rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr servo_sub_;
//   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
//   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

//   // TF broadcaster(s) and listener
//   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//   std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
//   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

//   // Laser projector for converting scans to point clouds
//   laser_geometry::LaserProjection projector_;

//   // Stored current servo angle (degrees)
//   int current_servo_angle_deg_;
// };

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<TiltLaserNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }