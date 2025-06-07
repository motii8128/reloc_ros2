#ifndef RAILOC_HPP_
#define RAILOC_HPP_

#include "common.hpp"
#include "visual_odometry.hpp"
#include "ekf.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <realsense2_camera_msgs/msg/rgbd.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <string>
#include <cv_bridge/cv_bridge.hpp>

using visual_odometry::VisualOdometry;
using ekf::EKF;

using std::placeholders::_1;

namespace reloc_ros2
{
    class ReLoc : public rclcpp::Node
    {
        public:
        explicit ReLoc(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void rgbd_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg);

        private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr rgbd_subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
        rclcpp::Time last_ekf_time_;
        nav_msgs::msg::Path path_;
        std::shared_ptr<VisualOdometry> vo_;
        common::vec7_t visual_odometry_;
        bool success_vo_;
        std::shared_ptr<EKF> ekf_;

        bool enable_log_, initalize_vo_, initalize_ekf_;
        std::string frame_id_;
    };
}

#endif