#include "reloc_ros2/reloc.hpp"

namespace reloc_ros2
{
    ReLoc::ReLoc(const rclcpp::NodeOptions& options) : Node("reloc", options)
    {
        enable_log_ = this->declare_parameter("enable_log", false);
        initalize_vo_ = false;
        initalize_ekf_ = false;
        success_vo_ = false;
        frame_id_ = this->declare_parameter("frame_id", "map");

        path_ = nav_msgs::msg::Path();
        path_.header.frame_id = frame_id_;
        
        vo_ = std::make_shared<VisualOdometry>();
        visual_odometry_ = common::vec7_t();
        ekf_ = std::make_shared<EKF>();
        posture_ekf_ = std::make_shared<ImuPostureEKF>();
        posture_ = common::quat_t(1.0, 0.0, 0.0, 0.0);

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 0);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 0);

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            rclcpp::SensorDataQoS(),
            std::bind(&ReLoc::imu_callback, this, _1)
        );

        rgbd_subscriber_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
            "/rgbd",
            10,
            std::bind(&ReLoc::rgbd_callback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "Start %s", this->get_name());

        last_ekf_time_ = this->get_clock()->now();
    }

    void ReLoc::rgbd_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg)
    {
        if(!initalize_vo_)
        {
            vo_->setCameraMatrix(
                msg->rgb_camera_info.k[0],
                msg->rgb_camera_info.k[4],
                msg->rgb_camera_info.k[2],
                msg->rgb_camera_info.k[5]
            );

            if(enable_log_)RCLCPP_INFO(this->get_logger(), "set camera info.");

            initalize_vo_ = true;

            return;
        }

        success_vo_ = vo_->compute(
            cv_bridge::toCvCopy(msg->rgb, msg->rgb.encoding)->image,
            cv_bridge::toCvCopy(msg->depth, msg->depth.encoding)->image,
            posture_
        );

        if(success_vo_)
        {
            if(enable_log_)RCLCPP_INFO(this->get_logger(), "compute visual odometry.");

            visual_odometry_ = vo_->getOdometry();
        }
    }

    void ReLoc::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const auto delta_time = this->get_clock()->now() - last_ekf_time_;

        if(!initalize_ekf_)
        {
            ekf_->setEstNoise(
                1.0,
                0.00001,
                1.0
            );

            initalize_ekf_ = true;
        }

        auto accel = vec3_t(
            -1.0*msg->linear_acceleration.z,
            -1.0*msg->linear_acceleration.x,
            msg->linear_acceleration.y
        );

        auto gyro = vec3_t(
            0.0,
            0.0,
            -1.0*msg->angular_velocity.y
        );

        if(abs(gyro.z()) < 0.01)
        {
            gyro(2) = 0.0;
        }
        auto euler_estimation = posture_ekf_->estimate(gyro, accel, delta_time.seconds());
        posture_ = angle2quat(euler_estimation);

        if(success_vo_)
        {
            if(enable_log_)RCLCPP_INFO(this->get_logger(), "compute ekf...");
            ekf_->predictUpdate(accel, gyro, delta_time.seconds());
            ekf_->measurementUpdate(visual_odometry_, 0.25);
        }

        auto pose = common::createPoseMsg(ekf_->getOdometry());
        pose.header.frame_id = frame_id_;
        pose.header.stamp = this->get_clock()->now();

        pose_publisher_->publish(pose);
        path_.poses.push_back(pose);
        path_publisher_->publish(path_);

        last_ekf_time_ = this->get_clock()->now();
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(reloc_ros2::ReLoc)