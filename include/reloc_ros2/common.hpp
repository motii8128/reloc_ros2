#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace common
{
    typedef Eigen::Matrix<float, 6, 6> mat6x6_t;
    typedef Eigen::Matrix<float, 9, 9> mat9x9_t;
    typedef Eigen::Matrix<float, 9, 6> mat9x6_t;
    typedef Eigen::Matrix<float, 3, 9> mat3x9_t;
    typedef Eigen::Matrix<float, 9, 3> mat9x3_t;
    typedef Eigen::Matrix3f mat3x3_t;
    typedef Eigen::Vector3f vec3_t;
    typedef Eigen::Matrix<float, 7, 1> vec7_t;
    typedef Eigen::Matrix<float, 9, 1> vec9_t;
    typedef Eigen::Quaternionf quat_t;


    quat_t cv2quat(cv::Mat rotation_matrix);
    vec3_t create_vec3(const float& x, const float& y, const float& z);
    quat_t angle2quat(vec3_t& angle);
    quat_t gyro2quat(vec3_t& gyro, const float& delta_time);
    geometry_msgs::msg::PoseStamped createPoseMsg(const vec7_t& v);
}

#endif