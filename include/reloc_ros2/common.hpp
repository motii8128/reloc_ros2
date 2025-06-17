#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace common
{
    typedef Eigen::Matrix<double, 6, 6> mat6x6_t;
    typedef Eigen::Matrix<double, 7, 7> mat7x7_t;
    typedef Eigen::Matrix<double, 9, 9> mat9x9_t;
    typedef Eigen::Matrix<double, 9, 6> mat9x6_t;
    typedef Eigen::Matrix<double, 7, 9> mat7x9_t;
    typedef Eigen::Matrix<double, 9, 7> mat9x7_t;
    typedef Eigen::Matrix<double, 3, 9> mat3x9_t;
    typedef Eigen::Matrix<double, 4, 3> mat4x3_t;
    typedef Eigen::Matrix<double, 9, 3> mat9x3_t;
    typedef Eigen::Matrix<double, 3, 2> mat3x2_t;
    typedef Eigen::Matrix<double, 2, 3> mat2x3_t;
    typedef Eigen::Matrix3d mat3x3_t;
    typedef Eigen::Matrix2d mat2x2_t;
    typedef Eigen::Vector3d vec3_t;
    typedef Eigen::Vector2d vec2_t;
    typedef Eigen::Matrix<double, 7, 1> vec7_t;
    typedef Eigen::Matrix<double, 9, 1> vec9_t;
    typedef Eigen::Quaterniond quat_t;


    quat_t cv2quat(cv::Mat rotation_matrix);
    vec3_t create_vec3(const double& x, const double& y, const double& z);
    quat_t angle2quat(vec3_t& angle);
    quat_t gyro2quat(vec3_t& gyro, const double& delta_time);
    vec3_t quat2angle(const quat_t& q);
    geometry_msgs::msg::PoseStamped createPoseMsg(const vec7_t& v);
}

#endif