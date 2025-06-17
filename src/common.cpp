#include "reloc_ros2/common.hpp"

namespace common
{
    quat_t cv2quat(cv::Mat rotation_matrix)
    {
        mat3x3_t mat = mat3x3_t::Zero();

        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                mat(i, j) = rotation_matrix.at<double>(i,j);
            }
        }

        quat_t q(mat);

        return q;
    }

    vec3_t create_vec3(const double& x, const double& y, const double& z)
    {
        return vec3_t(x,y,z);
    }

    quat_t angle2quat(vec3_t& angle)
    {
        Eigen::AngleAxisd aa(angle.norm(), angle.normalized());

        return quat_t(aa);
    }

    quat_t gyro2quat(vec3_t& gyro, const double& delta_time)
    {
        Eigen::AngleAxisd aa(delta_time * gyro.norm(), gyro.normalized());

        return quat_t(aa);
    }

    vec3_t quat2angle(const quat_t& q)
    {
        quat_t q_norm = q.normalized();
        Eigen::AngleAxisd angle_axis(q_norm);
        return angle_axis.axis() * angle_axis.angle();
    }

    geometry_msgs::msg::PoseStamped createPoseMsg(const vec7_t& v)
    {
        auto p = geometry_msgs::msg::PoseStamped();
        p.pose.position.x = v(0);
        p.pose.position.y = v(1);
        p.pose.position.z = v(2);

        p.pose.orientation.x = v(3);
        p.pose.orientation.y = v(4);
        p.pose.orientation.z = v(5);
        p.pose.orientation.w = v(6);

        return p;
    }
}