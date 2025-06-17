#ifndef EKF_HPP_
#define EKF_HPP_

#include "common.hpp"

using namespace common;

namespace ekf
{

    class EKF
    {    
        public:
        EKF();

        void setEstNoise(const double& accel_var, const double& gyro_var, const double delta_time);
        void predictUpdate(vec3_t& imu_accel,vec3_t& imu_gyro, const double& delta_time);
        void measurementUpdate(const vec7_t& observation, const double& obs_var);

        vec7_t getOdometry();

        private:
        vec3_t x_position_, x_velocity_;
        quat_t x_quat_;
        mat9x9_t cov_;
        mat6x6_t estimation_noise_;
    };

    mat3x3_t skew_symmetric(const vec3_t& v);
}

#endif