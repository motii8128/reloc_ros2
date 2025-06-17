#ifndef MOTISLAM_POSTUER_EKF_HPP_
#define MOTISLAM_POSTUER_EKF_HPP_

#include "common.hpp"

using namespace common;

namespace posture_ekf
{
    class ImuPostureEKF
    {
        public:
        /// @brief コンストラクタ
        ImuPostureEKF();

        /// @brief 加速度と角速度から姿勢を推定する
        /// @param angular 角速度[rad]
        /// @param linear_accel 加速度[m/s^2]
        /// @param dt 時間間隔
        /// @return オイラー角(roll, pitch, yaw)
        vec3_t estimate(vec3_t &angular, vec3_t &linear_accel, const double &dt);

        private:
        mat3x3_t cov_;
        mat3x3_t estimation_noise_;
        mat2x2_t observation_noise_;
        mat3x2_t kalman_gain_;
        vec3_t estimation_;
    };
    vec3_t getInputMatrix(const vec3_t& angular_velocity, const double &dt);

    mat3x2_t h();

    mat3x3_t jacob(const vec3_t &input_matrix, const vec3_t &estimation);

    vec3_t predictX(const vec3_t &input_matrix, const vec3_t &estimation);

    mat3x3_t predictCov(const mat3x3_t &jacob, const mat3x3_t &cov, const mat3x3_t &est_noise);

    vec2_t updateResidual(const vec2_t &obs, const vec3_t &est);

    mat2x2_t updateS(const mat3x3_t &cov_, const mat2x2_t &obs_noise);

    mat3x2_t updateKalmanGain(const mat2x2_t &s, const mat3x3_t &cov);

    vec3_t updateX(const vec3_t &est, const mat3x2_t &kalman_gain_, const vec2_t &residual);

    mat3x3_t updateCov(const mat3x2_t &kalman_gain, const mat3x3_t &cov);

    vec2_t obsModel(const vec3_t &linear_accel);
}

#endif