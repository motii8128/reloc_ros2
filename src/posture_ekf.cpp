#include "reloc_ros2/posture_ekf.hpp"

namespace posture_ekf
{
    ImuPostureEKF::ImuPostureEKF()
    : estimation_(vec3_t(0.0, 0.0, 0.0)),
    cov_(mat3x3_t::Identity()),
    estimation_noise_(mat3x3_t::Identity()),
    observation_noise_(mat2x2_t::Zero()),
    kalman_gain_(mat3x2_t::Zero())
    {
        cov_(0, 0) = 0.0174*0.001;
        cov_(1, 1) = 0.0174*0.001;
        cov_(2, 2) = 0.0174*0.001;

        estimation_noise_(0, 0) = 0.0174*0.001;
        estimation_noise_(1, 1) = 0.0174*0.001;
        estimation_noise_(2, 2) = 0.0174*0.001;
    }

    vec3_t ImuPostureEKF::estimate(vec3_t &angular, vec3_t &linear_accel, const double &dt)
    {
        const auto input_matrix = getInputMatrix(angular, dt);

        const auto jacob_ = jacob(input_matrix, estimation_);
        estimation_ = predictX(input_matrix, estimation_);
        cov_ = predictCov(jacob_, cov_, estimation_noise_);
        const auto obs_model = obsModel(linear_accel);
        const auto residual = updateResidual(obs_model, estimation_);
        const auto s = updateS(cov_, observation_noise_);
        kalman_gain_ = updateKalmanGain(s, cov_);
        estimation_ = updateX(estimation_, kalman_gain_, residual);
        cov_ = updateCov(kalman_gain_, cov_);

        return estimation_;
    }

    vec3_t getInputMatrix(const vec3_t& angular_velocity, const double &dt)
    {
        return vec3_t(
            angular_velocity.x()* dt,
            angular_velocity.y()* dt,
            angular_velocity.z()* dt
        );
    }

    mat3x2_t h()
    {
        mat3x2_t h_;
        h_.setZero();
        h_(0, 0) = 1.0;
        h_(1, 1) = 1.0;
        
        return h_;
    }

    mat3x3_t jacob(const vec3_t &input_matrix, const vec3_t &estimation)
    {
        auto cos_roll = cos(estimation.x());
        auto sin_roll = sin(estimation.x());
        auto cos_pitch = cos(estimation.y());
        auto sin_pitch = sin(estimation.y());

        auto m_11 = 1.0 + input_matrix.y() * ((cos_roll*sin_pitch)/cos_pitch) - input_matrix.z() * ((sin_roll*sin_pitch)/cos_pitch);
        auto m_12 = input_matrix.y()*(sin_roll/(cos_pitch*cos_pitch))+input_matrix.z()*((cos_roll/(cos_pitch*cos_pitch)));
        auto m_21 = -1.0*input_matrix.y()*sin_roll - input_matrix.z()*cos_roll;
        auto m_31 = input_matrix.y()*(cos_roll/cos_pitch) - input_matrix.z()*(sin_roll/cos_pitch);
        auto m_32 = input_matrix.y()*((sin_roll*sin_pitch)/(cos_pitch*cos_pitch))+input_matrix.z()*((cos_roll*sin_pitch)/(cos_pitch*cos_pitch));

        mat3x3_t mat;
        mat.setZero();
        mat(0, 0) = m_11;
        mat(0, 1) = m_12;
        mat(1, 0) = m_21;
        mat(1, 1) = 1.0;
        mat(2, 0) = m_31;
        mat(2, 1) = m_32;

        return mat;
    }

    vec3_t predictX(const vec3_t &input_matrix, const vec3_t &estimation)
    {
        auto cos_roll = cos(estimation.x());
        auto sin_roll = sin(estimation.x());
        auto cos_pitch = cos(estimation.y());
        auto sin_pitch = sin(estimation.y());

        vec3_t est;
        est(0) = estimation.x() + input_matrix.x() + input_matrix.y()*((sin_roll*sin_pitch)/cos_pitch)+input_matrix.z()*((cos_roll*sin_pitch)/cos_pitch);
        est(1) = estimation.y() + input_matrix.y() * cos_roll - input_matrix.z()*sin_roll;
        est(2) = estimation.z() + input_matrix.z() + input_matrix.y()*(sin_roll/cos_pitch) + input_matrix.z()*(cos_roll/cos_pitch);

        return est;
    }

    mat3x3_t predictCov(const mat3x3_t &jacob, const mat3x3_t &cov, const mat3x3_t &est_noise)
    {
        mat3x3_t t_jacob = jacob.transpose();
        mat3x3_t jacob_cov = jacob * cov;

        mat3x3_t new_cov;
        new_cov.setZero();

        mat3x3_t multiplied = jacob_cov * t_jacob;

        new_cov = multiplied + est_noise;

        return new_cov;
    }

    vec2_t updateResidual(const vec2_t &obs, const vec3_t &est)
    {
        vec2_t result;
        mat2x3_t h_ = h().transpose();
        vec2_t h_est = h_ * est;

        result(0) = obs.x() - h_est.x();
        result(1) = obs.y() - h_est.y();

        return result;
    }

    mat2x2_t updateS(const mat3x3_t &cov_, const mat2x2_t &obs_noise)
    {
        mat2x3_t h_ = h().transpose();
        mat2x3_t h_cov_ = h_ * cov_;
        mat2x2_t convert_cov_ = h_cov_ * h();

        return obs_noise + convert_cov_;
    }

    mat3x2_t updateKalmanGain(const mat2x2_t &s, const mat3x3_t &cov)
    {
        auto h_ = h();

        mat2x2_t inverse_s = s.inverse();

        mat3x2_t cov_and_h = cov * h_;

        return cov_and_h * inverse_s;
    }

    vec3_t updateX(const vec3_t &est, const mat3x2_t &kalman_gain_, const vec2_t &residual)
    {
        vec3_t kalman_res = kalman_gain_ * residual;

        vec3_t result;
        result.setZero();

        result(0) = est.x() + kalman_res.x();
        result(1) = est.y() + kalman_res.y();
        result(2) = est.z() + kalman_res.z();

        return result;
    }

    mat3x3_t updateCov(const mat3x2_t &kalman_gain, const mat3x3_t &cov)
    {
        mat3x3_t i;
        i.setIdentity();

        mat2x3_t h_ = h().transpose();

        mat3x3_t kalman_h = kalman_gain * h_;

        mat3x3_t i_k_h = i - kalman_h;

        return i_k_h * cov;
    }

    vec2_t obsModel(const vec3_t &linear_accel)
    {
        vec2_t model;

        if(linear_accel.z() == 0.0)
        {
            if(linear_accel.y() > 0.0)
            {
                model(0) = acos(-1.0) / 2.0;
            }
            else
            {
                model(0) = -1.0 * acos(-1.0) / 2.0;
            }
        }
        else
        {
            model(0) = atan(linear_accel.y() / linear_accel.z());
        }

        if(sqrt(linear_accel.y()*linear_accel.y() + linear_accel.z()*linear_accel.z()) == 0.0)
        {
            if(-1.0*linear_accel.x() > 0.0)
            {
                model(1) = acos(-1.0) / 2.0;
            }
            else
            {
                model(1) = -1.0*acos(-1.0) / 2.0;
            }
        }
        else
        {
            model(1) = (-1.0 * linear_accel.x()) / atan(sqrt(linear_accel.y()*linear_accel.y() + linear_accel.z()*linear_accel.z()));
        }

        return model;
    }
}