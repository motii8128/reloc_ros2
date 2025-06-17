#include "reloc_ros2/visual_odometry.hpp"

namespace visual_odometry
{
    VisualOdometry::VisualOdometry()
    {
        odom_ = rgbd::RgbdICPOdometry::create();
        odom_->setMinDepth(0.6);
        odom_->setMaxDepth(6.0);

        update_pose_ = Mat::eye(4,4,CV_32F);

        pose_ = common::vec7_t::Zero();
        initalized_ = false;
    }

    void VisualOdometry::setCameraMatrix(float fx, float fy, float cx, float cy)
    {
        Mat camera_matrix = Mat::zeros(3,3,CV_32F);
        camera_matrix.at<float>(0,0) = fx;
        camera_matrix.at<float>(0,2) = cx;
        camera_matrix.at<float>(1,1) = fy;
        camera_matrix.at<float>(1,2) = cy;
        camera_matrix.at<float>(2,2) = 1.0;

        odom_->setCameraMatrix(camera_matrix);
    }

    bool VisualOdometry::compute(Mat& rgb_image, Mat& depth_image)
    {
        odom_->setTransformType(cv::rgbd::RgbdOdometry::RIGID_BODY_MOTION);
        Mat gray_image;
        cv::cvtColor(rgb_image, gray_image, COLOR_BGR2GRAY);

        depth_image.convertTo(depth_image, CV_32FC1, 1.0 / 1000.0);

        if(!initalized_)
        {
            gray_image.copyTo(last_rgb_image_);
            depth_image.copyTo(last_depth_image_);

            initalized_ = true;

            return true;
        }

        Mat Rt;

        const auto check = odom_->compute(last_rgb_image_, last_depth_image_, Mat(), gray_image, depth_image, Mat(), Rt);

        if(check)
        {
            Rt.convertTo(Rt, CV_32F);
            update_pose_ = update_pose_ * Rt.inv();

            pose_ =  result2vec7(update_pose_);

            gray_image.copyTo(last_rgb_image_);
            depth_image.copyTo(last_depth_image_);

            return true;
        }
        else
        {
            return false;
        }
    }

    bool VisualOdometry::compute(Mat& rgb_image, Mat& depth_image, common::quat_t posture)
    {
        odom_->setTransformType(cv::rgbd::RgbdOdometry::TRANSLATION);
        Mat gray_image;
        cv::cvtColor(rgb_image, gray_image, COLOR_BGR2GRAY);

        depth_image.convertTo(depth_image, CV_32FC1, 1.0 / 1000.0);

        if(!initalized_)
        {
            gray_image.copyTo(last_rgb_image_);
            depth_image.copyTo(last_depth_image_);

            initalized_ = true;

            return true;
        }

        Mat Rt;

        Mat initRt = Mat::eye(4,4, CV_64F);
        common::mat3x3_t R_imu = posture.toRotationMatrix();
        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                initRt.at<double>(i,j) = R_imu(i,j);
            }
        }

        const auto check = odom_->compute(last_rgb_image_, last_depth_image_, Mat(), gray_image, depth_image, Mat(), Rt, initRt);

        if(check)
        {
            Rt.convertTo(Rt, CV_32F);
            update_pose_ = update_pose_ * Rt.inv();

            pose_ =  result2vec7(update_pose_);

            gray_image.copyTo(last_rgb_image_);
            depth_image.copyTo(last_depth_image_);

            return true;
        }
        else
        {
            return false;
        }
    }

    common::vec7_t VisualOdometry::getOdometry()
    {
        return pose_;
    }

    common::vec7_t result2vec7(Mat mat)
    {
        tf2::Matrix3x3 rotation(
            mat.at<float>(0,0),mat.at<float>(0,1),mat.at<float>(0,2),
            mat.at<float>(1,0),mat.at<float>(1,1),mat.at<float>(1,2),
            mat.at<float>(2,0),mat.at<float>(2,1),mat.at<float>(2,2)
        );

        tf2::Quaternion q;
        double r,p,y;
        rotation.getRPY(r,p,y);
        q.setEuler(0.0, 0.0, -1.0*p);

        common::vec7_t v;
        v(0) = mat.at<float>(2,3);
        v(1) = -1.0*mat.at<float>(0,3);
        v(2) = -1.0*mat.at<float>(1,3);
        v(3) = q.x();
        v(4) = q.y();
        v(5) = q.z();
        v(6) = q.w();

        return v;
    }
}