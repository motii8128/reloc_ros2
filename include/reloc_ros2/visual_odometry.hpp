#ifndef VISUAL_ODOMETRY_HPP_
#define VISUAL_ODOMETRY_HPP_

#include "common.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace visual_odometry
{
    using namespace cv;
    
    class VisualOdometry
    {
        public:
        VisualOdometry();

        void setCameraMatrix(float fx, float fy, float cx, float cy);
        bool compute(Mat& rgb_image, Mat& depth_image);
        bool compute(Mat& rgb_image, Mat& depth_image, common::quat_t posture);
        common::vec7_t getOdometry();

        private:
        Ptr<rgbd::RgbdICPOdometry> odom_;
        cv::Mat last_rgb_image_, last_depth_image_;
        bool initalized_;
        common::vec7_t pose_;
        Mat update_pose_;
    };

    common::vec7_t result2vec7(Mat mat);
}

#endif