//
// Created by 范宏昌 on 2020/1/16.
//

#ifndef TOYSLAM_CAMERA_H
#define TOYSLAM_CAMERA_H

#include <sophus/se3.h>
#include <toySLAM.h>
#include <memory>
#include "Config.h"

namespace TinySLAM {

// Pinhole stereo camera model
class Camera {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0, k1 = 0, k2 = 0, p1 = 0, p2 = 0,k3 = 0,
            baseline_ = 0;  // Camera intrinsics
    Sophus::SE3 pose_;             // extrinsic, from stereo camera to single camera
    Sophus::SE3 pose_inv_;         // inverse of extrinsics

    Camera();

    Camera(double fx, double fy, double cx, double cy, double baseline,
           const Sophus::SE3 &pose)
            : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
        pose_inv_ = pose_.inverse();
    }

    Sophus::SE3 pose() const { return pose_; }

    // return intrinsic matrix
    Mat33 K() const {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }

    cv::Mat K_CV() const {
        cv::Mat k = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
        return k;
    }
    cv::Mat D_CV() const {
        cv::Mat d = (cv::Mat_<double>(5, 1) << k1,k2,p1,p2,k3);
        return d;
    }

    // coordinate transform: world, camera, pixel
    Vec3 world2camera(const Vec3 &p_w, const Sophus::SE3 &T_c_w);

    Vec3 camera2world(const Vec3 &p_c, const Sophus::SE3 &T_c_w);

    Vec2 camera2pixel(const Vec3 &p_c);

    cv::Point2f pixel2camera(const cv::Point2f &p_p);

    Vec2 world2pixel(const Vec3 &p_w, const Sophus::SE3 &T_c_w);
};
}

#endif //TOYSLAM_CAMERA_H
