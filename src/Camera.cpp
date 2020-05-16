//
// Created by 范宏昌 on 2020/1/16.
//

#include "Camera.h"
#include "Config.h"

namespace TinySLAM {

Camera::Camera() {
    fx_ = Config::get<double>("camera.fx");
    fy_ = Config::get<double>("camera.fy");
    cx_ = Config::get<double>("camera.cx");
    cy_ = Config::get<double>("camera.cy");
    k1 = Config::get<double>("camera.k1");
    k2 = Config::get<double>("camera.k2");
    p1 = Config::get<double>("camera.p1");
    p2 = Config::get<double>("camera.p2");
//    k3 = Config::get<double>("camera.k3");
}

Vec3 Camera::world2camera(const Vec3 &p_w, const Sophus::SE3 &T_c_w) {
    return T_c_w * p_w;
}

Vec3 Camera::camera2world(const Vec3 &p_c, const Sophus::SE3 &T_c_w) {
    return T_c_w.inverse() * p_c;
}

Vec2 Camera::camera2pixel(const Vec3 &p_c) {
    return Vec2(
            fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
            fy_ * p_c(1, 0) / p_c(2, 0) + cy_
    );
}

cv::Point2f Camera::pixel2camera(const cv::Point2f &p_p) {
    cv::Point2f point2D;
    point2D.x = (p_p.x - cx_) / fx_;
    point2D.y = (p_p.y - cy_) / fy_;
    return point2D;
}

Vec2 Camera::world2pixel(const Vec3 &p_w, const Sophus::SE3 &T_c_w) {
    return camera2pixel(world2camera(p_w, T_c_w));
}


}