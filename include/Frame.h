//
// Created by 范宏昌 on 2020/1/16.
//

#ifndef TOYSLAM_FRAME_H
#define TOYSLAM_FRAME_H

#include "toySLAM.h"
#include "Camera.h"
#include <sophus/se3.h>
#include <opencv2/features2d.hpp>

namespace TinySLAM {
class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Frame> Ptr;
    uint64_t id;
    cv::Mat image;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    Sophus::SE3 Tcw;
    Camera::Ptr mCamera;
    std::string mTimeStamp;
private:
    bool orb_computed= false;

public:
    Frame(cv::Mat &image,Camera::Ptr &camera,std::string TimeStamp="");

    void computeORB();

    inline void set_pose(Sophus::SE3 &pose){
        Tcw = pose;
    }

    inline Eigen::Vector3d get_center_world(){
        return Tcw.inverse().translation();
    }

    bool is_in_frame(const Vec3 &point_world);

public:
    static uint64_t next_id;
};


}

#endif //TOYSLAM_FRAME_H
