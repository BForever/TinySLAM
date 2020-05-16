//
// Created by 范宏昌 on 2020/1/17.
//

#ifndef TOYSLAM_MAPPOINT_H
#define TOYSLAM_MAPPOINT_H

#include "Frame.h"
#include "KeyPose.h"
#include <opencv2/opencv.hpp>
#include <opencv/cv.hpp>
#include <Eigen/Geometry>

using namespace std;

namespace TinySLAM {
class KeyPose;
class MapPoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef shared_ptr<MapPoint> Ptr;
    static uint64_t next_id;
    uint64_t id;
    cv::Mat descriptor;
    vector<shared_ptr<KeyPose>> observers;
    Eigen::Vector3d pos;
    Eigen::Vector3d norm;
    int matched_time;
    int oberserved_time;

    MapPoint(cv::Point3d &point,Eigen::Vector3d &norm,cv::Mat &descriptor);

    inline cv::Point3f get_pos_CV() const{
        return cv::Point3f(pos(0,0),pos(1,0),pos(2,0));
    }
};

}
#endif //TOYSLAM_MAPPOINT_H
