//
// Created by 范宏昌 on 2020/1/17.
//

#include "KeyPose.h"
using namespace Eigen;
namespace TinySLAM {

uint64_t KeyPose::next_id=0;

void KeyPose::add_point(MapPoint* keyPoint) {
    points.push_back(shared_ptr<MapPoint>(keyPoint));
    keyPoint->observers.push_back(shared_ptr<KeyPose>(this));
}

void KeyPose::set_pose(Mat R, Mat t) {
    Vector3d new_t(t.at<double>(0, 0),\
                        t.at<double>(1, 0), \
                        t.at<double>(2, 0));
    Matrix3d new_R;
    new_R << R.at<double>(0, 0), R.at<double>(0, 1), \
                        R.at<double>(0, 2),R.at<double>(1, 0), \
                        R.at<double>(1, 1), R.at<double>(1,2), \
                        R.at<double>(2, 0), R.at<double>(2, 1), \
                        R.at<double>(2,2);
    Sophus::SE3 new_pose(new_R, new_t);
    this->mpFrame->set_pose(new_pose);
}

void KeyPose::set_pose(Sophus::SE3 &pose) {
    this->mpFrame->set_pose(pose);
}
}