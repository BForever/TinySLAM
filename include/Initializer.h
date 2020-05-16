//
// Created by 范宏昌 on 2020/1/16.
//

#ifndef TOYSLAM_INITIALIZER_H
#define TOYSLAM_INITIALIZER_H

#include "toySLAM.h"
#include "Frame.h"
#include "Camera.h"
#include "KeyPose.h"
#include "Map.h"
#include <queue>

using namespace std;
namespace TinySLAM {
class Initializer {
private:
    int init_min_good_matches;
    std::vector<cv::DMatch> good_matches;
    std::vector<cv::Point3d> points;
    std::vector<cv::Mat> descrptors;
    Camera::Ptr camera;
    int init_window_size;
    int init_min_translation;
    queue<Frame::Ptr> init_window;
    shared_ptr<Map> mMap;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    cv::Mat R, t;
    KeyPose::Ptr result;

    explicit Initializer(shared_ptr<Camera> &camera,shared_ptr<Map> &map);

    bool add_frame(Frame::Ptr frame);

    void match(Frame &frame1, Frame &frame2);

    bool estimate_pose_2d2d(Frame &frame1, Frame &frame2);

    void triangulate(Frame &frame1, Frame &frame2);

    void complete_keyPose(KeyPose::Ptr &keyPose);

    void clear();
};
}

#endif //TOYSLAM_INITIALIZER_H
