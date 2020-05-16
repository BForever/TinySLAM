//
// Created by 范宏昌 on 2020/1/17.
//

#ifndef TOYSLAM_VISUALODOMETRY_H
#define TOYSLAM_VISUALODOMETRY_H

#include <sophus/se3.h>
#include <opencv2/viz.hpp>
#include <queue>
#include "Frame.h"
#include "Camera.h"
#include "Initializer.h"
#include "Map.h"
using namespace std;
using namespace cv;
using namespace Sophus;


namespace TinySLAM {
class VisualOdometry {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef enum {
        UNINITIALIZED,
        LOST,
        TRACKING
    } STATE;

    Map::Ptr mMap;
    STATE mState;
    int lost_count=0;
    shared_ptr<Camera> mCamera;

private:
    shared_ptr<Initializer>mInitializer;
    vector<cv::DMatch> matches;
    vector<cv::DMatch> good_matches;
    vector<MapPoint::Ptr> map_points;
    Mat map_descrptors;
    Frame::Ptr frame1,frame2;
    KeyPose::Ptr keyPose1,keyPose2;

    vector<MapPoint::Ptr> pts_3d_ptr;
    set<int> pts_2d_index;
    std::vector<cv::Point3f> pts_3d;
    std::vector<cv::Point2f> pts_2d;
    std::vector<cv::KeyPoint> pts_2dkp;

    Mat rvec, tvec, R, inliers;
    SE3 T_c_w_estimated;
    bool trust;
    int max_translation,min_translation;

    int trajectory_count = 0;

    void match_with_map();
    bool estimate_pose();
    bool bundle_adjustment();
    void match_two_frame();
    void triangulation();
    void clear();

public:
    explicit VisualOdometry(shared_ptr<Camera> &camera);

    int track(shared_ptr<Frame> &frame);

    void draw_image(KeyPose::Ptr keyPose);

    void save_trajectory(string filename);

public:
    KeyPose::Ptr estimate_pose_3d2d(KeyPose::Ptr &keyPose1, Frame::Ptr &frame2);
};

}
#endif //TOYSLAM_VISUALODOMETRY_H
