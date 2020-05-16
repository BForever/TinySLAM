//
// Created by 范宏昌 on 2020/5/14.
//

#ifndef TOYSLAM_MAPVISUALIZER_H
#define TOYSLAM_MAPVISUALIZER_H

#include <opencv2/viz.hpp>
#include <sophus/se3.h>
#include <MapPoint.h>

using namespace cv;
namespace TinySLAM{

class MapVisualizer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef shared_ptr<MapVisualizer> Ptr;
    // 可视化
    viz::Viz3d vis;
    viz::WCoordinateSystem world_coor,camera_coor;
    Point3d camera_pos,camera_fp,camera_y_dir;
    cv::Affine3d camera_pose;
    KeyPose::Ptr last_keyPose;

    MapVisualizer();

    void update(Sophus::SE3 Tcw);

    void add_map_point(MapPoint::Ptr &mapPoint);

    void add_key_pose(KeyPose::Ptr &keyPose);

    void remove_map_point(MapPoint::Ptr &mapPoint);

    void clear();
};
}

#endif //TOYSLAM_MAPVISUALIZER_H
