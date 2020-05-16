//
// Created by 范宏昌 on 2020/2/24.
//

#ifndef TOYSLAM_MAP_H
#define TOYSLAM_MAP_H

#include "KeyPose.h"
#include "MapPoint.h"
#include "MapVisualizer.h"
using namespace std;
namespace TinySLAM {

class Map {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef shared_ptr<Map> Ptr;

    unsigned long latest_pose_num=0;
    unsigned long latest_point_num=0;
    unordered_map<unsigned long,KeyPose::Ptr>poses;
    unordered_map<unsigned long,MapPoint::Ptr>points;

    bool mbVisual= false;
    MapVisualizer::Ptr mMapVisualizer;

    explicit Map(bool visualize){
        mbVisual = visualize;
        if(mbVisual){
            mMapVisualizer = MapVisualizer::Ptr(new MapVisualizer());
        }
    }

    void add_pose(KeyPose::Ptr &keyPose);
    void add_point(MapPoint::Ptr &keyPoint);
    KeyPose::Ptr get_latest_keypose();

    void optimize(KeyPose::Ptr &keyPose);
    void refresh_visual(Sophus::SE3 camera_pose);

    void clear();

    double map_point_erase_ratio =0;
};


}
#endif //TOYSLAM_MAP_H
