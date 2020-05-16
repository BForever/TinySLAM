//
// Created by 范宏昌 on 2020/1/17.
//

#ifndef TOYSLAM_KEYPOSE_H
#define TOYSLAM_KEYPOSE_H

#include "MapPoint.h"
#include "Frame.h"
#include <vector>
#include <sophus/se3.h>
#include <map>


using namespace std;
using namespace cv;
namespace TinySLAM {
class MapPoint;
class Frame;

class KeyPose {
public:
    static uint64_t next_id;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef shared_ptr<KeyPose> Ptr;

    uint64_t id;
    Frame::Ptr mpFrame;
    vector<shared_ptr<MapPoint>> points;

    KeyPose(Frame::Ptr frame) : mpFrame(frame) { id = next_id++; }

    void add_point(MapPoint *keyPoint);

    void set_pose(Mat R, Mat t);

    void set_pose(Sophus::SE3 &pose);

    inline Sophus::SE3 get_pose(){
        return mpFrame->Tcw;
    }

};
}

#endif //TOYSLAM_KEYPOSE_H
