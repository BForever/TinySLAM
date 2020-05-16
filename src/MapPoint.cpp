//
// Created by 范宏昌 on 2020/1/17.
//

#include "MapPoint.h"
namespace TinySLAM{

uint64_t MapPoint::next_id=0;

MapPoint::MapPoint(cv::Point3d &point,Eigen::Vector3d &norm,cv::Mat &descriptor) {
    this->id = next_id++;
    this->pos(0,0) = point.x;
    this->pos(1,0) = point.y;
    this->pos(2,0) = point.z;
    this->norm = norm;
    this->descriptor = descriptor;
    oberserved_time = 0;
    matched_time = 0;
}
}