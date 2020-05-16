//
// Created by 范宏昌 on 2020/1/16.
//

#ifndef TOYSLAM_TOYSLAM_H
#define TOYSLAM_TOYSLAM_H

#include <cstdint>
#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>

namespace TinySLAM{


typedef Eigen::Matrix<double,3,3> Mat33;

typedef Eigen::Matrix<double,3,1> Vec3;
typedef Eigen::Matrix<double,2,1> Vec2;

}


#endif //TOYSLAM_TOYSLAM_H
