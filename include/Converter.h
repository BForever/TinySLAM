//
// Created by 范宏昌 on 2020/2/16.
//

#ifndef TOYSLAM_CONVERTER_H
#define TOYSLAM_CONVERTER_H

#include <opencv/cv.hpp>
#include <Eigen/Geometry>

namespace TinySLAM{
typedef Eigen::Matrix<double, 6, 1> Vec6;

class Converter {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static cv::Mat Matrix3dtocvMat(const Eigen::Matrix3d &mat);
    static cv::Mat Matrix4dtocvMat(const Eigen::Matrix4d &mat);
    static Eigen::Matrix3d cvMattoMatrix3d(cv::Mat &mat);
    static Eigen::Vector3d cvMattoVector3d(cv::Mat &mat);
    static cv::Mat combine_pose(cv::Mat &R,cv::Mat &t);
    static std::vector<float> toQuaternion(Eigen::Matrix3d &eigMat);
};
}



#endif //TOYSLAM_CONVERTER_H
