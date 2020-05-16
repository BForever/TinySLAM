//
// Created by 范宏昌 on 2020/2/16.
//

#include <iostream>
#include "Converter.h"
namespace TinySLAM{
cv::Mat Converter::Matrix3dtocvMat(const Eigen::Matrix3d &mat) {
    return (cv::Mat_<float>(3, 3) <<
            mat(0, 0), mat(0, 1), mat(0, 2),
            mat(1, 0), mat(1, 1), mat(1, 2),
            mat(2, 0), mat(2, 1), mat(2, 2)
    );
}
cv::Mat Converter::Matrix4dtocvMat(const Eigen::Matrix4d &mat) {
    return (cv::Mat_<float>(3, 4) <<
            mat(0, 0), mat(0, 1), mat(0, 2),mat(0, 3),
            mat(1, 0), mat(1, 1), mat(1, 2),mat(1, 3),
            mat(2, 0), mat(2, 1), mat(2, 2),mat(2, 3)
    );
}

Eigen::Matrix3d Converter::cvMattoMatrix3d(cv::Mat &mat) {
    Eigen::Matrix3d res;
    res<<
       mat.at<double>(0, 0), mat.at<double>(0, 1), mat.at<double>(0, 2),
       mat.at<double>(1, 0), mat.at<double>(1, 1), mat.at<double>(1, 2),
       mat.at<double>(2, 0), mat.at<double>(2, 1), mat.at<double>(2, 2);
    return res;
}

Eigen::Vector3d Converter::cvMattoVector3d(cv::Mat &mat) {
    Eigen::Vector3d res;
    res<<
        mat.at<double>(0, 0),
        mat.at<double>(1, 0),
        mat.at<double>(2, 0);
    return res;
}

cv::Mat Converter::combine_pose(cv::Mat &R, cv::Mat &t) {
    cv::Mat pose = cv::Mat(4,4,CV_64F);
    pose.at<double>(0,0)=R.at<double>(0,0);pose.at<double>(0,1)=R.at<double>(0,1);pose.at<double>(0,2)=R.at<double>(0,2);
    pose.at<double>(1,0)=R.at<double>(1,0);pose.at<double>(1,1)=R.at<double>(1,1);pose.at<double>(1,2)=R.at<double>(1,2);
    pose.at<double>(2,0)=R.at<double>(2,0);pose.at<double>(2,1)=R.at<double>(2,1);pose.at<double>(2,2)=R.at<double>(2,2);
    pose.at<double>(0,3)=t.at<double>(0,0);pose.at<double>(1,3)=t.at<double>(1,0);pose.at<double>(2,3)=t.at<double>(2,0);
    pose.at<double>(3,3)=1;
    return pose;
}
std::vector<float> Converter::toQuaternion(Eigen::Matrix3d &eigMat)
{
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}
}

