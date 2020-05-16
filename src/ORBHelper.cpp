//
// Created by 范宏昌 on 2020/1/17.
//

#include "ORBHelper.h"
#include "opencv2/xfeatures2d.hpp"

namespace TinySLAM {

std::shared_ptr<ORBHelper> ORBHelper::orbhelper = nullptr;

ORBHelper::ORBHelper() {
//    orb = cv::ORB::create(1000);
    orb =cv::xfeatures2d::SURF::create(100,4,3, true) ;
}

void ORBHelper::detect(const cv::_InputArray &image, std::vector<cv::KeyPoint> &keypoints) {
    if (orbhelper == nullptr) {
        orbhelper = std::shared_ptr<ORBHelper>(new ORBHelper);
    }
    orbhelper->orb->detect(image, keypoints);
}

void
ORBHelper::compute(const cv::_InputArray &image, std::vector<cv::KeyPoint> &keypoints, const cv::_OutputArray &descriptors) {
    if (orbhelper == nullptr) {
        orbhelper = std::shared_ptr<ORBHelper>(new ORBHelper);
    }
    orbhelper->orb->compute(image, keypoints, descriptors);
}

void ORBHelper::match(const cv::_InputArray &queryDescriptors,const cv::_InputArray &trainDescriptors,std::vector<cv::DMatch>&matches,double ratio) {
    if (orbhelper == nullptr) {
        orbhelper = std::shared_ptr<ORBHelper>(new ORBHelper);
    }
    orbhelper->matcher.match(queryDescriptors,trainDescriptors,matches);

    std::vector<cv::DMatch> good_matches;
    double min_dist = 10000;
    double max_dist = 0;

    for (auto &match:matches) {
        if (match.distance < min_dist) {
            min_dist = match.distance;
        }
        if (match.distance > max_dist) {
            max_dist = match.distance;
        }
    }
    for (auto &match:matches) {
        if (match.distance < std::max(3 * min_dist, 0.05)) {
            good_matches.push_back(match);
        }
    }
    matches = good_matches;
}
}