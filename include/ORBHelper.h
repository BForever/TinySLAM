//
// Created by 范宏昌 on 2020/1/17.
//

#ifndef TOYSLAM_ORBHELPER_H
#define TOYSLAM_ORBHELPER_H


#include <memory>
#include <opencv/cv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>


namespace TinySLAM {
class ORBHelper {

private:
    static std::shared_ptr<ORBHelper> orbhelper;
    // Singleton
    ORBHelper();

public:
//    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::xfeatures2d::SURF> orb;
    cv::BFMatcher matcher;
    static void detect(const cv::_InputArray &image,std::vector<cv::KeyPoint> &keypoints);
    static void compute(const cv::_InputArray &image,std::vector<cv::KeyPoint> &keypoints,const cv::_OutputArray &descriptors);
    static void match(const cv::_InputArray &queryDescriptors,const cv::_InputArray &trainDescriptors,
            std::vector<cv::DMatch>&matches,double ratio);
};
}

#endif //TOYSLAM_ORBHELPER_H
