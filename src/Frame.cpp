//
// Created by 范宏昌 on 2020/1/16.
//

#include <iostream>
#include "Frame.h"
#include "ORBHelper.h"
namespace TinySLAM{

uint64_t Frame::next_id = 0;

void Frame::computeORB() {
    if(!orb_computed){
        ORBHelper::detect(image,keypoints);
        ORBHelper::compute(image,keypoints,descriptors);
        orb_computed= true;
    }
}

Frame::Frame(cv::Mat &image,Camera::Ptr &camera, std::string TimeStamp) {
    id = next_id++;
    this->image = image;
    this->mCamera = camera;
    this->mTimeStamp = TimeStamp;
}

bool Frame::is_in_frame(const Vec3 &point_world) {
//    std::cout<<"before w2c: "<<point_world<<std::endl;
    auto point_camera = mCamera->world2camera(point_world, Tcw);

//    std::cout<<"after:  "<<point_camera<<std::endl;
    if(point_camera(2, 0) < 0){
        // 深度为副，表示在相机背后，剔除
        return false;
    }
    auto pixel = mCamera->world2pixel(point_world,Tcw);
//    std::cout<<"pixel:  "<<pixel<<std::endl;
    // 是否在画面内
    return pixel(0, 0) > 0 && pixel(0, 0) < image.cols && pixel(1, 0) > 0 && pixel(1, 0) < image.rows;
}
}