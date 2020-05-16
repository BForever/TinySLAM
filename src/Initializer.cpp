//
// Created by 范宏昌 on 2020/1/16.
//

#include "Initializer.h"
#include "Frame.h"
#include <opencv2/imgcodecs.hpp>
#include <opencv/cv.hpp>
#include <iostream>
#include "Config.h"
#include "ORBHelper.h"

namespace TinySLAM {

Initializer::Initializer(shared_ptr<Camera> &camera, shared_ptr<Map> &map) {
    this->camera = camera;
    this->mMap = map;
    init_window_size = Config::get<int>("init.window_size");
    init_min_good_matches = Config::get<int>("init.min_good_matches");
    init_min_translation = Config::get<int>("init.min_translation");
}


void Initializer::match(Frame &frame1, Frame &frame2) {
    ORBHelper::match(frame1.descriptors, frame2.descriptors, good_matches,2);

    double min_dist = 10000;
    double max_dist = 0;

    for (auto &match:good_matches) {
        if (match.distance < min_dist) {
            min_dist = match.distance;
        }
        if (match.distance > max_dist) {
            max_dist = match.distance;
        }
    }

    std::cout << "min:" << min_dist << std::endl;
    std::cout << "max:" << max_dist << std::endl;

}

bool Initializer::estimate_pose_2d2d(Frame &frame1, Frame &frame2) {
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    double average_dist = 0;
    int count = 0;
    for (auto &good_match : good_matches) {
        auto &pt1 = frame1.keypoints[good_match.queryIdx].pt;
        auto &pt2 = frame2.keypoints[good_match.trainIdx].pt;
        points1.push_back(pt1);
        points2.push_back(pt2);
        auto dist = pt1 - pt2;
        average_dist += abs(dist.x) + abs(dist.y);
        count++;
    }
    average_dist /= count;
    cout<<"average moving: "<<average_dist<<endl;
    if (average_dist < init_min_translation) {
        return false;
    }


    Mat show;
    drawMatches(frame1.image, frame1.keypoints, frame2.image, frame2.keypoints, good_matches, show);
    imshow("match for Initializer", show);
    cout << "使用 " << points1.size() << "组点进行初始化" << endl;
    // 计算本质矩阵
    cv::Point2d principal_point(camera->cx_, camera->cy_);  //相机光心, TUM dataset标定值
    double focal_length = camera->fy_;      //相机焦距, TUM dataset标定值
    cv::Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
//    std::cout << "essential_matrix is " << std::endl << essential_matrix << std::endl;

    // 从本质矩阵中恢复旋转和平移信息.
    recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
    std::cout << "R is " << std::endl << R << std::endl;
    std::cout << "t is " << std::endl << t << std::endl;

    // 反转位姿:如果要求相机在世界坐标系下的位置
//    R = R.t();  // rotation of inverse
//    t = -R * t; // translation of inverse
//    T = cv::Mat::eye(4, 4, R.type()); // T is 4x4
//    T( cv::Range(0,3), cv::Range(0,3) ) = new_R * 1; // copies R into T
//    T( cv::Range(0,3), cv::Range(3,4) ) = new_t * 1; // copies tvec into T

//    std::cout << "T is " << std::endl << T << std::endl;
    return true;
}

void Initializer::triangulate(Frame &frame1, Frame &frame2) {
    cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
                                        1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0);

    cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
//    cout<<"T1:"<<T1<<endl;
//    cout<<"T2:"<<T2<<endl;
    std::vector<cv::Point2f> pts_1, pts_2;
    for (cv::DMatch m:good_matches) {
        // 将像素坐标转换至相机坐标
//        std::cout<<frame1.keypoints[m.queryIdx].pt<<std::endl;
//        std::cout<<camera->pixel2camera(frame1.keypoints[m.queryIdx].pt)<<std::endl;
        pts_1.push_back(camera->pixel2camera(frame1.keypoints[m.queryIdx].pt));
        pts_2.push_back(camera->pixel2camera(frame2.keypoints[m.trainIdx].pt));
        descrptors.push_back(frame2.descriptors.row(m.trainIdx));
    }

    cv::Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

    points.clear();
    // 转换成非齐次坐标
    for (int i = 0; i < pts_4d.cols; i++) {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0); // 归一化
        cv::Point3d p(
                x.at<float>(0, 0),
                x.at<float>(1, 0),
                x.at<float>(2, 0)
        );
        points.push_back(p);
    }
}

void Initializer::complete_keyPose(KeyPose::Ptr &keyPose) {
    int count = 0;
    for (auto &p:points) {
        Eigen::Vector3d p_world(p.x, p.y, p.z);
        Eigen::Vector3d norm = p_world - keyPose->mpFrame->get_center_world();
        norm.normalize();
        auto mapPoint = MapPoint::Ptr(new MapPoint(p, norm, descrptors[count++]));
        mapPoint->observers.push_back(keyPose);
        keyPose->points.push_back(mapPoint);
        mMap->add_point(mapPoint);
    }
    keyPose->set_pose(R, t);
}

bool Initializer::add_frame(Frame::Ptr frame) {
    init_window.push(frame);
    while (init_window.size() > init_window_size) {
        init_window.pop();
    }
    if (init_window.size() == init_window_size) {
        Frame::Ptr front = init_window.front();
        Frame::Ptr back = init_window.back();

        front->computeORB();
        back->computeORB();

        match(*front, *back);
        cout << "match size: " << good_matches.size() << endl;
        if (good_matches.size() < init_min_good_matches) {
            return false;
        }
        if (!estimate_pose_2d2d(*front, *back)) {
            return false;
        }
        triangulate(*front, *back);
        result = KeyPose::Ptr(new KeyPose(frame));
        complete_keyPose(result);
        return true;
    }
    return false;
}

void Initializer::clear() {
    while (!init_window.empty()){
        init_window.pop();
    }
}


}