//
// Created by 范宏昌 on 2020/1/17.
//

#include "VisualOdometry.h"
#include "Initializer.h"
#include "ORBHelper.h"
#include <opencv2/calib3d/calib3d.hpp>
#include "g2o_types.h"
#include "Converter.h"


namespace TinySLAM {
VisualOdometry::VisualOdometry(shared_ptr<Camera> &camera) {
    mCamera = camera;
    mState = UNINITIALIZED;
    mMap = Map::Ptr(new Map(true));
    mInitializer = make_shared<Initializer>(camera, mMap);
    max_translation=Config::get<double >("track.max_translation");
    min_translation=Config::get<double >("track.min_translation");
}

int VisualOdometry::track(Frame::Ptr &frame) {
    switch (mState) {
        case UNINITIALIZED: {
            if (mInitializer->add_frame(frame)) {
                mState = TRACKING;
                mMap->add_pose(mInitializer->result);
            }
            imshow("image", frame->image);
            break;
        }
        case LOST: {
            // no break, still try track
            lost_count++;
            if(lost_count>20){
                this->save_trajectory("trajectory"+to_string(this->trajectory_count++)+".txt");
                this->clear();
                lost_count = 0;
                break;
            }
        }
        case TRACKING: {
            KeyPose::Ptr ref_pose = mMap->get_latest_keypose();
            KeyPose::Ptr cur_pose = estimate_pose_3d2d(ref_pose, frame);
            if (cur_pose != nullptr) {
                mState = TRACKING;
                lost_count=0;
            } else {
                // Track fail
//                imshow("image", frame->image);
                mState = LOST;
            }
            break;
        }
    }
    return 0;
}
double extract_translation(Frame::Ptr &frame1,Frame::Ptr &frame2){
    auto t1 = frame1->Tcw.translation();
    auto t2 = frame2->Tcw.translation();
    auto d = t1-t2;
    double distance = sqrt(d(0,0)*d(0,0)+d(1,0)*d(1,0)+d(2,0)*d(2,0));
    return distance;
}
KeyPose::Ptr VisualOdometry::estimate_pose_3d2d(KeyPose::Ptr &keyPose1, Frame::Ptr &frame2) {
    this->keyPose1 = keyPose1;
    this->frame1 = keyPose1->mpFrame;
    this->frame2 = frame2;

    match_with_map();
    if (!estimate_pose()) {
        return nullptr;
    }

    if (!bundle_adjustment()) {
        return nullptr;
    }
    match_two_frame();
    Mat show;
    drawMatches(frame1->image, frame1->keypoints, frame2->image, frame2->keypoints, matches, show);
    imshow("match for map", show);

    auto distance = extract_translation(frame1,frame2);
    cout<<"translation: "<<distance<<endl;
    if(distance>max_translation){
        return nullptr;
    }
    if(trust) {
        if(distance>min_translation){
            triangulation();
        }
        // Track success
        mMap->optimize(keyPose2);
        mMap->add_pose(keyPose2);
        return keyPose2;
    }
    return nullptr;
}

void VisualOdometry::match_with_map() {
    // 计算当前帧ORB特征点
    frame2->computeORB();
    map_points.clear();
    map_descrptors = Mat();
    // 获取地图点及其特征描述子，增加观测次数
    for (auto &point_it:mMap->points) {
        auto &p = point_it.second;
        p->oberserved_time++;
        map_points.push_back(p);
        map_descrptors.push_back(p->descriptor);
    }
    // 与地图点进行进行匹配
    matches.clear();
    ORBHelper::match(map_descrptors, frame2->descriptors, matches, 2);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;
}

bool VisualOdometry::estimate_pose() {
    // 将匹配点对用于PnP解算位姿
    pts_3d_ptr.clear();
    pts_2d_index.clear();
    pts_3d.clear();
    pts_2d.clear();
    pts_2dkp.clear();
    for (DMatch m:matches) {
        pts_3d_ptr.push_back(map_points[m.queryIdx]);
        map_points[m.queryIdx]->matched_time++;
        auto &pos = map_points[m.queryIdx]->pos;
        pts_3d.emplace_back(pos(0, 0), pos(1, 0), pos(2, 0));
        pts_2d_index.insert(m.trainIdx);
        pts_2d.push_back(frame2->keypoints[m.trainIdx].pt);
        pts_2dkp.push_back(frame2->keypoints[m.trainIdx]);
    }
    Mat keyPoints_image;
    drawKeypoints(frame2->image, pts_2dkp, keyPoints_image, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("image", keyPoints_image);
    // 3d-2d 点对如果数目不足3对则无法进行PnP，跳过此帧
    cout << "3d-2d pairs: " << pts_3d.size() << endl;
    if (pts_3d.size() < 3) {
        return false;
    }

    // PnP确定位姿
//    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    try {
        solvePnPRansac(pts_3d, pts_2d, mCamera->K_CV(), Mat(), rvec, tvec, false, 200, 4, 0.99,
                       inliers); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
//        solvePnP(pts_3d, pts_2d, mCamera->K_CV(), mCamera->D_CV(), rvec, tvec, false); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
    } catch (Exception e) {
        cout << e.msg << endl;
        return false;
    }
    cv::Rodrigues(rvec, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
//    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
//    cout << "solve pnp in opencv cost time: " << time_used.count() << " seconds." << endl;

//    cout << "R=" << endl << R << endl;
//    cout << "t=" << endl << tvec << endl;
    int num_inliers_ = inliers.rows;
    cout << "pnp inliers: " << num_inliers_ << endl;

    trust = num_inliers_ > 10 || (num_inliers_ > 5 && double(num_inliers_) / pts_3d.size() > 0.5);

    T_c_w_estimated = SE3(
            SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
            Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0))
    );
    return true;
}

bool VisualOdometry::bundle_adjustment() {
    if (inliers.rows < 2) {
        mMap->refresh_visual(T_c_w_estimated.inverse());
        return false;
    }
    // BA 优化位姿
    // 求解器
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 2>> Block;
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block *solver_ptr = new Block(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    // 位姿
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
            T_c_w_estimated.rotation_matrix(), T_c_w_estimated.translation()
    ));
    optimizer.addVertex(pose);

    // 观测边
    for (int i = 0; i < inliers.rows; i++) {
        int index = inliers.at<int>(i, 0);
//    for (int i = 0; i < pts_3d.size(); i++) {
//        int index = i;
        // 3D -> 2D projection
        EdgeProjectXYZ2UVPoseOnly *edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId(i);
        edge->setVertex(0, pose);
        edge->camera_ = mCamera;
        edge->point_ = Vector3d(pts_3d[index].x, pts_3d[index].y, pts_3d[index].z);
        edge->setMeasurement(Vector2d(pts_2d[index].x, pts_2d[index].y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
    }
    // 进行优化
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // 优化结果
    T_c_w_estimated = SE3(
            pose->estimate().rotation(),
            pose->estimate().translation()
    );
//    cout << "T_c_w_estimated: " << endl << T_c_w_estimated.matrix() << endl;
    mMap->refresh_visual(T_c_w_estimated.inverse());
    return true;
}

void VisualOdometry::match_two_frame() {
// 创建新的关键帧
    keyPose2 = KeyPose::Ptr(new KeyPose(frame2));
    keyPose2->set_pose(T_c_w_estimated);
    // 与参考帧配合进行三角化
    //   与前一帧进行匹配
    matches.clear();
    ORBHelper::match(frame1->descriptors, frame2->descriptors, matches, 2);
}

void VisualOdometry::triangulation() {
    vector<Point2f> pts_1, pts_2;
    vector<int> pts_index;
    good_matches.clear();
    good_matches.reserve(200);
    for (DMatch m:matches) {
        // 筛选未在地图中的点
        if (pts_2d_index.find(m.trainIdx) == pts_2d_index.end()) {
            good_matches.push_back(m);
//            cout << "pts1: " << frame1->keypoints[m.queryIdx].pt << endl;
//            cout << "pts1 in camera: " << mCamera->pixel2camera(frame1->keypoints[m.queryIdx].pt) << endl;
            pts_1.push_back(mCamera->pixel2camera(frame1->keypoints[m.queryIdx].pt));
            pts_2.push_back(mCamera->pixel2camera(frame2->keypoints[m.trainIdx].pt));
            pts_index.push_back(m.trainIdx);
        }
    }
    Mat pts_4d;
    Matrix4d T1 = keyPose1->get_pose().matrix();
    Matrix4d T2 = keyPose2->get_pose().matrix();
//    cout << "T1:" << T1 << endl;
//    cout << "T2:" << T2 << endl;
    auto tT1 = Converter::Matrix4dtocvMat(T1);
    auto tT2 = Converter::Matrix4dtocvMat(T2);
//    cout << "tT1:" << tT1 << endl;
//    cout << "tT2:" << tT2 << endl;
    try {
        cv::triangulatePoints(tT1, tT2, pts_1, pts_2, pts_4d);
    } catch (Exception &e) {
        cout << e.msg << endl;
        return;
    }


    // 将新三角化得到的3d点加入地图，并加入当前帧
    keyPose2->points.clear();
    for (int i = 0; i < pts_4d.cols; i++) {
        // 转换成非齐次坐标
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0); // 归一化
        cv::Point3d p(
                x.at<float>(0, 0),
                x.at<float>(1, 0),
                x.at<float>(2, 0)
        );
//        cout << "new mappoint: " << p << endl;
        // 计算法向
        Eigen::Vector3d p_world(p.x, p.y, p.z);
        Eigen::Vector3d norm = p_world - frame2->get_center_world();
        norm.normalize();
        Mat descriptor = frame2->descriptors.row(pts_index[i]).clone();
        auto mapPoint = MapPoint::Ptr(new MapPoint(p, norm, descriptor));
        mMap->add_point(mapPoint);
        mapPoint->observers.push_back(keyPose2);
        mapPoint->matched_time++;
        keyPose2->points.push_back(mapPoint);
    }
}
void VisualOdometry::save_trajectory(string filename){
    auto keyPoses = mMap->poses;
    vector<KeyPose::Ptr> poses;
    poses.reserve(keyPoses.size());
    int i = 0;
    for(int count=0;count<keyPoses.size();){
        auto pose = keyPoses.find(i++);
        if(pose==keyPoses.end()){
            continue;
        }
        poses.push_back(pose->second);
        count++;
    }

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<poses.size(); i++)
    {
        KeyPose::Ptr pose = poses[i];
        Matrix3d rot = pose->get_pose().rotation_matrix().inverse();
        vector<float> q = Converter::toQuaternion(rot);
        auto t = pose->mpFrame->get_center_world();
        f << setprecision(6) << pose->mpFrame->mTimeStamp << setprecision(7) << " " << t(0,0) << " " << t(1,0) << " " << t(2,0)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }

    f.close();
    cout << endl << "trajectory saved to "<<filename<<" !" << endl;
}

void VisualOdometry::clear() {
    mState = UNINITIALIZED;
    mMap->clear();
    mInitializer = make_shared<Initializer>(mCamera, mMap);
}
}