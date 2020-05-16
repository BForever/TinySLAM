//
// Created by 范宏昌 on 2020/5/14.
//

#include "MapVisualizer.h"
namespace TinySLAM{


MapVisualizer::MapVisualizer() {
    vis = viz::Viz3d("TinySLAM");
    world_coor = viz::WCoordinateSystem(5);
    camera_coor = viz::WCoordinateSystem(5);
    camera_pos = Point3d(0, 0, -1);
    camera_fp = Point3d(0, 0, 0);
    camera_y_dir = Point3d(0, 1, 0);
    camera_pose = viz::makeCameraPose(camera_pos, camera_fp, camera_y_dir);
    vis.setViewerPose(camera_pose);
    world_coor.setRenderingProperty(viz::LINE_WIDTH,2.);
    camera_coor.setRenderingProperty(viz::LINE_WIDTH,1.);
    vis.showWidget("World",world_coor);
    vis.showWidget("Camera",camera_coor);
    last_keyPose = nullptr;
}

void MapVisualizer::update(Sophus::SE3 Tcw) {
    auto R = Tcw.rotation_matrix();
    auto t = Tcw.translation();
    cv::Affine3d M(
            cv::Affine3d::Mat3(R(0,0),R(0,1),R(0,2),
                               R(1,0),R(1,1),R(1,2),
                               R(2,0),R(2,1),R(2,2)),
            cv::Affine3d::Vec3(t(0,0),t(1,0),t(2,0)));
    vis.setWidgetPose("Camera",M);
    vis.spinOnce(1, false);
}

void MapVisualizer::add_map_point(TinySLAM::MapPoint::Ptr &mapPoint) {
    viz::WSphere sphere(Point3d(mapPoint->pos(0,0),mapPoint->pos(1,0),mapPoint->pos(2,0)),0.2);
    vis.showWidget("MapPoint_"+to_string(mapPoint->id), sphere);
}

void MapVisualizer::add_key_pose(KeyPose::Ptr &keyPose){
    if(last_keyPose== nullptr){
        last_keyPose = keyPose;
    } else{
        auto t1 = last_keyPose->get_pose().inverse().translation();
        auto t2 = keyPose->get_pose().inverse().translation();
        auto point1 = Point3d(t1(0,0),t1(1,0),t1(2,0));
        auto point2 = Point3d(t2(0,0),t2(1,0),t2(2,0));
        viz::WLine line(point1,point2,viz::Color::yellow());
        vis.showWidget("Line_"+to_string(keyPose->id),line);
        last_keyPose = keyPose;
    }
}

void MapVisualizer::remove_map_point(MapPoint::Ptr &mapPoint) {
    vis.removeWidget("MapPoint_"+to_string(mapPoint->id));
}

void MapVisualizer::clear() {
    vis.removeAllWidgets();
    world_coor = viz::WCoordinateSystem(5);
    camera_coor = viz::WCoordinateSystem(5);
    camera_pos = Point3d(0, 0, -1);
    camera_fp = Point3d(0, 0, 0);
    camera_y_dir = Point3d(0, 1, 0);
    camera_pose = viz::makeCameraPose(camera_pos, camera_fp, camera_y_dir);
    vis.setViewerPose(camera_pose);
    world_coor.setRenderingProperty(viz::LINE_WIDTH,2.);
    camera_coor.setRenderingProperty(viz::LINE_WIDTH,1.);
    vis.showWidget("World",world_coor);
    vis.showWidget("Camera",camera_coor);
    last_keyPose = nullptr;
}
}