//
// Created by 范宏昌 on 2020/2/24.
//

#include "Map.h"
namespace TinySLAM{
void Map::add_pose(KeyPose::Ptr &keyPose) {
    poses.insert(pair<uint64_t,KeyPose::Ptr>(keyPose->id,keyPose));
    latest_pose_num = keyPose->id;
    if(mbVisual){
        mMapVisualizer->add_key_pose(keyPose);
    }
}

void Map::add_point(MapPoint::Ptr &keyPoint) {
    points.insert(pair<uint64_t,MapPoint::Ptr>(keyPoint->id, keyPoint));
    latest_point_num = keyPoint->id;

    if(mbVisual){
        mMapVisualizer->add_map_point(keyPoint);
    }
}

KeyPose::Ptr Map::get_latest_keypose() {
    return poses.at(latest_pose_num);
}

double getViewAngle ( Frame::Ptr &frame, MapPoint::Ptr &point )
{
    Vec3 n = point->pos - frame->get_center_world();
    n.normalize();
    return acos( n.transpose()*point->norm );
}

void Map::optimize(KeyPose::Ptr &keyPose) {
    for(auto point_it = points.begin(); point_it!=points.end();){

        // 能观察到时被匹配到的次数少，剔除
        double match_ratio = double(point_it->second->matched_time)/point_it->second->oberserved_time;
        if(point_it->second->oberserved_time>5&&match_ratio<map_point_erase_ratio){
            if(mbVisual){
                mMapVisualizer->remove_map_point(point_it->second);
            }
            point_it = points.erase(point_it);
//            cout<<"point removed because of match"<<endl;
            continue;
        }
//        // 无法被观察到，剔除
//        if(!keyPose->mpFrame->is_in_frame(point_it->second->pos)){
//            if(mbVisual){
//                mMapVisualizer->remove_map_point(point_it->second);
//            }
//            point_it = points.erase(point_it);
//            cout<<"point removed because of visibility"<<endl;
//            continue;
//        }
////         观察角度变化太大，剔除
//        double angle = getViewAngle(keyPose->mpFrame,point_it->second);
//        if(angle>M_PI/6.0){
//            if(mbVisual){
//                mMapVisualizer->remove_map_point(point_it->second);
//            }
//            point_it = points.erase(point_it);
//            cout<<"point removed because of visibility"<<endl;
//            continue;
//        }
        point_it++;

    }
    // 地图大小控制
    if(points.size()<200){
        map_point_erase_ratio = max<double>(0,map_point_erase_ratio-0.05);
    }
    if(points.size()>300){
        map_point_erase_ratio = min<double>(0.5,map_point_erase_ratio+0.05);
    }
    cout<<"Map now has "<<points.size()<<" points: "<<endl;
}

void Map::refresh_visual(Sophus::SE3 camera_pose) {
    if(mbVisual){
        mMapVisualizer->update(camera_pose);
    }
}

void Map::clear() {
    latest_pose_num=0;
    latest_point_num=0;
    poses.clear();
    points.clear();
    if(mbVisual){
        mMapVisualizer->clear();
    }
    KeyPose::next_id=0;
    MapPoint::next_id=0;
}

}