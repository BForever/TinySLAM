
#include <opencv/cv.hpp>
#include "Test.h"
#include "Camera.h"
#include "Config.h"
#include "Initializer.h"
#include "VisualOdometry.h"
#include <thread>
#include <ctime>
#include <unistd.h>

using namespace cv;
using namespace std;
using namespace TinySLAM;
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages,vector<string> &timeStamps);
int main(int argc, char **argv) {
    Config::setParameterFile(argv[1]);
    vector<string> vstrImageFilenames,timeStamps;
    LoadImages(Config::get<string>("data_dir"), Config::get<string>("time_dir"), vstrImageFilenames,timeStamps);

    auto camera = Camera::Ptr(new Camera());
    VisualOdometry visualOdometry(camera);

    for(auto &filename : vstrImageFilenames){
        cout<<filename<<endl;
        auto im = imread(filename,CV_LOAD_IMAGE_GRAYSCALE);
        Mat undistorted;
        undistort(im,undistorted,camera->K_CV(),camera->D_CV());
//        imshow("undistort",undistorted);
        auto frame = Frame::Ptr(new Frame(undistorted,camera));
        visualOdometry.track(frame);
    }




//    bool finish = false;
//    VideoCapture cap;
//    Mat frame;
//    mutex bufferMutex;
//    clock_t last_time = 0;
//    int count = 0;
//    std::thread bufferthread([&]() {
//        int key = 0;
//        while (!finish) {
//            if (cap.open("rtsp://192.168.1.101:8086"), CV_CAP_DSHOW) {
//                cout << "video stream reconnected!" << endl;
////                cap.set(CV_CAP_PROP_FPS, 30);
//                while (true) {
//                    cap.read(frame);
//                    count += 1;
//
//
//                    if (frame.empty()) {
//                        key++;
//                        if (key >= 25) {
//                            key = 0;
//                            break;
//                        } else {
//                            continue;
//                        }
//                    } else {
//                        key = 0;
//                    }
//
//                    if (last_time == 0) {
//                        last_time = clock();
//                    } else if (clock() - last_time >= 1 * CLOCKS_PER_SEC) {
//                        double flush_rate = count / (double(clock() - last_time) / double(CLOCKS_PER_SEC));
//                        cout << flush_rate << endl;
//                        last_time = clock();
//                        count = 0;
//                    }
//                }
//            }
//            cout << "video stream disconnected! sleep 1s." << endl;
//            sleep(1);
//        }
//    });
//
//    cout << endl << "-------" << endl;
//    cout << "Start processing sequence ..." << endl;
//    int inited = 0;
//
//    while (!finish) {
//        if (!frame.empty()) {
//            {
//                imshow("frame", frame);
//            }
//
//            cvWaitKey(16);
//            if (inited == 0) {
////                osmap.mapLoad("myHome.yaml");
//                inited++;
//            }
//        }
//    }
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages,vector<string> &timeStamps) {
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vstrImages.reserve(5000);
    timeStamps.reserve(5000);
    string s;
    getline(fTimes, s);
    getline(fTimes, s);
    getline(fTimes, s);
    while (!fTimes.eof()) {
        getline(fTimes, s);
        if (!s.empty()) {
            vstrImages.push_back(strImagePath + "/" +s.substr(18) );
            timeStamps.push_back(s.substr(0,17));
        }
    }
}