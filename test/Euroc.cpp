
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
                vector<string> &vstrImages, vector<string> &timeStamps);
int main(int argc, char **argv) {
    Config::setParameterFile(argv[1]);
    vector<string> vstrImageFilenames,timeStamps;
    LoadImages(Config::get<string>("data_dir"), Config::get<string>("time_dir"), vstrImageFilenames,timeStamps);

    auto camera = Camera::Ptr(new Camera());
    VisualOdometry visualOdometry(camera);

    for(int i=0;i<vstrImageFilenames.size();i++){
        auto filename = vstrImageFilenames[i];
        cout<<filename<<endl;
        auto im = imread(filename,CV_LOAD_IMAGE_GRAYSCALE);
        Mat undistorted;
        undistort(im,undistorted,camera->K_CV(),camera->D_CV());
//        imshow("undistort",undistorted);
        auto frame = Frame::Ptr(new Frame(undistorted,camera,timeStamps[i]));
        visualOdometry.track(frame);
        cvWaitKey(1);
    }
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<string> &timeStamps) {
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vstrImages.reserve(5000);
    timeStamps.reserve(5000);
    while (!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            timeStamps.push_back(ss.str());
        }
    }
}