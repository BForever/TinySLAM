
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
                vector<string> &vstrImages);
int main(int argc, char **argv) {
    Config::setParameterFile(argv[1]);
    vector<string> vstrImageFilenames;
    LoadImages(Config::get<string>("data_dir"), Config::get<string>("time_dir"), vstrImageFilenames);

    auto camera = Camera::Ptr(new Camera());
    VisualOdometry visualOdometry(camera);

    for(auto &filename : vstrImageFilenames){
        cout<<filename<<endl;
        auto im = imread(filename,CV_LOAD_IMAGE_GRAYSCALE);
//        undistort(im,undistorted,camera->K_CV(),camera->D_CV());
//        imshow("undistort",im);
        auto frame = Frame::Ptr(new Frame(im,camera));
        visualOdometry.track(frame);
    }
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages) {
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vstrImages.reserve(5000);
    string s;
    getline(fTimes, s);
    while (!fTimes.eof()) {
        getline(fTimes, s);
        if (!s.empty()) {
            vstrImages.push_back(strImagePath + "/" +s.substr(21) );
        }
    }
}