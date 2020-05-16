//
// Created by 范宏昌 on 2020/1/16.
//

#ifndef TOYSLAM_CONFIG_H
#define TOYSLAM_CONFIG_H

#include <opencv2/core/core.hpp>

namespace TinySLAM {

class Config {
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {} // private constructor makes a singleton
public:
    ~Config();  // close the file when deconstructing

    // set a new config file
    static void setParameterFile(const std::string &filename);

    // access the parameter values
    template<typename T>
    static T get(const std::string &key) {
        return T(Config::config_->file_[key]);
    }
};
}

#endif //TOYSLAM_CONFIG_H
