/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef MULTICAM_TOOLS_TOOLS_H
#define MULTICAM_TOOLS_TOOLS_H

#include "multicam_tools/calibration_data.h"
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core.hpp>

namespace multicam_tools {
  namespace tools {
    using sensor_msgs::CameraInfoConstPtr;

    CameraInfoConstPtr calib_data_to_camera_info(const CalibrationData &calib);

    //
    // wp:  world points matrix of shape [npoints, 1], type is CV_64FC3.
    //      To set it up, use code like this:
    //
    //       cv::Mat wp(wpe.size(), 1, CV_64FC3);
    //       for (unsigned int i = 0; i < wpe.size(); i++) {
    //         wp.at<cv::Vec3d>(i, 0) = cv::Vec3d(wpe[i].x, wpe[i].y, wpe[i].z);
    //       }
    //
    // imagePoints: will return [npoints, 2] of type CV_64FC1

    void project_points(const cv::Mat &wp,
                        const Eigen::Matrix<double, 4,4> &te,
                        const std::vector<double> &intrinsics,
                        const std::string &distModel,
                        const std::vector<double> &distcoeff,
                        cv::Mat *imagePoints);
  }
}

#endif
