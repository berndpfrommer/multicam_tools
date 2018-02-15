/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef MULTICAM_TOOLS_CALIBRATION_DATA_H
#define MULTICAM_TOOLS_CALIBRATION_DATA_H

#include "multicam_tools/camera_intrinsics.h"
#include "multicam_tools/camera_extrinsics.h"
#include <Eigen/StdVector>
#include <vector>
#include <string>
#include <ostream>
#include <ros/ros.h>


namespace multicam_tools {
  struct CalibrationData {
    typedef std::vector<CalibrationData, Eigen::aligned_allocator<CalibrationData> > CalibDataVec;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string       name;
    CameraIntrinsics  intrinsics;
    CameraExtrinsics  T_cn_cnm1;
    std::string       rostopic;
    bool              fixIntrinsics{false};
    bool              fixExtrinsics{false};
    // 
    static CalibDataVec parse_cameras(const ros::NodeHandle &nh);
  };
  using CalibDataVec=CalibrationData::CalibDataVec;
  std::ostream &operator<<(std::ostream &os, const CalibrationData &cd);
}
#endif
