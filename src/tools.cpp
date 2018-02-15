/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include "multicam_tools/tools.h"
#include <opencv2/calib3d/calib3d.hpp>

namespace multicam_tools {
  namespace tools {
    using CameraInfo = sensor_msgs::CameraInfo;
    CameraInfoConstPtr calib_data_to_camera_info(const CalibrationData &calib) {
      sensor_msgs::CameraInfoPtr cinfo(new CameraInfo());
      const CameraIntrinsics &ci = calib.intrinsics;
      const auto &K = ci.intrinsics;
      const auto &H = calib.T_cn_cnm1;
      cinfo->width  = ci.resolution[0];
      cinfo->height = ci.resolution[1];
      cinfo->distortion_model = ci.distortion_model;
      cinfo->D = ci.distortion_coeffs;
      cinfo->K = {K[0],    0,       K[2],
                 0,       K[1],    K[3],
                 0,       0,       1.0};
      cinfo->R = {H(0,0), H(0,1), H(0,2),
                 H(1,0), H(1,1), H(1,2),
                 H(2,0), H(2,1), H(2,2)};
      cinfo->P = {K[0],    0,       K[2], 0.0,
                 0,       K[1],    K[3], 0.0,
                 0,       0,       1.0,  0.0};
      return (cinfo);
    }

    void project_points(const cv::Mat &wp,
                        const Eigen::Matrix<double, 4,4> &te,
                        const std::vector<double> &intrinsics,
                        const std::string &distModel,
                        const std::vector<double> &distcoeff,
                        cv::Mat *imagePoints) {
      double Kd[9] =  {intrinsics[0], 0.0, intrinsics[2],
                       0.0, intrinsics[1], intrinsics[3],
                       0.0, 0.0, 1.0};
      cv::Mat K(3, 3, CV_64FC1, Kd);
      std::vector<double> dc = distcoeff;
      while (dc.size() < 4) {
        dc.push_back(0);
      }
      cv::Mat dist(dc.size(), 1, CV_64FC1, &dc[0]);

      cv::Affine3d::Mat4 T_mat = {te(0,0), te(0,1), te(0,2), te(0,3),
                                  te(1,0), te(1,1), te(1,2), te(1,3),
                                  te(2,0), te(2,1), te(2,2), te(2,3),
                                  te(3,0), te(3,1), te(3,2), te(3,3)};
      cv::Affine3d T_cam_world(T_mat);
      cv::Affine3f::Vec3 arvec = T_cam_world.rvec();
      cv::Affine3f::Vec3 atvec = T_cam_world.translation();
      if (distModel == "equidistant") {
        cv::fisheye::projectPoints(wp, *imagePoints, arvec, atvec, K, dist);
      } else if (distModel == "radtan") {
        cv::projectPoints(wp, arvec, atvec, K, dist, *imagePoints);
      } else {
        std::cout << "WARNING: unknown distortion model: " << distModel << " using radtan!" << std::endl;
        cv::projectPoints(wp, arvec, atvec, K, dist, *imagePoints);
      }
    }
  }
}
