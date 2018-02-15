/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#include "multicam_tools/undistort_nodelet.h"
#include "multicam_tools/tools.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>

namespace multicam_tools {
  void UndistortNodelet::onInit() {
    ros::NodeHandle nh = getPrivateNodeHandle();
    cameraInfoSub_ = nh.subscribe("camera_info_raw", 1,
                                  &UndistortNodelet::cameraInfoCallback, this);
    cameraInfoPub_ = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    imageTrans_.reset(new image_transport::ImageTransport(nh));
    imageSub_ = imageTrans_->subscribe("image", 1, &UndistortNodelet::imageCallback, this);
    imagePub_ = imageTrans_->advertise("undist_image", 1);
    nh.param<bool>("use_calibration_param", useCalibrationParam_, true);
    nh.param<bool>("center_image", centerImage_, true);
    if (useCalibrationParam_) {
      CalibDataVec cdv = CalibrationData::parse_cameras(nh);
      if (cdv.empty()) {
        throw (std::runtime_error("cannot find camera calib data in ros parameters"));
      }
      if (cdv.size() > 1) {
        ROS_WARN("using only camera 0 from calib file!");
      }
      CameraInfoConstPtr ci = tools::calib_data_to_camera_info(cdv[0]);
      initializeMap(ci);
      setUndistortedCameraInfo(ci);
    }
  }

  void UndistortNodelet::imageCallback(const ImageConstPtr &img) {
    if (mapx_.rows == 0) {
      ROS_WARN("no camera info received yet!");
      return;
    }
    bool isBayer = (img->encoding == "bayer_rggb8");
    bool outputColor = isBayer;
    std::string target_encoding = isBayer ?
      sensor_msgs::image_encodings::BGR8 :
      sensor_msgs::image_encodings::MONO8;
    cv::Mat im = cv_bridge::toCvCopy(img, target_encoding)->image;
    
    cv::Mat rectim;
    cv::remap(im, rectim, mapx_, mapy_, cv::INTER_LINEAR);
    std::string output_encoding = outputColor ? "bgr8" : "mono8";
    imagePub_.publish(cv_bridge::CvImage(img->header, output_encoding,
                                         rectim).toImageMsg());
    cameraInfo_.header = img->header;
    cameraInfoPub_.publish(cameraInfo_);
  }

  static cv::Mat centerK(const cv::Mat &K, int width, int height) {
    cv::Mat newK(K.clone());
    newK.at<double>(0,2) = width/2;
    newK.at<double>(1,2) = height/2;
    return (newK);
  }

  void UndistortNodelet::initializeMap(const CameraInfoConstPtr &camInfo) {
    cv::Mat mapx, mapy;
    cv::Size new_sz(camInfo->width, camInfo->height);
    cv::Mat K(3, 3, CV_64F, (void *)&camInfo->K[0]);
    std::cout << "K:" << std::endl << K << std::endl;
    cv::Mat D(camInfo->D.size(), 1, CV_64F, (void *)&camInfo->D[0]);
    std::cout << "D:" <<  D << std::endl;
    cv::Mat R = cv::Mat::eye(3,3, CV_64F);
    std::cout << "R:" <<  std::endl << R << std::endl;
    cv::Mat newK = centerImage_ ? centerK(K, camInfo->width, camInfo->height) : K;
    std::cout << "new K:" << std::endl << newK << std::endl;
    if (camInfo->distortion_model == "plumb_bob" ||
        camInfo->distortion_model == "radtan") {
      cv::initUndistortRectifyMap(K, D, R, newK, new_sz,
                                  CV_32FC1, mapx_, mapy_);
      ROS_INFO_STREAM("using radtan distortion model!");
    } else if (camInfo->distortion_model == "equidistant") {
      cv::fisheye::initUndistortRectifyMap(K, D, R, newK, new_sz,
                                           CV_32FC1, mapx_, mapy_);
      ROS_INFO_STREAM("using equidistant distortion model!");
    } else {
      ROS_ERROR_STREAM("unknown distortion model:" << camInfo->distortion_model);
      throw (std::runtime_error("unknown distortion model!"));
    }
  }

  void UndistortNodelet::setUndistortedCameraInfo(const CameraInfoConstPtr &camInfo) {
    cameraInfo_ = *camInfo;
    cameraInfo_.distortion_model = "plumb_bob";
    cameraInfo_.D.clear();  // no distortion!
    cameraInfo_.R = {1.0, 0.0, 0.0,   0.0, 1.0, 0.0,   0.0, 0.0, 1.0};
    cv::Mat K(3, 3, CV_64F, (void *)&camInfo->K[0]);
    cv::Mat newK = centerImage_? centerK(K, camInfo->width, camInfo->height) : K.clone();
    const double *cK = &newK.at<double>(0,0);
    cameraInfo_.K = {cK[0],  cK[1],  cK[2],
                     cK[3],  cK[4],  cK[5],
                     cK[6],  cK[7],  cK[8]};
    cameraInfo_.P = {cK[0],  cK[1],  cK[2], 0.0,
                     cK[3],  cK[4],  cK[5], 0.0,
                     cK[6],  cK[7],  cK[8], 0.0};
  }
  
  void UndistortNodelet::cameraInfoCallback(const CameraInfoConstPtr &camInfo) {
    if (useCalibrationParam_) {
      ROS_WARN_ONCE("ignoring camera info on rostopic!");
      return;
    }
    initializeMap(camInfo);
    setUndistortedCameraInfo(camInfo);
    cameraInfoSub_.shutdown();
  }
}
