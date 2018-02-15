/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#include "multicam_tools/overlay_point_cloud_nodelet.h"
#include "multicam_tools/tools.h"
#include <std_msgs/UInt32.h>
#include <boost/range/irange.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <sstream>
#include <memory>
#include <ctime>
#include <chrono>
#include <cmath>

namespace multicam_tools {
  using boost::irange;

  void OverlayPointCloudNodelet::onInit() {
    ros::NodeHandle nh = getPrivateNodeHandle();
    bool useCalibrationParam(true);
    nh.param<bool>("use_calibration_param", useCalibrationParam, true);
    if (useCalibrationParam) {
      CalibDataVec cdv = CalibrationData::parse_cameras(nh);
      if (cdv.empty()) {
        throw (std::runtime_error("cannot find camera calib data in ros parameters"));
      }
      if (cdv.size() > 1) {
        ROS_WARN("using only camera 0 from calib file!");
      }
      calib_ = cdv[0];
    }
    bool useApproxSync(false);
    nh.getParam("use_approximate_sync", useApproxSync);
    ROS_INFO_STREAM((useApproxSync ? "" : "not ") <<  "using approximate sync");
    nh.param<double>("alpha", alpha_, 0.5);
    image_transport::ImageTransport it(nh);
    std::string img_topic = ros::names::resolve("image");
    std::string points_topic = ros::names::resolve("points");
    ROS_INFO_STREAM("subscribing to image: " << img_topic);
    ROS_INFO_STREAM("subscribing to points: " << points_topic);
    imageSub_.reset(new image_transport::SubscriberFilter());
    imageSub_->subscribe(it,img_topic, 1);
    cloudSub_.reset(new message_filters::Subscriber<PointCloud2Msg>(nh, points_topic, 1));
    if (useApproxSync) {
      approxSync_.reset(new ApproxTimeSynchronizer2(
                          ApproxSyncPolicy2(60/*q size*/), *(imageSub_), *(cloudSub_)));
      approxSync_->registerCallback(&OverlayPointCloudNodelet::syncCallback, this);
    } else {
      exactSync_.reset(new ExactSynchronizer2(*(imageSub_), *(cloudSub_), 2));
      exactSync_->registerCallback(&OverlayPointCloudNodelet::syncCallback, this);
    }
    imagePub_ = nh.advertise<ImageMsg>("image_overlayed", 1);
  }


  void OverlayPointCloudNodelet::syncCallback(ImageConstPtr const &img0, PointCloud2ConstPtr const &pc0) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*pc0, cloud);
    cv::Mat projected;
    cv::Mat wp(cloud.points.size(),1, CV_64FC3);
    int pc_idx = 0;
    for (const pcl::PointXYZI &p: cloud.points) {
      wp.at<cv::Vec3d>(pc_idx, 0) = cv::Vec3d(p.x, p.y, p.z);
      pc_idx++;
    }
    const CameraIntrinsics &ci = calib_.intrinsics;
    tools::project_points(wp, identity(), ci.intrinsics,
                          ci.distortion_model, ci.distortion_coeffs,
                          &projected);
    if (imagePub_.getNumSubscribers() > 0) {
      cv_bridge::CvImageConstPtr const cv_ptr = cv_bridge::toCvShare(
        img0, sensor_msgs::image_encodings::MONO8);
      const cv::Mat gray = cv_ptr->image;
      if (gray.rows == 0) {
        ROS_ERROR("cannot decode image, not MONO8!");
        return;
      }
      cv::Mat img(gray.rows, gray.cols, CV_8UC3, cv::Scalar::all(0));
      for (int row = 0; row < projected.rows; row++) {
        int ix = std::nearbyint(projected.at<double>(row, 0));
        int iy = std::nearbyint(projected.at<double>(row, 1));
        if (ix >= 0 && ix < img.cols &&
            iy >= 0 && iy < img.rows) {
          img.at<cv::Vec3b>(iy, ix) = cv::Vec3b(255, 0, 255);
        }
      }
      cv::Mat img_out, gray_rgb;
      cv::cvtColor(gray, gray_rgb, CV_GRAY2BGR);
      addWeighted(img, alpha_, gray_rgb, (1-alpha_), 0.0, img_out);
      cv_bridge::CvImage cv_img(img0->header, sensor_msgs::image_encodings::BGR8, img_out);
      imagePub_.publish(cv_img.toImageMsg());
    }
  }

}
