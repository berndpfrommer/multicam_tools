/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef MULTICAM_TOOLS_OVERLAY_POINT_CLOUD_NODELET_H
#define MULTICAM_TOOLS_OVERLAY_POINT_CLOUD_NODELET_H

#include "multicam_tools/calibration_data.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <iostream>
#include <memory>


namespace multicam_tools {
  using sensor_msgs::ImageConstPtr;
  using sensor_msgs::PointCloud2ConstPtr;  
  using sensor_msgs::CameraInfo;
  using sensor_msgs::CameraInfoConstPtr;
  using ImageMsg       = sensor_msgs::Image;
  using PointCloud2Msg = sensor_msgs::PointCloud2;
  typedef message_filters::sync_policies::ApproximateTime<ImageMsg, PointCloud2Msg> ApproxSyncPolicy2;
  typedef message_filters::Synchronizer<ApproxSyncPolicy2> ApproxTimeSynchronizer2;
  typedef message_filters::TimeSynchronizer<ImageMsg, PointCloud2Msg> ExactSynchronizer2;
  class OverlayPointCloudNodelet : public nodelet::Nodelet {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void onInit() override;
  private:
    void syncCallback(ImageConstPtr const &img0, PointCloud2ConstPtr const &pc0);
    // ---------- variables
    ros::Publisher               imagePub_;
    std::shared_ptr<message_filters::Subscriber<PointCloud2Msg>> cloudSub_;
    std::shared_ptr<image_transport::SubscriberFilter> imageSub_;
    std::unique_ptr<ApproxTimeSynchronizer2>  approxSync_;
    std::unique_ptr<ExactSynchronizer2>       exactSync_;
    CalibrationData                           calib_;
    double                                    alpha_{0.5};
  };
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(multicam_tools::OverlayPointCloudNodelet, nodelet::Nodelet)

#endif
