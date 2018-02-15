/* -*-c++-*--------------------------------------------------------------------
 * 2017 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef MULTICAM_TOOLS_CAMERA_EXTRINSICS_H
#define MULTICAM_TOOLS_CAMERA_EXTRINSICS_H

#include <vector>
#include <Eigen/Core>

namespace multicam_tools {
  using  CameraExtrinsics = Eigen::Matrix<double, 4, 4>;
  CameraExtrinsics zeros();
  CameraExtrinsics identity();
  bool isNonZero(const CameraExtrinsics &T);
  typedef std::vector<CameraExtrinsics, Eigen::aligned_allocator<CameraExtrinsics> > CameraExtrinsicsVec;
}
#endif
