// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SVO_INITIALIZATION_H
#define SVO_INITIALIZATION_H

#include <svo/global.h>

namespace svo {

class FrameHandlerMono;

/// Bootstrapping the map from the first two views.
namespace initialization {

enum InitResult { FAILURE, NO_KEYFRAME, SUCCESS };


/// Tracks features using Lucas-Kanade tracker and then estimates a homography.
class KltHomographyInit {
  friend class svo::FrameHandlerMono;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FramePtr frame_ref_;
  KltHomographyInit() {};
  ~KltHomographyInit() {};
  InitResult addFirstFrame(FramePtr frame_ref);
  InitResult addSecondFrame(FramePtr frame_ref);
  void reset();

protected:
  vector<cv::Point2f> px_ref_;      //!< keypoints to be tracked in reference frame.
  vector<cv::Point2f> px_cur_;      //!< tracked keypoints in current frame.

  cv::Mat img_prev_;   //  hyj  optical flow tracking frame by frame.
  vector<cv::Point2f> px_prev_;  //  hyj  optical flow tracking frame by frame.

  vector<Vector3d> f_ref_;          //!< bearing vectors corresponding to the keypoints in the reference image.
  vector <Vector3d> fts_type_;   // hyj : save feature type
  vector<Vector3d> f_cur_;          //!< bearing vectors corresponding to the keypoints in the current image.
  vector<double> disparities_;      //!< disparity between first and second frame.
  vector<int> inliers_;             //!< inliers after the geometric check (e.g., Homography).
  vector<Vector3d> xyz_in_cur_;     //!< 3D points computed during the geometric check.
  SE3d T_cur_from_ref_;              //!< computed transformation between the first two frames.
};

/// Detect Fast corners in the image.
void detectFeatures(FramePtr frame, vector<Vector3d> &fts_type,
    vector<cv::Point2f>& px_vec,
    vector<Vector3d>& f_vec);

/// Compute optical flow (Lucas Kanade) for selected keypoints.
///
/*
void trackKlt(FramePtr frame_ref,
    FramePtr frame_cur,
    vector<cv::Point2f>& px_ref,
    vector<cv::Point2f>& px_cur,
    vector<Vector3d>& f_ref,
    vector<Vector3d>& f_cur,
    vector<double>& disparities);
*/
void trackKlt(FramePtr frame_ref,
    FramePtr frame_cur, vector<Vector3d> &fts_type,
    vector<cv::Point2f>& px_ref,
    vector<cv::Point2f>& px_cur,
    vector<Vector3d>& f_ref,
    vector<Vector3d>& f_cur,
    vector<double>& disparities, cv::Mat & img_prev, vector<cv::Point2f> & px_prev);

void computeHomography(const vector<Vector3d>& f_ref,
    const vector<Vector3d>& f_cur,
    double focal_length,
    double reprojection_threshold,
    vector<int>& inliers,
    vector<Vector3d>& xyz_in_cur,
    SE3d& T_cur_from_ref, int method_choose);

} // namespace initialization
} // namespace nslam

#endif // NSLAM_INITIALIZATION_H
