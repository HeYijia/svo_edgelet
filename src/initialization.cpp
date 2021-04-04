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

#include <svo/config.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/initialization.h>
#include <svo/feature_detection.h>
#include <svo/math_lib.h>
#include <svo/homography.h>

#include <opencv2/calib3d.hpp>
#include <numeric>

namespace svo {
namespace initialization {

InitResult KltHomographyInit::addFirstFrame(FramePtr frame_ref)
{
  reset();
  detectFeatures(frame_ref,fts_type_, px_ref_, f_ref_);
  if(px_ref_.size() < 100)
  {
    SVO_WARN_STREAM_THROTTLE(2.0, "First image has less than 100 features. Retry in more textured environment.");
    return FAILURE;
  }

  int fts_center_img = 0;
  for(size_t i=0, i_max=px_ref_.size(); i<i_max; ++i)
  {

        if(px_ref_[i].x>640/4 && px_ref_[i].x<640 *3/4  &&px_ref_[i].y>480/4 && px_ref_[i].y<480*3/4)
        {
                fts_center_img++;
        }
  }
  if(fts_center_img<50)
  {
        SVO_WARN_STREAM_THROTTLE(2.0, "First image,  features in center img are less. Retry in more textured environment.");
        return FAILURE;
  }

  frame_ref_ = frame_ref;

  img_prev_ = frame_ref_->img_pyr_[0].clone();   // init img_prev_:  used to track feature frame by frame
  px_prev_ = px_ref_;
  px_cur_.insert(px_cur_.begin(), px_ref_.begin(), px_ref_.end());
  return SUCCESS;
}

InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)
{
  //trackKlt(frame_ref_, frame_cur, px_ref_, px_cur_, f_ref_, f_cur_, disparities_);
  trackKlt(frame_ref_, frame_cur,fts_type_, px_ref_, px_cur_, f_ref_, f_cur_, disparities_,img_prev_,px_prev_);
  SVO_INFO_STREAM("Init: KLT tracked "<< disparities_.size() <<" features");

  if(disparities_.size() < Config::initMinTracked())
    return FAILURE;

  double disparity = svo::getMedian(disparities_);

  SVO_INFO_STREAM("Init: KLT "<<disparity<<"px average disparity.");
  if(disparity < Config::initMinDisparity())
    return NO_KEYFRAME;

  double sum = std::accumulate(disparities_.begin(), disparities_.end(), 0.0);
  double mean = sum / disparities_.size();
  double sq_sum = std::inner_product(disparities_.begin(), disparities_.end(), disparities_.begin(), 0.0);
  double stdev = std::sqrt(sq_sum / disparities_.size() - mean * mean);

  int method_flag(1);  // H method
  if(stdev > 15)
  {
    method_flag = 2;   // E method
  }

  computeHomography(
      f_ref_, f_cur_,
      frame_ref_->cam_->errorMultiplier2(), Config::poseOptimThresh(),
      inliers_, xyz_in_cur_, T_cur_from_ref_,method_flag);
  SVO_INFO_STREAM("Init: Homography RANSAC "<<inliers_.size()<<" inliers.");

  if(inliers_.size() < Config::initMinInliers())
  {
    SVO_WARN_STREAM("Init WARNING: "<<Config::initMinInliers()<<" inliers minimum required.");
    return FAILURE;
  }

  // Rescale the map such that the mean scene depth is equal to the specified scale
  vector<double> depth_vec;
  for(size_t i=0; i<xyz_in_cur_.size(); ++i)
    depth_vec.push_back((xyz_in_cur_[i]).z());
  double scene_depth_median = svo::getMedian(depth_vec);
  double scale = Config::mapScale()/scene_depth_median;
  frame_cur->T_f_w_ = T_cur_from_ref_ * frame_ref_->T_f_w_;
  frame_cur->T_f_w_.translation() =
      -frame_cur->T_f_w_.rotationMatrix()*(frame_ref_->pos() + scale*(frame_cur->pos() - frame_ref_->pos()));

  // For each inlier create 3D point and add feature in both frames
  SE3d T_world_cur = frame_cur->T_f_w_.inverse();

  for(vector<int>::iterator it=inliers_.begin(); it!=inliers_.end(); ++it)
  {

    Vector2d px_cur(px_cur_[*it].x, px_cur_[*it].y);
    Vector2d px_ref(px_ref_[*it].x, px_ref_[*it].y);
    Vector3d fts_type(fts_type_[*it][0],fts_type_[*it][1],fts_type_[*it][2]);
    if(frame_ref_->cam_->isInFrame(px_cur.cast<int>(), 10) && frame_ref_->cam_->isInFrame(px_ref.cast<int>(), 10) && xyz_in_cur_[*it].z() > 0)
    {
      Vector3d pos = T_world_cur * (xyz_in_cur_[*it]*scale);
      Point* new_point = new Point(pos);

      if(fts_type[2] == 0)
      {
          Feature* ftr_cur(new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it], 0));
          frame_cur->addFeature(ftr_cur);
          new_point->addFrameRef(ftr_cur);

          Feature* ftr_ref(new Feature(frame_ref_.get(), new_point, px_ref, f_ref_[*it], 0));
          frame_ref_->addFeature(ftr_ref);
          new_point->addFrameRef(ftr_ref);
      }else
      {
        Vector2d g(fts_type[0],fts_type[1]);
        Feature* ftr_cur(new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it],g, 0));
        frame_cur->addFeature(ftr_cur);
        new_point->addFrameRef(ftr_cur);

        Feature* ftr_ref(new Feature(frame_ref_.get(), new_point, px_ref, f_ref_[*it],g, 0));
        frame_ref_->addFeature(ftr_ref);
        new_point->addFrameRef(ftr_ref);
      }

    }
  }
  return SUCCESS;
}

void KltHomographyInit::reset()
{
  px_cur_.clear();
  frame_ref_.reset();
}

void detectFeatures(
    FramePtr frame,
    vector<Vector3d>& fts_type,
    vector<cv::Point2f>& px_vec,
    vector<Vector3d>& f_vec)
{
  Features new_features;
  feature_detection::FastDetector detector(
      frame->img().cols, frame->img().rows, Config::gridSize(), Config::nPyrLevels());
  detector.detect(frame.get(), frame->img_pyr_, Config::triangMinCornerScore(), new_features);


  feature_detection::EdgeDetector edge_detector( frame->img().cols, frame->img().rows, Config::gridSize(), Config::nPyrLevels());
  edge_detector.detect(frame.get(), frame->img_pyr_, 10, new_features);


  // now for all maximum corners, initialize a new seed
  px_vec.clear(); px_vec.reserve(new_features.size());
  f_vec.clear(); f_vec.reserve(new_features.size());
  std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){

    Vector3d fts_type_temp;
    if(ftr->type == Feature::EDGELET)
    {
         fts_type_temp[0] = ftr->grad[0];
         fts_type_temp[1] = ftr->grad[1];
         fts_type_temp[2] = 1;
    }
    else
    {
      fts_type_temp[0] = ftr->grad[0];
      fts_type_temp[1] = ftr->grad[1];
      fts_type_temp[2] = 0;
    }

    fts_type.push_back(fts_type_temp);
    px_vec.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
    f_vec.push_back(ftr->f);
    delete ftr;
  });
}
/*
void trackKlt(
    FramePtr frame_ref,
    FramePtr frame_cur,
    vector<cv::Point2f>& px_ref,
    vector<cv::Point2f>& px_cur,
    vector<Vector3d>& f_ref,
    vector<Vector3d>& f_cur,
    vector<double>& disparities)
{
  const double klt_win_size = 30.0; //30
  const int klt_max_iter = 30;
  const double klt_eps = 0.001;
  vector<uchar> status;
  vector<float> error;
  vector<float> min_eig_vec;
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
  cv::calcOpticalFlowPyrLK(frame_ref->img_pyr_[0], frame_cur->img_pyr_[0],
                           px_ref, px_cur,
                           status, error,
                           cv::Size2i(klt_win_size, klt_win_size),
                           3, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);

  vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
  vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
  vector<Vector3d>::iterator f_ref_it = f_ref.begin();
  f_cur.clear(); f_cur.reserve(px_cur.size());
  disparities.clear(); disparities.reserve(px_cur.size());
  for(size_t i=0; px_ref_it != px_ref.end(); ++i)
  {
    if(!status[i])
    {
      px_ref_it = px_ref.erase(px_ref_it);
      px_cur_it = px_cur.erase(px_cur_it);
      f_ref_it = f_ref.erase(f_ref_it);
      continue;
    }
    f_cur.push_back(frame_cur->c2f(px_cur_it->x, px_cur_it->y));
    disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());
    ++px_ref_it;
    ++px_cur_it;
    ++f_ref_it;
  }
}
*/

void trackKlt(
    FramePtr frame_ref,
    FramePtr frame_cur,
    vector<Vector3d>& fts_type,
    vector<cv::Point2f>& px_ref,
    vector<cv::Point2f>& px_cur,
    vector<Vector3d>& f_ref,
    vector<Vector3d>& f_cur,
    vector<double>& disparities,
    cv::Mat& img_prev,
    vector<cv::Point2f>& px_prev)
{
  const double klt_win_size = 30.0; //30
  const int klt_max_iter = 30;
  const double klt_eps = 0.001;
  vector<uchar> status;
  vector<float> error;
  vector<float> min_eig_vec;
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);

  cv::calcOpticalFlowPyrLK(img_prev, frame_cur->img_pyr_[0],
                           px_prev, px_cur,
                           status, error,
                           cv::Size2i(klt_win_size, klt_win_size),
                           3, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);

  vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
  vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
  vector<Vector3d>::iterator f_ref_it = f_ref.begin();

  vector<Vector3d>::iterator fts_type_it = fts_type.begin();

  f_cur.clear(); f_cur.reserve(px_cur.size());
  disparities.clear(); disparities.reserve(px_cur.size());
  for(size_t i=0; px_ref_it != px_ref.end(); ++i)
  {
    if(!status[i])
    {
      px_ref_it = px_ref.erase(px_ref_it);
      px_cur_it = px_cur.erase(px_cur_it);
      f_ref_it = f_ref.erase(f_ref_it);

      fts_type_it = fts_type.erase(fts_type_it);
      continue;
    }
    f_cur.push_back(frame_cur->c2f(px_cur_it->x, px_cur_it->y));
    disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());
    ++fts_type_it;
    ++px_ref_it;
    ++px_cur_it;
    ++f_ref_it;
  }

  img_prev = frame_cur->img_pyr_[0].clone();
  px_prev = px_cur;
}

#define ESSENTIAL
//#define FUNDAMENTA
#define HOMOGRAPHY
void computeHomography(
    const vector<Vector3d>& f_ref,
    const vector<Vector3d>& f_cur,
    double focal_length,
    double reprojection_threshold,
    vector<int>& inliers,
    vector<Vector3d>& xyz_in_cur,
    SE3d& T_cur_from_ref,
    int method_choose)
{
  vector<int> outliers;

  if(2 == method_choose)    //  E method
  {
#ifdef ESSENTIAL
  // ** 5 point algorithm find E

  /*
  cv::Mat x1(f_ref.size(), 2,CV_64F);
  cv::Mat x2(f_cur.size(), 2,CV_64F);
  for(size_t i=0, i_max=f_ref.size(); i<i_max; ++i)
  {
    Vector3d v = f_ref[i];
    x1.ptr<double>(i)[0] = v[0]/v[2];
    x1.ptr<double>(i)[1] = v[1]/v[2];

    v = f_cur[i];
    x2.ptr<double>(i)[0] = v[0]/v[2];
    x2.ptr<double>(i)[1] = v[1]/v[2];

  }
  */

  vector<cv::Point2f> x1(f_ref.size()), x2(f_cur.size());
  for(size_t i=0; i<f_ref.size(); ++i)
  {
    x1[i] = cv::Point2f(f_ref[i][0] / f_ref[i][2], f_ref[i][1]/f_ref[i][2]);
    x2[i] = cv::Point2f(f_cur[i][0] / f_cur[i][2], f_cur[i][1]/f_cur[i][2]);
  }

  cv::Point2d pp(0,0);
  double focal = 1;
  cv::Mat E = cv::findEssentialMat(x1, x2, focal, pp, CV_RANSAC, 0.99, 2.0/focal_length, cv::noArray() );
  std::cout << "=======================findEssentialMat==============================" << std::endl;
  cv::Mat R_cf,t_cf;
  recoverPose(E,x1,x2,R_cf,t_cf,focal,pp);
  Vector3d t;
  Matrix3d R;
  R << R_cf.at<double>(0,0), R_cf.at<double>(0,1), R_cf.at<double>(0,2),
       R_cf.at<double>(1,0), R_cf.at<double>(1,1), R_cf.at<double>(1,2),
       R_cf.at<double>(2,0), R_cf.at<double>(2,1), R_cf.at<double>(2,2);
  t<< t_cf.at<double>(0),t_cf.at<double>(1),t_cf.at<double>(2);

  SE3d Tcf(R,t);
  T_cur_from_ref = Tcf;

  svo::computeInliers(f_cur, f_ref,
                     R, t,
                     reprojection_threshold, focal_length,
                     xyz_in_cur, inliers, outliers);

  SVO_INFO_STREAM("Init: findEssentialMat RANSAC "<<inliers.size()<<" inliers.");
  //std::cout<<"R: "<<R_cf<<std::endl;
  //std::cout<<"t: "<<t_cf<<std::endl;
  std::cout<<R<<std::endl<<t<<std::endl;
  // std::cout<<"T_cur_from_ref: "<<T_cur_from_ref<<std::endl;

  int depth_error(0);
   for(vector<int>::iterator it=inliers.begin(); it!=inliers.end(); ++it)
   {
       if( xyz_in_cur[*it].z() < 0)
         depth_error ++;
   }
    std::cout << "depth error num: "  << depth_error << std::endl;
   if(depth_error * 1.0/ inliers.size() > 0.3 )
     method_choose = 1;
/*
  std::ofstream yj_file;
  yj_file.open("E_xyz.txt");
  if(yj_file.is_open())
  {
      for(vector<int>::iterator it=inliers.begin(); it!=inliers.end(); ++it)
      {
          yj_file<<"id: "<<*it<<"\t"
                          <<std::fixed
                           << xyz_in_cur[*it].x() << "\t"
                           << xyz_in_cur[*it].y() << "\t"
                           << xyz_in_cur[*it].z() << "\t"
                           << std::endl;
      }

  }
    yj_file.close();
*/
#endif

#ifdef FUNDAMENTA
    vector<cv::Point2f> x1(f_ref.size()), x2(f_cur.size());
    for(size_t i=0; i<f_ref.size(); ++i)
    {
      x1[i] = cv::Point2f(f_ref[i][0] / f_ref[i][2], f_ref[i][1]/f_ref[i][2]);
      x2[i] = cv::Point2f(f_cur[i][0] / f_cur[i][2], f_cur[i][1]/f_cur[i][2]);
    }

    cv::Point2d pp(0,0);
    double focal = 1;
   std::cout << "=======================findFundamentalMat==============================" << std::endl;
   cv::Mat F = cv::findFundamentalMat(x1,x2,FM_RANSAC,2.0/focal_length , 0.99, noArray() );

   Mat R_cf,t_cf;
   recoverPose(F,x1,x2,R_cf,t_cf,focal,pp);
   Vector3d t;
   Matrix3d R;
   R << R_cf.at<double>(0,0), R_cf.at<double>(0,1), R_cf.at<double>(0,2),
        R_cf.at<double>(1,0), R_cf.at<double>(1,1), R_cf.at<double>(1,2),
        R_cf.at<double>(2,0), R_cf.at<double>(2,1), R_cf.at<double>(2,2);
   t<< t_cf.at<double>(0),t_cf.at<double>(1),t_cf.at<double>(2);

   SE3d Tcf(R,t);
   T_cur_from_ref = Tcf;

   svo::computeInliers(f_cur, f_ref,
                      R, t,
                      reprojection_threshold, focal_length,
                      xyz_in_cur, inliers, outliers);

   SVO_INFO_STREAM("Init: findFundamentalMat  RANSAC "<<inliers.size()<<" inliers.");
   //std::cout<<"R: "<<R_cf<<std::endl;
   //std::cout<<"t: "<<t_cf<<std::endl;
   std::cout<<R<<std::endl<<t<<std::endl;
  //  std::cout<<"T_cur_from_ref: "<<T_cur_from_ref<<std::endl;

   int depth_error(0);
    for(vector<int>::iterator it=inliers.begin(); it!=inliers.end(); ++it)
    {
        if( xyz_in_cur[*it].z() < 0)
          depth_error ++;
    }
    std::cout << "depth error num: "  << depth_error << std::endl;
    if(depth_error>4)
      method_choose = 1;

#endif

  }

  if(1 == method_choose)
  {
#ifdef HOMOGRAPHY
  vector<Vector2d, aligned_allocator<Vector2d> > uv_ref(f_ref.size());
  vector<Vector2d, aligned_allocator<Vector2d> > uv_cur(f_cur.size());
  for(size_t i=0, i_max=f_ref.size(); i<i_max; ++i)
  {
    uv_ref[i] = svo::project2d(f_ref[i]);
    uv_cur[i] = svo::project2d(f_cur[i]);
  }
  svo::Homography Homography(uv_ref, uv_cur, focal_length, reprojection_threshold);
  Homography.computeSE3fromMatches();
 svo::computeInliers(f_cur, f_ref,
                     Homography.T_c2_from_c1.rotationMatrix(), Homography.T_c2_from_c1.translation(),
                     reprojection_threshold, focal_length,
                     xyz_in_cur, inliers, outliers);
  T_cur_from_ref = Homography.T_c2_from_c1;

  std::cout << "======================Homography===============================" << std::endl;
/*
  std::ofstream hyj_file;
  hyj_file.open("H_xyz.txt");
  if(hyj_file.is_open())
  {
      for(vector<int>::iterator it=inliers.begin(); it!=inliers.end(); ++it)
      {
          hyj_file<<"id: "<<*it<<"\t"
                 <<std::fixed
                  << xyz_in_cur[*it].x() << "\t"
                  << xyz_in_cur[*it].y() << "\t"
                  << xyz_in_cur[*it].z() << "\t"
                  << std::endl;
      }
  }
  hyj_file.close();
*/
  std::cout<<T_cur_from_ref.rotationMatrix()<<std::endl;
  std::cout<<T_cur_from_ref.translation()<<std::endl;
  // std::cout<<"T_cur_from_ref: "<<T_cur_from_ref<<std::endl;
  SVO_INFO_STREAM("Init: Homography RANSAC "<<inliers.size()<<" inliers.");

#endif
  }

   int iiii=0;

}


} // namespace initialization
} // namespace svo
