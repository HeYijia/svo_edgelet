/*
 * blender_utils.h
 *
 *  Created on: Feb 13, 2014
 *      Author: cforster
 */

#ifndef BLENDER_UTILS_H_
#define BLENDER_UTILS_H_

#include <list>
#include <string>
#include <svo/camera_model.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <Eigen/Core>

namespace svo {

/**
 * Entry has to support the following operator
 *   std::istream& operator >>(std::istream&, Entry&);
 */
template<class Entry>
class FileReader
{
public:
  FileReader(const std::string& file) :
    hasEntry_(false),
    file_(file),
    file_stream_(file.c_str())
  {}

  virtual ~FileReader()
  {
    file_stream_.close();
  }

  void skip(int num_lines)
  {
    for(int idx = 0; idx < num_lines; ++idx)
    {
      if(!file_stream_.good())  continue;
      file_stream_.ignore(1024, '\n');
      assert(file_stream_.gcount() < 1024);
    }
  }

  void skipComments()
  {
    while(file_stream_.good() && file_stream_.peek() == '#')
      skip(1);
  }

  /// Moves to the next entry in the file. Returns true, if there was a next entry, false otherwise.
  bool next()
  {
    if(file_stream_.good() && !file_stream_.eof())
    {
      file_stream_ >> entry_;
      hasEntry_ = true;
      return true;
    }
    return false;
  }

  /// Read all entries at once.
  void readAllEntries(std::vector<Entry>& entries)
  {
    if(!hasEntry()) next();
    do
      entries.push_back(entry());
    while(next());
  }

  /// Gets the current entry
  const Entry& entry() const { return entry_; }

  /// Determines whether the first entry was read
  const bool& hasEntry() const { return hasEntry_; }

private:
  bool hasEntry_;
  std::string file_;
  std::ifstream file_stream_;
  Entry entry_;
};

namespace blender_utils {

void loadBlenderDepthmap(
    const std::string file_name,
    const svo::AbstractCamera& cam,
    cv::Mat& img)
{
  std::ifstream file_stream(file_name.c_str());
  assert(file_stream.is_open());
  img = cv::Mat(cam.height(), cam.width(), CV_32FC1);
  float * img_ptr = img.ptr<float>();
  float depth;
  for(int y=0; y<cam.height(); ++y)
  {
    for(int x=0; x<cam.width(); ++x, ++img_ptr)
    {
      file_stream >> depth;
      // blender:
      Eigen::Vector3d xyz ( cam.cam2world(x,y)) ;
      Eigen::Vector2d uv = xyz.head<2>()/xyz[2];;
      *img_ptr = depth * sqrt(uv[0]*uv[0] + uv[1]*uv[1] + 1.0);

      // povray
      // *img_ptr = depth/100.0; // depth is in [cm], we want [m]

      if(file_stream.peek() == '\n' && x != cam.width()-1 && y != cam.height()-1)
        printf("WARNING: did not read the full depthmap!\n");
    }
  }
}

bool getDepthmapNormalAtPoint(
    const Vector2i& px,
    const cv::Mat& depth,
    const int halfpatch_size,
    const svo::AbstractCamera& cam,
    Vector3d& normal)
{
  assert(cam.width() == depth.cols && cam.height() == depth.rows);
  if(!cam.isInFrame(px, halfpatch_size+1))
    return false;

  const size_t n_meas = (halfpatch_size*2+1)*(halfpatch_size*2+1);
  list<Vector3d> pts;
  for(int y = px[1]-halfpatch_size; y<=px[1]+halfpatch_size; ++y)
    for(int x = px[0]-halfpatch_size; x<=px[0]+halfpatch_size; ++x)
      pts.push_back(cam.cam2world(x,y)*depth.at<float>(y,x));

  assert(n_meas == pts.size());
  Eigen::Matrix<double, Dynamic, 4> A; A.resize(n_meas, Eigen::NoChange);
  Eigen::Matrix<double, Dynamic, 1> b; b.resize(n_meas, Eigen::NoChange);

  size_t i = 0;
  for(list<Vector3d>::iterator it=pts.begin(); it!=pts.end(); ++it)
  {
    A.row(i) << it->x(), it->y(), it->z(), 1.0;
    b[i] = 0;
    ++i;
  }

  JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

  Eigen::Matrix<double, 4, 4> V = svd.matrixV();
  normal = V.block<3,1>(0,3);
  normal.normalize();
  return true;
}

namespace file_format
{

class ImageNameAndPose
{
public:
  ImageNameAndPose() {}
  virtual ~ImageNameAndPose() {}
  double timestamp_;
  std::string image_name_;
  Eigen::Vector3d t_;
  Eigen::Quaterniond q_;
  friend std::ostream& operator <<(std::ostream& out, const ImageNameAndPose& pair);
  friend std::istream& operator >>(std::istream& in, ImageNameAndPose& pair);
};

std::ostream& operator <<(std::ostream& out, const ImageNameAndPose& gt)
{
  out << gt.timestamp_ << " " << gt.image_name_ << " "
      << gt.t_.x()   << " " << gt.t_.y()   << " " << gt.t_.z()   << " "
      << gt.q_.x()   << " " << gt.q_.y()   << " " << gt.q_.z()   << " " << gt.q_.w()   << " " << std::endl;
  return out;
}

std::istream& operator >>(std::istream& in, ImageNameAndPose& gt)
{
  in >> gt.timestamp_;
  in >> gt.image_name_;
  double tx, ty, tz, qx, qy, qz, qw;
  in >> tx;
  in >> ty;
  in >> tz;
  in >> qx;
  in >> qy;
  in >> qz;
  in >> qw;
  gt.t_ = Eigen::Vector3d(tx, ty, tz);
  gt.q_ = Eigen::Quaterniond(qw, qx, qy, qz);
  gt.q_.normalize();
  return in;
}

} // namespace file_format
} // namespace blender_utils
} // namespace vk

#endif // VIKIT_BLENDER_UTILS_H_
