#ifndef CAMERA_MODEL_H_
#define CAMERA_MODEL_H_

#include <stdlib.h>
#include <string>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

namespace svo {

using namespace std;
using namespace Eigen;

class AbstractCamera
{
protected:

  int width_;   // TODO cannot be const because of omni-camera model
  int height_;

public:

  AbstractCamera() {}; // need this constructor for omni camera
  AbstractCamera(int width, int height) : width_(width), height_(height) {};

  virtual ~AbstractCamera() {};

  /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Vector3d
  cam2world(const double& x, const double& y) const = 0;

  /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
  virtual Vector3d
  cam2world(const Vector2d& px) const = 0;

  virtual Vector2d
  world2cam(const Vector3d& xyz_c) const = 0;

  /// projects unit plane coordinates to camera coordinates
  virtual Vector2d
  world2cam(const Vector2d& uv) const = 0;

  // undistort a point
  virtual Vector2d
  undistortpoint(const double& u, const double& v)  const=0;  //  hyj

  virtual double
  errorMultiplier2() const = 0;

  virtual double
  errorMultiplier() const = 0;

  inline int width() const { return width_; }

  inline int height() const { return height_; }

  inline bool isInFrame(const Vector2i & obs, int boundary=0) const
  {
    if(obs[0]>=boundary && obs[0]<width()-boundary
        && obs[1]>=boundary && obs[1]<height()-boundary)
      return true;
    return false;
  }

  inline bool isInFrame(const Vector2i &obs, int boundary, int level) const
  {
    if(obs[0] >= boundary && obs[0] < width()/(1<<level)-boundary
        && obs[1] >= boundary && obs[1] <height()/(1<<level)-boundary)
      return true;
    return false;
  }
};

class PinholeCamera : public AbstractCamera {

private:
  const double fx_, fy_;
  const double cx_, cy_;
  bool distortion_;             //!< is it pure pinhole model or has it radial distortion?
  double d_[5];                 //!< distortion parameters, see http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
  cv::Mat cvK_, cvD_;
  cv::Mat undist_map1_, undist_map2_;
  bool use_optimization_;
  Matrix3d K_;
  Matrix3d K_inv_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PinholeCamera(double width, double height,
                double fx, double fy, double cx, double cy,
                double d0=0.0, double d1=0.0, double d2=0.0, double d3=0.0, double d4=0.0);

  ~PinholeCamera();

  void
  initUnistortionMap();

  virtual Vector3d
  cam2world(const double& x, const double& y) const;

  virtual Vector3d
  cam2world(const Vector2d& px) const;

  virtual Vector2d
  world2cam(const Vector3d& xyz_c) const;

  virtual Vector2d
  world2cam(const Vector2d& uv) const;

  virtual Vector2d
  undistortpoint(const double& u, const double& v)  const;  //  hyj

  const Vector2d focal_length() const
  {
    return Vector2d(fx_, fy_);
  }

  virtual double errorMultiplier2() const
  {
    return fabs(fx_);
  }

  virtual double errorMultiplier() const
  {
    return fabs(4.0*fx_*fy_);
  }

  inline const Matrix3d& K() const { return K_; };
  inline const Matrix3d& K_inv() const { return K_inv_; };
  inline double fx() const { return fx_; };
  inline double fy() const { return fy_; };
  inline double cx() const { return cx_; };
  inline double cy() const { return cy_; };
  inline double d0() const { return d_[0]; };
  inline double d1() const { return d_[1]; };
  inline double d2() const { return d_[2]; };
  inline double d3() const { return d_[3]; };
  inline double d4() const { return d_[4]; };

  void undistortImage(const cv::Mat& raw, cv::Mat& rectified);

};

} // end namespace svo


#endif /* CAMERA_MODEL_H_ */

