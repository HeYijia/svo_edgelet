#ifndef SPARSE_ALIGN_H
#define SPARSE_ALIGN_H

//#include <vikit/nlls_solver.h>
//#include <vikit/performance_monitor.h>
#include <svo/global.h>
#define D 6
namespace svo {

class AbstractCamera;
class Feature;

/// Optimize the pose of the frame by minimizing the photometric error of feature patches.
class SparseAlign
{
  static const int patch_halfsize_ = 2;
  static const int patch_size_ = 2*patch_halfsize_;
  static const int patch_area_ = patch_size_*patch_size_;  //  16 pixels

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  cv::Mat resimg_;

  SparseAlign(
      int n_levels,
      int min_level,
      int n_iter,
      //Method method,
      bool display,
      bool verbose);

  size_t run(
      FramePtr ref_frame,
      FramePtr cur_frame);

  /// Return fisher information matrix, i.e. the Hessian of the log-likelihood
  /// at the converged state.
  Matrix<double, 6, 6> getFisherInformation();

protected:
  FramePtr ref_frame_;            //!< reference frame, has depth for gradient pixels.
  FramePtr cur_frame_;            //!< only the image is known!
  int level_;                     //!< current pyramid level on which the optimization runs.
  bool display_;                  //!< display residual image.
  int max_level_;                 //!< coarsest pyramid level for the alignment.
  int min_level_;                 //!< finest pyramid level for the alignment.

  Matrix<double, D, D>  H_;       //!< Hessian approximation
  Matrix<double, D, 1>  Jres_;    //!< Jacobian x Residual
  Matrix<double, D, 1>  x_;       //!< update step

  size_t                n_iter_init_, n_iter_;  //!< Number of Iterations
  size_t               n_meas_;                //!< Number of measurements
  bool                  stop_;                  //!< Stop flag
  bool                  verbose_;               //!< Output Statistics
  double             eps_;                   //!< Stop if update norm is smaller than eps
  size_t                iter_;                  //!< Current Iteration

  double                chi2_;
  double                rho_;

  // robust least squares
  bool                  use_weights_;
  float                 scale_;
  //robust_cost::ScaleEstimatorPtr scale_estimator_;
  //robust_cost::WeightFunctionPtr weight_function_;


  // cache:
  Matrix<double, 6, Dynamic, ColMajor> jacobian_cache_;
  bool have_ref_patch_cache_;
  cv::Mat ref_patch_cache_;
  std::vector<bool> visible_fts_;

  void precomputeReferencePatches();
  void optimize(SE3& model);
  void reset();
  double computeResiduals(const SE3& model, bool linearize_system, bool compute_weight_scale = false);
  int solve();
  void update (const SE3& old_model, SE3& new_model);
  void startIteration();
  void finishIteration();
};

} // namespace svo

#endif // SPARSE_ALIGN_H
