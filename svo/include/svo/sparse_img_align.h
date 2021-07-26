
/*
 ** 直接法优化位姿（最小化光度误差）
*/
#ifndef SVO_SPARSE_IMG_ALIGN_H_
#define SVO_SPARSE_IMG_ALIGN_H_

#include <vikit/nlls_solver.h>
#include <vikit/performance_monitor.h>
#include <svo/global.h>

namespace vk {
class AbstractCamera;
}

namespace svo {

class Feature;

/// Optimize the pose of the frame by minimizing the photometric error of feature patches.
class SparseImgAlign : public vk::NLLSSolver<6, SE3>
{
  static const int patch_halfsize_ = 2;
  static const int patch_size_ = 2*patch_halfsize_;
  static const int patch_area_ = patch_size_*patch_size_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  cv::Mat resimg_;
  
  /*
  * 在frame_handler_mono.cpp文件中的FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()函数中初始化
  */
  SparseImgAlign(
      int n_levels,
      int min_level,
      int n_iter, // 迭代次数
      Method method,
      bool display,
      bool verbose);

  size_t run(
      FramePtr ref_frame,
      FramePtr cur_frame);

  /// Return fisher information matrix, i.e. the Hessian of the log-likelihood
  /// at the converged state.
  Matrix<double, 6, 6> getFisherInformation();

protected:
  FramePtr ref_frame_;            //!< reference frame, has depth for gradient pixels. 参考帧
  FramePtr cur_frame_;            //!< only the image is known!当前帧？？？ 
  int level_;                     //!< current pyramid level on which the optimization runs.当前优化所在的金字塔层数
  bool display_;                  //!< display residual image.是否显示残差图像
  int max_level_;                 //!< coarsest pyramid level for the alignment.图像对齐（匹配）的最大金字塔层数，粗对齐
  int min_level_;                 //!< finest pyramid level for the alignment.图像对齐（匹配）的最小金字塔层数，精对齐

  // cache:
  Matrix<double, 6, Dynamic, ColMajor> jacobian_cache_; // double:数值类型，6:行数，Dynamic:动态矩阵，其大小根据运算决定，ColMajor:按列存储
  bool have_ref_patch_cache_;
  cv::Mat ref_patch_cache_;
  std::vector<bool> visible_fts_; // 

  void precomputeReferencePatches();
  virtual double computeResiduals(const SE3& model, bool linearize_system, bool compute_weight_scale = false);
  virtual int solve();
  virtual void update (const ModelType& old_model, ModelType& new_model);
  virtual void startIteration();
  virtual void finishIteration();
};

} // namespace svo

#endif // SVO_SPARSE_IMG_ALIGN_H_
