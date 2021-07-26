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

/*
** 重投影匹配与极线搜索
*/
#ifndef SVO_MATCHER_H_
#define SVO_MATCHER_H_

#include <svo/global.h>

namespace vk {
  class AbstractCamera;
  namespace patch_score {
    template<int HALF_PATCH_SIZE> class ZMSSD;
  }
}

namespace svo {

class Point;
class Frame;
class Feature;

/// Warp a patch from the reference view to the current view.
// 将patch从参考视图扭曲到当前视图
namespace warp {

void getWarpMatrixAffine(
    const vk::AbstractCamera& cam_ref,
    const vk::AbstractCamera& cam_cur,
    const Vector2d& px_ref,
    const Vector3d& f_ref,
    const double depth_ref,
    const SE3& T_cur_ref,
    const int level_ref,
    Matrix2d& A_cur_ref);

int getBestSearchLevel(
    const Matrix2d& A_cur_ref,
    const int max_level);

void warpAffine(
    const Matrix2d& A_cur_ref,
    const cv::Mat& img_ref,
    const Vector2d& px_ref,
    const int level_ref,
    const int level_cur,
    const int halfpatch_size,
    uint8_t* patch);

} // namespace warp

/// Patch-matcher for reprojection-matching and epipolar search in triangulation.
// 三角化中用于重投影匹配和极线搜索的patch匹配器
class Matcher
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // 这是搜索patch的尺寸吧
  static const int halfpatch_size_ = 4;
  static const int patch_size_ = 8;

  typedef vk::patch_score::ZMSSD<halfpatch_size_> PatchScore;

  struct Options
  {
    bool align_1d;              //!< in epipolar search: align patch 1D along epipolar line 标志位，极线搜索：沿着极线进行patch匹配
    int align_max_iter;         //!< number of iterations for aligning the feature patches in gauss newton
    double max_epi_length_optim;//!< max length of epipolar line to skip epipolar search and directly go to img align
    size_t max_epi_search_steps;//!< max number of evaluations along epipolar line 极线搜索中的最大步数
    bool subpix_refinement;     //!< do gauss newton feature patch alignment after epipolar search
    bool epi_search_edgelet_filtering;
    double epi_search_edgelet_max_angle;
    Options() :
      align_1d(false),
      align_max_iter(10),
      max_epi_length_optim(2.0),
      max_epi_search_steps(1000),
      subpix_refinement(true),
      epi_search_edgelet_filtering(true),
      epi_search_edgelet_max_angle(0.7)
    {}
  } options_;

  uint8_t patch_[patch_size_*patch_size_] __attribute__ ((aligned (16)));
  uint8_t patch_with_border_[(patch_size_+2)*(patch_size_+2)] __attribute__ ((aligned (16)));
  Matrix2d A_cur_ref_;          //!< affine warp matrix 仿射变换矩阵？？？
  Vector2d epi_dir_;			// 极线方向？？？ 
  double epi_length_;           //!< length of epipolar line segment in pixels (only used for epipolar search)
								// 以像素为单位的极线长度（只能用于极线搜索）
  double h_inv_;                //!< hessian of 1d image alignment along epipolar line
  int search_level_;
  bool reject_;
  Feature* ref_ftr_;
  Vector2d px_cur_;

  Matcher() = default;
  ~Matcher() = default;

  /// Find a match by directly applying subpix refinement.
  /// IMPORTANT! This function assumes that px_cur is already set to an estimate that is within ~2-3 pixel of the final result!
  // 通过直接应用像素精炼寻找匹配点。注意！！！此函数假设px_cur已设置为最终结果的~2-3像素范围内的估计值！
  bool findMatchDirect(
      const Point& pt,
      const Frame& frame,
      Vector2d& px_cur);

  /// Find a match by searching along the epipolar line without using any features.
  // 在不使用任何特征的情况下，通过沿极线搜索来查找匹配项
  bool findEpipolarMatchDirect(
      const Frame& ref_frame,
      const Frame& cur_frame,
      const Feature& ref_ftr,
      const double d_estimate,
      const double d_min,
      const double d_max,
      double& depth);

  void createPatchFromPatchWithBorder();
};

} // namespace svo

#endif // SVO_MATCHER_H_
