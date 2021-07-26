
#include <cstdlib>
#include <vikit/abstract_camera.h>
#include <vikit/vision.h>
#include <vikit/math_utils.h>
#include <vikit/patch_score.h>
#include <svo/matcher.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/config.h>
#include <svo/feature_alignment.h>

namespace svo {

namespace warp {

/*
** parameters
* cam_ref: 参考相机
* cam_cur: 当前相机
* px_ref:
* f_ref:
* depth_ref:
* T_cur_ref:
* level_ref:
* A_cur_ref: 输出参数，当前帧与参考帧之间的仿射变换矩阵
* 在本文件中的bool Matcher::findMatchDirect()函数中调用
* 在本文件中的bool Matcher::findEpipolarMatchDirect()函数中调用
** function: 是在计算当前帧与参考帧之间的仿射变换吗？？？
*/
void getWarpMatrixAffine(
    const vk::AbstractCamera& cam_ref,
    const vk::AbstractCamera& cam_cur,
    const Vector2d& px_ref,
    const Vector3d& f_ref,
    const double depth_ref,
    const SE3& T_cur_ref,
    const int level_ref,
    Matrix2d& A_cur_ref)
{
  // Compute affine warp matrix A_ref_cur
  const int halfpatch_size = 5;
  const Vector3d xyz_ref(f_ref*depth_ref);
  Vector3d xyz_du_ref(cam_ref.cam2world(px_ref + Vector2d(halfpatch_size,0)*(1<<level_ref)));
  Vector3d xyz_dv_ref(cam_ref.cam2world(px_ref + Vector2d(0,halfpatch_size)*(1<<level_ref)));
  xyz_du_ref *= xyz_ref[2]/xyz_du_ref[2];
  xyz_dv_ref *= xyz_ref[2]/xyz_dv_ref[2];
  const Vector2d px_cur(cam_cur.world2cam(T_cur_ref*(xyz_ref)));
  const Vector2d px_du(cam_cur.world2cam(T_cur_ref*(xyz_du_ref)));
  const Vector2d px_dv(cam_cur.world2cam(T_cur_ref*(xyz_dv_ref)));
  A_cur_ref.col(0) = (px_du - px_cur)/halfpatch_size;
  A_cur_ref.col(1) = (px_dv - px_cur)/halfpatch_size;
}

/*
** parameters
* A_cur_ref: 当前帧与参考帧之间的仿射变换矩阵
* max_level: 最大的搜索层数 2层 因为从第0层开始 优化层数为3层
* 在本文件中的bool Matcher::findMatchDirect()函数中调用
* 在本文件中的bool Matcher::findEpipolarMatchDirect()函数中调用
** function: 获得最佳的搜索层数
*/
int getBestSearchLevel(
    const Matrix2d& A_cur_ref,
    const int max_level)
{
  // Compute patch level in other image
  int search_level = 0;
  // 计算矩阵的行列式
  double D = A_cur_ref.determinant();
  while(D > 3.0 && search_level < max_level)
  {
    search_level += 1;
    D *= 0.25;
  }
  return search_level;
}

/*
** parameters
* A_cur_ref: 当前帧与参考帧之间的仿射变换矩阵
* img_ref: 参考图像
* px_ref：
* level_ref：
* search_level：最佳搜索层数
* halfpatch_size：
* patch: 
* 在本文件中的bool Matcher::findMatchDirect()函数中调用
* 在本文件中的bool Matcher::findEpipolarMatchDirect()函数中调用
** function: 
*/
void warpAffine(
    const Matrix2d& A_cur_ref,
    const cv::Mat& img_ref,
    const Vector2d& px_ref,
    const int level_ref,
    const int search_level,
    const int halfpatch_size,
    uint8_t* patch)
{
  const int patch_size = halfpatch_size*2 ;
  const Matrix2f A_ref_cur = A_cur_ref.inverse().cast<float>();
  // isnan() 判断是不是NAN值（not a number非法数字）
  if(isnan(A_ref_cur(0,0)))
  {
    printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
    return;
  }

  // Perform the warp on a larger patch.
  // 在一个较大的patch中应用warp
  uint8_t* patch_ptr = patch;
  const Vector2f px_ref_pyr = px_ref.cast<float>() / (1<<level_ref);
  for (int y=0; y<patch_size; ++y)
  {
    for (int x=0; x<patch_size; ++x, ++patch_ptr)
    {
      Vector2f px_patch(x-halfpatch_size, y-halfpatch_size);
      px_patch *= (1<<search_level);
      const Vector2f px(A_ref_cur*px_patch + px_ref_pyr);
      if (px[0]<0 || px[1]<0 || px[0]>=img_ref.cols-1 || px[1]>=img_ref.rows-1)
        *patch_ptr = 0;
      else
        *patch_ptr = (uint8_t) vk::interpolateMat_8u(img_ref, px[0], px[1]);// vikit库中的函数
    }
  }
}

} // namespace warp

/*
** parameters
* T_search_ref: 参考帧与当前帧之间的变换矩阵？？？
* f_ref: 参考帧中特征点的方向向量
* f_cur: 当前帧中的像素在世界坐标系下的值
* depth: 返回值，z轴值，即深度
* 在本文件中的bool Matcher::findEpipolarMatchDirect()函数中的调用
** function: 用于在匹配完成后的三角化深度估计
*/
bool depthFromTriangulation(
    const SE3& T_search_ref,
    const Vector3d& f_ref,
    const Vector3d& f_cur,
    double& depth)
{
  Matrix<double,3,2> A; A << T_search_ref.rotation_matrix() * f_ref, f_cur;
  const Matrix2d AtA = A.transpose()*A;
  // 如果行列式为负数，则三角化失败
  if(AtA.determinant() < 0.000001)
    return false;
  const Vector2d depth2 = - AtA.inverse()*A.transpose()*T_search_ref.translation();
  depth = fabs(depth2[0]);
  return true; // 获得深度depth，返回true
}

/*
* 在本文件中的bool Matcher::findMatchDirect()函数中调用
* 在本文件中的bool Matcher::findEpipolarMatchDirect()函数中调用
** function: 这个好像是在创建一个带有边缘的patch？？？
*/
void Matcher::createPatchFromPatchWithBorder()
{
  uint8_t* ref_patch_ptr = patch_;
  for(int y=1; y<patch_size_+1; ++y, ref_patch_ptr += patch_size_)
  {
    uint8_t* ref_patch_border_ptr = patch_with_border_ + y*(patch_size_+2) + 1;
    for(int x=0; x<patch_size_; ++x)
      ref_patch_ptr[x] = ref_patch_border_ptr[x];
  }
}

/*
** parameters
* pt: 3D点
* cur_frame: 当前帧
* px_cur: 当前帧中的关键点的坐标
* 在文件reprojector.cpp文件中的bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)函数中调用
** function: 通过直接应用像素精炼寻找匹配点
*/
bool Matcher::findMatchDirect(
    const Point& pt,
    const Frame& cur_frame,
    Vector2d& px_cur)
{
	// 筛选出在每个特征点60度观察范围内的特征点
  if(!pt.getCloseViewObs(cur_frame.pos(), ref_ftr_))
    return false;

  // isInFrame()这个函数在哪定义？？？根据函数名，初步判断应该是判断点是否在帧内？？？
  if(!ref_ftr_->frame->cam_->isInFrame(ref_ftr_->px.cast<int>()/(1<<ref_ftr_->level), halfpatch_size_+2, ref_ftr_->level))
    return false;

  // warp affine
  // 是在计算当前帧与参考帧之间的仿射变换吗？？？
  warp::getWarpMatrixAffine(
      *ref_ftr_->frame->cam_, 
	  *cur_frame.cam_, 
	  ref_ftr_->px, ref_ftr_->f,
      (ref_ftr_->frame->pos() - pt.pos_).norm(),
      cur_frame.T_f_w_ * ref_ftr_->frame->T_f_w_.inverse(), 
	  ref_ftr_->level, A_cur_ref_);
  
  // 仿射变换矩阵、最大搜索层数，获得最佳的搜索层数
  search_level_ = warp::getBestSearchLevel(A_cur_ref_, Config::nPyrLevels()-1);

  warp::warpAffine(A_cur_ref_, ref_ftr_->frame->img_pyr_[ref_ftr_->level], ref_ftr_->px,
                   ref_ftr_->level, search_level_, halfpatch_size_+1, patch_with_border_);

  createPatchFromPatchWithBorder();

  // px_cur should be set
  Vector2d px_scaled(px_cur/(1<<search_level_));

  bool success = false;
  // 判断参考帧中的特征点的类型，选择不同的对齐(匹配)算法
  if(ref_ftr_->type == Feature::EDGELET) // 边上的点
  {
    Vector2d dir_cur(A_cur_ref_*ref_ftr_->grad);
    dir_cur.normalize();
	// 特征点对齐（匹配）
    success = feature_alignment::align1D(
          cur_frame.img_pyr_[search_level_], dir_cur.cast<float>(),
          patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
  }
  else // 角上的点
  {
    success = feature_alignment::align2D(
      cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
      options_.align_max_iter, px_scaled);
  }
  px_cur = px_scaled * (1<<search_level_);
  return success;
}

/*
** parameters
* ref_frame: 参考帧
* cur_frame: 当前帧
* ref_ftr：
* d_estimate: 正态分布均值的倒数
* d_min: z轴逆深度最小值的倒数
* d_max: z轴逆深度最大值的倒数
* depth: z轴值
* 在depth_filter.cpp文件中的void DepthFilter::updateSeeds(FramePtr frame)函数中调用
** function: 在不使用任何特征的情况下，通过沿极线搜索来查找匹配项
*/
bool Matcher::findEpipolarMatchDirect(
    const Frame& ref_frame,
    const Frame& cur_frame,
    const Feature& ref_ftr,
    const double d_estimate,
    const double d_min,
    const double d_max,
    double& depth)
{
	// 这个是两帧之间的变换矩阵吗？？？
  SE3 T_cur_ref = cur_frame.T_f_w_ * ref_frame.T_f_w_.inverse();
  int zmssd_best = PatchScore::threshold();
  Vector2d uv_best;

  // Compute start and end of epipolar line in old_kf for match search, on unit plane!
  // 在旧帧中计算极线的起始A和终点B，以便后面的搜索匹配
  Vector2d A = vk::project2d(T_cur_ref * (ref_ftr.f*d_min));
  Vector2d B = vk::project2d(T_cur_ref * (ref_ftr.f*d_max));
  epi_dir_ = A - B;// 极线方向？？？

  // Compute affine warp matrix
  // 计算仿射变换矩阵
  warp::getWarpMatrixAffine(
      *ref_frame.cam_, *cur_frame.cam_, ref_ftr.px, ref_ftr.f,
      d_estimate, T_cur_ref, ref_ftr.level, A_cur_ref_);

  // feature pre-selection
  // 特征预搜索
  reject_ = false;
  if(ref_ftr.type == Feature::EDGELET && options_.epi_search_edgelet_filtering)
  {
    const Vector2d grad_cur = (A_cur_ref_ * ref_ftr.grad).normalized();
    const double cosangle = fabs(grad_cur.dot(epi_dir_.normalized()));
    if(cosangle < options_.epi_search_edgelet_max_angle) {
      reject_ = true;
      return false;
    }
  }

  // 获得最佳的搜索层
  search_level_ = warp::getBestSearchLevel(A_cur_ref_, Config::nPyrLevels()-1);

  // Find length of search range on epipolar line
  // 在极线上获取搜索长度范围，当前帧中
  Vector2d px_A(cur_frame.cam_->world2cam(A));
  Vector2d px_B(cur_frame.cam_->world2cam(B));
  epi_length_ = (px_A-px_B).norm() / (1<<search_level_);

  // Warp reference patch at ref_level
  // 在当前层中，warp参考patch
  warp::warpAffine(A_cur_ref_, ref_frame.img_pyr_[ref_ftr.level], ref_ftr.px,
                   ref_ftr.level, search_level_, halfpatch_size_+1, patch_with_border_);
  createPatchFromPatchWithBorder();

  // 如果极线长度小于2.0，单位是像素
  if(epi_length_ < 2.0)
  {
    px_cur_ = (px_A+px_B)/2.0;
    Vector2d px_scaled(px_cur_/(1<<search_level_));
    bool res;
    if(options_.align_1d)
      res = feature_alignment::align1D(
          cur_frame.img_pyr_[search_level_], (px_A-px_B).cast<float>().normalized(),
          patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
    else
      res = feature_alignment::align2D(
          cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
          options_.align_max_iter, px_scaled);

	// 如果匹配成功，就执行三角化测量深度
    if(res)
    {
      px_cur_ = px_scaled*(1<<search_level_);
      if(depthFromTriangulation(T_cur_ref, ref_ftr.f, cur_frame.cam_->cam2world(px_cur_), depth))
        return true;// 三角化成功
    }
    return false; // 三角化失败
  }

  // 获取极线搜索的步数，每0.7个像素为一步
  // 获取在极线方向上的搜索步长，每步多少个像素
  size_t n_steps = epi_length_/0.7; // one step per pixel
  Vector2d step = epi_dir_/n_steps;

  // 如果大于最大搜索步数，打印警告信息，返回false
  if(n_steps > options_.max_epi_search_steps)
  {
    printf("WARNING: skip epipolar search: %zu evaluations, px_lenght=%f, d_min=%f, d_max=%f.\n",
           n_steps, epi_length_, d_min, d_max);
    return false;
  }

  // for matching, precompute sum and sum2 of warped reference patch
  int pixel_sum = 0;
  int pixel_sum_square = 0;
  PatchScore patch_score(patch_);// 这是什么类？？？

  // now we sample along the epipolar line
  // 沿着极线采样？？？B是参考帧中极线上的最大值，应该是逐步的减去步长
  Vector2d uv = B-step;
  Vector2i last_checked_pxi(0,0);
  ++n_steps;
  for(size_t i=0; i<n_steps; ++i, uv+=step)
  {
    Vector2d px(cur_frame.cam_->world2cam(uv));
    Vector2i pxi(px[0]/(1<<search_level_)+0.5,
                 px[1]/(1<<search_level_)+0.5); // +0.5 to round to closest int
	// 如果像素已经递减至与最小值相等，则退出
    if(pxi == last_checked_pxi)
      continue;
    last_checked_pxi = pxi;

    // check if the patch is full within the new frame
	// 检查patch是否全在新图像中
    if(!cur_frame.cam_->isInFrame(pxi, patch_size_, search_level_))
      continue;

    // TODO interpolation would probably be a good idea
	// 唔，没看懂。。。。。。zmssd？？？ zmssd_best？？？
    uint8_t* cur_patch_ptr = cur_frame.img_pyr_[search_level_].data
                             + (pxi[1]-halfpatch_size_)*cur_frame.img_pyr_[search_level_].cols
                             + (pxi[0]-halfpatch_size_);
    int zmssd = patch_score.computeScore(cur_patch_ptr, cur_frame.img_pyr_[search_level_].cols);

    if(zmssd < zmssd_best) {
      zmssd_best = zmssd;
      uv_best = uv;
    }
  }

  if(zmssd_best < PatchScore::threshold())
  {
    if(options_.subpix_refinement)
    {
      px_cur_ = cur_frame.cam_->world2cam(uv_best);
      Vector2d px_scaled(px_cur_/(1<<search_level_));
      bool res;
      if(options_.align_1d)
        res = feature_alignment::align1D(
            cur_frame.img_pyr_[search_level_], (px_A-px_B).cast<float>().normalized(),
            patch_with_border_, patch_, options_.align_max_iter, px_scaled, h_inv_);
      else
        res = feature_alignment::align2D(
            cur_frame.img_pyr_[search_level_], patch_with_border_, patch_,
            options_.align_max_iter, px_scaled);
      if(res)
      {
        px_cur_ = px_scaled*(1<<search_level_);
        if(depthFromTriangulation(T_cur_ref, ref_ftr.f, cur_frame.cam_->cam2world(px_cur_), depth))
          return true;
      }
      return false;
    }
    px_cur_ = cur_frame.cam_->world2cam(uv_best);
    if(depthFromTriangulation(T_cur_ref, ref_ftr.f, vk::unproject2d(uv_best).normalized(), depth))
      return true;
  }
  return false;
}

} // namespace svo
