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
** SVO的全局配置
*/

#ifndef SVO_CONFIG_H_
#define SVO_CONFIG_H_

#include <string>
#include <stdint.h>
#include <stdio.h>

namespace svo {

using std::string;

/// Global configuration file of SVO.
/// Implements the Singleton design pattern to allow global access and to ensure
/// that only one instance exists.
class Config
{
public:

	// 实际上所有对外函数都是在调用这一个函数
  static Config& getInstance();

  /// Base-name of the tracefiles.
  // 跟踪文件的名称
  static string& traceName() { return getInstance().trace_name; }

  /// Directory where the tracefiles are saved.
  // 跟踪文件的保存路径
  static string& traceDir() { return getInstance().trace_dir; }

  /// Number of pyramid levels used for features.
  // 金字塔层数，论文里不是说5层吗？？？猜测是：论文里写的是一共5层，但是在第3层时已经收敛了，所以只使用3层即可。
  static size_t& nPyrLevels() { return getInstance().n_pyr_levels; }

  /// Use the IMU to get relative rotations.
  // 是否使用IMU获得相对旋转
  static bool& useImu() { return getInstance().use_imu; }

  /// Number of keyframes in the core. The core-kfs are optimized through bundle adjustment.
  // 在核心中的关键帧数量，核心关键帧通过BA优化。这个核心应该是指滑动窗口吧。。。
  static size_t& coreNKfs() { return getInstance().core_n_kfs; }

  /// Initial scale of the map. Depends on the distance the camera is moved for the initialization.
  // 地图的初始尺度，
  static double& mapScale() { return getInstance().map_scale; }

  /// Feature grid size of a cell in [px].
  // 一个cell中的网格数量
  static size_t& gridSize() { return getInstance().grid_size; }

  /// Initialization: Minimum required disparity between the first two frames.
  // 阈值，头两个关键帧之间的最小距离
  static double& initMinDisparity() { return getInstance().init_min_disparity; }

  /// Initialization: Minimum number of tracked features.
  // 阈值，最少的跟踪点的数量
  static size_t& initMinTracked() { return getInstance().init_min_tracked; }

  /// Initialization: Minimum number of inliers after RANSAC.
  // 阈值，RANSAC后最少的内点数量
  static size_t& initMinInliers() { return getInstance().init_min_inliers; }

  /// Maximum level of the Lucas Kanade tracker.
  // LK跟踪的最大层数
  static size_t& kltMaxLevel() { return getInstance().klt_max_level; }

  /// Minimum level of the Lucas Kanade tracker.
  // LK跟踪的最小层数
  static size_t& kltMinLevel() { return getInstance().klt_min_level; }

  /// Reprojection threshold [px].
  // 
  static double& reprojThresh() { return getInstance().reproj_thresh; }

  /// Reprojection threshold after pose optimization.
  // 阈值，重投影误差阈值，用于判定位姿优化
  static double& poseOptimThresh() { return getInstance().poseoptim_thresh; }

  /// Number of iterations in local bundle adjustment.
  // 局部BA的迭代次数
  static size_t& poseOptimNumIter() { return getInstance().poseoptim_num_iter; }

  /// Maximum number of points to optimize at every iteration.
  // 每次迭代能够优化的最大数量的点
  static size_t& structureOptimMaxPts() { return getInstance().structureoptim_max_pts; }

  /// Number of iterations in structure optimization.
  // 在结构优化中的迭代次数
  static size_t& structureOptimNumIter() { return getInstance().structureoptim_num_iter; }

  /// Reprojection threshold after bundle adjustment.
  // BA的重投影误差阈值
  static double& lobaThresh() { return getInstance().loba_thresh; }

  /// Threshold for the robust Huber kernel of the local bundle adjustment.
  // BA的Huber核阈值
  static double& lobaRobustHuberWidth() { return getInstance().loba_robust_huber_width; }

  /// Number of iterations in the local bundle adjustment.
  // 局部BA的迭代次数
  static size_t& lobaNumIter() { return getInstance().loba_num_iter; }

  /// Minimum distance between two keyframes. Relative to the average height in the map.
  // 阈值，两关键帧之间的最小距离。
  static double& kfSelectMinDist() { return getInstance().kfselect_mindist; }

  /// Select only features with a minimum Harris corner score for triangulation.
  // 阈值，用于选定特征点中用来三角化的Harris角点的最低得分
  static double& triangMinCornerScore() { return getInstance().triang_min_corner_score; }

  /// Subpixel refinement of reprojection and triangulation. Set to 0 if no subpix refinement required!
  // 
  static size_t& subpixNIter() { return getInstance().subpix_n_iter; }

  /// Limit the number of keyframes in the map. This makes nslam essentially.
  /// a Visual Odometry. Set to 0 if unlimited number of keyframes are allowed.
  /// Minimum number of keyframes is 3.
  // 
  static size_t& maxNKfs() { return getInstance().max_n_kfs; }

  /// How much (in milliseconds) is the camera delayed with respect to the imu.
  // 
  static double& imgImuDelay() { return getInstance().img_imu_delay; }

  /// Maximum number of features that should be tracked.
  // 阈值，用于跟踪的最大数量的特征点
  static size_t& maxFts() { return getInstance().max_fts; }

  /// If the number of tracked features drops below this threshold. Tracking quality is bad.
  // 阈值，用于衡量跟踪质量的好坏
  // 如果跟踪的点小于该阈值，则认为跟踪的效果不好
  static size_t& qualityMinFts() { return getInstance().quality_min_fts; }

  /// If within one frame, this amount of features are dropped. Tracking quality is bad.
  // 阈值，丢失的特征点的数量，则认为跟踪的效果不好
  static int& qualityMaxFtsDrop() { return getInstance().quality_max_drop_fts; }

private:
  Config();
  Config(Config const&);
  void operator=(Config const&);
  string trace_name;
  string trace_dir;
  size_t n_pyr_levels;
  bool use_imu;
  size_t core_n_kfs;
  double map_scale;
  size_t grid_size;
  double init_min_disparity;
  size_t init_min_tracked;
  size_t init_min_inliers;
  size_t klt_max_level;
  size_t klt_min_level;
  double reproj_thresh;
  double poseoptim_thresh;
  size_t poseoptim_num_iter;
  size_t structureoptim_max_pts;
  size_t structureoptim_num_iter;
  double loba_thresh;
  double loba_robust_huber_width;
  size_t loba_num_iter;
  double kfselect_mindist;
  double triang_min_corner_score;
  size_t triang_half_patch_size;
  size_t subpix_n_iter;
  size_t max_n_kfs;
  double img_imu_delay;
  size_t max_fts;
  size_t quality_min_fts;
  int quality_max_drop_fts;
};

} // namespace svo

#endif // SVO_CONFIG_H_
