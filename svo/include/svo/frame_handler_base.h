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
** 视觉前端基础类
*/
#ifndef SVO_FRAME_HANDLER_BASE_H_
#define SVO_FRAME_HANDLER_BASE_H_

#include <queue>
#include <vikit/timer.h>
#include <vikit/ringbuffer.h>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <svo/global.h>
#include <svo/map.h>

namespace vk
{
class AbstractCamera;
class PerformanceMonitor;
}

namespace svo
{
class Point;
class Matcher;
class DepthFilter;

/// Base class for various VO pipelines. Manages the map and the state machine.
class FrameHandlerBase : boost::noncopyable
{
public:
	// 系统当前状态
  enum Stage {
    STAGE_PAUSED,	//暂停
    STAGE_FIRST_FRAME,	//第一帧
    STAGE_SECOND_FRAME,	//第二帧
    STAGE_DEFAULT_FRAME,	//默认帧？？？
    STAGE_RELOCALIZING	//重定位？？？
  };

  // 跟踪质量
  enum TrackingQuality {
    TRACKING_INSUFFICIENT,	//不足？？？
    TRACKING_BAD,	//差
    TRACKING_GOOD	//优
  };
  enum UpdateResult {
    RESULT_NO_KEYFRAME,
    RESULT_IS_KEYFRAME,
    RESULT_FAILURE
  };

  FrameHandlerBase();

  virtual ~FrameHandlerBase();

  /// Get the current map.
  const Map& map() const { return map_; }

  /// Will reset the map as soon as the current frame is finished processing.
  // 当当前帧完成处理后就会触发地图重置功能，用户可自定义
  void reset() { set_reset_ = true; }

  /// Start processing.
  // 当有图像传入时，就开启系统，用户可自定义
  // 在vo_node.cpp文件中的VoNode::VoNode()函数中调用
  void start() { set_start_ = true; }

  /// Get the current stage of the algorithm.
  // 当前状态
  Stage stage() const { return stage_; }

  /// Get tracking quality.
  // 跟踪的质量
  TrackingQuality trackingQuality() const { return tracking_quality_; }

  /// Get the processing time of the previous iteration.
  // 前一次迭代的时间
  double lastProcessingTime() const { return timer_.getTime(); }

  /// Get the number of feature observations of the last frame.
  // 最后一帧中的特征点数量
  size_t lastNumObservations() const { return num_obs_last_; }

protected:
  Stage stage_;                 //!< Current stage of the algorithm.系统当前状态
  bool set_reset_;              //!< Flag that the user can set. Will reset the system before the next iteration.复位标志位
  bool set_start_;              //!< Flag the user can set to start the system when the next image is received.开启系统标志位
  Map map_;                     //!< Map of keyframes created by the slam system.关键帧产生的地图
  vk::Timer timer_;             //!< Stopwatch to measure time to process frame.
  vk::RingBuffer<double> acc_frame_timings_;    //!< Total processing time of the last 10 frames, used to give some user feedback on the performance.最近10帧的处理时间
  vk::RingBuffer<size_t> acc_num_obs_;          //!< Number of observed features of the last 10 frames, used to give some user feedback on the tracking performance.最近10帧中的特征点数量
  size_t num_obs_last_;                         //!< Number of observations in the previous frame.前一帧中特征点的数量
  TrackingQuality tracking_quality_;            //!< An estimate of the tracking quality based on the number of tracked features.根据跟踪点的数量判断跟踪的质量

  /// Before a frame is processed, this function is called.
  bool startFrameProcessingCommon(const double timestamp);

  /// When a frame is finished processing, this function is called.
  int finishFrameProcessingCommon(
      const size_t update_id,
      const UpdateResult dropout,
      const size_t num_observations);

  /// Reset the map and frame handler to start from scratch.
  void resetCommon();

  /// Reset the frame handler. Implement in derived class.
  virtual void resetAll() { resetCommon(); }

  /// Set the tracking quality based on the number of tracked features.
  virtual void setTrackingQuality(const size_t num_observations);

  /// Optimize some of the observed 3D points.
  // 优化观察到的3D点
  virtual void optimizeStructure(FramePtr frame, size_t max_n_pts, int max_iter);
};

} // namespace nslam

#endif // SVO_FRAME_HANDLER_BASE_H_
