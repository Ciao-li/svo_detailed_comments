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
** �Ӿ�ǰ�˻�����
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
	// ϵͳ��ǰ״̬
  enum Stage {
    STAGE_PAUSED,	//��ͣ
    STAGE_FIRST_FRAME,	//��һ֡
    STAGE_SECOND_FRAME,	//�ڶ�֡
    STAGE_DEFAULT_FRAME,	//Ĭ��֡������
    STAGE_RELOCALIZING	//�ض�λ������
  };

  // ��������
  enum TrackingQuality {
    TRACKING_INSUFFICIENT,	//���㣿����
    TRACKING_BAD,	//��
    TRACKING_GOOD	//��
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
  // ����ǰ֡��ɴ����ͻᴥ����ͼ���ù��ܣ��û����Զ���
  void reset() { set_reset_ = true; }

  /// Start processing.
  // ����ͼ����ʱ���Ϳ���ϵͳ���û����Զ���
  // ��vo_node.cpp�ļ��е�VoNode::VoNode()�����е���
  void start() { set_start_ = true; }

  /// Get the current stage of the algorithm.
  // ��ǰ״̬
  Stage stage() const { return stage_; }

  /// Get tracking quality.
  // ���ٵ�����
  TrackingQuality trackingQuality() const { return tracking_quality_; }

  /// Get the processing time of the previous iteration.
  // ǰһ�ε�����ʱ��
  double lastProcessingTime() const { return timer_.getTime(); }

  /// Get the number of feature observations of the last frame.
  // ���һ֡�е�����������
  size_t lastNumObservations() const { return num_obs_last_; }

protected:
  Stage stage_;                 //!< Current stage of the algorithm.ϵͳ��ǰ״̬
  bool set_reset_;              //!< Flag that the user can set. Will reset the system before the next iteration.��λ��־λ
  bool set_start_;              //!< Flag the user can set to start the system when the next image is received.����ϵͳ��־λ
  Map map_;                     //!< Map of keyframes created by the slam system.�ؼ�֡�����ĵ�ͼ
  vk::Timer timer_;             //!< Stopwatch to measure time to process frame.
  vk::RingBuffer<double> acc_frame_timings_;    //!< Total processing time of the last 10 frames, used to give some user feedback on the performance.���10֡�Ĵ���ʱ��
  vk::RingBuffer<size_t> acc_num_obs_;          //!< Number of observed features of the last 10 frames, used to give some user feedback on the tracking performance.���10֡�е�����������
  size_t num_obs_last_;                         //!< Number of observations in the previous frame.ǰһ֡�������������
  TrackingQuality tracking_quality_;            //!< An estimate of the tracking quality based on the number of tracked features.���ݸ��ٵ�������жϸ��ٵ�����

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
  // �Ż��۲쵽��3D��
  virtual void optimizeStructure(FramePtr frame, size_t max_n_pts, int max_iter);
};

} // namespace nslam

#endif // SVO_FRAME_HANDLER_BASE_H_
