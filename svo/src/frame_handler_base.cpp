
/*
** 视觉前端基础类
*/
#include <vikit/abstract_camera.h>
#include <stdlib.h>
#include <Eigen/StdVector>
#include <boost/bind.hpp>
#include <fstream>
#include <svo/frame_handler_base.h>
#include <svo/config.h>
#include <svo/feature.h>
#include <svo/matcher.h>
#include <svo/map.h>
#include <svo/point.h>

namespace svo
{

// definition of global and static variables which were declared in the header
#ifdef SVO_TRACE
vk::PerformanceMonitor* g_permon = NULL;
#endif

/*
在frame_handler_mono.cpp文件中的FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam)调用
*/
FrameHandlerBase::FrameHandlerBase() :
  stage_(STAGE_PAUSED),
  set_reset_(false),
  set_start_(false),
  acc_frame_timings_(10),
  acc_num_obs_(10),
  num_obs_last_(0),
  tracking_quality_(TRACKING_INSUFFICIENT)
{
#ifdef SVO_TRACE
  // Initialize Performance Monitor
  // 初始化性能监视器
  g_permon = new vk::PerformanceMonitor();
  g_permon->addTimer("pyramid_creation");
  g_permon->addTimer("sparse_img_align");
  g_permon->addTimer("reproject");
  g_permon->addTimer("reproject_kfs");
  g_permon->addTimer("reproject_candidates");
  g_permon->addTimer("feature_align");
  g_permon->addTimer("pose_optimizer");
  g_permon->addTimer("point_optimizer");
  g_permon->addTimer("local_ba");
  g_permon->addTimer("tot_time");
  g_permon->addLog("timestamp");
  g_permon->addLog("img_align_n_tracked");
  g_permon->addLog("repr_n_mps");
  g_permon->addLog("repr_n_new_references");
  g_permon->addLog("sfba_thresh");
  g_permon->addLog("sfba_error_init");
  g_permon->addLog("sfba_error_final");
  g_permon->addLog("sfba_n_edges_final");
  g_permon->addLog("loba_n_erredges_init");
  g_permon->addLog("loba_n_erredges_fin");
  g_permon->addLog("loba_err_init");
  g_permon->addLog("loba_err_fin");
  g_permon->addLog("n_candidates");
  g_permon->addLog("dropout");
  g_permon->init(Config::traceName(), Config::traceDir());
#endif

  SVO_INFO_STREAM("SVO initialized");
}

FrameHandlerBase::~FrameHandlerBase()
{
  SVO_INFO_STREAM("SVO destructor invoked");
#ifdef SVO_TRACE
  delete g_permon;
#endif
}

/*
** parameter
* timestamp:时间戳
* 在frame_handler_mono.cpp文件中的void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)函数中调用
*/
bool FrameHandlerBase::startFrameProcessingCommon(const double timestamp)
{
  if(set_start_)
  {
    resetAll();// 转frame_handler_mono.cpp文件
    stage_ = STAGE_FIRST_FRAME;// 系统状态置为STAGE_FIRST_FRAME
  }

  // 若系统状态为STAGE_PAUSED，则结束startFrameProcessingCommon，返回false
  if(stage_ == STAGE_PAUSED)
    return false;

  // 传进函数的系统时间和“New Frame”等信息记录至日志文件中
  SVO_LOG(timestamp);
  SVO_DEBUG_STREAM("New Frame");
  SVO_START_TIMER("tot_time");

  // 启动vk::Timer型变量timer_，用于计量程序执行时间，可精确到毫秒级
  timer_.start();

  // some cleanup from last iteration, can't do before because of visualization
  // 清空map_的垃圾箱trash，restCommon()函数已经做过了吧？？？
  map_.emptyTrash();

  // 返回true
  return true;
}

/*
** parameters
* update_id: 关键帧id
* dropout: 系统相应状态的返回
* num_observations: 观察到的特征点数
* 在frame_handler_mono.cpp文件中的void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)函数中调用
*/
int FrameHandlerBase::finishFrameProcessingCommon(
    const size_t update_id,
    const UpdateResult dropout,
    const size_t num_observations)
{
	// 将last_frame_信息记录至日志
  SVO_DEBUG_STREAM("Frame: "<<update_id<<"\t fps-avg = "<< 1.0/acc_frame_timings_.getMean()<<"\t nObs = "<<acc_num_obs_.getMean());
  SVO_LOG(dropout);

  // save processing time to calculate fps
  // 统计最近十帧的处理时间
  acc_frame_timings_.push_back(timer_.stop());

  // 若系统状态为STAGE_DEFAULT_FRAME，则统计最近十帧所获得的特征点总数
  if(stage_ == STAGE_DEFAULT_FRAME)
    acc_num_obs_.push_back(num_observations);
  num_obs_last_ = num_observations;
  SVO_STOP_TIMER("tot_time");

  // 条件编译
#ifdef SVO_TRACE
  // writeToFlie()函数在vikit库中，rpg_vikit/vikit_common/src/performance_monitor.cpp
  g_permon->writeToFile();
  {
	// 线程锁
    boost::unique_lock<boost::mutex> lock(map_.point_candidates_.mut_);
	// 记录特征点数量
    size_t n_candidates = map_.point_candidates_.candidates_.size();
    SVO_LOG(n_candidates);
  }
#endif

  // 判断系统状态
  if(dropout == RESULT_FAILURE &&
      (stage_ == STAGE_DEFAULT_FRAME || stage_ == STAGE_RELOCALIZING ))
  {
    stage_ = STAGE_RELOCALIZING;
    tracking_quality_ = TRACKING_INSUFFICIENT;
  }
  else if (dropout == RESULT_FAILURE)
	// 进行Map型变量map_的初始化（包括关键帧和候选点的清空等）
    // 同时stage_被改为STAGE_PAUSED，set_reset和set_start都被设置为false，还有其它一些设置。
    resetAll();// 转frame_handler_mono.cpp文件中
  if(set_reset_)
    resetAll(); // 同上

  return 0;
}

/*
在frame_handler_mono.cpp文件中的void FrameHandlerMono::resetAll()函数中调用
*/
void FrameHandlerBase::resetCommon()
{
  // Map型变量map_的初始化,包括关键帧和候选点的清空等
  map_.reset();

  // 将系统状态置为STAGE_PAUSED
  stage_ = STAGE_PAUSED;
  set_reset_ = false;
  set_start_ = false;
  tracking_quality_ = TRACKING_INSUFFICIENT;
  num_obs_last_ = 0;
  SVO_INFO_STREAM("RESET");
}

/*
** parameter
* num_observations: 观察到的3D点的数量
* 在frame_handler_mono.cpp文件中的FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()函数中调用
** function: 根据观察到的3D点的数量，判定跟踪的质量
*/
void FrameHandlerBase::setTrackingQuality(const size_t num_observations)
{
  // 先假设是好的跟踪
  tracking_quality_ = TRACKING_GOOD;
  // 若点数量小于最小的阈值，则认为跟踪的效果不好
  if(num_observations < Config::qualityMinFts())
  {
    SVO_WARN_STREAM_THROTTLE(0.5, "Tracking less than "<< Config::qualityMinFts() <<" features!");
    tracking_quality_ = TRACKING_INSUFFICIENT;
  }

  // 若丢失的特征点的数量大于阈值，则认为跟踪的效果不好
  // 总的点数减去观察到的点数，剩下的就是丢失的点数
  const int feature_drop = static_cast<int>(std::min(num_obs_last_, Config::maxFts())) - num_observations;
  if(feature_drop > Config::qualityMaxFtsDrop())
  {
    SVO_WARN_STREAM("Lost "<< feature_drop <<" features!");
    tracking_quality_ = TRACKING_INSUFFICIENT;
  }
}

bool ptLastOptimComparator(Point* lhs, Point* rhs)
{
  return (lhs->last_structure_optim_ < rhs->last_structure_optim_);
}

/*
** parameters
* frame: 输入帧
* max_n_pts: 每次迭代能够优化的最大数量的点
* max_iter: 在结构优化中的迭代次数
* 在frame_handler_mono.cpp文件中的FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()函数中调用
** function: 优化3D点信息，也可以说是优化位姿吗？？？
*/
void FrameHandlerBase::optimizeStructure(
    FramePtr frame,
    size_t max_n_pts,
    int max_iter)
{
  deque<Point*> pts;
  for(Features::iterator it=frame->fts_.begin(); it!=frame->fts_.end(); ++it)
  {
    if((*it)->point != NULL)
      pts.push_back((*it)->point);
  }
  max_n_pts = min(max_n_pts, pts.size());
  /*
  nth_element(first, nth, last, compare)
  求[first, last]这个区间中第n大小的元素，如果参数加入了compare函数，就按compare函数的方式比较。
  */
  nth_element(pts.begin(), pts.begin() + max_n_pts, pts.end(), ptLastOptimComparator);
  for(deque<Point*>::iterator it=pts.begin(); it!=pts.begin()+max_n_pts; ++it)
  {
    (*it)->optimize(max_iter);
    (*it)->last_structure_optim_ = frame->id_;
  }
}


} // namespace svo
