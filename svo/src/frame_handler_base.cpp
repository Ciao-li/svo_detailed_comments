
/*
** �Ӿ�ǰ�˻�����
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
��frame_handler_mono.cpp�ļ��е�FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam)����
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
  // ��ʼ�����ܼ�����
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
* timestamp:ʱ���
* ��frame_handler_mono.cpp�ļ��е�void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)�����е���
*/
bool FrameHandlerBase::startFrameProcessingCommon(const double timestamp)
{
  if(set_start_)
  {
    resetAll();// תframe_handler_mono.cpp�ļ�
    stage_ = STAGE_FIRST_FRAME;// ϵͳ״̬��ΪSTAGE_FIRST_FRAME
  }

  // ��ϵͳ״̬ΪSTAGE_PAUSED�������startFrameProcessingCommon������false
  if(stage_ == STAGE_PAUSED)
    return false;

  // ����������ϵͳʱ��͡�New Frame������Ϣ��¼����־�ļ���
  SVO_LOG(timestamp);
  SVO_DEBUG_STREAM("New Frame");
  SVO_START_TIMER("tot_time");

  // ����vk::Timer�ͱ���timer_�����ڼ�������ִ��ʱ�䣬�ɾ�ȷ�����뼶
  timer_.start();

  // some cleanup from last iteration, can't do before because of visualization
  // ���map_��������trash��restCommon()�����Ѿ������˰ɣ�����
  map_.emptyTrash();

  // ����true
  return true;
}

/*
** parameters
* update_id: �ؼ�֡id
* dropout: ϵͳ��Ӧ״̬�ķ���
* num_observations: �۲쵽����������
* ��frame_handler_mono.cpp�ļ��е�void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)�����е���
*/
int FrameHandlerBase::finishFrameProcessingCommon(
    const size_t update_id,
    const UpdateResult dropout,
    const size_t num_observations)
{
	// ��last_frame_��Ϣ��¼����־
  SVO_DEBUG_STREAM("Frame: "<<update_id<<"\t fps-avg = "<< 1.0/acc_frame_timings_.getMean()<<"\t nObs = "<<acc_num_obs_.getMean());
  SVO_LOG(dropout);

  // save processing time to calculate fps
  // ͳ�����ʮ֡�Ĵ���ʱ��
  acc_frame_timings_.push_back(timer_.stop());

  // ��ϵͳ״̬ΪSTAGE_DEFAULT_FRAME����ͳ�����ʮ֡����õ�����������
  if(stage_ == STAGE_DEFAULT_FRAME)
    acc_num_obs_.push_back(num_observations);
  num_obs_last_ = num_observations;
  SVO_STOP_TIMER("tot_time");

  // ��������
#ifdef SVO_TRACE
  // writeToFlie()������vikit���У�rpg_vikit/vikit_common/src/performance_monitor.cpp
  g_permon->writeToFile();
  {
	// �߳���
    boost::unique_lock<boost::mutex> lock(map_.point_candidates_.mut_);
	// ��¼����������
    size_t n_candidates = map_.point_candidates_.candidates_.size();
    SVO_LOG(n_candidates);
  }
#endif

  // �ж�ϵͳ״̬
  if(dropout == RESULT_FAILURE &&
      (stage_ == STAGE_DEFAULT_FRAME || stage_ == STAGE_RELOCALIZING ))
  {
    stage_ = STAGE_RELOCALIZING;
    tracking_quality_ = TRACKING_INSUFFICIENT;
  }
  else if (dropout == RESULT_FAILURE)
	// ����Map�ͱ���map_�ĳ�ʼ���������ؼ�֡�ͺ�ѡ�����յȣ�
    // ͬʱstage_����ΪSTAGE_PAUSED��set_reset��set_start��������Ϊfalse����������һЩ���á�
    resetAll();// תframe_handler_mono.cpp�ļ���
  if(set_reset_)
    resetAll(); // ͬ��

  return 0;
}

/*
��frame_handler_mono.cpp�ļ��е�void FrameHandlerMono::resetAll()�����е���
*/
void FrameHandlerBase::resetCommon()
{
  // Map�ͱ���map_�ĳ�ʼ��,�����ؼ�֡�ͺ�ѡ�����յ�
  map_.reset();

  // ��ϵͳ״̬��ΪSTAGE_PAUSED
  stage_ = STAGE_PAUSED;
  set_reset_ = false;
  set_start_ = false;
  tracking_quality_ = TRACKING_INSUFFICIENT;
  num_obs_last_ = 0;
  SVO_INFO_STREAM("RESET");
}

/*
** parameter
* num_observations: �۲쵽��3D�������
* ��frame_handler_mono.cpp�ļ��е�FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()�����е���
** function: ���ݹ۲쵽��3D����������ж����ٵ�����
*/
void FrameHandlerBase::setTrackingQuality(const size_t num_observations)
{
  // �ȼ����Ǻõĸ���
  tracking_quality_ = TRACKING_GOOD;
  // ��������С����С����ֵ������Ϊ���ٵ�Ч������
  if(num_observations < Config::qualityMinFts())
  {
    SVO_WARN_STREAM_THROTTLE(0.5, "Tracking less than "<< Config::qualityMinFts() <<" features!");
    tracking_quality_ = TRACKING_INSUFFICIENT;
  }

  // ����ʧ�������������������ֵ������Ϊ���ٵ�Ч������
  // �ܵĵ�����ȥ�۲쵽�ĵ�����ʣ�µľ��Ƕ�ʧ�ĵ���
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
* frame: ����֡
* max_n_pts: ÿ�ε����ܹ��Ż�����������ĵ�
* max_iter: �ڽṹ�Ż��еĵ�������
* ��frame_handler_mono.cpp�ļ��е�FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()�����е���
** function: �Ż�3D����Ϣ��Ҳ����˵���Ż�λ���𣿣���
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
  ��[first, last]��������е�n��С��Ԫ�أ��������������compare�������Ͱ�compare�����ķ�ʽ�Ƚϡ�
  */
  nth_element(pts.begin(), pts.begin() + max_n_pts, pts.end(), ptLastOptimComparator);
  for(deque<Point*>::iterator it=pts.begin(); it!=pts.begin()+max_n_pts; ++it)
  {
    (*it)->optimize(max_iter);
    (*it)->last_structure_optim_ = frame->id_;
  }
}


} // namespace svo
