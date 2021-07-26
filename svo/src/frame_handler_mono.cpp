
/*
** �Ӿ�ǰ��ԭ��
*/
#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/pose_optimizer.h>
#include <svo/sparse_img_align.h>
#include <vikit/performance_monitor.h>
#include <svo/depth_filter.h>
#ifdef USE_BUNDLE_ADJUSTMENT
#include <svo/bundle_adjustment.h>
#endif

namespace svo {

/*
* parameter: cam  vk::AbstractCamera��
* ��svo_ros/src/vo_node.cpp�ļ��е�VoNode::VoNode()���캯������
*/
FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam) :
  FrameHandlerBase(),
  cam_(cam),// �������ģ��
  reprojector_(cam_, map_),// ��ͶӰ��ʼ����תreprojector.cpp�ļ����캯��
  depth_filter_(NULL) // ����˲�����ʼ����תdepth_filter.cpp�ļ����캯��
{
  initialize();
}

/*
* ��ʼ��
* �ڱ��ļ��е�FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam)�����е���
*/
void FrameHandlerMono::initialize()
{
  feature_detection::DetectorPtr feature_detector(
      new feature_detection::FastDetector(
          cam_->width(), cam_->height(), Config::gridSize(), Config::nPyrLevels())); // ����fast���������
  DepthFilter::callback_t depth_filter_cb = boost::bind(
      &MapPointCandidates::newCandidatePoint, &map_.point_candidates_, _1, _2);
  /*
  �����˻ص�����ָ����ָ���ݣ�����bind�������ɵ�ָ��
  bind������newCandidatePoint�ͱ���point_candidates_��map_�ĳ�Ա�������������������һ�𹹳ɺ���ָ��depth_filter_cb
  ����depth_filter_cb�������������������������������point_candidates���м��㡣
  */
  depth_filter_ = new DepthFilter(feature_detector, depth_filter_cb);// �������ָ��ͻص�����ָ����һ������һ������˲���depth_filter_
  depth_filter_->startThread();// ����˲����߳�����
}

FrameHandlerMono::~FrameHandlerMono()
{
  delete depth_filter_;
}

/*
** parameters
* img:ͼ��
* timestamp:ʱ���
* ��vo_node.cpp�ļ��е�void VoNode::imgCb()�����е���
* ����ͼ��һЩ��ʼ������������ͼ���������������stage״̬ѡ��ͬ�Ĵ���ģʽ
*/
void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)
{
	// ������frame_handler_base.cpp�ļ���
	// ���startFrameProcessingCommon()��������false�����˳�addImage()����
  if(!startFrameProcessingCommon(timestamp))
    return;

  // some cleanup from last iteration, can't do before because of visualization
  /*
	  ���core_kfs_��overlap_kfs_������������ͬ��ָ��ļ��ϡ�
	  core_kfs_��Frame���͵�����ָ��shared_ptr�����ڱ�ʾһ֡��Χ�Ĺؼ�֡��
	  overlap_kfs_��һ���������洢������һ��ָ���һ����ֵ���ɵ���ϱ��������ڱ�ʾ�����ص���Ұ�Ĺؼ�֡����ֵ�������ص���Ұ�еĵر�����
  */
  core_kfs_.clear();
  overlap_kfs_.clear();

  // create new frame
  // ������֡����¼����־�ļ���
  SVO_START_TIMER("pyramid_creation");

  // �¹���һ��Frame����Ȼ����Frame������ָ�����new_frameָ������.reset����������ָ��ԭ����ָ�������ٲ�ָ���¶���
  // Frame()���캯����frame.cpp�ļ���
  new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
  SVO_STOP_TIMER("pyramid_creation");

  // process frame
  // ����֡
  UpdateResult res = RESULT_FAILURE;// ����UpdateResult��ö�ٱ���res��ֵΪRESULT_FAILURE��

  // ����stage״̬��ѡ����Ӧ�Ĵ����ֶ�
  if(stage_ == STAGE_DEFAULT_FRAME)
	  // ���������ؼ�֮֡�������֡
	  // ת���ļ���
    res = processFrame();

  else if(stage_ == STAGE_SECOND_FRAME)
	  // �����1֡��������֡��ֱ���ҵ�һ���µĹؼ�֡
	  // ת���ļ���
    res = processSecondFrame();

  else if(stage_ == STAGE_FIRST_FRAME)
	  // �����1֡����������Ϊ�ؼ�֡
	  // ת���ļ���
    res = processFirstFrame();

  else if(stage_ == STAGE_RELOCALIZING)
	  // �����λ���ض�λ֡���ṩ�ؼ�֡
	  // ת���ļ���
    res = relocalizeFrame(SE3(Matrix3d::Identity(), Vector3d::Zero()),
                          map_.getClosestKeyframe(last_frame_));

  // set last frame
  last_frame_ = new_frame_;
  // �������֡�����´�ʹ��
  new_frame_.reset();
  // finish processing
  finishFrameProcessingCommon(last_frame_->id_, res, last_frame_->nObs()); // תframe_handler_base.cpp�ļ�
}

/*
* �����1֡����������Ϊ�ؼ�֡
* �ڱ��ļ��е�void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)�����е���
*/
FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()
{
	//  �½�һ���任����SE3Ȼ�󸳸�new_frame_��T_f_w_�����ڱ�ʾ���������굽�������ı任����
  new_frame_->T_f_w_ = SE3(Matrix3d::Identity(), Vector3d::Zero());

  /*
  �жϺ���klt_homography_init_.addFirstFrame(new_frame_)����ֵ�Ƿ�Ϊinitialization::FAILURE������ʼ���Ƿ�ʧ��
  ����ǣ������processFirstFrame������RESULT_NO_KEYFRAME����˼�ǵ�ǰ֡�ǹؼ�֡��
  ����klt_homography_init_��KltHomographyInit��FrameHandlerMono����Ԫ�ࣩ���͵��࣬���ڼ��㵥Ӧ�Ծ��󣨸���ǰ�����ؼ�֡������ʼ��λ��
  addFirstFrame()������initialization.cpp��
  */
  if(klt_homography_init_.addFirstFrame(new_frame_) == initialization::FAILURE)
    return RESULT_NO_KEYFRAME;

  // ���ٳɹ������õ�ǰ֡Ϊ�ؼ�֡
  new_frame_->setKeyframe();
  // ��new_frame_�ؼ�֡����keyframes_��
  map_.addKeyframe(new_frame_);
  // ��stage״̬��Ϊ�ڶ�֡
  stage_ = STAGE_SECOND_FRAME;
  // ����Ϣ��Init: Selected first frame����¼����־��
  SVO_INFO_STREAM("Init: Selected first frame.");
  // ����processFirstFrame������RESULT_IS_KEYFRAME
  return RESULT_IS_KEYFRAME;
}

/*
* �����1֡��������֡��ֱ���ҵ�һ���µĹؼ�֡
* �ڱ��ļ��е�void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)�����е���
*/
FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()
{
  // ��ӵڶ����ؼ�֡
  initialization::InitResult res = klt_homography_init_.addSecondFrame(new_frame_);
  // �ж���ӵڶ����ؼ�֡�Ľ��
  if(res == initialization::FAILURE)
	  return RESULT_FAILURE; // ΪFAILUREʱ������processSecondFrame()������������RESULT_FAILURE��
  else if(res == initialization::NO_KEYFRAME)
    return RESULT_NO_KEYFRAME; // ΪNO_KEYFRAMEʱ������processSecondFrame()������������RESULT_NO_KEYFRAME��

  // two-frame bundle adjustment
  // �������룬���������USE_BUNDLE_ADJUSTMENT���ͽ���BA�Ż�������ba::twoViewBA����
#ifdef USE_BUNDLE_ADJUSTMENT
  ba::twoViewBA(new_frame_.get(), map_.lastKeyframe().get(), Config::lobaThresh(), &map_);
#endif

  /*
   ִ�к���new_frame_->setKeyframe()��������Ϊ�ؼ�֡������is_keyframe_����Ϊtrue����ִ��setKeyPoints()����
   setKeyPoints������ͨ������checkKeyPoints�����Ե�ǰͼ����ÿ����������б����ͱȽϣ�����ѡ������д����Ե�5����Ϊ�ؼ��㡣
   ʵ������1������ͼ�����ĵĵ��4������ͼ���ĸ��ǵĵ㡣
  */
  new_frame_->setKeyframe();

  // ���֡�е�ƽ����Ⱥ���С���
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);

  // ������˲���depth_filter����ӹؼ�֡����ǰ֡�����������depth_mean��0.5 * depth_min����֪��Ϊɶ����2�����г�ʼ��
  depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

  // add frame to map
  // ��map_����ӵ�ǰ�ؼ�֡
  map_.addKeyframe(new_frame_);

  /*
  ����stage_ΪSTAGE_DEFAULT_FRAME��
  ����klt_homography_init_.reset()����ʼ��px_cur_��frame_ref_��
  ����processSecondFrame()����������RESULT_IS_KEYFRAME��
  */
  stage_ = STAGE_DEFAULT_FRAME;
  klt_homography_init_.reset();// תinitilization.cpp�ļ���
  SVO_INFO_STREAM("Init: Selected second frame, triangulated initial map.");
  return RESULT_IS_KEYFRAME;
}

/*
* �ڱ��ļ��е�void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)�����е���
** function: ���������ؼ�֮֡�������֡
*/
FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()
{
  // Set initial pose TODO use prior
  // ���ó�ʼλ�ˣ�������֡��last_frame_���ı任����T_f_w_��������ǰ֡�ı任����T_f_w_��
  new_frame_->T_f_w_ = last_frame_->T_f_w_;

  // sparse image align
  // ͼ���ϡ����루Ӧ����ƥ�����˼��
  SVO_START_TIMER("sparse_img_align");
  // ���ȴ���SparseImgAlign���ͱ���img_align�������ù��캯�����г�ʼ����ͼ��������������С�㡢�������������ø�˹ţ�ٷ���display_��verbose
  SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                           30, SparseImgAlign::GaussNewton, false, false);
  // ����run�������ж��룬�������Ϊ��֡�͵�ǰָ֡��
  size_t img_align_n_tracked = img_align.run(last_frame_, new_frame_);
  SVO_STOP_TIMER("sparse_img_align");
  SVO_LOG(img_align_n_tracked);
  SVO_DEBUG_STREAM("Img Align:\t Tracked = " << img_align_n_tracked);

  // map reprojection & feature alignment
  // ��ͼ��ͶӰ���������루������ƥ�䣩
  SVO_START_TIMER("reproject");

  // �ӵ�ͼ��ͼ��ͶӰ�����Ȳ��Ҿ����ص���Ұ�Ĺؼ�֡������ͶӰ��Щ�ؼ�֡�ĵ�ͼ�㣬תreprojector.cpp�ļ�
  // �������µ�֡���������ο�֡�����ص���Ұ�Ĺؼ�֡��Ӧ���������İɡ�����
  reprojector_.reprojectMap(new_frame_, overlap_kfs_);

  SVO_STOP_TIMER("reproject");
  const size_t repr_n_new_references = reprojector_.n_matches_;// ƥ��������
  const size_t repr_n_mps = reprojector_.n_trials_;// ��ͶӰ�˶��ٵ�ͼ��
  SVO_LOG2(repr_n_mps, repr_n_new_references);
  SVO_DEBUG_STREAM("Reprojection:\t nPoints = "<<repr_n_mps<<"\t \t nMatches = "<<repr_n_new_references);

  // ���ƥ��������С����ֵ������Ϊ���ٵ���������
  if(repr_n_new_references < Config::qualityMinFts())
  {
    SVO_WARN_STREAM_THROTTLE(1.0, "Not enough matched features.");
	// ����һ֡�ı任���󸳸���ǰ֡
    new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
    tracking_quality_ = TRACKING_INSUFFICIENT;
    return RESULT_FAILURE;// ����RESULT_FAILURE
  }

  // pose optimization
  // ������λ�˵��Ż����̣���ʱ��������Ӧ��������Ҫ�����
  SVO_START_TIMER("pose_optimizer");
  // ���۲쵽��3D�������
  size_t sfba_n_edges_final;
  // ����ʼ��������
  double sfba_thresh, sfba_error_init, sfba_error_final;

  // ���ø�˹ţ�ٷ������Ż����Ż������������λ�ã���ͶӰ������
  pose_optimizer::optimizeGaussNewton(
      Config::poseOptimThresh(), Config::poseOptimNumIter(), false,
      new_frame_, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
  SVO_STOP_TIMER("pose_optimizer");
  SVO_LOG4(sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrInit = "<<sfba_error_init<<"px\t thresh = "<<sfba_thresh);
  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrFin. = "<<sfba_error_final<<"px\t nObsFin. = "<<sfba_n_edges_final);
  // ������۲쵽�ĵ����������20���򷵻ش���
  if(sfba_n_edges_final < 20)
    return RESULT_FAILURE;

  // structure optimization
  // �ṹ�Ż����Ż�����λ�ˣ�����frame_handler_bash.hע�ͣ�Ӧ�����Ż�3D��ɣ�����
  SVO_START_TIMER("point_optimizer");
  optimizeStructure(new_frame_, Config::structureOptimMaxPts(), Config::structureOptimNumIter());
  SVO_STOP_TIMER("point_optimizer");

  // select keyframe
  // ����ǰ֡����core_kfs_�����ڴ洢�����ؼ�֡��
  core_kfs_.insert(new_frame_);
  // �ж����ٵ�Ч���û�
  setTrackingQuality(sfba_n_edges_final);
  // �ж�tracking_quality_ ��������TRACKING_INSUFFICIENT�������õ�ǰ֡�任����Ϊ��֡�任���󣬲�����RESULT_FAILURE
  if(tracking_quality_ == TRACKING_INSUFFICIENT)
  {
    new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
    return RESULT_FAILURE;
  }

  // ��Ⱦ�ֵ����Сֵ
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);

  // �����ǰ֡������Ϊ�ؼ�֡���߸���Ч�����ã��򽫵�ǰ֡��������˲�����
  if(!needNewKf(depth_mean) || tracking_quality_ == TRACKING_BAD)
  {
    depth_filter_->addFrame(new_frame_);
	// ���ؽ�����޹ؼ�֡
    return RESULT_NO_KEYFRAME;
  }
  // ����ǰ֡����Ϊ�ؼ�֡
  new_frame_->setKeyframe();
  SVO_DEBUG_STREAM("New keyframe selected.");

  // new keyframe selected
  // ��ǰ֡�Ѿ���ѡΪ�ؼ�֡�������ؼ�֡�е�������
  for(Features::iterator it=new_frame_->fts_.begin(); it!=new_frame_->fts_.end(); ++it)
	  // ������������Ӧ�ĵ�ͼ��ָ�벻Ϊ�գ������֡����Ӹ�������
    if((*it)->point != NULL)
      (*it)->point->addFrameRef(*it);

  // ��map_.point_candidates_���뵱ǰ֡��ص���������ӵ���ǰ֡
  map_.point_candidates_.addCandidatePointToFrame(new_frame_);

  // optional bundle adjustment
  // �������룬���������USE_BUNDLE_ADJUSTMENT�������BA�Ż�
#ifdef USE_BUNDLE_ADJUSTMENT
  // ����ֲ�BA�ĵ�����������0���Ϳ�ʼ�ֲ�BA
  if(Config::lobaNumIter() > 0)
  {
    SVO_START_TIMER("local_ba");
	// ���ص������֡�е���������뻬�����ڵĹؼ�֡��
    setCoreKfs(Config::coreNKfs());

    size_t loba_n_erredges_init, loba_n_erredges_fin;
    double loba_err_init, loba_err_fin;
	// 
    ba::localBA(new_frame_.get(), &core_kfs_, &map_,
                loba_n_erredges_init, loba_n_erredges_fin,
                loba_err_init, loba_err_fin);
    SVO_STOP_TIMER("local_ba");
    SVO_LOG4(loba_n_erredges_init, loba_n_erredges_fin, loba_err_init, loba_err_fin);
    SVO_DEBUG_STREAM("Local BA:\t RemovedEdges {"<<loba_n_erredges_init<<", "<<loba_n_erredges_fin<<"} \t "
                     "Error {"<<loba_err_init<<", "<<loba_err_fin<<"}");
  }
#endif

  // init new depth-filters
  depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

  // if limited number of keyframes, remove the one furthest apart
  if(Config::maxNKfs() > 2 && map_.size() >= Config::maxNKfs())
  {
    FramePtr furthest_frame = map_.getFurthestKeyframe(new_frame_->pos());
    depth_filter_->removeKeyframe(furthest_frame); // TODO this interrupts the mapper thread, maybe we can solve this better
    map_.safeDeleteFrame(furthest_frame);
  }

  // add keyframe to map
  map_.addKeyframe(new_frame_);

  return RESULT_IS_KEYFRAME;
}

/*
* �����λ���ض�λ֡���ṩ�ؼ�֡
* �ڱ��ļ��е�void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)�����е���
*/
FrameHandlerMono::UpdateResult FrameHandlerMono::relocalizeFrame(
    const SE3& T_cur_ref,
    FramePtr ref_keyframe)
{
  SVO_WARN_STREAM_THROTTLE(1.0, "Relocalizing frame");
  if(ref_keyframe == nullptr)
  {
    SVO_INFO_STREAM("No reference keyframe.");
    return RESULT_FAILURE;
  }
  SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                           30, SparseImgAlign::GaussNewton, false, false);
  size_t img_align_n_tracked = img_align.run(ref_keyframe, new_frame_);
  if(img_align_n_tracked > 30)
  {
    SE3 T_f_w_last = last_frame_->T_f_w_;
    last_frame_ = ref_keyframe;
    FrameHandlerMono::UpdateResult res = processFrame();
    if(res != RESULT_FAILURE)
    {
      stage_ = STAGE_DEFAULT_FRAME;
      SVO_INFO_STREAM("Relocalization successful.");
    }
    else
      new_frame_->T_f_w_ = T_f_w_last; // reset to last well localized pose
    return res;
  }
  return RESULT_FAILURE;
}

bool FrameHandlerMono::relocalizeFrameAtPose(
    const int keyframe_id,
    const SE3& T_f_kf,
    const cv::Mat& img,
    const double timestamp)
{
  FramePtr ref_keyframe;
  if(!map_.getKeyframeById(keyframe_id, ref_keyframe))
    return false;
  new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
  UpdateResult res = relocalizeFrame(T_f_kf, ref_keyframe);
  if(res != RESULT_FAILURE) {
    last_frame_ = new_frame_;
    return true;
  }
  return false;
}

/*
* ��frame_handler_base.cpp�ļ��е�bool FrameHandlerBase::startFrameProcessingCommon(const double timestamp)�����е���
* ��frame_handler_base.cpp�ļ��е�int FrameHandlerBase::finishFrameProcessingCommon()�����е���
* ��Ҫ����һЩ��������õĹ���
*/
void FrameHandlerMono::resetAll()
{
  resetCommon();//תframe_handler_base.cpp�ļ�
  last_frame_.reset();
  new_frame_.reset();
  core_kfs_.clear();
  overlap_kfs_.clear();
  depth_filter_->reset();
}

void FrameHandlerMono::setFirstFrame(const FramePtr& first_frame)
{
  resetAll();
  last_frame_ = first_frame;
  last_frame_->setKeyframe();
  map_.addKeyframe(last_frame_);
  stage_ = STAGE_DEFAULT_FRAME;
}

/*
** parameter
* scene_depth_mean: ��������Ⱦ�ֵ
* �ڱ��ļ��е�FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()�����е���
** function: �ж���ǰ֡�Ƿ���Ա�ѡΪ�ؼ�֡
*/
bool FrameHandlerMono::needNewKf(double scene_depth_mean)
{
  for(auto it=overlap_kfs_.begin(), ite=overlap_kfs_.end(); it!=ite; ++it)
  {
	// �Ȱѵ����������ϵת��ͼ������ϵ
    Vector3d relpos = new_frame_->w2f(it->first->pos());
	// ���μ���x��y��z��ƽ����ȵı�ֵ�����ж��Ƿ�С����ֵ����С�ڣ��򷵻�false
    if(fabs(relpos.x())/scene_depth_mean < Config::kfSelectMinDist() &&
       fabs(relpos.y())/scene_depth_mean < Config::kfSelectMinDist()*0.8 &&
       fabs(relpos.z())/scene_depth_mean < Config::kfSelectMinDist()*1.3)
      return false;
  }
  return true;// ����������ֵ���򷵻�true�����Ա�ѡΪ�ؼ�֡
}

/*
** parameter
* n_closest: ���������еĹؼ�֡����
* �ڱ��ļ��е�FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()�����е���
** function: ���ص������֡�е���������뻬�����ڵĹؼ�֡��
*/
void FrameHandlerMono::setCoreKfs(size_t n_closest)
{
  // ȡ�������ڹؼ�֡�����;����ص�����ؼ�֡��������Сֵ
  size_t n = min(n_closest, overlap_kfs_.size()-1);
  // �����оֲ�Ԫ�ؽ�������Ĭ�����������򡣿����ǰ���ǰʮ֡������������İɣ�����
  std::partial_sort(overlap_kfs_.begin(), overlap_kfs_.begin()+n, overlap_kfs_.end(),
                    boost::bind(&pair<FramePtr, size_t>::second, _1) >
                    boost::bind(&pair<FramePtr, size_t>::second, _2));
  std::for_each(overlap_kfs_.begin(), overlap_kfs_.end(), [&](pair<FramePtr,size_t>& i){ core_kfs_.insert(i.first); });
}

} // namespace svo
