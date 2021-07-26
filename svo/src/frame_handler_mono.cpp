
/*
** 视觉前端原理
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
* parameter: cam  vk::AbstractCamera类
* 在svo_ros/src/vo_node.cpp文件中的VoNode::VoNode()构造函数调用
*/
FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam) :
  FrameHandlerBase(),
  cam_(cam),// 加载相机模型
  reprojector_(cam_, map_),// 重投影初始化，转reprojector.cpp文件构造函数
  depth_filter_(NULL) // 深度滤波器初始化，转depth_filter.cpp文件构造函数
{
  initialize();
}

/*
* 初始化
* 在本文件中的FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam)函数中调用
*/
void FrameHandlerMono::initialize()
{
  feature_detection::DetectorPtr feature_detector(
      new feature_detection::FastDetector(
          cam_->width(), cam_->height(), Config::gridSize(), Config::nPyrLevels())); // 设置fast特征检测器
  DepthFilter::callback_t depth_filter_cb = boost::bind(
      &MapPointCandidates::newCandidatePoint, &map_.point_candidates_, _1, _2);
  /*
  设置了回调函数指针所指内容，即由bind函数生成的指针
  bind函数将newCandidatePoint型变量point_candidates_（map_的成员变量）与输入参数绑定在一起构成函数指针depth_filter_cb
  最终depth_filter_cb有两个输入参数，这两个参数被传入point_candidates进行计算。
  */
  depth_filter_ = new DepthFilter(feature_detector, depth_filter_cb);// 特征检测指针和回调函数指针在一起生成一个深度滤波器depth_filter_
  depth_filter_->startThread();// 深度滤波器线程启动
}

FrameHandlerMono::~FrameHandlerMono()
{
  delete depth_filter_;
}

/*
** parameters
* img:图像
* timestamp:时间戳
* 在vo_node.cpp文件中的void VoNode::imgCb()函数中调用
* 加载图像，一些初始化工作，创建图像金字塔，最后根据stage状态选择不同的处理模式
*/
void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)
{
	// 定义在frame_handler_base.cpp文件中
	// 如果startFrameProcessingCommon()函数返回false，则退出addImage()函数
  if(!startFrameProcessingCommon(timestamp))
    return;

  // some cleanup from last iteration, can't do before because of visualization
  /*
	  清空core_kfs_和overlap_kfs_，这两个都是同种指针的集合。
	  core_kfs_是Frame类型的智能指针shared_ptr，用于表示一帧周围的关键帧。
	  overlap_kfs_是一个向量，存储的是由一个指针和一个数值构成的组合变量，用于表示具有重叠视野的关键帧，数值代表了重叠视野中的地标数。
  */
  core_kfs_.clear();
  overlap_kfs_.clear();

  // create new frame
  // 创建新帧并记录至日志文件。
  SVO_START_TIMER("pyramid_creation");

  // 新构造一个Frame对象，然后用Frame型智能指针变量new_frame指向它。.reset函数将智能指针原来所指对象销毁并指向新对象。
  // Frame()构造函数在frame.cpp文件中
  new_frame_.reset(new Frame(cam_, img.clone(), timestamp));
  SVO_STOP_TIMER("pyramid_creation");

  // process frame
  // 处理帧
  UpdateResult res = RESULT_FAILURE;// 设置UpdateResult型枚举变量res，值为RESULT_FAILURE。

  // 根据stage状态，选择相应的处理手段
  if(stage_ == STAGE_DEFAULT_FRAME)
	  // 处理两个关键帧之后的所有帧
	  // 转本文件中
    res = processFrame();

  else if(stage_ == STAGE_SECOND_FRAME)
	  // 处理第1帧后面所有帧，直至找到一个新的关键帧
	  // 转本文件中
    res = processSecondFrame();

  else if(stage_ == STAGE_FIRST_FRAME)
	  // 处理第1帧并将其设置为关键帧
	  // 转本文件中
    res = processFirstFrame();

  else if(stage_ == STAGE_RELOCALIZING)
	  // 在相关位置重定位帧以提供关键帧
	  // 转本文件中
    res = relocalizeFrame(SE3(Matrix3d::Identity(), Vector3d::Zero()),
                          map_.getClosestKeyframe(last_frame_));

  // set last frame
  last_frame_ = new_frame_;
  // 清空最新帧，供下次使用
  new_frame_.reset();
  // finish processing
  finishFrameProcessingCommon(last_frame_->id_, res, last_frame_->nObs()); // 转frame_handler_base.cpp文件
}

/*
* 处理第1帧并将其设置为关键帧
* 在本文件中的void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)函数中调用
*/
FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()
{
	//  新建一个变换矩阵SE3然后赋给new_frame_的T_f_w_，用于表示从世界坐标到相机坐标的变换矩阵。
  new_frame_->T_f_w_ = SE3(Matrix3d::Identity(), Vector3d::Zero());

  /*
  判断函数klt_homography_init_.addFirstFrame(new_frame_)返回值是否为initialization::FAILURE，即初始化是否失败
  如果是，则结束processFirstFrame并返回RESULT_NO_KEYFRAME（意思是当前帧非关键帧）
  其中klt_homography_init_是KltHomographyInit（FrameHandlerMono的友元类）类型的类，用于计算单应性矩阵（根据前两个关键帧）来初始化位姿
  addFirstFrame()函数在initialization.cpp中
  */
  if(klt_homography_init_.addFirstFrame(new_frame_) == initialization::FAILURE)
    return RESULT_NO_KEYFRAME;

  // 跟踪成功，设置当前帧为关键帧
  new_frame_->setKeyframe();
  // 将new_frame_关键帧存入keyframes_中
  map_.addKeyframe(new_frame_);
  // 将stage状态设为第二帧
  stage_ = STAGE_SECOND_FRAME;
  // 将信息“Init: Selected first frame”记录至日志。
  SVO_INFO_STREAM("Init: Selected first frame.");
  // 结束processFirstFrame并返回RESULT_IS_KEYFRAME
  return RESULT_IS_KEYFRAME;
}

/*
* 处理第1帧后面所有帧，直至找到一个新的关键帧
* 在本文件中的void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)函数中调用
*/
FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()
{
  // 添加第二个关键帧
  initialization::InitResult res = klt_homography_init_.addSecondFrame(new_frame_);
  // 判断添加第二个关键帧的结果
  if(res == initialization::FAILURE)
	  return RESULT_FAILURE; // 为FAILURE时，结束processSecondFrame()函数，并返回RESULT_FAILURE。
  else if(res == initialization::NO_KEYFRAME)
    return RESULT_NO_KEYFRAME; // 为NO_KEYFRAME时，结束processSecondFrame()函数，并返回RESULT_NO_KEYFRAME。

  // two-frame bundle adjustment
  // 条件编译，如果定义了USE_BUNDLE_ADJUSTMENT，就进行BA优化，调用ba::twoViewBA函数
#ifdef USE_BUNDLE_ADJUSTMENT
  ba::twoViewBA(new_frame_.get(), map_.lastKeyframe().get(), Config::lobaThresh(), &map_);
#endif

  /*
   执行函数new_frame_->setKeyframe()将其设置为关键帧（即将is_keyframe_设置为true，并执行setKeyPoints()）。
   setKeyPoints函数中通过调用checkKeyPoints函数对当前图像中每个特征点进行遍历和比较，最终选出最具有代表性的5个作为关键点。
   实质上是1个靠近图像中心的点和4个靠近图像四个角的点。
  */
  new_frame_->setKeyframe();

  // 获得帧中的平均深度和最小深度
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);

  // 向深度滤波器depth_filter中添加关键帧（当前帧），传入参数depth_mean、0.5 * depth_min（不知道为啥除以2）进行初始化
  depth_filter_->addKeyframe(new_frame_, depth_mean, 0.5*depth_min);

  // add frame to map
  // 向map_中添加当前关键帧
  map_.addKeyframe(new_frame_);

  /*
  设置stage_为STAGE_DEFAULT_FRAME。
  调用klt_homography_init_.reset()，初始化px_cur_和frame_ref_。
  结束processSecondFrame()函数，返回RESULT_IS_KEYFRAME。
  */
  stage_ = STAGE_DEFAULT_FRAME;
  klt_homography_init_.reset();// 转initilization.cpp文件中
  SVO_INFO_STREAM("Init: Selected second frame, triangulated initial map.");
  return RESULT_IS_KEYFRAME;
}

/*
* 在本文件中的void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)函数中调用
** function: 处理两个关键帧之后的所有帧
*/
FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()
{
  // Set initial pose TODO use prior
  // 设置初始位姿，即将上帧（last_frame_）的变换矩阵（T_f_w_）赋给当前帧的变换矩阵（T_f_w_）
  new_frame_->T_f_w_ = last_frame_->T_f_w_;

  // sparse image align
  // 图像的稀疏对齐（应该是匹配的意思）
  SVO_START_TIMER("sparse_img_align");
  // 首先创建SparseImgAlign类型变量img_align，并利用构造函数进行初始化：图像金字塔最大层和最小层、迭代次数、采用高斯牛顿法、display_和verbose
  SparseImgAlign img_align(Config::kltMaxLevel(), Config::kltMinLevel(),
                           30, SparseImgAlign::GaussNewton, false, false);
  // 调用run函数进行对齐，传入参数为上帧和当前帧指针
  size_t img_align_n_tracked = img_align.run(last_frame_, new_frame_);
  SVO_STOP_TIMER("sparse_img_align");
  SVO_LOG(img_align_n_tracked);
  SVO_DEBUG_STREAM("Img Align:\t Tracked = " << img_align_n_tracked);

  // map reprojection & feature alignment
  // 地图重投影和特征对齐（或许是匹配）
  SVO_START_TIMER("reproject");

  // 从地图向图像投影，首先查找具有重叠视野的关键帧，并仅投影这些关键帧的地图点，转reprojector.cpp文件
  // 输入是新的帧，输出是与参考帧具有重叠视野的关键帧，应该是这样的吧。。。
  reprojector_.reprojectMap(new_frame_, overlap_kfs_);

  SVO_STOP_TIMER("reproject");
  const size_t repr_n_new_references = reprojector_.n_matches_;// 匹配点的数量
  const size_t repr_n_mps = reprojector_.n_trials_;// 重投影了多少地图点
  SVO_LOG2(repr_n_mps, repr_n_new_references);
  SVO_DEBUG_STREAM("Reprojection:\t nPoints = "<<repr_n_mps<<"\t \t nMatches = "<<repr_n_new_references);

  // 如果匹配点的数量小于阈值，则认为跟踪的质量不好
  if(repr_n_new_references < Config::qualityMinFts())
  {
    SVO_WARN_STREAM_THROTTLE(1.0, "Not enough matched features.");
	// 将上一帧的变换矩阵赋给当前帧
    new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
    tracking_quality_ = TRACKING_INSUFFICIENT;
    return RESULT_FAILURE;// 返回RESULT_FAILURE
  }

  // pose optimization
  // 下面是位姿的优化过程，此时跟踪质量应该是满足要求的了
  SVO_START_TIMER("pose_optimizer");
  // 最后观察到的3D点的数量
  size_t sfba_n_edges_final;
  // 、初始误差、最后误差
  double sfba_thresh, sfba_error_init, sfba_error_final;

  // 调用高斯牛顿法进行优化，优化的是特征点的位置，重投影误差？？？
  pose_optimizer::optimizeGaussNewton(
      Config::poseOptimThresh(), Config::poseOptimNumIter(), false,
      new_frame_, sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
  SVO_STOP_TIMER("pose_optimizer");
  SVO_LOG4(sfba_thresh, sfba_error_init, sfba_error_final, sfba_n_edges_final);
  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrInit = "<<sfba_error_init<<"px\t thresh = "<<sfba_thresh);
  SVO_DEBUG_STREAM("PoseOptimizer:\t ErrFin. = "<<sfba_error_final<<"px\t nObsFin. = "<<sfba_n_edges_final);
  // 如果最后观察到的点的数量少于20，则返回错误
  if(sfba_n_edges_final < 20)
    return RESULT_FAILURE;

  // structure optimization
  // 结构优化，优化的是位姿，根据frame_handler_bash.h注释，应该是优化3D点吧？？？
  SVO_START_TIMER("point_optimizer");
  optimizeStructure(new_frame_, Config::structureOptimMaxPts(), Config::structureOptimNumIter());
  SVO_STOP_TIMER("point_optimizer");

  // select keyframe
  // 将当前帧插入core_kfs_（用于存储附近关键帧）
  core_kfs_.insert(new_frame_);
  // 判定跟踪的效果好坏
  setTrackingQuality(sfba_n_edges_final);
  // 判断tracking_quality_ ，若等于TRACKING_INSUFFICIENT。则设置当前帧变换矩阵为上帧变换矩阵，并返回RESULT_FAILURE
  if(tracking_quality_ == TRACKING_INSUFFICIENT)
  {
    new_frame_->T_f_w_ = last_frame_->T_f_w_; // reset to avoid crazy pose jumps
    return RESULT_FAILURE;
  }

  // 深度均值和最小值
  double depth_mean, depth_min;
  frame_utils::getSceneDepth(*new_frame_, depth_mean, depth_min);

  // 如果当前帧不能作为关键帧或者跟踪效果不好，则将当前帧存入深度滤波器中
  if(!needNewKf(depth_mean) || tracking_quality_ == TRACKING_BAD)
  {
    depth_filter_->addFrame(new_frame_);
	// 返回结果，无关键帧
    return RESULT_NO_KEYFRAME;
  }
  // 将当前帧设置为关键帧
  new_frame_->setKeyframe();
  SVO_DEBUG_STREAM("New keyframe selected.");

  // new keyframe selected
  // 当前帧已经被选为关键帧，遍历关键帧中的特征点
  for(Features::iterator it=new_frame_->fts_.begin(); it!=new_frame_->fts_.end(); ++it)
	  // 如果与特征点对应的地图点指针不为空，就向该帧中添加该特征点
    if((*it)->point != NULL)
      (*it)->point->addFrameRef(*it);

  // 将map_.point_candidates_中与当前帧相关的特征点添加到当前帧
  map_.point_candidates_.addCandidatePointToFrame(new_frame_);

  // optional bundle adjustment
  // 条件编译，如果定义了USE_BUNDLE_ADJUSTMENT，则进行BA优化
#ifdef USE_BUNDLE_ADJUSTMENT
  // 如果局部BA的迭代次数大于0，就开始局部BA
  if(Config::lobaNumIter() > 0)
  {
    SVO_START_TIMER("local_ba");
	// 将重叠区域的帧中的特征点存入滑动窗口的关键帧中
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
* 在相关位置重定位帧以提供关键帧
* 在本文件中的void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)函数中调用
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
* 在frame_handler_base.cpp文件中的bool FrameHandlerBase::startFrameProcessingCommon(const double timestamp)函数中调用
* 在frame_handler_base.cpp文件中的int FrameHandlerBase::finishFrameProcessingCommon()函数中调用
* 主要是做一些清理和重置的工作
*/
void FrameHandlerMono::resetAll()
{
  resetCommon();//转frame_handler_base.cpp文件
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
* scene_depth_mean: 场景的深度均值
* 在本文件中的FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()函数中调用
** function: 判定当前帧是否可以被选为关键帧
*/
bool FrameHandlerMono::needNewKf(double scene_depth_mean)
{
  for(auto it=overlap_kfs_.begin(), ite=overlap_kfs_.end(); it!=ite; ++it)
  {
	// 先把点从世界坐标系转到图像坐标系
    Vector3d relpos = new_frame_->w2f(it->first->pos());
	// 依次计算x、y、z与平均深度的比值，并判断是否小于阈值。若小于，则返回false
    if(fabs(relpos.x())/scene_depth_mean < Config::kfSelectMinDist() &&
       fabs(relpos.y())/scene_depth_mean < Config::kfSelectMinDist()*0.8 &&
       fabs(relpos.z())/scene_depth_mean < Config::kfSelectMinDist()*1.3)
      return false;
  }
  return true;// 若都大于阈值，则返回true，可以被选为关键帧
}

/*
** parameter
* n_closest: 滑动窗口中的关键帧数量
* 在本文件中的FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()函数中调用
** function: 将重叠区域的帧中的特征点存入滑动窗口的关键帧中
*/
void FrameHandlerMono::setCoreKfs(size_t n_closest)
{
  // 取滑动窗口关键帧数量和具有重叠区域关键帧数量的最小值
  size_t n = min(n_closest, overlap_kfs_.size()-1);
  // 对序列局部元素进行排序，默认排序是升序。可能是按照前十帧函数那里排序的吧？？？
  std::partial_sort(overlap_kfs_.begin(), overlap_kfs_.begin()+n, overlap_kfs_.end(),
                    boost::bind(&pair<FramePtr, size_t>::second, _1) >
                    boost::bind(&pair<FramePtr, size_t>::second, _2));
  std::for_each(overlap_kfs_.begin(), overlap_kfs_.end(), [&](pair<FramePtr,size_t>& i){ core_kfs_.insert(i.first); });
}

} // namespace svo
