
/*
 ** 初始化
*/
#include <svo/config.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/initialization.h>
#include <svo/feature_detection.h>
#include <vikit/math_utils.h>
#include <vikit/homography.h>

namespace svo {
namespace initialization {

/*
** parameter
* frame_ref: 传入的图像帧
* 在frame_handler_mono.cpp文件中的FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()函数中调用
* 添加第一帧
* InitResult是枚举类型，包含FAILURE, NO_KEYFRAME, SUCCESS，代表初始化的结果
*/
InitResult KltHomographyInit::addFirstFrame(FramePtr frame_ref)
{
  // 初始化px_cur_（一个二维点向量，存储当前帧中被用于跟踪的关键点坐标）和frame_ref_（一个frame型智能指针）
  // 转本文件
  reset();

  // 对new_frame进行Fast特征检测，返回值是得分大于阈值的特征点的坐标px_ref_及方向f_ref_
  // 转本文件
  detectFeatures(frame_ref, px_ref_, f_ref_);

  // 如果特征点数目少于100，则返回错误，结束本函数
  if(px_ref_.size() < 100)
  {
    SVO_WARN_STREAM_THROTTLE(2.0, "First image has less than 100 features. Retry in more textured environment.");
    return FAILURE;
  }

  // 否则，特征点数目大于100
  // 将传入的new_frame_赋值给frame_ref_
  frame_ref_ = frame_ref;
  // 将px_ref_的值赋给px_cur_
  px_cur_.insert(px_cur_.begin(), px_ref_.begin(), px_ref_.end());
  // 返回SUCCESS
  return SUCCESS;
}

/*
** parameter
* frame_cur: 当前帧
* 在frame_handler_mono.cpp文件中的FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()函数中调用
* 添加第二帧，返回结果：失败、无关键帧、成功
*/
InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)
{
	// LK光流，返回跟踪到的点
  trackKlt(frame_ref_, frame_cur, px_ref_, px_cur_, f_ref_, f_cur_, disparities_);
  SVO_INFO_STREAM("Init: KLT tracked "<< disparities_.size() <<" features");

  // 判断跟踪到的特征点的数目是否小于设定的阈值，是的话就结束addSecondFrame函数，并返回FAILURE。
  // 为什么要用disparities_？？？不应该是px_cur_吗？？？
  if(disparities_.size() < Config::initMinTracked())
    return FAILURE;

  // 调用getMedian函数得到disparities_中的中位数作为平均距离，幅值给变量disparity。
  // 然后判断disparity，若值小于设定值，结束addSecondFrame函数，并返回NO_KEYFRAME。
  double disparity = vk::getMedian(disparities_);
  SVO_INFO_STREAM("Init: KLT "<<disparity<<"px average disparity.");
  if(disparity < Config::initMinDisparity())
    return NO_KEYFRAME;

  // 计算单应矩阵
  computeHomography(
      f_ref_, f_cur_,
      frame_ref_->cam_->errorMultiplier2(), Config::poseOptimThresh(),
      inliers_, xyz_in_cur_, T_cur_from_ref_);
  SVO_INFO_STREAM("Init: Homography RANSAC "<<inliers_.size()<<" inliers.");

  // 判断内点数量，若小于设定阈值，则结束addSecondFrame函数，并返回FAILURE。
  if(inliers_.size() < Config::initMinInliers())
  {
    SVO_WARN_STREAM("Init WARNING: "<<Config::initMinInliers()<<" inliers minimum required.");
    return FAILURE;
  }

  // Rescale the map such that the mean scene depth is equal to the specified scale
  // 调节地图大小以使地图平均深度等于指定比例
  vector<double> depth_vec;
  // 从xyz_in_cur中提取3D点深度信息
  for(size_t i=0; i<xyz_in_cur_.size(); ++i)
    depth_vec.push_back((xyz_in_cur_[i]).z());
  // 选其中位数作为均值
  double scene_depth_median = vk::getMedian(depth_vec);
  // 归一化地图尺度？？？
  double scale = Config::mapScale()/scene_depth_median;
  // 计算当前帧的变换矩阵（可表示相机位姿的变化）
  // SE3中对乘法“*”进行了重载以实现矩阵乘法，所以T_cur_from_ref_ * frame_ref_->T_f_w_表示对后者进行前者形式的变换
  frame_cur->T_f_w_ = T_cur_from_ref_ * frame_ref_->T_f_w_;
  /*
   T_f_w.rotation_matrix()和T_f_w.translation()分别返回SE3型变量的R和t
   pos()是frame类成员函数（程序注释是返回帧的世界位置坐标），其实它的本质是返回T_f_w_的逆矩阵的平移参数t
   明明已经通过T_cur_from_ref_和frame_ref_->T_f_w_得到frame_cur的旋转和平移参数了，为什么还要再重新计算frame_cur的平移参数呢？
   实际上是为了调整地图规模（scale）。进行一个简单（好吧，其实也挺复杂的）的推导就可以推出来，
   我们把T_cur_from_ref_.rotation_matrix()简称为R1，T_cur_from_ref_.translation()简称为t1，frame_ref_->T_f_w_.translation()简称为t2，
   那个复杂的计算式子最后结果相当于：frame_cur->T_f_w_.translation() =R1*t2 - scale*t1（应该没推错。。。）
  */
  frame_cur->T_f_w_.translation() =
      -frame_cur->T_f_w_.rotation_matrix()*(frame_ref_->pos() + scale*(frame_cur->pos() - frame_ref_->pos()));

  // For each inlier create 3D point and add feature in both frames
  // 创建欧式群矩阵T_world_cur，赋值为frame_cur->T_f_w_的逆矩阵
  SE3 T_world_cur = frame_cur->T_f_w_.inverse();
  // 通过for循环向两帧都添加内点和对应的特征
  for(vector<int>::iterator it=inliers_.begin(); it!=inliers_.end(); ++it)
  {
	// 定义二维向量px_cur和px_ref用于存储内点、特征点的像素坐标，px_cur_存储的是当前帧中各特征点的序号以及坐标，inliers_存储的是内点对应的下标
	// 应该是获得参考帧以及当前帧中的内点的坐标的吧？？？
    Vector2d px_cur(px_cur_[*it].x, px_cur_[*it].y);
    Vector2d px_ref(px_ref_[*it].x, px_ref_[*it].y);

	// 进行判断，若超出可视范围或者深度为负，则跳过，进行下一轮
    if(frame_ref_->cam_->isInFrame(px_cur.cast<int>(), 10) && frame_ref_->cam_->isInFrame(px_ref.cast<int>(), 10) && xyz_in_cur_[*it].z() > 0)
    {
	  // 将内点特征点坐标（相机坐标系）乘以scale后，得到其世界坐标系坐标，存入指针变量new_point
      Vector3d pos = T_world_cur * (xyz_in_cur_[*it]*scale);
      Point* new_point = new Point(pos); // Point构造函数初始化

	  // 新建Feature型指针ftr_cur，并用new_point、px_cur、f_cur_等进行初始化，0指的是金字塔0层
      Feature* ftr_cur(new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it], 0));
	  // 调用Frame类成员函数addFeature，将ftr_cur添加进frame_cur的fts_（特征点列表）
      frame_cur->addFeature(ftr_cur);
	  // 调用Point类成员函数addFrameRef，将ftr_cur添加进new_point的obs_（可以观测到此特征点的帧的指针的列表）
      new_point->addFrameRef(ftr_cur);

	  // 新建Feature型指针ftr_ref并进行同上操作
      Feature* ftr_ref(new Feature(frame_ref_.get(), new_point, px_ref, f_ref_[*it], 0));
      frame_ref_->addFeature(ftr_ref);
      new_point->addFrameRef(ftr_ref);
    }
  }

  // 结束addSecondFrame函数，并返回SUCCESS
  return SUCCESS;
}

/*
* 在本文件中的InitResult KltHomographyInit::addFirstFrame(FramePtr frame_ref)函数中调用
* 在frame_handler_mono.cpp文件中的FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()函数中调用
* 执行清空操作
*/
void KltHomographyInit::reset()
{
	/*
	px_cyr_存储的是当前帧中用于跟踪的关键点坐标
	frame_ref_是一个frame型的智能指针
	*/
  px_cur_.clear();
  frame_ref_.reset();
}

/*
** parameters
* frame: 传入的图像帧
* px_vec: 返回值，参考帧中用于跟踪的关键点的坐标值
* f_vec: 返回值，参考帧中关键点的方向向量
* 在本文件中的InitResult KltHomographyInit::addFirstFrame(FramePtr frame_ref)函数中调用
* 调用FastDetector::detect函数对new_frame进行Fast特征检测
* 并将其关键点坐标集和对应的向量集（关键特征点对应的世界坐标系下的向量）分别赋给px_ref_和f_ref_。
*/
void detectFeatures(
    FramePtr frame,
    vector<cv::Point2f>& px_vec,
    vector<Vector3d>& f_vec)
{
  Features new_features;

  // 构造feature_detection::FastDetector对象
  // Config::gridSize()和Config::nPyrLevels()函数最后都是调用了config.cpp文件中的getInstance()函数，分别指一个cell的数量和迭代收敛的金字塔层数
  feature_detection::FastDetector detector(frame->img().cols, frame->img().rows, Config::gridSize(), Config::nPyrLevels());

  // 调用FastDetector::detect函数对new_frame进行Fast特征检测，返回值new_features是大于阈值的特征点
  // detect()函数转feature_detection.cpp文件，Config::triangMinCornerScore()调用了config.cpp文件中的getInstance()函数
  detector.detect(frame.get(), frame->img_pyr_, Config::triangMinCornerScore(), new_features);

  // now for all maximum corners, initialize a new seed
  // 保存检测到的关键点
  px_vec.clear(); px_vec.reserve(new_features.size());
  f_vec.clear(); f_vec.reserve(new_features.size());

  // 将检测到的关键点的坐标赋值给px_vec
  // 将检测到的关键点的方向向量赋值给f_vec
  std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
    px_vec.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
    f_vec.push_back(ftr->f);
    delete ftr;
  });
}

/*
** parameters
* frame_ref: 参考帧
* frame_cur: 当前帧
* px_ref: 参考帧中用于跟踪的关键点
* px_cur: 当前帧中跟踪到的点
* f_ref: 参考帧中关键点的方向向量
* f_cur: 当前帧中关键点的方向向量
* disparities: 参考帧与当前帧的差异
* 在本函数中的InitResult KltHomographyInit::addSecondFrame()函数中调用
* LK光流跟踪，返回跟踪到的关键点
*/
void trackKlt(
    FramePtr frame_ref,
    FramePtr frame_cur,
    vector<cv::Point2f>& px_ref,
    vector<cv::Point2f>& px_cur,
    vector<Vector3d>& f_ref,
    vector<Vector3d>& f_cur,
    vector<double>& disparities)
{
  // 搜索窗口大小
  const double klt_win_size = 30.0;
  // 最大迭代次数
  const int klt_max_iter = 30;
  // 迭代精度
  const double klt_eps = 0.001;
  // 输出状态向量status，用于表示每个特征是否被找到，找到就置1。
  vector<uchar> status;
  // 输出错误向量error，用于表示每个特征的错误（距离误差）。
  vector<float> error;
  // 
  vector<float> min_eig_vec;

  /*
   创建cv::TermCriteria类型变量termcrit（用于设置迭代算法终止条件），通过重载构造函数实现初始化，
   其中cv::TermCriteria::COUNT表示达到最大迭代次数终止迭代，cv::TermCriteria::EPS表示达到精度后终止，两个加起来表示两个任意一个满足就终止。
  */
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);

  /*
  ** 调用cv::calcOpticalFlowPyrLK实现LK光流跟踪。
  上一帧图像的金字塔frame_ref->img_pyr_[0]
  当前帧图像金字塔frame_cur->img_pyr_[0]
  被跟踪特征点（参考帧）坐标集合px_ref
  当前帧特征点坐标集合px_cur，px_cur用来存储光流法计算出的当前帧中特征点坐标。
  顺便提一下px_cur是三维向量（确切说应该是一个存储2维坐标的向量）：特征序号、特征坐标。
  输出状态向量status，用于表示每个特征是否被找到，找到就置1。
  输出错误向量error，用于表示每个特征的错误（距离误差）。
  设置搜索窗口的大小为klt_win_size*klt_win_size。
  金字塔总层数为4。
  迭代终止条件为termcrit（前面设置的那个）
  flag变量，用于设置光流跟踪方法。设置为cv::OPTFLOW_USE_INITIAL_FLOW，表示利用px_cur中存储的坐标信息进行初步估计。
  */
  cv::calcOpticalFlowPyrLK(frame_ref->img_pyr_[0], frame_cur->img_pyr_[0],
                           px_ref, px_cur,
                           status, error,
                           cv::Size2i(klt_win_size, klt_win_size),
                           4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);

  vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
  vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
  vector<Vector3d>::iterator f_ref_it = f_ref.begin();
  f_cur.clear(); f_cur.reserve(px_cur.size());
  disparities.clear(); disparities.reserve(px_cur.size());
  // 剔除没有被跟踪到的关键点
  for(size_t i=0; px_ref_it != px_ref.end(); ++i)
  {
    if(!status[i])
    {
      px_ref_it = px_ref.erase(px_ref_it);
      px_cur_it = px_cur.erase(px_cur_it);
      f_ref_it = f_ref.erase(f_ref_it);
      continue;
    }
	// 将剩余特征点的像素坐标转换为世界坐标，这个通过frame类的函数c2f（调用cam2world函数，在vikit库中）实现。然后再存入f_cur中。
    f_cur.push_back(frame_cur->c2f(px_cur_it->x, px_cur_it->y));
	// 将特征点移动的像素距离存入disparities_中。参考帧-当前帧
    disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());
    ++px_ref_it;
    ++px_cur_it;
    ++f_ref_it;
  }
}

/*
** parameters
* f_ref: 参考帧中关键点的方向向量，参考帧特征点球坐标（这个球坐标应该指世界坐标系下单位向量）
* f_cur: 当前帧中关键点的方向向量，当前帧特征点球坐标
* focal_length: 焦距
* reprojection_threshold: 重投影阈值（小于该阈值则判断为内点）
* inliers: 输出参数，存储内点下标
* xyz_in_cur: 输出参数，存储3D点按估计位姿投影得到的像素坐标，好像不对，像素坐标怎么会有深度信息呢？？？
* T_cur_from_ref: 输出参数，存储从上帧到当前帧的位姿变化的变换矩阵
* 在本文件中的InitResult KltHomographyInit::addSecondFrame()函数中调用
* 计算单应矩阵
*/
void computeHomography(
    const vector<Vector3d>& f_ref,
    const vector<Vector3d>& f_cur,
    double focal_length,
    double reprojection_threshold,
    vector<int>& inliers,
    vector<Vector3d>& xyz_in_cur,
    SE3& T_cur_from_ref)
{
  vector<Vector2d > uv_ref(f_ref.size());
  vector<Vector2d > uv_cur(f_cur.size());
  for(size_t i=0, i_max=f_ref.size(); i<i_max; ++i)
  {
    uv_ref[i] = vk::project2d(f_ref[i]);
    uv_cur[i] = vk::project2d(f_cur[i]);
  }
  vk::Homography Homography(uv_ref, uv_cur, focal_length, reprojection_threshold);
  Homography.computeSE3fromMatches();
  vector<int> outliers;
  vk::computeInliers(f_cur, f_ref,
                     Homography.T_c2_from_c1.rotation_matrix(), Homography.T_c2_from_c1.translation(),
                     reprojection_threshold, focal_length,
                     xyz_in_cur, inliers, outliers);
  T_cur_from_ref = Homography.T_c2_from_c1;
}


} // namespace initialization
} // namespace svo
