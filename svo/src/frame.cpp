
/*
** frame定义
*/
#include <stdexcept>
#include <svo/frame.h>
#include <svo/feature.h>
#include <svo/point.h>
#include <svo/config.h>
#include <boost/bind.hpp>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/performance_monitor.h>
#include <fast/fast.h>

namespace svo {

int Frame::frame_counter_ = 0;

/*
** parameters
* cam:
* img:
* timestamp:
* 构造函数
* 在frame_handler_mono.cpp文件中的void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)函数中调用
*/
Frame::Frame(vk::AbstractCamera* cam, const cv::Mat& img, double timestamp) :
    // 一些变量的初始化，如id、系统时间、相机模型、用于判两帧是否有重叠视野的特征数量、关键帧标志等。
	id_(frame_counter_++),
    timestamp_(timestamp),
    cam_(cam),
    key_pts_(5),
    is_keyframe_(false),
    v_kf_(NULL)
{
  initFrame(img);// 转本文件
}

Frame::~Frame()
{
  std::for_each(fts_.begin(), fts_.end(), [&](Feature* i){delete i;});
}

/*
** parameter
* img: 输入图像
* 在本文件中的Frame::Frame()构造函数中调用
*/
void Frame::initFrame(const cv::Mat& img)
{
  // check image
	// 检查图像是否符合规范，是否为空、数据格式（要求是灰度图）、行列值
  if(img.empty() || img.type() != CV_8UC1 || img.cols != cam_->width() || img.rows != cam_->height())
    throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");

  // Set keypoints to NULL
  // 将特征点变量清空
  std::for_each(key_pts_.begin(), key_pts_.end(), [&](Feature* ftr){ ftr=NULL; });

  // Build Image Pyramid
  // 创建图像金字塔
  frame_utils::createImgPyramid(img, max(Config::nPyrLevels(), Config::kltMaxLevel()+1), img_pyr_);
}

/*
* 在frame_handler_mono.cpp文件中的FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()函数中调用
* 在frame_handler_mono.cpp文件中的FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()函数中调用
* 在frame_handler_mono.cpp文件中的FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()函数中调用
** function: 将当前帧设为关键帧
*/
void Frame::setKeyframe()
{
  is_keyframe_ = true;// 标志位
  setKeyPoints();
}

/*
* 在initilization.cpp文件中的InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)函数中调用
* 在reprojector.cpp文件中的bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)函数中调用
* 在map.cpp文件中的void MapPointCandidates::addCandidatePointToFrame(FramePtr frame)函数中调用
* function: 将ftr特征点加入fts_特征点链表中
*/
void Frame::addFeature(Feature* ftr)
{
  fts_.push_back(ftr);
}

/*
* 在本文件中的void Frame::setKeyFrame()函数中调用
* 在本文件中的void Frame::removeKeyPoint(Feature* ftr)函数中调用
** function: 关键点是最靠近4个图像角点和中心并指定了一个三维点的五个特征。这些点用于快速检查两帧的视野是否重叠。
*/
void Frame::setKeyPoints()
{
  for(size_t i = 0; i < 5; ++i)
    if(key_pts_[i] != NULL)
      if(key_pts_[i]->point == NULL)
        key_pts_[i] = NULL;

  std::for_each(fts_.begin(), fts_.end(), [&](Feature* ftr){ if(ftr->point != NULL) checkKeyPoints(ftr); });
}

/*
** parameter
* ftr: 用于帧间跟踪的特征点
* 在本文件中的void Frame::setKeyPoints()函数中调用
** function: 对当前图像中每个特征点进行遍历和比较，最终选出最具有代表性的5个作为关键点，实质上是1个靠近图像中心的点和4个靠近图像四个角的点。
*/
void Frame::checkKeyPoints(Feature* ftr)
{
	// 取中心点
  const int cu = cam_->width()/2;
  const int cv = cam_->height()/2;

  // center pixel
  // 中心像素点
  // key_pts_是指待搜寻的五个特征点
  if(key_pts_[0] == NULL)
    key_pts_[0] = ftr;
  else if(std::max(std::fabs(ftr->px[0]-cu), std::fabs(ftr->px[1]-cv))
        < std::max(std::fabs(key_pts_[0]->px[0]-cu), std::fabs(key_pts_[0]->px[1]-cv)))
    key_pts_[0] = ftr;

  if(ftr->px[0] >= cu && ftr->px[1] >= cv)
  {
    if(key_pts_[1] == NULL)
      key_pts_[1] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[1]->px[0]-cu) * (key_pts_[1]->px[1]-cv))
      key_pts_[1] = ftr;
  }
  if(ftr->px[0] >= cu && ftr->px[1] < cv)
  {
    if(key_pts_[2] == NULL)
      key_pts_[2] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[2]->px[0]-cu) * (key_pts_[2]->px[1]-cv))
      key_pts_[2] = ftr;
  }
  if(ftr->px[0] < cv && ftr->px[1] < cv)
  {
    if(key_pts_[3] == NULL)
      key_pts_[3] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[3]->px[0]-cu) * (key_pts_[3]->px[1]-cv))
      key_pts_[3] = ftr;
  }
  if(ftr->px[0] < cv && ftr->px[1] >= cv)
  {
    if(key_pts_[4] == NULL)
      key_pts_[4] = ftr;
    else if((ftr->px[0]-cu) * (ftr->px[1]-cv)
          > (key_pts_[4]->px[0]-cu) * (key_pts_[4]->px[1]-cv))
      key_pts_[4] = ftr;
  }
}

/*
** parameter
* ftr: 输入的帧的特征点
* 在map.cpp文件中的void Map::safeDeletePoint(Point* pt)函数中调用
** function: 移除删除的地图点所对应的关键点
*/
void Frame::removeKeyPoint(Feature* ftr)
{
  bool found = false;
  // 搜寻关键点中待删除的关键点
  std::for_each(key_pts_.begin(), key_pts_.end(), [&](Feature*& i){
    if(i == ftr) {
      i = NULL;
      found = true;
    }
  });
  if(found)
    setKeyPoints();
}

/*
** parameter
* xyz_w: 世界坐标系下的坐标
* 在map.cpp文件中的void Map::getCloseKeyframes()函数中调用
** function: 检查某3d点是否能在图像中被观察到
*/
bool Frame::isVisible(const Vector3d& xyz_w) const
{
  Vector3d xyz_f = T_f_w_*xyz_w;
  if(xyz_f.z() < 0.0)
    return false; // point is behind the camera
  Vector2d px = f2c(xyz_f);
  if(px[0] >= 0.0 && px[1] >= 0.0 && px[0] < cam_->width() && px[1] < cam_->height())
    return true;
  return false;
}


/// Utility functions for the Frame class
namespace frame_utils {

/*
** parameters
* img_level_0:原始图像
* n_levels:金字塔层数
* pyr:输出参数，创建的图像金字塔
* 在本文件中的void Frame::initFrame(const cv::Mat& img)函数中调用
* 创建图像金字塔
*/
void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr)
{
  // 获取层数
  pyr.resize(n_levels);
  // 将原始图像作为第0层
  pyr[0] = img_level_0;
  for(int i=1; i<n_levels; ++i)
  {
	  // 每层的缩放系数为1/2
    pyr[i] = cv::Mat(pyr[i-1].rows/2, pyr[i-1].cols/2, CV_8U);

	// 这个函数目前还不知道具体是在做什么？？？
    vk::halfSample(pyr[i-1], pyr[i]);
  }
}

/*
** parameters
* frame: 图像帧
* depth_mean: 输出参数，平均深度
* depth_min: 输出参数，最小深度
* 在文件frame_handler_mono.cpp文件中的FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()函数中调用
* 在文件frame_handler_mono.cpp文件中的FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()函数中调用
* function: 获得当前场景的平均深度和最小深度
*/
bool getSceneDepth(const Frame& frame, double& depth_mean, double& depth_min)
{
  // 定义depth_vec用于存放特征点的深度信息
  vector<double> depth_vec;
  // depth_vec容量应与特征点数目保持一致
  depth_vec.reserve(frame.fts_.size());
  // 好吧，这个没看懂？？？
  depth_min = std::numeric_limits<double>::max();
  // 开始循环查找
  for(auto it=frame.fts_.begin(), ite=frame.fts_.end(); it!=ite; ++it)
  {
	// 若特征点不为空
    if((*it)->point != NULL)
    {
	  // 通过调用w2f()函数，将点从世界坐标系转为相机坐标系，而后获得在相机坐标系下的深度信息
      const double z = frame.w2f((*it)->point->pos_).z();
	  // 将深度信息存入
      depth_vec.push_back(z);
	  // 获得最小的深度
      depth_min = fmin(z, depth_min);
    }
  }

  // 这里应该是指没有特征点吧，返回false
  if(depth_vec.empty())
  {
    SVO_WARN_STREAM("Cannot set scene depth. Frame has no point-observations!");
    return false;
  }

  // 调用vikit库中的getMidan()函数，获得平均深度
  depth_mean = vk::getMedian(depth_vec);
  // 返回true
  return true;
}

} // namespace frame_utils
} // namespace svo
