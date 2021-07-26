
/*
** frame����
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
* ���캯��
* ��frame_handler_mono.cpp�ļ��е�void FrameHandlerMono::addImage(const cv::Mat& img, const double timestamp)�����е���
*/
Frame::Frame(vk::AbstractCamera* cam, const cv::Mat& img, double timestamp) :
    // һЩ�����ĳ�ʼ������id��ϵͳʱ�䡢���ģ�͡���������֡�Ƿ����ص���Ұ�������������ؼ�֡��־�ȡ�
	id_(frame_counter_++),
    timestamp_(timestamp),
    cam_(cam),
    key_pts_(5),
    is_keyframe_(false),
    v_kf_(NULL)
{
  initFrame(img);// ת���ļ�
}

Frame::~Frame()
{
  std::for_each(fts_.begin(), fts_.end(), [&](Feature* i){delete i;});
}

/*
** parameter
* img: ����ͼ��
* �ڱ��ļ��е�Frame::Frame()���캯���е���
*/
void Frame::initFrame(const cv::Mat& img)
{
  // check image
	// ���ͼ���Ƿ���Ϲ淶���Ƿ�Ϊ�ա����ݸ�ʽ��Ҫ���ǻҶ�ͼ��������ֵ
  if(img.empty() || img.type() != CV_8UC1 || img.cols != cam_->width() || img.rows != cam_->height())
    throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");

  // Set keypoints to NULL
  // ��������������
  std::for_each(key_pts_.begin(), key_pts_.end(), [&](Feature* ftr){ ftr=NULL; });

  // Build Image Pyramid
  // ����ͼ�������
  frame_utils::createImgPyramid(img, max(Config::nPyrLevels(), Config::kltMaxLevel()+1), img_pyr_);
}

/*
* ��frame_handler_mono.cpp�ļ��е�FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()�����е���
* ��frame_handler_mono.cpp�ļ��е�FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()�����е���
* ��frame_handler_mono.cpp�ļ��е�FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()�����е���
** function: ����ǰ֡��Ϊ�ؼ�֡
*/
void Frame::setKeyframe()
{
  is_keyframe_ = true;// ��־λ
  setKeyPoints();
}

/*
* ��initilization.cpp�ļ��е�InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)�����е���
* ��reprojector.cpp�ļ��е�bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)�����е���
* ��map.cpp�ļ��е�void MapPointCandidates::addCandidatePointToFrame(FramePtr frame)�����е���
* function: ��ftr���������fts_������������
*/
void Frame::addFeature(Feature* ftr)
{
  fts_.push_back(ftr);
}

/*
* �ڱ��ļ��е�void Frame::setKeyFrame()�����е���
* �ڱ��ļ��е�void Frame::removeKeyPoint(Feature* ftr)�����е���
** function: �ؼ��������4��ͼ��ǵ�����Ĳ�ָ����һ����ά��������������Щ�����ڿ��ټ����֡����Ұ�Ƿ��ص���
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
* ftr: ����֡����ٵ�������
* �ڱ��ļ��е�void Frame::setKeyPoints()�����е���
** function: �Ե�ǰͼ����ÿ����������б����ͱȽϣ�����ѡ������д����Ե�5����Ϊ�ؼ��㣬ʵ������1������ͼ�����ĵĵ��4������ͼ���ĸ��ǵĵ㡣
*/
void Frame::checkKeyPoints(Feature* ftr)
{
	// ȡ���ĵ�
  const int cu = cam_->width()/2;
  const int cv = cam_->height()/2;

  // center pixel
  // �������ص�
  // key_pts_��ָ����Ѱ�����������
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
* ftr: �����֡��������
* ��map.cpp�ļ��е�void Map::safeDeletePoint(Point* pt)�����е���
** function: �Ƴ�ɾ���ĵ�ͼ������Ӧ�Ĺؼ���
*/
void Frame::removeKeyPoint(Feature* ftr)
{
  bool found = false;
  // ��Ѱ�ؼ����д�ɾ���Ĺؼ���
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
* xyz_w: ��������ϵ�µ�����
* ��map.cpp�ļ��е�void Map::getCloseKeyframes()�����е���
** function: ���ĳ3d���Ƿ�����ͼ���б��۲쵽
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
* img_level_0:ԭʼͼ��
* n_levels:����������
* pyr:���������������ͼ�������
* �ڱ��ļ��е�void Frame::initFrame(const cv::Mat& img)�����е���
* ����ͼ�������
*/
void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr)
{
  // ��ȡ����
  pyr.resize(n_levels);
  // ��ԭʼͼ����Ϊ��0��
  pyr[0] = img_level_0;
  for(int i=1; i<n_levels; ++i)
  {
	  // ÿ�������ϵ��Ϊ1/2
    pyr[i] = cv::Mat(pyr[i-1].rows/2, pyr[i-1].cols/2, CV_8U);

	// �������Ŀǰ����֪������������ʲô������
    vk::halfSample(pyr[i-1], pyr[i]);
  }
}

/*
** parameters
* frame: ͼ��֡
* depth_mean: ���������ƽ�����
* depth_min: �����������С���
* ���ļ�frame_handler_mono.cpp�ļ��е�FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()�����е���
* ���ļ�frame_handler_mono.cpp�ļ��е�FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()�����е���
* function: ��õ�ǰ������ƽ����Ⱥ���С���
*/
bool getSceneDepth(const Frame& frame, double& depth_mean, double& depth_min)
{
  // ����depth_vec���ڴ��������������Ϣ
  vector<double> depth_vec;
  // depth_vec����Ӧ����������Ŀ����һ��
  depth_vec.reserve(frame.fts_.size());
  // �ðɣ����û����������
  depth_min = std::numeric_limits<double>::max();
  // ��ʼѭ������
  for(auto it=frame.fts_.begin(), ite=frame.fts_.end(); it!=ite; ++it)
  {
	// �������㲻Ϊ��
    if((*it)->point != NULL)
    {
	  // ͨ������w2f()�������������������ϵתΪ�������ϵ�����������������ϵ�µ������Ϣ
      const double z = frame.w2f((*it)->point->pos_).z();
	  // �������Ϣ����
      depth_vec.push_back(z);
	  // �����С�����
      depth_min = fmin(z, depth_min);
    }
  }

  // ����Ӧ����ָû��������ɣ�����false
  if(depth_vec.empty())
  {
    SVO_WARN_STREAM("Cannot set scene depth. Frame has no point-observations!");
    return false;
  }

  // ����vikit���е�getMidan()���������ƽ�����
  depth_mean = vk::getMedian(depth_vec);
  // ����true
  return true;
}

} // namespace frame_utils
} // namespace svo
