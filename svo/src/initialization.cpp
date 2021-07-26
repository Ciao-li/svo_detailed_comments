
/*
 ** ��ʼ��
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
* frame_ref: �����ͼ��֡
* ��frame_handler_mono.cpp�ļ��е�FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()�����е���
* ��ӵ�һ֡
* InitResult��ö�����ͣ�����FAILURE, NO_KEYFRAME, SUCCESS�������ʼ���Ľ��
*/
InitResult KltHomographyInit::addFirstFrame(FramePtr frame_ref)
{
  // ��ʼ��px_cur_��һ����ά���������洢��ǰ֡�б����ڸ��ٵĹؼ������꣩��frame_ref_��һ��frame������ָ�룩
  // ת���ļ�
  reset();

  // ��new_frame����Fast������⣬����ֵ�ǵ÷ִ�����ֵ�������������px_ref_������f_ref_
  // ת���ļ�
  detectFeatures(frame_ref, px_ref_, f_ref_);

  // �����������Ŀ����100���򷵻ش��󣬽���������
  if(px_ref_.size() < 100)
  {
    SVO_WARN_STREAM_THROTTLE(2.0, "First image has less than 100 features. Retry in more textured environment.");
    return FAILURE;
  }

  // ������������Ŀ����100
  // �������new_frame_��ֵ��frame_ref_
  frame_ref_ = frame_ref;
  // ��px_ref_��ֵ����px_cur_
  px_cur_.insert(px_cur_.begin(), px_ref_.begin(), px_ref_.end());
  // ����SUCCESS
  return SUCCESS;
}

/*
** parameter
* frame_cur: ��ǰ֡
* ��frame_handler_mono.cpp�ļ��е�FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()�����е���
* ��ӵڶ�֡�����ؽ����ʧ�ܡ��޹ؼ�֡���ɹ�
*/
InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)
{
	// LK���������ظ��ٵ��ĵ�
  trackKlt(frame_ref_, frame_cur, px_ref_, px_cur_, f_ref_, f_cur_, disparities_);
  SVO_INFO_STREAM("Init: KLT tracked "<< disparities_.size() <<" features");

  // �жϸ��ٵ������������Ŀ�Ƿ�С���趨����ֵ���ǵĻ��ͽ���addSecondFrame������������FAILURE��
  // ΪʲôҪ��disparities_��������Ӧ����px_cur_�𣿣���
  if(disparities_.size() < Config::initMinTracked())
    return FAILURE;

  // ����getMedian�����õ�disparities_�е���λ����Ϊƽ�����룬��ֵ������disparity��
  // Ȼ���ж�disparity����ֵС���趨ֵ������addSecondFrame������������NO_KEYFRAME��
  double disparity = vk::getMedian(disparities_);
  SVO_INFO_STREAM("Init: KLT "<<disparity<<"px average disparity.");
  if(disparity < Config::initMinDisparity())
    return NO_KEYFRAME;

  // ���㵥Ӧ����
  computeHomography(
      f_ref_, f_cur_,
      frame_ref_->cam_->errorMultiplier2(), Config::poseOptimThresh(),
      inliers_, xyz_in_cur_, T_cur_from_ref_);
  SVO_INFO_STREAM("Init: Homography RANSAC "<<inliers_.size()<<" inliers.");

  // �ж��ڵ���������С���趨��ֵ�������addSecondFrame������������FAILURE��
  if(inliers_.size() < Config::initMinInliers())
  {
    SVO_WARN_STREAM("Init WARNING: "<<Config::initMinInliers()<<" inliers minimum required.");
    return FAILURE;
  }

  // Rescale the map such that the mean scene depth is equal to the specified scale
  // ���ڵ�ͼ��С��ʹ��ͼƽ����ȵ���ָ������
  vector<double> depth_vec;
  // ��xyz_in_cur����ȡ3D�������Ϣ
  for(size_t i=0; i<xyz_in_cur_.size(); ++i)
    depth_vec.push_back((xyz_in_cur_[i]).z());
  // ѡ����λ����Ϊ��ֵ
  double scene_depth_median = vk::getMedian(depth_vec);
  // ��һ����ͼ�߶ȣ�����
  double scale = Config::mapScale()/scene_depth_median;
  // ���㵱ǰ֡�ı任���󣨿ɱ�ʾ���λ�˵ı仯��
  // SE3�жԳ˷���*��������������ʵ�־���˷�������T_cur_from_ref_ * frame_ref_->T_f_w_��ʾ�Ժ��߽���ǰ����ʽ�ı任
  frame_cur->T_f_w_ = T_cur_from_ref_ * frame_ref_->T_f_w_;
  /*
   T_f_w.rotation_matrix()��T_f_w.translation()�ֱ𷵻�SE3�ͱ�����R��t
   pos()��frame���Ա����������ע���Ƿ���֡������λ�����꣩����ʵ���ı����Ƿ���T_f_w_��������ƽ�Ʋ���t
   �����Ѿ�ͨ��T_cur_from_ref_��frame_ref_->T_f_w_�õ�frame_cur����ת��ƽ�Ʋ����ˣ�Ϊʲô��Ҫ�����¼���frame_cur��ƽ�Ʋ����أ�
   ʵ������Ϊ�˵�����ͼ��ģ��scale��������һ���򵥣��ðɣ���ʵҲͦ���ӵģ����Ƶ��Ϳ����Ƴ�����
   ���ǰ�T_cur_from_ref_.rotation_matrix()���ΪR1��T_cur_from_ref_.translation()���Ϊt1��frame_ref_->T_f_w_.translation()���Ϊt2��
   �Ǹ����ӵļ���ʽ��������൱�ڣ�frame_cur->T_f_w_.translation() =R1*t2 - scale*t1��Ӧ��û�ƴ�������
  */
  frame_cur->T_f_w_.translation() =
      -frame_cur->T_f_w_.rotation_matrix()*(frame_ref_->pos() + scale*(frame_cur->pos() - frame_ref_->pos()));

  // For each inlier create 3D point and add feature in both frames
  // ����ŷʽȺ����T_world_cur����ֵΪframe_cur->T_f_w_�������
  SE3 T_world_cur = frame_cur->T_f_w_.inverse();
  // ͨ��forѭ������֡������ڵ�Ͷ�Ӧ������
  for(vector<int>::iterator it=inliers_.begin(); it!=inliers_.end(); ++it)
  {
	// �����ά����px_cur��px_ref���ڴ洢�ڵ㡢��������������꣬px_cur_�洢���ǵ�ǰ֡�и������������Լ����꣬inliers_�洢�����ڵ��Ӧ���±�
	// Ӧ���ǻ�òο�֡�Լ���ǰ֡�е��ڵ������İɣ�����
    Vector2d px_cur(px_cur_[*it].x, px_cur_[*it].y);
    Vector2d px_ref(px_ref_[*it].x, px_ref_[*it].y);

	// �����жϣ����������ӷ�Χ�������Ϊ������������������һ��
    if(frame_ref_->cam_->isInFrame(px_cur.cast<int>(), 10) && frame_ref_->cam_->isInFrame(px_ref.cast<int>(), 10) && xyz_in_cur_[*it].z() > 0)
    {
	  // ���ڵ����������꣨�������ϵ������scale�󣬵õ�����������ϵ���꣬����ָ�����new_point
      Vector3d pos = T_world_cur * (xyz_in_cur_[*it]*scale);
      Point* new_point = new Point(pos); // Point���캯����ʼ��

	  // �½�Feature��ָ��ftr_cur������new_point��px_cur��f_cur_�Ƚ��г�ʼ����0ָ���ǽ�����0��
      Feature* ftr_cur(new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it], 0));
	  // ����Frame���Ա����addFeature����ftr_cur��ӽ�frame_cur��fts_���������б�
      frame_cur->addFeature(ftr_cur);
	  // ����Point���Ա����addFrameRef����ftr_cur��ӽ�new_point��obs_�����Թ۲⵽���������֡��ָ����б�
      new_point->addFrameRef(ftr_cur);

	  // �½�Feature��ָ��ftr_ref������ͬ�ϲ���
      Feature* ftr_ref(new Feature(frame_ref_.get(), new_point, px_ref, f_ref_[*it], 0));
      frame_ref_->addFeature(ftr_ref);
      new_point->addFrameRef(ftr_ref);
    }
  }

  // ����addSecondFrame������������SUCCESS
  return SUCCESS;
}

/*
* �ڱ��ļ��е�InitResult KltHomographyInit::addFirstFrame(FramePtr frame_ref)�����е���
* ��frame_handler_mono.cpp�ļ��е�FrameHandlerBase::UpdateResult FrameHandlerMono::processSecondFrame()�����е���
* ִ����ղ���
*/
void KltHomographyInit::reset()
{
	/*
	px_cyr_�洢���ǵ�ǰ֡�����ڸ��ٵĹؼ�������
	frame_ref_��һ��frame�͵�����ָ��
	*/
  px_cur_.clear();
  frame_ref_.reset();
}

/*
** parameters
* frame: �����ͼ��֡
* px_vec: ����ֵ���ο�֡�����ڸ��ٵĹؼ��������ֵ
* f_vec: ����ֵ���ο�֡�йؼ���ķ�������
* �ڱ��ļ��е�InitResult KltHomographyInit::addFirstFrame(FramePtr frame_ref)�����е���
* ����FastDetector::detect������new_frame����Fast�������
* ������ؼ������꼯�Ͷ�Ӧ�����������ؼ��������Ӧ����������ϵ�µ��������ֱ𸳸�px_ref_��f_ref_��
*/
void detectFeatures(
    FramePtr frame,
    vector<cv::Point2f>& px_vec,
    vector<Vector3d>& f_vec)
{
  Features new_features;

  // ����feature_detection::FastDetector����
  // Config::gridSize()��Config::nPyrLevels()��������ǵ�����config.cpp�ļ��е�getInstance()�������ֱ�ָһ��cell�������͵��������Ľ���������
  feature_detection::FastDetector detector(frame->img().cols, frame->img().rows, Config::gridSize(), Config::nPyrLevels());

  // ����FastDetector::detect������new_frame����Fast������⣬����ֵnew_features�Ǵ�����ֵ��������
  // detect()����תfeature_detection.cpp�ļ���Config::triangMinCornerScore()������config.cpp�ļ��е�getInstance()����
  detector.detect(frame.get(), frame->img_pyr_, Config::triangMinCornerScore(), new_features);

  // now for all maximum corners, initialize a new seed
  // �����⵽�Ĺؼ���
  px_vec.clear(); px_vec.reserve(new_features.size());
  f_vec.clear(); f_vec.reserve(new_features.size());

  // ����⵽�Ĺؼ�������긳ֵ��px_vec
  // ����⵽�Ĺؼ���ķ���������ֵ��f_vec
  std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
    px_vec.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
    f_vec.push_back(ftr->f);
    delete ftr;
  });
}

/*
** parameters
* frame_ref: �ο�֡
* frame_cur: ��ǰ֡
* px_ref: �ο�֡�����ڸ��ٵĹؼ���
* px_cur: ��ǰ֡�и��ٵ��ĵ�
* f_ref: �ο�֡�йؼ���ķ�������
* f_cur: ��ǰ֡�йؼ���ķ�������
* disparities: �ο�֡�뵱ǰ֡�Ĳ���
* �ڱ������е�InitResult KltHomographyInit::addSecondFrame()�����е���
* LK�������٣����ظ��ٵ��Ĺؼ���
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
  // �������ڴ�С
  const double klt_win_size = 30.0;
  // ����������
  const int klt_max_iter = 30;
  // ��������
  const double klt_eps = 0.001;
  // ���״̬����status�����ڱ�ʾÿ�������Ƿ��ҵ����ҵ�����1��
  vector<uchar> status;
  // �����������error�����ڱ�ʾÿ�������Ĵ��󣨾�������
  vector<float> error;
  // 
  vector<float> min_eig_vec;

  /*
   ����cv::TermCriteria���ͱ���termcrit���������õ����㷨��ֹ��������ͨ�����ع��캯��ʵ�ֳ�ʼ����
   ����cv::TermCriteria::COUNT��ʾ�ﵽ������������ֹ������cv::TermCriteria::EPS��ʾ�ﵽ���Ⱥ���ֹ��������������ʾ��������һ���������ֹ��
  */
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);

  /*
  ** ����cv::calcOpticalFlowPyrLKʵ��LK�������١�
  ��һ֡ͼ��Ľ�����frame_ref->img_pyr_[0]
  ��ǰ֡ͼ�������frame_cur->img_pyr_[0]
  �����������㣨�ο�֡�����꼯��px_ref
  ��ǰ֡���������꼯��px_cur��px_cur�����洢������������ĵ�ǰ֡�����������ꡣ
  ˳����һ��px_cur����ά������ȷ��˵Ӧ����һ���洢2ά�������������������š��������ꡣ
  ���״̬����status�����ڱ�ʾÿ�������Ƿ��ҵ����ҵ�����1��
  �����������error�����ڱ�ʾÿ�������Ĵ��󣨾�������
  �����������ڵĴ�СΪklt_win_size*klt_win_size��
  �������ܲ���Ϊ4��
  ������ֹ����Ϊtermcrit��ǰ�����õ��Ǹ���
  flag�������������ù������ٷ���������Ϊcv::OPTFLOW_USE_INITIAL_FLOW����ʾ����px_cur�д洢��������Ϣ���г������ơ�
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
  // �޳�û�б����ٵ��Ĺؼ���
  for(size_t i=0; px_ref_it != px_ref.end(); ++i)
  {
    if(!status[i])
    {
      px_ref_it = px_ref.erase(px_ref_it);
      px_cur_it = px_cur.erase(px_cur_it);
      f_ref_it = f_ref.erase(f_ref_it);
      continue;
    }
	// ��ʣ�����������������ת��Ϊ�������꣬���ͨ��frame��ĺ���c2f������cam2world��������vikit���У�ʵ�֡�Ȼ���ٴ���f_cur�С�
    f_cur.push_back(frame_cur->c2f(px_cur_it->x, px_cur_it->y));
	// ���������ƶ������ؾ������disparities_�С��ο�֡-��ǰ֡
    disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());
    ++px_ref_it;
    ++px_cur_it;
    ++f_ref_it;
  }
}

/*
** parameters
* f_ref: �ο�֡�йؼ���ķ����������ο�֡�����������꣨���������Ӧ��ָ��������ϵ�µ�λ������
* f_cur: ��ǰ֡�йؼ���ķ�����������ǰ֡������������
* focal_length: ����
* reprojection_threshold: ��ͶӰ��ֵ��С�ڸ���ֵ���ж�Ϊ�ڵ㣩
* inliers: ����������洢�ڵ��±�
* xyz_in_cur: ����������洢3D�㰴����λ��ͶӰ�õ����������꣬���񲻶ԣ�����������ô���������Ϣ�أ�����
* T_cur_from_ref: ����������洢����֡����ǰ֡��λ�˱仯�ı任����
* �ڱ��ļ��е�InitResult KltHomographyInit::addSecondFrame()�����е���
* ���㵥Ӧ����
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
