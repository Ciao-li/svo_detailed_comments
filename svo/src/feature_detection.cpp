
/*
** �������
*/
#include <svo/feature_detection.h>
#include <svo/feature.h>
#include <fast/fast.h>
#include <vikit/vision.h>

namespace svo {
namespace feature_detection {

/*
* ���캯��
* �ڱ��ļ��е�FastDetector::FastDetector()�����е���
*/
AbstractDetector::AbstractDetector(
    const int img_width, // ͼ����
    const int img_height, // ͼ��߶�
    const int cell_size, // cell�ĳߴ�
    const int n_pyr_levels) : // ����������
        cell_size_(cell_size),
        n_pyr_levels_(n_pyr_levels),
        grid_n_cols_(ceil(static_cast<double>(img_width)/cell_size_)), // ��������� ����ȡ��
        grid_n_rows_(ceil(static_cast<double>(img_height)/cell_size_)), // ���������
        grid_occupancy_(grid_n_cols_*grid_n_rows_, false) // ����ռ�ݵĳߴ� ��*��
{}

/*
* �ڱ��ļ��е�void FastDetector::detect()�����е���
* ��λgrid_occupancy_�������Ա��´�ʹ��
*/
void AbstractDetector::resetGrid()
{
  // vector<bool> grid_occupancy_;
  // fill������������:��һ�������Ԫ�ض�����valֵ������������fill(vec.begin(), vec.end(), val); valΪ��Ҫ�滻��ֵ
  std::fill(grid_occupancy_.begin(), grid_occupancy_.end(), false);
}

void AbstractDetector::setExistingFeatures(const Features& fts)
{
  std::for_each(fts.begin(), fts.end(), [&](Feature* i){
    grid_occupancy_.at(
        static_cast<int>(i->px[1]/cell_size_)*grid_n_cols_
        + static_cast<int>(i->px[0]/cell_size_)) = true;
  });
}

/*
** parameter
* px: 
* ��depth_filter.cpp�ļ��е�void DepthFilter::updateSeeds(FramePtr frame)�����е���
** function: ��������cellΪ��ռ��
*/
void AbstractDetector::setGridOccpuancy(const Vector2d& px)
{
  grid_occupancy_.at(
      static_cast<int>(px[1]/cell_size_)*grid_n_cols_
    + static_cast<int>(px[0]/cell_size_)) = true;
}

/*
* ���캯��
* ��frame_handler_mono.cpp�ļ��е�void FrameHandlerMono::initialize()�����е���
*/
FastDetector::FastDetector(
    const int img_width,// ͼ����
    const int img_height, // ͼ��߶�
    const int cell_size, // ����ͼ�����������
    const int n_pyr_levels) : // ���ڽ���������
        AbstractDetector(img_width, img_height, cell_size, n_pyr_levels)
{}

/*
** parameters
* frame: ��ǰ֡
* img_pyr: ͼ�������
* detection_threshold: ��ֵ������ѡ�����������������ǻ���Harris�ǵ����͵÷�
* fts: ����ֵ�������⵽��������
* ��initialization.cpp�ļ��е�void detectFeatures(FramePtr frame,vector<cv::Point2f>& px_vec,vector<Vector3d>& f_vec)�����е���
* ���ڼ��FAST��
*/
void FastDetector::detect(
    Frame* frame,
    const ImgPyr& img_pyr,
    const double detection_threshold,
    Features& fts)
{
	// ��ʼ���ǵ�����������ߴ磬����������꣬��ֵ�����ֵ�
  Corners corners(grid_n_cols_*grid_n_rows_, Corner(0,0,detection_threshold,0,0.0f));

  for(int L=0; L<n_pyr_levels_; ++L)
  {
    const int scale = (1<<L);

	// ����ǵ�����
    vector<fast::fast_xy> fast_corners;

	// �������룬�ðɣ������⼸����������û����������
	// �⼸��fast��FAST���йأ����廹û��
#if __SSE2__
      fast::fast_corner_detect_10_sse2(
          (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
          img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#elif HAVE_FAST_NEON
      fast::fast_corner_detect_9_neon(
          (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
          img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#else
      fast::fast_corner_detect_10(
          (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
          img_pyr[L].rows, img_pyr[L].cols, 20, fast_corners);
#endif

	// ����÷��Լ��ǵ���Ŀ����
    vector<int> scores, nm_corners;
    fast::fast_corner_score_10((fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols, fast_corners, 20, scores);
    fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

	// 
    for(auto it=nm_corners.begin(), ite=nm_corners.end(); it!=ite; ++it)
    {
      fast::fast_xy& xy = fast_corners.at(*it);
      const int k = static_cast<int>((xy.y*scale)/cell_size_)*grid_n_cols_
                  + static_cast<int>((xy.x*scale)/cell_size_);
      if(grid_occupancy_[k])
        continue;
      const float score = vk::shiTomasiScore(img_pyr[L], xy.x, xy.y);
      if(score > corners.at(k).score)
        corners.at(k) = Corner(xy.x*scale, xy.y*scale, score, L, 0.0f);
    }
  }

  // Create feature for every corner that has high enough corner score
  // �����е÷ִ�����ֵ�������㴫��fts�У���Ϊ����ֵ
  std::for_each(corners.begin(), corners.end(), [&](Corner& c) {
    if(c.score > detection_threshold)
      fts.push_back(new Feature(frame, Vector2d(c.x, c.y), c.level));
  });

  // ��λgrid_occupancy_����
  resetGrid();
}

} // namespace feature_detection
} // namespace svo

