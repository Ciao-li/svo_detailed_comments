
/*
** 特征检测
*/
#include <svo/feature_detection.h>
#include <svo/feature.h>
#include <fast/fast.h>
#include <vikit/vision.h>

namespace svo {
namespace feature_detection {

/*
* 构造函数
* 在本文件中的FastDetector::FastDetector()函数中调用
*/
AbstractDetector::AbstractDetector(
    const int img_width, // 图像宽度
    const int img_height, // 图像高度
    const int cell_size, // cell的尺寸
    const int n_pyr_levels) : // 金字塔层数
        cell_size_(cell_size),
        n_pyr_levels_(n_pyr_levels),
        grid_n_cols_(ceil(static_cast<double>(img_width)/cell_size_)), // 网格的列数 向上取整
        grid_n_rows_(ceil(static_cast<double>(img_height)/cell_size_)), // 网格的行数
        grid_occupancy_(grid_n_cols_*grid_n_rows_, false) // 网格占据的尺寸 列*行
{}

/*
* 在本文件中的void FastDetector::detect()函数中调用
* 复位grid_occupancy_变量，以便下次使用
*/
void AbstractDetector::resetGrid()
{
  // vector<bool> grid_occupancy_;
  // fill函数的作用是:将一个区间的元素都赋予val值。函数参数：fill(vec.begin(), vec.end(), val); val为将要替换的值
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
* 在depth_filter.cpp文件中的void DepthFilter::updateSeeds(FramePtr frame)函数中调用
** function: 设置网格cell为已占用
*/
void AbstractDetector::setGridOccpuancy(const Vector2d& px)
{
  grid_occupancy_.at(
      static_cast<int>(px[1]/cell_size_)*grid_n_cols_
    + static_cast<int>(px[0]/cell_size_)) = true;
}

/*
* 构造函数
* 在frame_handler_mono.cpp文件中的void FrameHandlerMono::initialize()函数中调用
*/
FastDetector::FastDetector(
    const int img_width,// 图像宽度
    const int img_height, // 图像高度
    const int cell_size, // 所在图像的网格数量
    const int n_pyr_levels) : // 所在金字塔层数
        AbstractDetector(img_width, img_height, cell_size, n_pyr_levels)
{}

/*
** parameters
* frame: 当前帧
* img_pyr: 图像金字塔
* detection_threshold: 阈值，用于选定特征点中用来三角化的Harris角点的最低得分
* fts: 返回值，代表检测到的特征点
* 在initialization.cpp文件中的void detectFeatures(FramePtr frame,vector<cv::Point2f>& px_vec,vector<Vector3d>& f_vec)函数中调用
* 用于检测FAST点
*/
void FastDetector::detect(
    Frame* frame,
    const ImgPyr& img_pyr,
    const double detection_threshold,
    Features& fts)
{
	// 初始化角点检测器，网格尺寸，特征点的坐标，阈值，评分等
  Corners corners(grid_n_cols_*grid_n_rows_, Corner(0,0,detection_threshold,0,0.0f));

  for(int L=0; L<n_pyr_levels_; ++L)
  {
    const int scale = (1<<L);

	// 定义角点容器
    vector<fast::fast_xy> fast_corners;

	// 条件编译，好吧，，，这几个条件编译没看懂？？？
	// 这几个fast与FAST库有关，具体还没看
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

	// 定义得分以及角点数目变量
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
  // 将所有得分大于阈值的特征点传入fts中，作为返回值
  std::for_each(corners.begin(), corners.end(), [&](Corner& c) {
    if(c.score > detection_threshold)
      fts.push_back(new Feature(frame, Vector2d(c.x, c.y), c.level));
  });

  // 复位grid_occupancy_变量
  resetGrid();
}

} // namespace feature_detection
} // namespace svo

