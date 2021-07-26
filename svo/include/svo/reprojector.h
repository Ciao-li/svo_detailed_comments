
/*
 ** 重投影
*/
#ifndef SVO_REPROJECTION_H_
#define SVO_REPROJECTION_H_

#include <svo/global.h>
#include <svo/matcher.h>

namespace vk {
class AbstractCamera;
}

namespace svo {

class Map;
class Point;

/// Project points from the map into the image and find the corresponding
/// feature (corner). We don't search a match for every point but only for one
/// point per cell. Thereby, we achieve a homogeneously distributed set of
/// matched features and at the same time we can save processing time by not
/// projecting all points.
class Reprojector
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Reprojector config parameters
  struct Options {
    size_t max_n_kfs;   //!< max number of keyframes to reproject from， 用于重投影的关键帧的最大数目
    bool find_match_direct; // ？？？
    Options()
    : max_n_kfs(10),
      find_match_direct(true)
    {}
  } options_;

  size_t n_matches_; // 匹配点
  size_t n_trials_; // 什么意思？？？

  Reprojector(vk::AbstractCamera* cam, Map& map);

  ~Reprojector();

  /// Project points from the map into the image. First finds keyframes with
  /// overlapping field of view and projects only those map-points.
  // 从地图向图像投影，首先查找具有重叠视野的关键帧，并仅投影这些关键帧的地图点
  void reprojectMap(
      FramePtr frame,
      std::vector< std::pair<FramePtr,std::size_t> >& overlap_kfs);

private:

  /// A candidate is a point that projects into the image plane and for which we will search a matching feature in the image.
	// 候选点（Candidate）是投影到图像平面上的点，我们将搜索图像中对应的匹配特征。
  struct Candidate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Point* pt;       //!< 3D point. 空间点
    Vector2d px;     //!< projected 2D pixel location. 投影点坐标
    Candidate(Point* pt, Vector2d& px) : pt(pt), px(px) {}
  };
  typedef std::list<Candidate > Cell; // Cell的数据类型是一系列候选点的链表
  typedef std::vector<Cell*> CandidateGrid;

  /// The grid stores a set of candidate matches. For every grid cell we try to find one match.
  // 网格存储一组候选匹配项。对于每个网格单元，我们试图找到一个匹配项。
  struct Grid
  {
    CandidateGrid cells;// 当前图像的网格
    vector<int> cell_order; // 网格索引
    int cell_size;// 网格数量
    int grid_n_cols;// 列
    int grid_n_rows;// 行
  };

  Grid grid_;
  Matcher matcher_;
  Map& map_;

  static bool pointQualityComparator(Candidate& lhs, Candidate& rhs);
  void initializeGrid(vk::AbstractCamera* cam);
  void resetGrid();
  bool reprojectCell(Cell& cell, FramePtr frame);
  bool reprojectPoint(FramePtr frame, Point* point);
};

} // namespace svo

#endif // SVO_REPROJECTION_H_
