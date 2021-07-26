
/*
 ** ��ͶӰ
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
    size_t max_n_kfs;   //!< max number of keyframes to reproject from�� ������ͶӰ�Ĺؼ�֡�������Ŀ
    bool find_match_direct; // ������
    Options()
    : max_n_kfs(10),
      find_match_direct(true)
    {}
  } options_;

  size_t n_matches_; // ƥ���
  size_t n_trials_; // ʲô��˼������

  Reprojector(vk::AbstractCamera* cam, Map& map);

  ~Reprojector();

  /// Project points from the map into the image. First finds keyframes with
  /// overlapping field of view and projects only those map-points.
  // �ӵ�ͼ��ͼ��ͶӰ�����Ȳ��Ҿ����ص���Ұ�Ĺؼ�֡������ͶӰ��Щ�ؼ�֡�ĵ�ͼ��
  void reprojectMap(
      FramePtr frame,
      std::vector< std::pair<FramePtr,std::size_t> >& overlap_kfs);

private:

  /// A candidate is a point that projects into the image plane and for which we will search a matching feature in the image.
	// ��ѡ�㣨Candidate����ͶӰ��ͼ��ƽ���ϵĵ㣬���ǽ�����ͼ���ж�Ӧ��ƥ��������
  struct Candidate {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Point* pt;       //!< 3D point. �ռ��
    Vector2d px;     //!< projected 2D pixel location. ͶӰ������
    Candidate(Point* pt, Vector2d& px) : pt(pt), px(px) {}
  };
  typedef std::list<Candidate > Cell; // Cell������������һϵ�к�ѡ�������
  typedef std::vector<Cell*> CandidateGrid;

  /// The grid stores a set of candidate matches. For every grid cell we try to find one match.
  // ����洢һ���ѡƥ�������ÿ������Ԫ��������ͼ�ҵ�һ��ƥ���
  struct Grid
  {
    CandidateGrid cells;// ��ǰͼ�������
    vector<int> cell_order; // ��������
    int cell_size;// ��������
    int grid_n_cols;// ��
    int grid_n_rows;// ��
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
