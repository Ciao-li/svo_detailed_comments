
/*
 ** ��ͶӰ
*/
#include <algorithm>
#include <stdexcept>
#include <svo/reprojector.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/map.h>
#include <svo/config.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <vikit/abstract_camera.h>
#include <vikit/math_utils.h>
#include <vikit/timer.h>

namespace svo {

/*
* ���캯��
* ��frame_handler_mono.pp�ļ��е�FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam)�����е���
*/
Reprojector::Reprojector(vk::AbstractCamera* cam, Map& map) :
    map_(map)
{
  initializeGrid(cam);
}

Reprojector::~Reprojector()
{
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ delete c; });
}

/*
* �ڱ��ļ��е�Reprojector::Reprojector()���캯���е���
** function: ��Ҫ�����ͶӰ�ĳ�ʼ������
*/
void Reprojector::initializeGrid(vk::AbstractCamera* cam)
{
  grid_.cell_size = Config::gridSize();// ��ȡ��������
  grid_.grid_n_cols = ceil(static_cast<double>(cam->width())/grid_.cell_size);// ceil()����ȡ������ȡ��ǰͼ���ж���������
  grid_.grid_n_rows = ceil(static_cast<double>(cam->height())/grid_.cell_size);// ceil()����ȡ������ȡ��ǰͼ���ж���������
  grid_.cells.resize(grid_.grid_n_cols*grid_.grid_n_rows);// ������������
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell*& c){ c = new Cell; });// Ϊÿһ��Cell�����ڴ�
  grid_.cell_order.resize(grid_.cells.size());// �����������
  for(size_t i=0; i<grid_.cells.size(); ++i)
    grid_.cell_order[i] = i; // ������������
  random_shuffle(grid_.cell_order.begin(), grid_.cell_order.end()); // maybe we should do it at every iteration! ������������
}

/*
* �ڱ��ļ��е�void Reprojector::reprojectMap()�����е���
** function: �������
*/
void Reprojector::resetGrid()
{
  n_matches_ = 0;
  n_trials_ = 0;
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ c->clear(); });
}

/*
** parameters
* frame: �����ͼ��֡
* overlap_kfs: �����������ο�֡�����ص���Ұ�������ؼ�֡
* ��frame_handler.mono.cpp�ļ��е�FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()�����е���
** function: �ӵ�ͼ��ͼ��ͶӰ�����Ȳ��Ҿ����ص���Ұ�Ĺؼ�֡��Ȼ��ѡ����������ǰʮλ�Ĺؼ�֡������ͶӰ��Щ�ؼ�֡�ĵ�ͼ�㣻�����ÿ��cell��ֻ����һ����ͶӰ��
*/
void Reprojector::reprojectMap(
    FramePtr frame,
    std::vector< std::pair<FramePtr,std::size_t> >& overlap_kfs)
{
  // ���������Ϣ
  resetGrid();

  // Identify those Keyframes which share a common field of view.
  // ʶ����й��ӹ�ϵ�Ĺؼ�֡
  SVO_START_TIMER("reproject_kfs");
  list< pair<FramePtr,double> > close_kfs;
  map_.getCloseKeyframes(frame, close_kfs);

  // Sort KFs with overlap according to their closeness
  // ���ݾ���Զ�����Ծ��й��ӹ�ϵ�Ĺؼ�֡�������򣬽�����ǰ��Զ���ں�
  close_kfs.sort(boost::bind(&std::pair<FramePtr, double>::second, _1) < boost::bind(&std::pair<FramePtr, double>::second, _2));

  // Reproject all mappoints of the closest N kfs with overlap. We only store
  // in which grid cell the points fall.
  // ��ͶӰ����֡�����еĵ�ͼ�㣬ֻ�����е㽵�������
  size_t n = 0;
  // options_��Options�ṹ��Ķ������ÿ�����ͶӰ�������Ŀ�Ĺؼ�֡
  overlap_kfs.reserve(options_.max_n_kfs);

  for(auto it_frame=close_kfs.begin(), ite_frame=close_kfs.end();
      it_frame!=ite_frame && n<options_.max_n_kfs; ++it_frame, ++n)
  {
	  // ����֮ǰ��������������ǰʮ���ؼ�֡��overlap_kfs�С�ʵ�����Ǳ���������֡�Ͻ���ǰʮ��֡
    FramePtr ref_frame = it_frame->first;
    overlap_kfs.push_back(pair<FramePtr,size_t>(ref_frame,0));

    // Try to reproject each mappoint that the other KF observes
	// ѡ��ʮ֡�󣬽���ʮ֡�еĵ�ͼ����ͶӰ������֡
    for(auto it_ftr=ref_frame->fts_.begin(), ite_ftr=ref_frame->fts_.end();
        it_ftr!=ite_ftr; ++it_ftr)
    {
      // check if the feature has a mappoint assigned
		// ����ͼ���Ƿ�Ϊ��
      if((*it_ftr)->point == NULL)
        continue;

      // make sure we project a point only once
	  // ȷ��ÿ����ֻ����ͶӰһ��
      if((*it_ftr)->point->last_projected_kf_id_ == frame->id_)
        continue;
      (*it_ftr)->point->last_projected_kf_id_ = frame->id_;
	  // ���ú�������ͶӰ������ͶӰ�ĵ����overlap_kfs��
      if(reprojectPoint(frame, (*it_ftr)->point))
        overlap_kfs.back().second++;
    }
  }
  SVO_STOP_TIMER("reproject_kfs");

  // Now project all point candidates
  SVO_START_TIMER("reproject_candidates");
  {
	  // �߳���
    boost::unique_lock<boost::mutex> lock(map_.point_candidates_.mut_);
	// ɾ����ѡ�㣿��������map.hע�ͣ������ѡ��Ӧ����ָδ�������ؼ�֡��ͬ�۲⵽�ĵ�ͼ��
    auto it=map_.point_candidates_.candidates_.begin();// ������ѡ��
    while(it!=map_.point_candidates_.candidates_.end())
    {
      if(!reprojectPoint(frame, it->first))
      {
        it->first->n_failed_reproj_ += 3;
        if(it->first->n_failed_reproj_ > 30)
        {
          map_.point_candidates_.deleteCandidate(*it);
          it = map_.point_candidates_.candidates_.erase(it);
          continue;
        }
      }
      ++it;
    }
  } // unlock the mutex when out of scope
  SVO_STOP_TIMER("reproject_candidates");

  // Now we go through each grid cell and select one point to match.
  // At the end, we should have at maximum one reprojected point per cell.
  // �������Ǳ���ÿ������Ԫ��ѡ��һ��Ҫƥ��ĵ㣬���ÿ������Ԫ���ֻ����һ����ͶӰ��
  // �ҵ��˹��ӵĹؼ������һ��������֮֡�������Щ���ӵ��ƥ���ˣ�˵��ͶӰ��Ӧ��׼ȷһЩ�ɣ�
  SVO_START_TIMER("feature_align");
  // ����������������
  for(size_t i=0; i<grid_.cells.size(); ++i)
  {
    // we prefer good quality points over unkown quality (more likely to match)
    // and unknown quality over candidates (position not optimized)
	// ���Ǹ�ϲ���������㣬�����δ֪�����ĵ㣨��Ϊ��������ƥ�䣩������Ǻ�ѡ�㣨��Ϊ��λ��δ�Ż���
    if(reprojectCell(*grid_.cells.at(grid_.cell_order[i]), frame))
      ++n_matches_;
	// ��ƥ��������������ֵ��������ͶӰ
    if(n_matches_ > (size_t) Config::maxFts())
      break;
  }
  SVO_STOP_TIMER("feature_align");
}

/*
** parameters
* lhs: 
* rhs: 
* �ڱ��ļ��е�bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)�����е���
** function: ���۵��������type_����������PointType,�ο�point.h
*/
bool Reprojector::pointQualityComparator(Candidate& lhs, Candidate& rhs)
{
	// enumö�����Ϳ��ԱȽϴ�Сô������
  if(lhs.pt->type_ > rhs.pt->type_)
    return true;
  return false;
}

/*
** parameters
* cell: ͼ������
* frame: ����֡
* ���ļ�reprojector.cpp�ļ��е�void Reprojector::reprojectMap()�����е���
** function: ����ÿ������Ԫ��ѡ��һ��Ҫƥ��ĵ㣬���ÿ������Ԫ���ֻ����һ����ͶӰ��
*/
bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)
{
	// �Ƚϵ��������ת���ļ�
  cell.sort(boost::bind(&Reprojector::pointQualityComparator, _1, _2));

  // ѭ������ÿ��cell
  Cell::iterator it=cell.begin();
  while(it!=cell.end())
  {
    ++n_trials_;

	// ����������������DELETED����ɾ���õ���������
    if(it->pt->type_ == Point::TYPE_DELETED)
    {
      it = cell.erase(it);
      continue;
    }

    bool found_match = true;
	// ���options_��reprojector.h�ļ��е�Options����
	// ͨ��ֱ��Ӧ�����ؾ���Ѱ��ƥ���
    if(options_.find_match_direct)
      found_match = matcher_.findMatchDirect(*it->pt, *frame, it->px);

    if(!found_match)
    {
      it->pt->n_failed_reproj_++;
	  // ������������δ֪�Ĳ�����ͶӰʧ�ܵĴ�������15�Σ���ɾ����
      if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_failed_reproj_ > 15)
        map_.safeDeletePoint(it->pt);
	  // �����������Ǻ�ѡ�Ĳ�����ͶӰʧ�ܵĴ�������30�Σ���ɾ����
      if(it->pt->type_ == Point::TYPE_CANDIDATE  && it->pt->n_failed_reproj_ > 30)
        map_.point_candidates_.deleteCandidatePoint(it->pt);
      it = cell.erase(it);
      continue;
    }

    it->pt->n_succeeded_reproj_++;
	// ������������TYPE_UNKNOWN������ͶӰʧ�ܵĴ�������10�Σ��򽫸õ��������¶���ΪTYPE_GOOD
    if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_succeeded_reproj_ > 10)
      it->pt->type_ = Point::TYPE_GOOD;

	// ���캯����ʼ��������һ��Feature���ָ�����new_feature
    Feature* new_feature = new Feature(frame.get(), it->px, matcher_.search_level_);
	// ��new_feature����frame��
    frame->addFeature(new_feature);

    // Here we add a reference in the feature to the 3D point, the other way
    // round is only done if this frame is selected as keyframe.
	// ��������ǽ������е�һ��������ӵ�3D�㣬��һ�ַ����ǣ�������֡��ѡΪ�ؼ�֡ʱ���Ž���round
    new_feature->point = it->pt;

    if(matcher_.ref_ftr_->type == Feature::EDGELET)
    {
      new_feature->type = Feature::EDGELET;
      new_feature->grad = matcher_.A_cur_ref_*matcher_.ref_ftr_->grad;
      new_feature->grad.normalize();// �ݶȹ�һ��
    }

    // If the keyframe is selected and we reproject the rest, we don't have to
    // check this point anymore.
	// ���ѡ���˹ؼ�֡������ͶӰ����Ĺؼ�֡���򲻱��ټ��õ�
    it = cell.erase(it);

    // Maximum one point per cell.
	// ����true����ʾ���ҵ�ÿ��cell����õ�һ��point
    return true;
  }
  return false;// ����false����ʾ����cell�������
}

/*
** parameters
* frame: ����֡����������Χѡȡ��ʮ��֡
* point: ���й��ӹ�ϵ�ĵ�ͼ��
* �ڱ��ļ��е�void Reprojector::reprojectMap()�����е���
** function�����ڽ�����֡���ӵĵ�ͼ����ͶӰ������֡��
*/
bool Reprojector::reprojectPoint(FramePtr frame, Point* point)
{
	// �Ȱѵ����������ϵת���������ϵ
  Vector2d px(frame->w2c(point->pos_));
  // ���ж�ɶ�������Ѳ�����ָ��8*8��patch��ĵ��Ǻ�ѡ��ô������
  if(frame->cam_->isInFrame(px.cast<int>(), 8)) // 8px is the patch size in the matcher
  {
    const int k = static_cast<int>(px[1]/grid_.cell_size)*grid_.grid_n_cols + static_cast<int>(px[0]/grid_.cell_size);
    grid_.cells.at(k)->push_back(Candidate(point, px));
    return true;
  }
  // ��patch֮�⣿����
  return false;
}
} // namespace svo
