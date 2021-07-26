
/*
** ��ͼ�����������
*/
#include <set>
#include <svo/map.h>
#include <svo/point.h>
#include <svo/frame.h>
#include <svo/feature.h>
#include <boost/bind.hpp>

namespace svo {

Map::Map() {}

Map::~Map()
{
  reset();
  SVO_INFO_STREAM("Map destructed");
}

void Map::reset()
{
  keyframes_.clear();
  point_candidates_.reset();
  emptyTrash();
}

/*
** parameter
* frame: ����֡
* ��reprojector.cpp�ļ��е�bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)�����е���
** function: 
*/
bool Map::safeDeleteFrame(FramePtr frame)
{
  bool found = false;
  // �����йؼ�֡����Ѱ��֡
  for(auto it=keyframes_.begin(), ite=keyframes_.end(); it!=ite; ++it)
  {
	  // ��������֡
    if(*it == frame)
    {
      std::for_each((*it)->fts_.begin(), (*it)->fts_.end(), [&](Feature* ftr){
        removePtFrameRef(it->get(), ftr);
      });
      keyframes_.erase(it);
      found = true;
      break;
    }
  }

  point_candidates_.removeFrameCandidates(frame);

  if(found)
    return true;

  SVO_ERROR_STREAM("Tried to delete Keyframe in map which was not there.");
  return false;
}

/*
** parameter
* frame: ����֡
* ftr: ����֡�е�������
* �ڱ��ļ��е�bool Map::safeDeleteFrame(FramePtr frame)�����е���
* ��bundle_adjustment.cpp�ļ��е�void localBA()�����е���
** function: ɾ�����֮֡���references����ϵ��������
*/
void Map::removePtFrameRef(Frame* frame, Feature* ftr)
{
  if(ftr->point == NULL)
    return; // mappoint may have been deleted in a previous ref. removal
  Point* pt = ftr->point;
  ftr->point = NULL;
  if(pt->obs_.size() <= 2)
  {
    // If the references list of mappoint has only size=2, delete mappoint
    safeDeletePoint(pt);
    return;
  }
  pt->deleteFrameRef(frame);  // Remove reference from map_point
  frame->removeKeyPoint(ftr); // Check if mp was keyMp in keyframe
}

/*
** parameter
* pt: ��ͼ��
* ��reprojector.cpp�ļ��е�bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)�����е���
** function: ɾ����ͼ�еĵ㲢ɾ���ؼ�֡�жԸõ������references
*/
void Map::safeDeletePoint(Point* pt)
{
  // Delete references to mappoints in all keyframes
  // ɾ�����йؼ�֡�жԵ�ͼ���references
  std::for_each(pt->obs_.begin(), pt->obs_.end(), [&](Feature* ftr){
    ftr->point=NULL;
    ftr->frame->removeKeyPoint(ftr);
  });
  pt->obs_.clear();

  // Delete mappoint
  // ɾ����ͼ��
  deletePoint(pt);
}

/*
** parameter
* pt: ��ɾ���ĵ�ͼ��
* �ڱ��ļ��е�void Map::safeDeletePoint(Point* pt)�����е���
** function: ɾ����ͼ��
*/
void Map::deletePoint(Point* pt)
{
	// �����ͼ�������������TYPE_DELETED���򽫸õ����trash_points_�����У��ñ���ר�����ڴ��Ҫɾ���ĵ�
  pt->type_ = Point::TYPE_DELETED;
  trash_points_.push_back(pt);
}

/*
** parameter
* new_keyframe������ֵ����⵽�Ĺؼ�֡
* ��frame_handler_mono.cpp�ļ��е�FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()�����е���
** function������⵽�Ĺؼ�֡����keyframes_��
*/
void Map::addKeyframe(FramePtr new_keyframe)
{
  keyframes_.push_back(new_keyframe);
}

/*
** parameters
* frame: �����֡
* close_kfs: ���������������֡���й��ӹ�ϵ�������ؼ�֡
* ��reprojector.cpp�ļ��е�void Reprojector::reprojectMap()�����е���
** function: ����һ��֡�����������������й��ӹ�ϵ�Ĺؼ�֡
*/
void Map::getCloseKeyframes(
    const FramePtr& frame,
    std::list< std::pair<FramePtr,double> >& close_kfs) const
{
  for(auto kf : keyframes_)
  {
    // check if kf has overlaping field of view with frame, use therefore KeyPoints
    // ���ؼ�֡��ο�֡�Ƿ����ص�����
    for(auto keypoint : kf->key_pts_)
    {
      if(keypoint == nullptr)
        continue;

	  // isVisible()���ڼ��ĳ3d���Ƿ�����ͼ���б��۲쵽
      if(frame->isVisible(keypoint->point->pos_))
      {
        close_kfs.push_back(
            std::make_pair(
                kf, (frame->T_f_w_.translation()-kf->T_f_w_.translation()).norm()));
        break; // this keyframe has an overlapping field of view -> add to close_kfs
      }
    }
  }
}

FramePtr Map::getClosestKeyframe(const FramePtr& frame) const
{
  list< pair<FramePtr,double> > close_kfs;
  getCloseKeyframes(frame, close_kfs);
  if(close_kfs.empty())
  {
    return nullptr;
  }


  // Sort KFs with overlap according to their closeness
  close_kfs.sort(boost::bind(&std::pair<FramePtr, double>::second, _1) <
                 boost::bind(&std::pair<FramePtr, double>::second, _2));

  if(close_kfs.front().first != frame)
    return close_kfs.front().first;
  close_kfs.pop_front();
  return close_kfs.front().first;
}

FramePtr Map::getFurthestKeyframe(const Vector3d& pos) const
{
  FramePtr furthest_kf;
  double maxdist = 0.0;
  for(auto it=keyframes_.begin(), ite=keyframes_.end(); it!=ite; ++it)
  {
    double dist = ((*it)->pos()-pos).norm();
    if(dist > maxdist) {
      maxdist = dist;
      furthest_kf = *it;
    }
  }
  return furthest_kf;
}

bool Map::getKeyframeById(const int id, FramePtr& frame) const
{
  bool found = false;
  for(auto it=keyframes_.begin(), ite=keyframes_.end(); it!=ite; ++it)
    if((*it)->id_ == id) {
      found = true;
      frame = *it;
      break;
    }
  return found;
}

void Map::transform(const Matrix3d& R, const Vector3d& t, const double& s)
{
  for(auto it=keyframes_.begin(), ite=keyframes_.end(); it!=ite; ++it)
  {
    Vector3d pos = s*R*(*it)->pos() + t;
    Matrix3d rot = R*(*it)->T_f_w_.rotation_matrix().inverse();
    (*it)->T_f_w_ = SE3(rot, pos).inverse();
    for(auto ftr=(*it)->fts_.begin(); ftr!=(*it)->fts_.end(); ++ftr)
    {
      if((*ftr)->point == NULL)
        continue;
      if((*ftr)->point->last_published_ts_ == -1000)
        continue;
      (*ftr)->point->last_published_ts_ = -1000;
      (*ftr)->point->pos_ = s*R*(*ftr)->point->pos_ + t;
    }
  }
}

void Map::emptyTrash()
{
  std::for_each(trash_points_.begin(), trash_points_.end(), [&](Point*& pt){
    delete pt;
    pt=NULL;
  });
  trash_points_.clear();
  point_candidates_.emptyTrash();
}

MapPointCandidates::MapPointCandidates()
{}

MapPointCandidates::~MapPointCandidates()
{
  reset();
}

void MapPointCandidates::newCandidatePoint(Point* point, double depth_sigma2)
{
  point->type_ = Point::TYPE_CANDIDATE;
  boost::unique_lock<boost::mutex> lock(mut_);
  candidates_.push_back(PointCandidate(point, point->obs_.front()));
}

/*
** parameter
* frame: ����ֵ����ǰ֡
* ��frame_handler_mono.cpp�ļ��е�FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()�����е���
** function: ����������ӵ�֡������������ɾ����ѡ����
*/
void MapPointCandidates::addCandidatePointToFrame(FramePtr frame)
{
	// �߳���
  boost::unique_lock<boost::mutex> lock(mut_);
  // ������ѡ��
  PointCandidateList::iterator it=candidates_.begin();
  while(it != candidates_.end())
  {
    if(it->first->obs_.front()->frame == frame.get())
    {
      // insert feature in the frame
      it->first->type_ = Point::TYPE_UNKNOWN;
      it->first->n_failed_reproj_ = 0;
	  // �����������������������
      it->second->frame->addFeature(it->second);
      it = candidates_.erase(it);
    }
    else
      ++it;
  }
}

/*
** parameter
* point: ��ͼ��
* ��reprojrctor.cpp�ļ��е�bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)�����е���
** function: ������ɾ����ѡ��
*/
bool MapPointCandidates::deleteCandidatePoint(Point* point)
{
	// �߳���
  boost::unique_lock<boost::mutex> lock(mut_);
  /*
  typedef pair<Point*, Feature*> PointCandidate;
  typedef list<PointCandidate> PointCandidateList;
  PointCandidateList candidates_;
  ����candidates_�����ϻ���һϵ��pair<Point*, Feature*>������
  */
  for(auto it=candidates_.begin(), ite=candidates_.end(); it!=ite; ++it)
  {
	  // �ж��Ƿ�ΪҪɾ���ĵ�
    if(it->first == point)
    {
      deleteCandidate(*it);// �������幦�ܲμ���������
      candidates_.erase(it);
      return true;// ����true��ʾ�Ѿ��ɹ�ɾ��
    }
  }
  return false;// ����false��ʾû��Ҫɾ���ĵ�ͼ��
}

void MapPointCandidates::removeFrameCandidates(FramePtr frame)
{
  boost::unique_lock<boost::mutex> lock(mut_);
  auto it=candidates_.begin();
  while(it!=candidates_.end())
  {
    if(it->second->frame == frame.get())
    {
      deleteCandidate(*it);
      it = candidates_.erase(it);
    }
    else
      ++it;
  }
}

void MapPointCandidates::reset()
{
  boost::unique_lock<boost::mutex> lock(mut_);
  std::for_each(candidates_.begin(), candidates_.end(), [&](PointCandidate& c){
    delete c.first;
    delete c.second;
  });
  candidates_.clear();
}

/*
** parameter
* c: ����ֵӦ���ǵ�ͼ�㼰���Ӧ��������
* ��reprojector.cpp�ļ��е�void Reprojector::reprojectMap()�����е���
* �ڱ��ļ��е�bool MapPointCandidates::deleteCandidatePoint(Point* point)�����е���
** function: �Ƴ���ɾ���ĺ�ѡ�㣬ע��c.first��c.second��ͬ����
*/
void MapPointCandidates::deleteCandidate(PointCandidate& c)
{
  // camera-rig: another frame might still be pointing to the candidate point
  // therefore, we can't delete it right now.
  /*
  c������������typedef pair<Point*, Feature*> PointCandidate;����Ӧ���ǵ�ͼ�㼰���Ӧ���������pair
  c.first��Point* �����ͼ�㣬c.second��Feature* ����������
  ��������֡�п����иõ�ͼ���ͶӰ�㣬���c.second�е�ֵ��������ɾ����ֻ�ܽ�c.second��ָ��ɾ����дΪNULL
  ע��delete c.second;�����delete������˵��ɾ������һ����˼
  */
  delete c.second; c.second=NULL;
  // ��ɾ���ĺ�ѡ��ı�־��ΪTYPE_DELETED
  c.first->type_ = Point::TYPE_DELETED;
  // Ȼ��Ѵ�ɾ���ĺ�ѡ���������Ͱ������������
  trash_points_.push_back(c.first);
}

void MapPointCandidates::emptyTrash()
{
  std::for_each(trash_points_.begin(), trash_points_.end(), [&](Point*& p){
    delete p; p=NULL;
  });
  trash_points_.clear();
}

namespace map_debug {

void mapValidation(Map* map, int id)
{
  for(auto it=map->keyframes_.begin(); it!=map->keyframes_.end(); ++it)
    frameValidation(it->get(), id);
}

void frameValidation(Frame* frame, int id)
{
  for(auto it = frame->fts_.begin(); it!=frame->fts_.end(); ++it)
  {
    if((*it)->point==NULL)
      continue;

    if((*it)->point->type_ == Point::TYPE_DELETED)
      printf("ERROR DataValidation %i: Referenced point was deleted.\n", id);

    if(!(*it)->point->findFrameRef(frame))
      printf("ERROR DataValidation %i: Frame has reference but point does not have a reference back.\n", id);

    pointValidation((*it)->point, id);
  }
  for(auto it=frame->key_pts_.begin(); it!=frame->key_pts_.end(); ++it)
    if(*it != NULL)
      if((*it)->point == NULL)
        printf("ERROR DataValidation %i: KeyPoints not correct!\n", id);
}

void pointValidation(Point* point, int id)
{
  for(auto it=point->obs_.begin(); it!=point->obs_.end(); ++it)
  {
    bool found=false;
    for(auto it_ftr=(*it)->frame->fts_.begin(); it_ftr!=(*it)->frame->fts_.end(); ++it_ftr)
     if((*it_ftr)->point == point) {
       found=true; break;
     }
    if(!found)
      printf("ERROR DataValidation %i: Point %i has inconsistent reference in frame %i, is candidate = %i\n", id, point->id_, (*it)->frame->id_, (int) point->type_);
  }
}

void mapStatistics(Map* map)
{
  // compute average number of features which each frame observes
  size_t n_pt_obs(0);
  for(auto it=map->keyframes_.begin(); it!=map->keyframes_.end(); ++it)
    n_pt_obs += (*it)->nObs();
  printf("\n\nMap Statistics: Frame avg. point obs = %f\n", (float) n_pt_obs/map->size());

  // compute average number of observations that each point has
  size_t n_frame_obs(0);
  size_t n_pts(0);
  std::set<Point*> points;
  for(auto it=map->keyframes_.begin(); it!=map->keyframes_.end(); ++it)
  {
    for(auto ftr=(*it)->fts_.begin(); ftr!=(*it)->fts_.end(); ++ftr)
    {
      if((*ftr)->point == NULL)
        continue;
      if(points.insert((*ftr)->point).second) {
        ++n_pts;
        n_frame_obs += (*ftr)->point->nRefs();
      }
    }
  }
  printf("Map Statistics: Point avg. frame obs = %f\n\n", (float) n_frame_obs/n_pts);
}

} // namespace map_debug
} // namespace svo
