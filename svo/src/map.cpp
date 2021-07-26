
/*
** 地图的生成与管理
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
* frame: 输入帧
* 在reprojector.cpp文件中的bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)函数中调用
** function: 
*/
bool Map::safeDeleteFrame(FramePtr frame)
{
  bool found = false;
  // 在所有关键帧中搜寻该帧
  for(auto it=keyframes_.begin(), ite=keyframes_.end(); it!=ite; ++it)
  {
	  // 若是输入帧
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
* frame: 输入帧
* ftr: 输入帧中的特征点
* 在本文件中的bool Map::safeDeleteFrame(FramePtr frame)函数中调用
* 在bundle_adjustment.cpp文件中的void localBA()函数中调用
** function: 删除点和帧之间的references（联系？？？）
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
* pt: 地图点
* 在reprojector.cpp文件中的bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)函数中调用
** function: 删除地图中的点并删除关键帧中对该点的所有references
*/
void Map::safeDeletePoint(Point* pt)
{
  // Delete references to mappoints in all keyframes
  // 删除所有关键帧中对地图点的references
  std::for_each(pt->obs_.begin(), pt->obs_.end(), [&](Feature* ftr){
    ftr->point=NULL;
    ftr->frame->removeKeyPoint(ftr);
  });
  pt->obs_.clear();

  // Delete mappoint
  // 删除地图点
  deletePoint(pt);
}

/*
** parameter
* pt: 待删除的地图点
* 在本文件中的void Map::safeDeletePoint(Point* pt)函数中调用
** function: 删除地图点
*/
void Map::deletePoint(Point* pt)
{
	// 如果地图点的数据类型是TYPE_DELETED，则将该点存入trash_points_变量中，该变量专门用于存放要删除的点
  pt->type_ = Point::TYPE_DELETED;
  trash_points_.push_back(pt);
}

/*
** parameter
* new_keyframe：输入值，检测到的关键帧
* 在frame_handler_mono.cpp文件中的FrameHandlerMono::UpdateResult FrameHandlerMono::processFirstFrame()函数中调用
** function：将检测到的关键帧存入keyframes_中
*/
void Map::addKeyframe(FramePtr new_keyframe)
{
  keyframes_.push_back(new_keyframe);
}

/*
** parameters
* frame: 输入的帧
* close_kfs: 输出参数，与输入帧具有共视关系的其他关键帧
* 在reprojector.cpp文件中的void Reprojector::reprojectMap()函数中调用
** function: 给定一个帧，返回所有与它具有共视关系的关键帧
*/
void Map::getCloseKeyframes(
    const FramePtr& frame,
    std::list< std::pair<FramePtr,double> >& close_kfs) const
{
  for(auto kf : keyframes_)
  {
    // check if kf has overlaping field of view with frame, use therefore KeyPoints
    // 检查关键帧与参考帧是否有重叠区域
    for(auto keypoint : kf->key_pts_)
    {
      if(keypoint == nullptr)
        continue;

	  // isVisible()用于检查某3d点是否能在图像中被观察到
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
* frame: 输入值，当前帧
* 在frame_handler_mono.cpp文件中的FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()函数中调用
** function: 将特征点添加到帧，并从链表中删除候选特征
*/
void MapPointCandidates::addCandidatePointToFrame(FramePtr frame)
{
	// 线程锁
  boost::unique_lock<boost::mutex> lock(mut_);
  // 遍历候选点
  PointCandidateList::iterator it=candidates_.begin();
  while(it != candidates_.end())
  {
    if(it->first->obs_.front()->frame == frame.get())
    {
      // insert feature in the frame
      it->first->type_ = Point::TYPE_UNKNOWN;
      it->first->n_failed_reproj_ = 0;
	  // 添加特征点至特征点链表中
      it->second->frame->addFeature(it->second);
      it = candidates_.erase(it);
    }
    else
      ++it;
  }
}

/*
** parameter
* point: 地图点
* 在reprojrctor.cpp文件中的bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)函数中调用
** function: 遍历，删除候选点
*/
bool MapPointCandidates::deleteCandidatePoint(Point* point)
{
	// 线程锁
  boost::unique_lock<boost::mutex> lock(mut_);
  /*
  typedef pair<Point*, Feature*> PointCandidate;
  typedef list<PointCandidate> PointCandidateList;
  PointCandidateList candidates_;
  所以candidates_本质上还是一系列pair<Point*, Feature*>的链表
  */
  for(auto it=candidates_.begin(), ite=candidates_.end(); it!=ite; ++it)
  {
	  // 判断是否为要删除的点
    if(it->first == point)
    {
      deleteCandidate(*it);// 函数具体功能参见函数定义
      candidates_.erase(it);
      return true;// 返回true表示已经成功删除
    }
  }
  return false;// 返回false表示没有要删除的地图点
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
* c: 输入值应该是地图点及其对应的特征点
* 在reprojector.cpp文件中的void Reprojector::reprojectMap()函数中调用
* 在本文件中的bool MapPointCandidates::deleteCandidatePoint(Point* point)函数中调用
** function: 移除待删除的候选点，注意c.first和c.second不同操作
*/
void MapPointCandidates::deleteCandidate(PointCandidate& c)
{
  // camera-rig: another frame might still be pointing to the candidate point
  // therefore, we can't delete it right now.
  /*
  c的数据类型是typedef pair<Point*, Feature*> PointCandidate;所以应该是地图点及其对应的特征点的pair
  c.first是Point* 代表地图点，c.second是Feature* 代表特征点
  由于其他帧中可能有该地图点的投影点，因此c.second中的值不能立刻删除，只能将c.second的指向删除并写为NULL
  注意delete c.second;这里的delete和上面说的删除不是一个意思
  */
  delete c.second; c.second=NULL;
  // 将删除的候选点的标志置为TYPE_DELETED
  c.first->type_ = Point::TYPE_DELETED;
  // 然后把待删除的候选点放入垃圾桶，辣鸡！！！
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
