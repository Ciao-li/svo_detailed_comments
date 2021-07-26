
/*
 ** 重投影
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
* 构造函数
* 在frame_handler_mono.pp文件中的FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam)函数中调用
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
* 在本文件中的Reprojector::Reprojector()构造函数中调用
** function: 主要完成重投影的初始化工作
*/
void Reprojector::initializeGrid(vk::AbstractCamera* cam)
{
  grid_.cell_size = Config::gridSize();// 获取网格数量
  grid_.grid_n_cols = ceil(static_cast<double>(cam->width())/grid_.cell_size);// ceil()向上取整。获取当前图像有多少网格列
  grid_.grid_n_rows = ceil(static_cast<double>(cam->height())/grid_.cell_size);// ceil()向上取整。获取当前图像有多少网格行
  grid_.cells.resize(grid_.grid_n_cols*grid_.grid_n_rows);// 重置网格数量
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell*& c){ c = new Cell; });// 为每一个Cell申请内存
  grid_.cell_order.resize(grid_.cells.size());// 获得网格索引
  for(size_t i=0; i<grid_.cells.size(); ++i)
    grid_.cell_order[i] = i; // 分配网格索引
  random_shuffle(grid_.cell_order.begin(), grid_.cell_order.end()); // maybe we should do it at every iteration! 打乱网格索引
}

/*
* 在本文件中的void Reprojector::reprojectMap()函数中调用
** function: 清除网格
*/
void Reprojector::resetGrid()
{
  n_matches_ = 0;
  n_trials_ = 0;
  std::for_each(grid_.cells.begin(), grid_.cells.end(), [&](Cell* c){ c->clear(); });
}

/*
** parameters
* frame: 输入的图像帧
* overlap_kfs: 输出参数，与参考帧具有重叠视野的其他关键帧
* 在frame_handler.mono.cpp文件中的FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()函数中调用
** function: 从地图向图像投影，首先查找具有重叠视野的关键帧；然后选出距离排在前十位的关键帧，并仅投影这些关键帧的地图点；最后在每个cell中只保留一个重投影点
*/
void Reprojector::reprojectMap(
    FramePtr frame,
    std::vector< std::pair<FramePtr,std::size_t> >& overlap_kfs)
{
  // 清空网格信息
  resetGrid();

  // Identify those Keyframes which share a common field of view.
  // 识别具有共视关系的关键帧
  SVO_START_TIMER("reproject_kfs");
  list< pair<FramePtr,double> > close_kfs;
  map_.getCloseKeyframes(frame, close_kfs);

  // Sort KFs with overlap according to their closeness
  // 根据距离远近，对具有共视关系的关键帧进行排序，近的在前，远的在后
  close_kfs.sort(boost::bind(&std::pair<FramePtr, double>::second, _1) < boost::bind(&std::pair<FramePtr, double>::second, _2));

  // Reproject all mappoints of the closest N kfs with overlap. We only store
  // in which grid cell the points fall.
  // 重投影共视帧中所有的地图点，只保存有点降落的网格
  size_t n = 0;
  // options_是Options结构体的对象，设置可用重投影的最大数目的关键帧
  overlap_kfs.reserve(options_.max_n_kfs);

  for(auto it_frame=close_kfs.begin(), ite_frame=close_kfs.end();
      it_frame!=ite_frame && n<options_.max_n_kfs; ++it_frame, ++n)
  {
	  // 根据之前的排序结果，插入前十个关键帧至overlap_kfs中。实际上是保存与输入帧较近的前十个帧
    FramePtr ref_frame = it_frame->first;
    overlap_kfs.push_back(pair<FramePtr,size_t>(ref_frame,0));

    // Try to reproject each mappoint that the other KF observes
	// 选择十帧后，将这十帧中的地图点重投影至输入帧
    for(auto it_ftr=ref_frame->fts_.begin(), ite_ftr=ref_frame->fts_.end();
        it_ftr!=ite_ftr; ++it_ftr)
    {
      // check if the feature has a mappoint assigned
		// 检查地图点是否为空
      if((*it_ftr)->point == NULL)
        continue;

      // make sure we project a point only once
	  // 确保每个点只能重投影一次
      if((*it_ftr)->point->last_projected_kf_id_ == frame->id_)
        continue;
      (*it_ftr)->point->last_projected_kf_id_ = frame->id_;
	  // 调用函数，重投影，将重投影的点存入overlap_kfs中
      if(reprojectPoint(frame, (*it_ftr)->point))
        overlap_kfs.back().second++;
    }
  }
  SVO_STOP_TIMER("reproject_kfs");

  // Now project all point candidates
  SVO_START_TIMER("reproject_candidates");
  {
	  // 线程锁
    boost::unique_lock<boost::mutex> lock(map_.point_candidates_.mut_);
	// 删除候选点？？？根据map.h注释，这个候选点应该是指未被两个关键帧共同观测到的地图点
    auto it=map_.point_candidates_.candidates_.begin();// 遍历候选点
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
  // 现在我们遍历每个网格单元并选择一个要匹配的点，最后，每个网格单元最多只能有一个重投影点
  // 找到了共视的关键点后，下一步就是两帧之间关于这些共视点的匹配了（说重投影点应该准确一些吧）
  SVO_START_TIMER("feature_align");
  // 在网格区域内搜索
  for(size_t i=0; i<grid_.cells.size(); ++i)
  {
    // we prefer good quality points over unkown quality (more likely to match)
    // and unknown quality over candidates (position not optimized)
	// 我们更喜欢高质量点，其次是未知质量的点（因为它更容易匹配），最后是候选点（因为其位置未优化）
    if(reprojectCell(*grid_.cells.at(grid_.cell_order[i]), frame))
      ++n_matches_;
	// 若匹配点大于最大跟踪阈值，结束重投影
    if(n_matches_ > (size_t) Config::maxFts())
      break;
  }
  SVO_STOP_TIMER("feature_align");
}

/*
** parameters
* lhs: 
* rhs: 
* 在本文件中的bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)函数中调用
** function: 评价点的质量，type_数据类型是PointType,参考point.h
*/
bool Reprojector::pointQualityComparator(Candidate& lhs, Candidate& rhs)
{
	// enum枚举类型可以比较大小么？？？
  if(lhs.pt->type_ > rhs.pt->type_)
    return true;
  return false;
}

/*
** parameters
* cell: 图像网格
* frame: 输入帧
* 在文件reprojector.cpp文件中的void Reprojector::reprojectMap()函数中调用
** function: 遍历每个网格单元并选择一个要匹配的点，最后，每个网格单元最多只能有一个重投影点
*/
bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)
{
	// 比较点的质量，转本文件
  cell.sort(boost::bind(&Reprojector::pointQualityComparator, _1, _2));

  // 循环遍历每个cell
  Cell::iterator it=cell.begin();
  while(it!=cell.end())
  {
    ++n_trials_;

	// 如果点的数据类型是DELETED，则删除该点所在网格
    if(it->pt->type_ == Point::TYPE_DELETED)
    {
      it = cell.erase(it);
      continue;
    }

    bool found_match = true;
	// 这个options_是reprojector.h文件中的Options对象
	// 通过直接应用像素精炼寻找匹配点
    if(options_.find_match_direct)
      found_match = matcher_.findMatchDirect(*it->pt, *frame, it->px);

    if(!found_match)
    {
      it->pt->n_failed_reproj_++;
	  // 如果点的类型是未知的并且重投影失败的次数大于15次，则删除点
      if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_failed_reproj_ > 15)
        map_.safeDeletePoint(it->pt);
	  // 如果点的类型是候选的并且重投影失败的次数大于30次，则删除点
      if(it->pt->type_ == Point::TYPE_CANDIDATE  && it->pt->n_failed_reproj_ > 30)
        map_.point_candidates_.deleteCandidatePoint(it->pt);
      it = cell.erase(it);
      continue;
    }

    it->pt->n_succeeded_reproj_++;
	// 如果点的类型是TYPE_UNKNOWN并且重投影失败的次数大于10次，则将该点类型重新定义为TYPE_GOOD
    if(it->pt->type_ == Point::TYPE_UNKNOWN && it->pt->n_succeeded_reproj_ > 10)
      it->pt->type_ = Point::TYPE_GOOD;

	// 构造函数初始化，创建一个Feature类的指针对象new_feature
    Feature* new_feature = new Feature(frame.get(), it->px, matcher_.search_level_);
	// 将new_feature存入frame中
    frame->addFeature(new_feature);

    // Here we add a reference in the feature to the 3D point, the other way
    // round is only done if this frame is selected as keyframe.
	// 在这里，我们将特征中的一个引用添加到3D点，另一种方法是，仅当该帧被选为关键帧时，才进行round
    new_feature->point = it->pt;

    if(matcher_.ref_ftr_->type == Feature::EDGELET)
    {
      new_feature->type = Feature::EDGELET;
      new_feature->grad = matcher_.A_cur_ref_*matcher_.ref_ftr_->grad;
      new_feature->grad.normalize();// 梯度归一化
    }

    // If the keyframe is selected and we reproject the rest, we don't have to
    // check this point anymore.
	// 如果选择了关键帧并重新投影其余的关键帧，则不必再检查该点
    it = cell.erase(it);

    // Maximum one point per cell.
	// 返回true，表示已找到每个cell中最好的一个point
    return true;
  }
  return false;// 返回false，表示所有cell遍历完成
}

/*
** parameters
* frame: 输入帧，在它的周围选取了十个帧
* point: 具有共视关系的地图点
* 在本文件中的void Reprojector::reprojectMap()函数中调用
** function：用于将其他帧共视的地图点重投影至输入帧中
*/
bool Reprojector::reprojectPoint(FramePtr frame, Point* point)
{
	// 先把点从世界坐标系转到相机坐标系
  Vector2d px(frame->w2c(point->pos_));
  // 先判断啥？？？难不成是指在8*8的patch里的点是候选点么？？？
  if(frame->cam_->isInFrame(px.cast<int>(), 8)) // 8px is the patch size in the matcher
  {
    const int k = static_cast<int>(px[1]/grid_.cell_size)*grid_.grid_n_cols + static_cast<int>(px[0]/grid_.cell_size);
    grid_.cells.at(k)->push_back(Candidate(point, px));
    return true;
  }
  // 在patch之外？？？
  return false;
}
} // namespace svo
