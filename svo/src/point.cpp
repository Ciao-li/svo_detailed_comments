
/*
** 3D点的定义
*/
#include <stdexcept>
#include <vikit/math_utils.h>
#include <svo/point.h>
#include <svo/frame.h>
#include <svo/feature.h>
 
namespace svo {

int Point::point_counter_ = 0;

/*
* 在initilization.cpp文件中的InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)函数中初始化
*/
Point::Point(const Vector3d& pos) :
  id_(point_counter_++),
  pos_(pos),
  normal_set_(false),
  n_obs_(0),
  v_pt_(NULL),
  last_published_ts_(0),
  last_projected_kf_id_(-1),
  type_(TYPE_UNKNOWN),
  n_failed_reproj_(0),
  n_succeeded_reproj_(0),
  last_structure_optim_(0)
{}

Point::Point(const Vector3d& pos, Feature* ftr) :
  id_(point_counter_++),
  pos_(pos),
  normal_set_(false),
  n_obs_(1),
  v_pt_(NULL),
  last_published_ts_(0),
  last_projected_kf_id_(-1),
  type_(TYPE_UNKNOWN),
  n_failed_reproj_(0),
  n_succeeded_reproj_(0),
  last_structure_optim_(0)
{
  obs_.push_front(ftr);
}

Point::~Point()
{}

/*
** parameter
* ftr: 特征点
* 在initilization.cpp文件中的InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)函数中调用
* 在frame_handler_mono.cpp文件中的FrameHandlerBase::UpdateResult FrameHandlerMono::processFrame()函数中调用
** function: 向帧中添加特征点
*/
void Point::addFrameRef(Feature* ftr)
{
  // obs_:可以观察到此特征点的帧的链表
  obs_.push_front(ftr);
  // 观测点数++
  ++n_obs_;
}

Feature* Point::findFrameRef(Frame* frame)
{
  for(auto it=obs_.begin(), ite=obs_.end(); it!=ite; ++it)
    if((*it)->frame == frame)
      return *it;
  return NULL;    // no keyframe found
}

bool Point::deleteFrameRef(Frame* frame)
{
  for(auto it=obs_.begin(), ite=obs_.end(); it!=ite; ++it)
  {
    if((*it)->frame == frame)
    {
      obs_.erase(it);
      return true;
    }
  }
  return false;
}

void Point::initNormal()
{
  assert(!obs_.empty());
  const Feature* ftr = obs_.back();
  assert(ftr->frame != NULL);
  normal_ = ftr->frame->T_f_w_.rotation_matrix().transpose()*(-ftr->f);
  normal_information_ = DiagonalMatrix<double,3,3>(pow(20/(pos_-ftr->frame->pos()).norm(),2), 1.0, 1.0);
  normal_set_ = true;
}

/*
** parameters
* framepos: 帧在世界坐标系下的坐标
* ftr: 特征点
* 在matcher.cpp文件中的bool Matcher::findMatchDirect()函数中调用
** function: 筛选出在每个特征点60度观察范围内的特征点？？？
*/
bool Point::getCloseViewObs(const Vector3d& framepos, Feature*& ftr) const
{
  // TODO: get frame with same point of view AND same pyramid level!
  Vector3d obs_dir(framepos - pos_); obs_dir.normalize();
  // obs_数据类型是list<Feature*>
  auto min_it=obs_.begin();
  double min_cos_angle = 0;
  for(auto it=obs_.begin(), ite=obs_.end(); it!=ite; ++it)
  {
    // T_f_w_的逆矩阵的平移参数t - 世界坐标系下的3D点坐标
    Vector3d dir((*it)->frame->pos() - pos_); dir.normalize();
    double cos_angle = obs_dir.dot(dir);
    if(cos_angle > min_cos_angle)
    {
      min_cos_angle = cos_angle;
      min_it = it;
    }
  }
  ftr = *min_it;
  // 假定大于60度的观察角度所观测的点是无用的点
  if(min_cos_angle < 0.5) // assume that observations larger than 60° are useless
    return false;
  return true;
}

/*
** parameter
* n_iter: 迭代次数
* 在frame_handler_base.cpp文件中的void FrameHandlerBase::optimizeStructure()函数中调用
** function: 通过最小化重投影误差来优化3D点的位置信息
*/
void Point::optimize(const size_t n_iter)
{
  Vector3d old_point = pos_;
  double chi2 = 0.0;
  Matrix3d A;
  Vector3d b;

  for(size_t i=0; i<n_iter; i++)
  {
    A.setZero();
    b.setZero();
    double new_chi2 = 0.0;

    // compute residuals
    for(auto it=obs_.begin(); it!=obs_.end(); ++it)
    {
      Matrix23d J;
      const Vector3d p_in_f((*it)->frame->T_f_w_ * pos_);
      Point::jacobian_xyz2uv(p_in_f, (*it)->frame->T_f_w_.rotation_matrix(), J);
      const Vector2d e(vk::project2d((*it)->f) - vk::project2d(p_in_f));
      new_chi2 += e.squaredNorm();
      A.noalias() += J.transpose() * J;
      b.noalias() -= J.transpose() * e;
    }

    // solve linear system
    const Vector3d dp(A.ldlt().solve(b));

    // check if error increased
    if((i > 0 && new_chi2 > chi2) || (bool) std::isnan((double)dp[0]))
    {
#ifdef POINT_OPTIMIZER_DEBUG
      cout << "it " << i
           << "\t FAILURE \t new_chi2 = " << new_chi2 << endl;
#endif
      pos_ = old_point; // roll-back
      break;
    }

    // update the model
    Vector3d new_point = pos_ + dp;
    old_point = pos_;
    pos_ = new_point;
    chi2 = new_chi2;
#ifdef POINT_OPTIMIZER_DEBUG
    cout << "it " << i
         << "\t Success \t new_chi2 = " << new_chi2
         << "\t norm(b) = " << vk::norm_max(b)
         << endl;
#endif

    // stop when converged
    if(vk::norm_max(dp) <= EPS)
      break;
  }
#ifdef POINT_OPTIMIZER_DEBUG
  cout << endl;
#endif
}

} // namespace svo
