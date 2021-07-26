
/*
 ** 光束法平差（图优化）
*/

#ifndef SVO_BUNDLE_ADJUSTMENT_H_
#define SVO_BUNDLE_ADJUSTMENT_H_

#include <svo/global.h>

namespace g2o {
class EdgeProjectXYZ2UV;
class SparseOptimizer;
class VertexSE3Expmap;
class VertexSBAPointXYZ;
}

namespace svo {

typedef g2o::EdgeProjectXYZ2UV g2oEdgeSE3;
typedef g2o::VertexSE3Expmap g2oFrameSE3;
typedef g2o::VertexSBAPointXYZ g2oPoint;

class Frame;
class Point;
class Feature;
class Map;

/// Local, global and 2-view bundle adjustment with g2o
// 局部，全局，两视图BA
namespace ba {

/// Temporary container to hold the g2o edge with reference to frame and point.
// 临时容器，存储帧和点有关系的g2o边
struct EdgeContainerSE3
{
  g2oEdgeSE3*     edge;
  Frame*          frame;
  Feature*        feature;
  bool            is_deleted;
  EdgeContainerSE3(g2oEdgeSE3* e, Frame* frame, Feature* feature) :
    edge(e), frame(frame), feature(feature), is_deleted(false)
  {}
};

/// Optimize two camera frames and their observed 3D points.
/// Is used after initialization.
// 两视图的BA,优化两帧和他们所观察到的3D点，在初始化头两帧后使用
void twoViewBA(Frame* frame1, Frame* frame2, double reproj_thresh, Map* map);

/// Local bundle adjustment.
/// Optimizes core_kfs and their observed map points while keeping the
/// neighbourhood fixed.
// 局部BA，优化滑动窗口内的关键帧以及观察到的3D点
void localBA(
    Frame* center_kf,
    set<FramePtr>* core_kfs,
    Map* map,
    size_t& n_incorrect_edges_1,
    size_t& n_incorrect_edges_2,
    double& init_error,
    double& final_error);

/// Global bundle adjustment.
/// Optimizes the whole map. Is currently not used in SVO.
void globalBA(Map* map);

/// Initialize g2o with solver type, optimization strategy and camera model.
// 使用求解器类型、优化策略和相机模型初始化g2o
void setupG2o(g2o::SparseOptimizer * optimizer);

/// Run the optimization on the provided graph.
// 在建立好的图中进行优化
void runSparseBAOptimizer(
    g2o::SparseOptimizer* optimizer,
    unsigned int num_iter,
    double& init_error,
    double& final_error);

/// Create a g2o vertice from a keyframe object.
// 从一个关键帧对象中创建一个g2o顶点
g2oFrameSE3* createG2oFrameSE3(
    Frame* kf,
    size_t id,
    bool fixed);

/// Creates a g2o vertice from a mappoint object.
// 从一个地图点对象中创建一个g2o顶点
g2oPoint* createG2oPoint(
    Vector3d pos,
    size_t id,
    bool fixed);

/// Creates a g2o edge between a g2o keyframe and mappoint vertice with the provided measurement.
// 在g2o关键帧和3D点之间创建一条边
g2oEdgeSE3* createG2oEdgeSE3(
    g2oFrameSE3* v_kf,
    g2oPoint* v_mp,
    const Vector2d& f_up,
    bool robust_kernel,
    double huber_width,
    double weight = 1);

} // namespace ba
} // namespace svo

#endif // SVO_BUNDLE_ADJUSTMENT_H_
