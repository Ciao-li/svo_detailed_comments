
/*
** 特征定义
*/
#ifndef SVO_FEATURE_H_
#define SVO_FEATURE_H_

#include <svo/frame.h>

namespace svo {

/// A salient image region that is tracked across frames.
// 用于帧间跟踪的特征点
struct Feature
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 特征点的类型（应该是指分布吧）
  enum FeatureType {
    CORNER, // 角
    EDGELET  // 边
  };

  FeatureType type;     //!< Type can be corner or edgelet. 表示该点的类型，在角上还是边上
  Frame* frame;         //!< Pointer to frame in which the feature was detected. 指向检测到特征点的帧的指针
  Vector2d px;          //!< Coordinates in pixels on pyramid level 0. 在金字塔第0层的像素坐标
  Vector3d f;           //!< Unit-bearing vector of the feature. 特征点的单位方向向量
  int level;            //!< Image pyramid level where feature was extracted. 特征点所在的金字塔层数
  Point* point;         //!< Pointer to 3D point which corresponds to the feature. 指向与特征点关联的地图点的指针
  Vector2d grad;        //!< Dominant gradient direction for edglets, normalized. 边的主要梯度方向，已归一化

  /*
  * 在reprojector.cpp文件中的bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)函数中初始化
  * 函数重载，构造函数的初始化操作
  */
  Feature(Frame* _frame, const Vector2d& _px, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(frame->cam_->cam2world(px)),
    level(_level),
    point(NULL),
    grad(1.0,0.0)
  {}

  Feature(Frame* _frame, const Vector2d& _px, const Vector3d& _f, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(_f),
    level(_level),
    point(NULL),
    grad(1.0,0.0)
  {}

  /*
  * 在initilization.cpp文件中的InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)函数中初始化
  * 函数重载，构造函数的初始化操作
  */
  Feature(Frame* _frame, Point* _point, const Vector2d& _px, const Vector3d& _f, int _level) :
    type(CORNER),
    frame(_frame),
    px(_px),
    f(_f),
    level(_level),
    point(_point),
    grad(1.0,0.0)
  {}
};

} // namespace svo

#endif // SVO_FEATURE_H_
