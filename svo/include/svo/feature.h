
/*
** ��������
*/
#ifndef SVO_FEATURE_H_
#define SVO_FEATURE_H_

#include <svo/frame.h>

namespace svo {

/// A salient image region that is tracked across frames.
// ����֡����ٵ�������
struct Feature
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // ����������ͣ�Ӧ����ָ�ֲ��ɣ�
  enum FeatureType {
    CORNER, // ��
    EDGELET  // ��
  };

  FeatureType type;     //!< Type can be corner or edgelet. ��ʾ�õ�����ͣ��ڽ��ϻ��Ǳ���
  Frame* frame;         //!< Pointer to frame in which the feature was detected. ָ���⵽�������֡��ָ��
  Vector2d px;          //!< Coordinates in pixels on pyramid level 0. �ڽ�������0�����������
  Vector3d f;           //!< Unit-bearing vector of the feature. ������ĵ�λ��������
  int level;            //!< Image pyramid level where feature was extracted. ���������ڵĽ���������
  Point* point;         //!< Pointer to 3D point which corresponds to the feature. ָ��������������ĵ�ͼ���ָ��
  Vector2d grad;        //!< Dominant gradient direction for edglets, normalized. �ߵ���Ҫ�ݶȷ����ѹ�һ��

  /*
  * ��reprojector.cpp�ļ��е�bool Reprojector::reprojectCell(Cell& cell, FramePtr frame)�����г�ʼ��
  * �������أ����캯���ĳ�ʼ������
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
  * ��initilization.cpp�ļ��е�InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)�����г�ʼ��
  * �������أ����캯���ĳ�ʼ������
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
