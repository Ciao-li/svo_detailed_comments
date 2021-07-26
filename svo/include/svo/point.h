
/*
** 3D��Ķ���
*/
#ifndef SVO_POINT_H_
#define SVO_POINT_H_

#include <boost/noncopyable.hpp>
#include <svo/global.h>

namespace g2o {
  class VertexSBAPointXYZ;
}
typedef g2o::VertexSBAPointXYZ g2oPoint;

namespace svo {

class Feature;

typedef Matrix<double, 2, 3> Matrix23d;

/// A 3D point on the surface of the scene.
// 3D�����������
class Point : boost::noncopyable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // �������
  enum PointType {
    TYPE_DELETED, // ɾ����
    TYPE_CANDIDATE, // ��ѡ��
    TYPE_UNKNOWN, // δ֪
    TYPE_GOOD // �õĵ�
  };

  static int                  point_counter_;           //!< Counts the number of created points. Used to set the unique id.
  int                         id_;                      //!< Unique ID of the point.���ID
  Vector3d                    pos_;                     //!< 3d pos of the point in the world coordinate frame.��������ϵ�µ�3D������
  Vector3d                    normal_;                  //!< Surface normal at point.
  Matrix3d                    normal_information_;      //!< Inverse covariance matrix of normal estimation.ʲô���Ƶ���Э�������
  bool                        normal_set_;              //!< Flag whether the surface normal was estimated or not.
  list<Feature*>              obs_;                     //!< References to keyframes which observe the point.���Թ۲쵽ĳ�������֡������
														// ��Ӧ���������������ô
  size_t                      n_obs_;                   //!< Number of obervations: Keyframes AND successful reprojections in intermediate frames.
														// �۲쵽�������������
  g2oPoint*                   v_pt_;                    //!< Temporary pointer to the point-vertex in g2o during bundle adjustment.
														// ��BA�����ڴ洢3D���g2o������Ϣ����ʱָ��
  int                         last_published_ts_;       //!< Timestamp of last publishing.
  int                         last_projected_kf_id_;    //!< Flag for the reprojection: don't reproject a pt twice. ��ͶӰ��־����������ͶӰ���μ�����
  PointType                   type_;                    //!< Quality of the point. �������
  int                         n_failed_reproj_;         //!< Number of failed reprojections. Used to assess the quality of the point.��ͶӰʧ�ܵĴ��������������������
  int                         n_succeeded_reproj_;      //!< Number of succeeded reprojections. Used to assess the quality of the point.��ͶӰ�ɹ��Ĵ��������������������
  int                         last_structure_optim_;    //!< Timestamp of last point optimization

  Point(const Vector3d& pos);
  Point(const Vector3d& pos, Feature* ftr);
  ~Point();

  /// Add a reference to a frame.
  // ��һ֡���������
  void addFrameRef(Feature* ftr);

  /// Remove reference to a frame.
  bool deleteFrameRef(Frame* frame);

  /// Initialize point normal. The inital estimate will point towards the frame.
  void initNormal();

  /// Check whether mappoint has reference to a frame.
  Feature* findFrameRef(Frame* frame);

  /// Get Frame with similar viewpoint.
  // ��þ������ƹ۲���֡
  bool getCloseViewObs(const Vector3d& pos, Feature*& obs) const;

  /// Get number of observations.
  inline size_t nRefs() const { return obs_.size(); }

  /// Optimize point position through minimizing the reprojection error.
  // ͨ����С����ͶӰ������Ż�3D���λ����Ϣ
  void optimize(const size_t n_iter);

  /// Jacobian of point projection on unit plane (focal length = 1) in frame (f).
  // �ڵ�ǰ֡�еĹ�һ��ƽ���ϵ�ͶӰ����ſɱȾ���
  inline static void jacobian_xyz2uv(
      const Vector3d& p_in_f,
      const Matrix3d& R_f_w,
      Matrix23d& point_jac)
  {
    const double z_inv = 1.0/p_in_f[2];
    const double z_inv_sq = z_inv*z_inv;
    point_jac(0, 0) = z_inv;
    point_jac(0, 1) = 0.0;
    point_jac(0, 2) = -p_in_f[0] * z_inv_sq;
    point_jac(1, 0) = 0.0;
    point_jac(1, 1) = z_inv;
    point_jac(1, 2) = -p_in_f[1] * z_inv_sq;
    point_jac = - point_jac * R_f_w;
  }
};

} // namespace svo

#endif // SVO_POINT_H_
