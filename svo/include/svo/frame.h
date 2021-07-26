
/*
** frame����
*/
#ifndef SVO_FRAME_H_
#define SVO_FRAME_H_

#include <sophus/se3.h>
#include <vikit/math_utils.h>
#include <vikit/abstract_camera.h>
#include <boost/noncopyable.hpp>
#include <svo/global.h>

namespace g2o {
class VertexSE3Expmap;
}
typedef g2o::VertexSE3Expmap g2oFrameSE3;

namespace svo {

class Point;
struct Feature;

typedef list<Feature*> Features; // Feature���ָ������
typedef vector<cv::Mat> ImgPyr;// ʲô��˼������

/// A frame saves the image, the associated features and the estimated pose.
class Frame : boost::noncopyable
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
  static int                    frame_counter_;         //!< Counts the number of created frames. Used to set the unique id.
  int                           id_;                    //!< Unique id of the frame.
  double                        timestamp_;             //!< Timestamp of when the image was recorded.
  vk::AbstractCamera*           cam_;                   //!< Camera model.���ģ�ͣ�����
  Sophus::SE3                   T_f_w_;                 //!< Transform (f)rame from (w)orld.����������ϵ��ͼ��֡����ϵ�µı任
  Matrix<double, 6, 6>          Cov_;                   //!< Covariance. ����
  ImgPyr                        img_pyr_;               //!< Image Pyramid. ͼ�������
  Features                      fts_;                   //!< List of features in the image.����������
  vector<Feature*>              key_pts_;               //!< Five features and associated 3D points which are used to detect if two frames have overlapping field of view.
														// �������ά����ص������㣬��ά�����ڼ������֡�Ƿ����ص�����Ұ
  bool                          is_keyframe_;           //!< Was this frames selected as keyframe?
  g2oFrameSE3*                  v_kf_;                  //!< Temporary pointer to the g2o node object of the keyframe.
														// �ؼ�֡��g2o�ڵ�������ʱָ��
  int                           last_published_ts_;     //!< Timestamp of last publishing.

  Frame(vk::AbstractCamera* cam, const cv::Mat& img, double timestamp);
  ~Frame();

  /*
  ** ������inline���ε�����������ֻ��ͷ�ļ��ж��� **
  */
  /// Initialize new frame and create image pyramid.
  void initFrame(const cv::Mat& img);

  /// Select this frame as keyframe.
  // ѡ��ǰ֡Ϊ�ؼ�֡
  void setKeyframe();

  /// Add a feature to the image
  // ��ͼ�����������
  void addFeature(Feature* ftr);

  /// The KeyPoints are those five features which are closest to the 4 image corners
  /// and to the center and which have a 3D point assigned. These points are used
  /// to quickly check whether two frames have overlapping field of view.
  /*
  �ؼ��������4��ͼ��ǵ�����Ĳ�ָ����һ����ά��������������Щ�����ڿ��ټ����֡����Ұ�Ƿ��ص���
  */
  void setKeyPoints();

  /// Check if we can select five better key-points.
  // ��������Ƿ���ѡ��������õĹؼ��㡣
  void checkKeyPoints(Feature* ftr);

  /// If a point is deleted, we must remove the corresponding key-point.
  // ���һ����ͼ�㱻ɾ���ˣ���Ҫɾ����Ӧ�Ĺؼ���
  void removeKeyPoint(Feature* ftr);

  /// Return number of point observations.
  inline size_t nObs() const { return fts_.size(); }

  /// Check if a point in (w)orld coordinate frame is visible in the image.
  // ���ĳ3d���Ƿ�����ͼ���б��۲쵽
  bool isVisible(const Vector3d& xyz_w) const;

  /// Full resolution image stored in the frame.
  inline const cv::Mat& img() const { return img_pyr_[0]; }

  /// Was this frame selected as keyframe ?
  // ȷ����ǰ֡�Ƿ�Ϊ�ؼ�֡
  inline bool isKeyframe() const { return is_keyframe_; }

  /// Transforms point coordinates in world-frame (w) to camera pixel coordinates (c).
  inline Vector2d w2c(const Vector3d& xyz_w) const { return cam_->world2cam( T_f_w_ * xyz_w ); }

  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
  inline Vector3d c2f(const Vector2d& px) const { return cam_->cam2world(px[0], px[1]); }

  /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
  // ����������任Ϊ�������꣬cam2world()������vikit����
  inline Vector3d c2f(const double x, const double y) const { return cam_->cam2world(x, y); }

  /// Transforms point coordinates in world-frame (w) to camera-frams (f).
  // ����������ϵ�µĵ�תΪ�������ϵ��
  inline Vector3d w2f(const Vector3d& xyz_w) const { return T_f_w_ * xyz_w; }

  /// Transforms point from frame unit sphere (f) frame to world coordinate frame (w).
  inline Vector3d f2w(const Vector3d& f) const { return T_f_w_.inverse() * f; }

  /// Projects Point from unit sphere (f) in camera pixels (c).
  // ��������أ�c��Ϊ��λ�ӵ�λ���壨f��ͶӰ��
  inline Vector2d f2c(const Vector3d& f) const { return cam_->world2cam( f ); }

  /// Return the pose of the frame in the (w)orld coordinate frame.
  // ����֡����������ϵ�µ����꣬�����Ƿ���T_f_w_��������ƽ�Ʋ���t
  inline Vector3d pos() const { return T_f_w_.inverse().translation(); }

  /// Frame jacobian for projection of 3D point in (f)rame coordinate to
  /// unit plane coordinates uv (focal length = 1).
  // ���ڽ���ǰ֡����ϵ�е�3D��ͶӰ��ͼ������ϵ�У�����uv��������Ϊ1�Ĺ�һ��ƽ�棩��֡�ſɱȾ���
  inline static void jacobian_xyz2uv(
      const Vector3d& xyz_in_f,
      Matrix<double,2,6>& J)
  {
    const double x = xyz_in_f[0];
    const double y = xyz_in_f[1];
    const double z_inv = 1./xyz_in_f[2];
    const double z_inv_2 = z_inv*z_inv;

    J(0,0) = -z_inv;              // -1/z
    J(0,1) = 0.0;                 // 0
    J(0,2) = x*z_inv_2;           // x/z^2
    J(0,3) = y*J(0,2);            // x*y/z^2
    J(0,4) = -(1.0 + x*J(0,2));   // -(1.0 + x^2/z^2)
    J(0,5) = y*z_inv;             // y/z

    J(1,0) = 0.0;                 // 0
    J(1,1) = -z_inv;              // -1/z
    J(1,2) = y*z_inv_2;           // y/z^2
    J(1,3) = 1.0 + y*J(1,2);      // 1.0 + y^2/z^2
    J(1,4) = -J(0,3);             // -x*y/z^2
    J(1,5) = -x*z_inv;            // x/z
  }
};


/// Some helper functions for the frame object.
namespace frame_utils {

/// Creates an image pyramid of half-sampled images.
void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr);

/// Get the average depth of the features in the image.
// �������������ͼ���ƽ����Ⱥ���С���
bool getSceneDepth(const Frame& frame, double& depth_mean, double& depth_min);

} // namespace frame_utils
} // namespace svo

#endif // SVO_FRAME_H_
