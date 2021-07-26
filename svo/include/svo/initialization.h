
/*
 ** 初始化
*/
#ifndef SVO_INITIALIZATION_H
#define SVO_INITIALIZATION_H

#include <svo/global.h>

namespace svo {

class FrameHandlerMono;

/// Bootstrapping the map from the first two views.
namespace initialization {

// 初始化的结果，错误，无关键帧，成功
enum InitResult { FAILURE, NO_KEYFRAME, SUCCESS };

/// Tracks features using Lucas-Kanade tracker and then estimates a homography.
// 使用Lucas-Kanade跟踪特征点，然后计算单应性矩阵估计相对变换
class KltHomographyInit {
  friend class svo::FrameHandlerMono;// 友元类
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FramePtr frame_ref_;
  KltHomographyInit() {};
  ~KltHomographyInit() {};
  InitResult addFirstFrame(FramePtr frame_ref);
  InitResult addSecondFrame(FramePtr frame_ref);
  void reset();

protected:
  vector<cv::Point2f> px_ref_;      //!< keypoints to be tracked in reference frame. 参考帧中用于跟踪的关键点
  vector<cv::Point2f> px_cur_;      //!< tracked keypoints in current frame. 当前帧中被跟踪到的关键点，应该有好几种状态，LK之前的关键点、LK之后的关键点、剔除没有被跟踪到的关键点后
  vector<Vector3d> f_ref_;          //!< bearing vectors corresponding to the keypoints in the reference image. 参考帧中关键点的方向向量？？？
  vector<Vector3d> f_cur_;          //!< bearing vectors corresponding to the keypoints in the current image. 当前帧中关键点的方向向量？？？
  vector<double> disparities_;      //!< disparity between first and second frame. 第一帧与第二帧的差异，两帧之间像素移动的距离
  vector<int> inliers_;             //!< inliers after the geometric check (e.g., Homography). 几何检查（单应矩阵）后正确点的数量
  vector<Vector3d> xyz_in_cur_;     //!< 3D points computed during the geometric check.几何检查过程中的地图点
  SE3 T_cur_from_ref_;              //!< computed transformation between the first two frames.由前两帧计算的变换矩阵
};

/// Detect Fast corners in the image.
// 检测图像中的FAST角点
void detectFeatures(
    FramePtr frame,
    vector<cv::Point2f>& px_vec,
    vector<Vector3d>& f_vec);

/// Compute optical flow (Lucas Kanade) for selected keypoints.
// 对选择的关键点计算光流
void trackKlt(
    FramePtr frame_ref,
    FramePtr frame_cur,
    vector<cv::Point2f>& px_ref,
    vector<cv::Point2f>& px_cur,
    vector<Vector3d>& f_ref,
    vector<Vector3d>& f_cur,
    vector<double>& disparities);

void computeHomography(
    const vector<Vector3d>& f_ref,
    const vector<Vector3d>& f_cur,
    double focal_length,
    double reprojection_threshold,
    vector<int>& inliers,
    vector<Vector3d>& xyz_in_cur,
    SE3& T_cur_from_ref);

} // namespace initialization
} // namespace svo

#endif // SVO_INITIALIZATION_H
