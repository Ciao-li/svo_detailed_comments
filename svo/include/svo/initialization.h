
/*
 ** ��ʼ��
*/
#ifndef SVO_INITIALIZATION_H
#define SVO_INITIALIZATION_H

#include <svo/global.h>

namespace svo {

class FrameHandlerMono;

/// Bootstrapping the map from the first two views.
namespace initialization {

// ��ʼ���Ľ���������޹ؼ�֡���ɹ�
enum InitResult { FAILURE, NO_KEYFRAME, SUCCESS };

/// Tracks features using Lucas-Kanade tracker and then estimates a homography.
// ʹ��Lucas-Kanade���������㣬Ȼ����㵥Ӧ�Ծ��������Ա任
class KltHomographyInit {
  friend class svo::FrameHandlerMono;// ��Ԫ��
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FramePtr frame_ref_;
  KltHomographyInit() {};
  ~KltHomographyInit() {};
  InitResult addFirstFrame(FramePtr frame_ref);
  InitResult addSecondFrame(FramePtr frame_ref);
  void reset();

protected:
  vector<cv::Point2f> px_ref_;      //!< keypoints to be tracked in reference frame. �ο�֡�����ڸ��ٵĹؼ���
  vector<cv::Point2f> px_cur_;      //!< tracked keypoints in current frame. ��ǰ֡�б����ٵ��Ĺؼ��㣬Ӧ���кü���״̬��LK֮ǰ�Ĺؼ��㡢LK֮��Ĺؼ��㡢�޳�û�б����ٵ��Ĺؼ����
  vector<Vector3d> f_ref_;          //!< bearing vectors corresponding to the keypoints in the reference image. �ο�֡�йؼ���ķ�������������
  vector<Vector3d> f_cur_;          //!< bearing vectors corresponding to the keypoints in the current image. ��ǰ֡�йؼ���ķ�������������
  vector<double> disparities_;      //!< disparity between first and second frame. ��һ֡��ڶ�֡�Ĳ��죬��֮֡�������ƶ��ľ���
  vector<int> inliers_;             //!< inliers after the geometric check (e.g., Homography). ���μ�飨��Ӧ���󣩺���ȷ�������
  vector<Vector3d> xyz_in_cur_;     //!< 3D points computed during the geometric check.���μ������еĵ�ͼ��
  SE3 T_cur_from_ref_;              //!< computed transformation between the first two frames.��ǰ��֡����ı任����
};

/// Detect Fast corners in the image.
// ���ͼ���е�FAST�ǵ�
void detectFeatures(
    FramePtr frame,
    vector<cv::Point2f>& px_vec,
    vector<Vector3d>& f_vec);

/// Compute optical flow (Lucas Kanade) for selected keypoints.
// ��ѡ��Ĺؼ���������
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
