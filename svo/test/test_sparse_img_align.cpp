// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <vikit/pinhole_camera.h>
#include <vikit/math_utils.h>
#include <vikit/file_reader.h>
#include <vikit/timer.h>
#include <vikit/blender_utils.h>
#include <iostream>
#include <svo/feature_detection.h>
#include <svo/sparse_img_align.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/config.h>

namespace {

/// SparseImgAlign Test-Fixture
class SparseImgAlignTest {
 public:

  SparseImgAlignTest();
  virtual ~SparseImgAlignTest();
  void testSequence(
      const std::string& dataset_dir,
      const std::string& experiment_name,
      svo::feature_detection::AbstractDetector* feature_detector);

  vk::PinholeCamera* cam_;
  svo::FramePtr frame_ref_;
  svo::FramePtr frame_cur_;
};

SparseImgAlignTest::SparseImgAlignTest() :
    cam_(new vk::PinholeCamera(752, 480, 217.083701215, 217.083701215, 376, 240))
{}

SparseImgAlignTest::~SparseImgAlignTest()
{}

void SparseImgAlignTest::testSequence(
    const std::string& dataset_dir,
    const std::string& experiment_name,
    svo::feature_detection::AbstractDetector* feature_detector)
{
  vk::FileReader<vk::blender_utils::file_format::ImageNameAndPose> sequence_file_reader(dataset_dir+"/trajectory.txt");
  std::vector<vk::blender_utils::file_format::ImageNameAndPose> sequence;
  sequence_file_reader.skipComments();
  if(!sequence_file_reader.next())
    std::runtime_error("Failed to open sequence file");
  sequence_file_reader.readAllEntries(sequence);
  printf("RUN EXPERIMENT: read %zu dataset entries.\n", sequence.size());
  std::vector<vk::blender_utils::file_format::ImageNameAndPose>::iterator iter = sequence.begin();
  std::list<double> translation_error;

  Sophus::SE3 T_prev_w, T_prevgt_w;
  std::string trace_name(std::string(TEST_TRACE_DIR) + "/sparse_img_align_" + experiment_name + "_trans_estimate.txt");
  std::ofstream ofs(trace_name.c_str());
  for(int i=0; iter != sequence.end() && i<30; ++iter, ++i)
  {
    // load img
    // IMPORTANT: We need to flip the image because the Povray dataset has a
    // negative focal length in y direction which we didn't set.
    std::string img_name(dataset_dir+"/img/" + iter->image_name_ + "_0.png");
    cv::Mat img(cv::imread(img_name, 0));
    assert(!img.empty());

    // load pose
    Sophus::SE3 T_w_gt = Sophus::SE3(iter->q_.toRotationMatrix(), iter->t_);
    Sophus::SE3 T_gt_w = T_w_gt.inverse(); // ground-truth

    if(i==0)
    {
      // set reference frame
      frame_ref_.reset(new svo::Frame(cam_, img));
      frame_ref_->T_f_w_ = T_gt_w;

      // load ground-truth depth
      cv::Mat depthmap;
      vk::blender_utils::loadBlenderDepthmap(
          dataset_dir+"/depth/"+iter->image_name_+"_0.depth", *cam_, depthmap);

      // extract features, generate features with 3D points
      svo::feature_detection::Corners corners;
      feature_detector->detect(
          frame_ref_->img_pyr_, frame_ref_->fts_, svo::Config::gridSize(),
          svo::Config::nPyrLevels(), svo::Config::triangMinCornerScore(), &corners);
      for(svo::feature_detection::Corners::iterator corner=corners.begin(); corner!=corners.end(); ++corner)
      {
        svo::Feature* ftr = new svo::Feature(frame_ref_.get(), Eigen::Vector2d(corner->x, corner->y), corner->level);
        Eigen::Vector3d pt_pos_cur = ftr->f*depthmap.at<float>(corner->y, corner->x);
        Eigen::Vector3d pt_pos_w = frame_ref_->T_f_w_.inverse()*pt_pos_cur;
        svo::Point* pt = new svo::Point(pt_pos_w, svo::Point::DEPTH);
        ftr->point = pt;
        frame_ref_->addFeature(ftr);
      }

      printf("Added %zu 3d pts to the reference frame.\n", corners.size());
      T_prev_w = frame_ref_->T_f_w_;
      T_prevgt_w = T_gt_w;
      continue;
    }

    frame_cur_.reset(new svo::Frame(cam_, img));
    //frame_cur_->T_f_w_ = frame_ref_->T_f_w_; // start at reference frame
    frame_cur_->T_f_w_ = T_prev_w; // start at last frame

    // run image align
    vk::Timer t;
    svo::SparseImgAlign img_align(svo::Config::kltMaxLevel(), svo::Config::kltMinLevel(),
                                    30, svo::SparseImgAlign::GaussNewton, false, false);
    img_align.run(frame_ref_, frame_cur_);

    // compute error
    Sophus::SE3 T_f_gt = frame_cur_->T_f_w_ * T_gt_w.inverse();
    translation_error.push_back(T_f_gt.translation().norm());
    printf("[%3.i] time = %f ms \t |t| = %f \t translation error = %f\n",
           i, t.stop()*1000, (frame_ref_->T_f_w_*T_gt_w.inverse()).translation().norm(),
           translation_error.back());

    // save old pose for next iteration
    T_prev_w = frame_cur_->T_f_w_;
    T_prevgt_w = T_gt_w;

    ofs << frame_cur_->T_f_w_.inverse().translation().transpose() << " "
        << T_gt_w.inverse().translation().transpose() << std::endl;
  }
  ofs.close();

  // trace error
  trace_name = std::string(TEST_TRACE_DIR) + "/sparse_img_align_" + experiment_name + "_trans_error.txt";
  ofs.open(trace_name.c_str());
  for(std::list<double>::iterator it=translation_error.begin(); it!=translation_error.end(); ++it)
    ofs << *it << std::endl;
  ofs.close();
}

}  // namespace


int main(int argc, char** argv)
{
  std::string experiment_name("flying_room_1_rig_1_fast_minlev0");
  std::string dataset_dir(std::string(TEST_DATA_DIR) + "/flying_room_1_rig_1");
  svo::feature_detection::FastDetector detector;
  svo::Config::triangMinCornerScore() = 20;
  svo::Config::kltMinLevel() = 0;
  SparseImgAlignTest test;
  test.testSequence(dataset_dir, experiment_name, &detector);
  return 0;
}