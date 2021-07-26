
/*
** VO主节点，主函数
** 参考链接：https://www.cnblogs.com/hxzkh/p/8607714.html 、
*/
#include <ros/package.h>
#include <string>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/config.h>
#include <svo_ros/visualizer.h>
#include <vikit/params_helper.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <vikit/abstract_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/user_input_thread.h>

namespace svo {

/// SVO Interface
class VoNode
{
public:
  svo::FrameHandlerMono* vo_;
  svo::Visualizer visualizer_;
  bool publish_markers_;                 //!< publish only the minimal amount of info (choice for embedded devices)
  bool publish_dense_input_;
  boost::shared_ptr<vk::UserInputThread> user_input_thread_;
  ros::Subscriber sub_remote_key_;
  std::string remote_input_;
  vk::AbstractCamera* cam_;	//要用到vikit库中的AbstractCamera类
  bool quit_;
  VoNode();
  ~VoNode();
  void imgCb(const sensor_msgs::ImageConstPtr& msg);
  void processUserActions();
  void remoteKeyCb(const std_msgs::StringConstPtr& key_input);
};

/*
* 构造函数
*/
VoNode::VoNode() :
  vo_(NULL),
  publish_markers_(vk::getParam<bool>("svo/publish_markers", true)),
  publish_dense_input_(vk::getParam<bool>("svo/publish_dense_input", false)),
  remote_input_(""),
  cam_(NULL),
  quit_(false)
{
  // Start user input thread in parallel thread that listens to console keys
  if(vk::getParam<bool>("svo/accept_console_user_input", true))
    user_input_thread_ = boost::make_shared<vk::UserInputThread>();//开起线程用于监听控制台输入，boost库

  // Create Camera
  // 加载摄像机参数，vikit库
  if(!vk::camera_loader::loadFromRosNs("svo", cam_))
    throw std::runtime_error("Camera model not correctly specified.");

  // Get initial position and orientation
  /* 
	初始化位姿，SE3(R,t)，R:旋转矩阵，t:平移向量。Sophus库
	vk::rpy2dcm(const Vector3d &rpy) 可将欧拉角 rpy 转换为旋转矩阵
	vikit库、Eigen库
  */
  visualizer_.T_world_from_vision_ = Sophus::SE3(
      vk::rpy2dcm(Vector3d(vk::getParam<double>("svo/init_rx", 0.0),
                           vk::getParam<double>("svo/init_ry", 0.0),
                           vk::getParam<double>("svo/init_rz", 0.0))),
      Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
                      vk::getParam<double>("svo/init_ty", 0.0),
                      vk::getParam<double>("svo/init_tz", 0.0)));

  // Init VO and start
  // 初始化视觉前端VO
  // 调用frame_handler_mono.cpp文件中的FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam)函数
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();// 转frame_handler_base.h文件 void start() { set_start_ = true; }
}

/*
* 析构函数
*/
VoNode::~VoNode()
{
  delete vo_;
  delete cam_;
  if(user_input_thread_ != NULL)
    user_input_thread_->stop();
}

/*
* 在本文件中的主函数中调用
*/
void VoNode::imgCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img;
  try {
    img = cv_bridge::toCvShare(msg, "mono8")->image;// 读取图像并将ROS数据转变为OpenCV中的图像数据
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  processUserActions();// 开辟控制台输入线程，并根据输入的字母进行相应的操作

  // msg->header.stamp.toSec()可获取系统时间（以秒为单位）
  // addImage()函数在frame_handler_mono.cpp中
  vo_->addImage(img, msg->header.stamp.toSec());
  // 调用Visualizer类成员函数publishMinimal
  // 进行ROS消息有关的设置
  visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, msg->header.stamp.toSec());

  if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
	  // 调用Visualizer类成员函数visualizeMarkers
	  // 里面又调用了publishTfTransform、publishCameraMarke、publishPointMarker、publishMapRegion等函数，进行Marker和关键帧的显示。
    visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

  if(publish_dense_input_)
	  // 调用Visualizer类成员函数exportToDense函数，稠密显示特征点。
    visualizer_.exportToDense(vo_->lastFrame());

  // 判断stage_，若值为STAGE_PAUSED，则将线程挂起100000微秒（0.1秒）
  if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
    usleep(100000);
}

/*
* 开辟控制台输入线程，并根据输入的字母进行相应的操作
* 在本文件中的void VoNode::imgCb(const sensor_msgs::ImageConstPtr& msg)函数中调用
*/
void VoNode::processUserActions()
{
  char input = remote_input_.c_str()[0];
  remote_input_ = "";

  if(user_input_thread_ != NULL)
  {
    char console_input = user_input_thread_->getInput();
    if(console_input != 0)
      input = console_input;
  }

  switch(input)
  {
    case 'q':
      quit_ = true;
      printf("SVO user input: QUIT\n");
      break;
    case 'r':
      vo_->reset();
      printf("SVO user input: RESET\n");
      break;
    case 's':
      vo_->start();
      printf("SVO user input: START\n");
      break;
    default: ;
  }
}

void VoNode::remoteKeyCb(const std_msgs::StringConstPtr& key_input)
{
  remote_input_ = key_input->data;
}

} // namespace svo

/*
* 主函数
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");// ros初始化
  ros::NodeHandle nh;// 创建节点句柄NodeHandle ，名为nh（创建节点前必须要有NodeHandle）
  std::cout << "create vo_node" << std::endl; 
  svo::VoNode vo_node;// 创建节点VoNode，名为vo_node，同时VoNode的构造函数完成一系列初始化操作

  // subscribe to cam msgs
  std::string cam_topic(vk::getParam<std::string>("svo/cam_topic", "camera/image_raw"));// topic名称
  image_transport::ImageTransport it(nh);//  创建图片发布/订阅器，名为it，使用了之前创建的节点句柄nh
  // 对于节点vo_node，一旦有图像（5代表队列长度，应该是5张图片）发布到主题cam_topic时，就执行svo::VoNode::imgCb函数
  // 然后it.subscribe返回值保存到image_transport::Subscriber型变量it_sub。
  // imgCb()在本文件中
  image_transport::Subscriber it_sub = it.subscribe(cam_topic, 5, &svo::VoNode::imgCb, &vo_node);

  // subscribe to remote input
  // 订阅远程输入消息（应该指的就是键盘输入）
  // nh.subscribe("svo/remote_key", 5, &svo::VoNode::remoteKeyCb, &vo_node)
  // 意思跟上面差不多，有消息发布到主题svo / remote_key（队列长度是5，如果输入6个数据，那么第6个就会被舍弃），就执行svo::VoNode::remoteKeyCb函数
  // 返回值保存到vo_node.sub_remote_key_
  vo_node.sub_remote_key_ = nh.subscribe("svo/remote_key", 5, &svo::VoNode::remoteKeyCb, &vo_node);

  // start processing callbacks
  // 无图像信息输入或者键盘输入q，则停止程序，打印 SVO终止 信息
  while(ros::ok() && !vo_node.quit_)
  {
    ros::spinOnce();
    // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
  }

  printf("SVO terminated.\n");
  return 0;
}
