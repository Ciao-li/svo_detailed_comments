
/*
** VO���ڵ㣬������
** �ο����ӣ�https://www.cnblogs.com/hxzkh/p/8607714.html ��
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
  vk::AbstractCamera* cam_;	//Ҫ�õ�vikit���е�AbstractCamera��
  bool quit_;
  VoNode();
  ~VoNode();
  void imgCb(const sensor_msgs::ImageConstPtr& msg);
  void processUserActions();
  void remoteKeyCb(const std_msgs::StringConstPtr& key_input);
};

/*
* ���캯��
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
    user_input_thread_ = boost::make_shared<vk::UserInputThread>();//�����߳����ڼ�������̨���룬boost��

  // Create Camera
  // ���������������vikit��
  if(!vk::camera_loader::loadFromRosNs("svo", cam_))
    throw std::runtime_error("Camera model not correctly specified.");

  // Get initial position and orientation
  /* 
	��ʼ��λ�ˣ�SE3(R,t)��R:��ת����t:ƽ��������Sophus��
	vk::rpy2dcm(const Vector3d &rpy) �ɽ�ŷ���� rpy ת��Ϊ��ת����
	vikit�⡢Eigen��
  */
  visualizer_.T_world_from_vision_ = Sophus::SE3(
      vk::rpy2dcm(Vector3d(vk::getParam<double>("svo/init_rx", 0.0),
                           vk::getParam<double>("svo/init_ry", 0.0),
                           vk::getParam<double>("svo/init_rz", 0.0))),
      Eigen::Vector3d(vk::getParam<double>("svo/init_tx", 0.0),
                      vk::getParam<double>("svo/init_ty", 0.0),
                      vk::getParam<double>("svo/init_tz", 0.0)));

  // Init VO and start
  // ��ʼ���Ӿ�ǰ��VO
  // ����frame_handler_mono.cpp�ļ��е�FrameHandlerMono::FrameHandlerMono(vk::AbstractCamera* cam)����
  vo_ = new svo::FrameHandlerMono(cam_);
  vo_->start();// תframe_handler_base.h�ļ� void start() { set_start_ = true; }
}

/*
* ��������
*/
VoNode::~VoNode()
{
  delete vo_;
  delete cam_;
  if(user_input_thread_ != NULL)
    user_input_thread_->stop();
}

/*
* �ڱ��ļ��е��������е���
*/
void VoNode::imgCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat img;
  try {
    img = cv_bridge::toCvShare(msg, "mono8")->image;// ��ȡͼ�񲢽�ROS����ת��ΪOpenCV�е�ͼ������
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  processUserActions();// ���ٿ���̨�����̣߳��������������ĸ������Ӧ�Ĳ���

  // msg->header.stamp.toSec()�ɻ�ȡϵͳʱ�䣨����Ϊ��λ��
  // addImage()������frame_handler_mono.cpp��
  vo_->addImage(img, msg->header.stamp.toSec());
  // ����Visualizer���Ա����publishMinimal
  // ����ROS��Ϣ�йص�����
  visualizer_.publishMinimal(img, vo_->lastFrame(), *vo_, msg->header.stamp.toSec());

  if(publish_markers_ && vo_->stage() != FrameHandlerBase::STAGE_PAUSED)
	  // ����Visualizer���Ա����visualizeMarkers
	  // �����ֵ�����publishTfTransform��publishCameraMarke��publishPointMarker��publishMapRegion�Ⱥ���������Marker�͹ؼ�֡����ʾ��
    visualizer_.visualizeMarkers(vo_->lastFrame(), vo_->coreKeyframes(), vo_->map());

  if(publish_dense_input_)
	  // ����Visualizer���Ա����exportToDense������������ʾ�����㡣
    visualizer_.exportToDense(vo_->lastFrame());

  // �ж�stage_����ֵΪSTAGE_PAUSED�����̹߳���100000΢�루0.1�룩
  if(vo_->stage() == FrameHandlerMono::STAGE_PAUSED)
    usleep(100000);
}

/*
* ���ٿ���̨�����̣߳��������������ĸ������Ӧ�Ĳ���
* �ڱ��ļ��е�void VoNode::imgCb(const sensor_msgs::ImageConstPtr& msg)�����е���
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
* ������
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "svo");// ros��ʼ��
  ros::NodeHandle nh;// �����ڵ���NodeHandle ����Ϊnh�������ڵ�ǰ����Ҫ��NodeHandle��
  std::cout << "create vo_node" << std::endl; 
  svo::VoNode vo_node;// �����ڵ�VoNode����Ϊvo_node��ͬʱVoNode�Ĺ��캯�����һϵ�г�ʼ������

  // subscribe to cam msgs
  std::string cam_topic(vk::getParam<std::string>("svo/cam_topic", "camera/image_raw"));// topic����
  image_transport::ImageTransport it(nh);//  ����ͼƬ����/����������Ϊit��ʹ����֮ǰ�����Ľڵ���nh
  // ���ڽڵ�vo_node��һ����ͼ��5������г��ȣ�Ӧ����5��ͼƬ������������cam_topicʱ����ִ��svo::VoNode::imgCb����
  // Ȼ��it.subscribe����ֵ���浽image_transport::Subscriber�ͱ���it_sub��
  // imgCb()�ڱ��ļ���
  image_transport::Subscriber it_sub = it.subscribe(cam_topic, 5, &svo::VoNode::imgCb, &vo_node);

  // subscribe to remote input
  // ����Զ��������Ϣ��Ӧ��ָ�ľ��Ǽ������룩
  // nh.subscribe("svo/remote_key", 5, &svo::VoNode::remoteKeyCb, &vo_node)
  // ��˼�������࣬����Ϣ����������svo / remote_key�����г�����5���������6�����ݣ���ô��6���ͻᱻ����������ִ��svo::VoNode::remoteKeyCb����
  // ����ֵ���浽vo_node.sub_remote_key_
  vo_node.sub_remote_key_ = nh.subscribe("svo/remote_key", 5, &svo::VoNode::remoteKeyCb, &vo_node);

  // start processing callbacks
  // ��ͼ����Ϣ������߼�������q����ֹͣ���򣬴�ӡ SVO��ֹ ��Ϣ
  while(ros::ok() && !vo_node.quit_)
  {
    ros::spinOnce();
    // TODO check when last image was processed. when too long ago. publish warning that no msgs are received!
  }

  printf("SVO terminated.\n");
  return 0;
}
