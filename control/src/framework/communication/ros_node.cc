/******************************************************************************
 ** ROS节点封装
 ******************************************************************************
 *
 *  ROS节点封装，用于ROS节点间通讯
 *
 *  @file       ros_node.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "communication/ros_node.h"


#if (ENABLE_ROS_NODE)

#include <string>

namespace phoenix {
namespace framework {


/******************************************************************************/
/** 构造函数
 ******************************************************************************
 *  @param      argc            (IN)           程序输入参数
 *              argv            (IN)           程序输入参数
 *              name            (IN)           节点名称
 *
 *  @retval     none
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       构造函数
 *
 *  <Attention>
 *       在整个系统中，节点名称必须是唯一的
 *
 */
RosNode::RosNode(int argc, char** argv, const std::string& name) :
  init_argc_(argc),
  init_argv_(argv),
  node_name_(name) {
  // nothing to do
}

/******************************************************************************/
/** 析构函数
 ******************************************************************************
 *  @param      none
 *
 *  @retval     none
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       析构函数
 *
 *  <Attention>
 *       none
 *
 */
RosNode::~RosNode() {
  Stop();
}

/******************************************************************************/
/** 启动ROS节点
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       启动ROS节点
 *
 *  <Attention>
 *       None
 *
 */
bool RosNode::Start() {
  if (ros::isStarted()) {
    return (true);
  }

  LOG_INFO(3) << "Start ROS node ...";

  subscribers_.clear();
  publishers_.clear();

  ros::init(init_argc_, init_argv_, node_name_);
  if (!ros::master::check()) {
    LOG_ERR << "Failed to try to contact the ros master.";
    LOG_INFO(3) << "Start ROS node ... [NG]";
    return false;
  }

  // explicitly needed since our nodehandle is going out of scope.
  // ros::start();
  // ros::NodeHandle n;
  ros_node_handle_.reset(new ros::NodeHandle());
  // Add your ros communications here.
  // chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

  thread_receiving_messages_ = boost::thread(
      boost::bind(&RosNode::TheadReceivingMessages, this));

  LOG_INFO(3) << "Start ROS node ... [OK]";

  return (true);
}

/******************************************************************************/
/** 关闭ROS节点
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       关闭ROS节点
 *
 *  <Attention>
 *       None
 *
 */
bool RosNode::Stop() {
  bool ret = true;

  subscribers_.clear();
  publishers_.clear();

  if (ros::isStarted()) {
    LOG_INFO(3) << "Stop ROS node ...";

    // explicitly needed since we use ros::start();
    ros::shutdown();
    ros::waitForShutdown();

    // wait();
    ret = thread_receiving_messages_.timed_join(boost::posix_time::seconds(2));
    if (false == ret) {
      LOG_ERR << "Failed to wait for ros node to be shutdown.";
    }

    if (ret) {
      LOG_INFO(3) << "Stop ROS node ... [OK]";
    } else {
      LOG_INFO(3) << "Stop ROS node ... [NG]";
    }
  }

  return (ret);
}

/******************************************************************************/
/** 取消订阅某个报文
 ******************************************************************************
 *  @param      topic            报文名称
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       取消订阅某个报文
 *
 *  <Attention>
 *       none
 *
 */
bool RosNode::Unsubscribe(const std::string& topic) {
  const auto& pos = subscribers_.find(topic);
  if (pos == subscribers_.end()) {
    LOG_WARN << "Have not such topic \"" << topic.c_str() << "\".";
    return (false);
  }

  subscribers_.erase(pos);

  return (true);
}

/******************************************************************************/
/** 移除某个报文发布
 ******************************************************************************
 *  @param      topic            报文名称
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       移除某个报文发布
 *
 *  <Attention>
 *       none
 *
 */
bool RosNode::RemovePublisher(const std::string& topic) {
  const auto& pos = publishers_.find(topic);
  if (pos == publishers_.end()) {
    LOG_WARN << "Have not such topic \"" << topic.c_str() << "\".";
    return (false);
  }

  publishers_.erase(pos);

  return (true);
}

/******************************************************************************/
/** 线程函数
 ******************************************************************************
 *  @param      none
 *
 *  @retval     none
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       线程函数，用于从总线中获取报文
 *
 *  <Attention>
 *       none
 *
 */
void RosNode::TheadReceivingMessages() {
  LOG_INFO(3) << "ROS Receiving Thead ... [Started]";

  ros::spin();

  LOG_INFO(3) << "ROS Receiving Thead ... [Stopped]";
}


}  // namespace framework
}  // namespace phoenix

#endif // #if (ENABLE_ROS_NODE)


