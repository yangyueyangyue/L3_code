/******************************************************************************
 ** ROS节点封装
 ******************************************************************************
 *
 *  ROS节点封装，用于ROS节点间通讯
 *
 *  @file       ros_node.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_ROS_NODE_H_
#define PHOENIX_FRAMEWORK_ROS_NODE_H_


#include <memory>
#include <map>
#include <string>

#include "utils/macros.h"

#if (ENABLE_ROS_NODE)

#include "boost/thread.hpp"
#include "ros/ros.h"
#include "utils/log.h"


namespace phoenix {
namespace framework {

class RosNode {
 public:
  RosNode(int argc, char** argv, const std::string& name);
  virtual ~RosNode();

  bool Start();
  bool Stop();

  template <typename MessageType, typename MessageHandlerClass>
  bool Subscribe(
      const std::string& topic, unsigned int queue_size,
      void (MessageHandlerClass::*handlerMethod)(const MessageType msg),
      MessageHandlerClass* handler);

  bool Unsubscribe(const std::string& topic);

  template <typename MessageType>
  bool AddPublisher(const std::string& topic, unsigned int queue_size);

  bool RemovePublisher(const std::string& topic);

  template <typename MessageType>
  bool Publish(const std::string& topic, const MessageType& msg);

 private:
  void TheadReceivingMessages();

 private:
  int init_argc_;
  char** init_argv_;
  std::string node_name_;

  std::unique_ptr<ros::NodeHandle> ros_node_handle_;
  std::map<std::string, ros::Subscriber> subscribers_;
  std::map<std::string, ros::Publisher> publishers_;

  boost::thread thread_receiving_messages_;
};

template <typename MessageType, typename MessageHandlerClass>
bool RosNode::Subscribe(
    const std::string& topic, unsigned int queue_size,
    void (MessageHandlerClass::*handlerMethod)(const MessageType msg),
    MessageHandlerClass* handler) {
  if (Nullptr_t == ros_node_handle_) {
    LOG_ERR << "Ros node is not ready.";
    return (false);
  }

  const auto& pos = subscribers_.find(topic);
  if (pos != subscribers_.end()) {
    LOG_WARN << "Alread subscribe to \"" << topic.c_str() << "\".";
    return (true);
  }

  subscribers_.insert(std::make_pair(topic,
      ros_node_handle_->subscribe<MessageType>(
        topic, queue_size, handlerMethod, handler,
        ros::TransportHints().tcpNoDelay())));

  return (true);
}

template <typename MessageType>
bool RosNode::AddPublisher(const std::string& topic, unsigned int queue_size) {
  if (Nullptr_t == ros_node_handle_) {
    LOG_ERR << "Ros node is not ready.";
    return (false);
  }

  const auto& pos = publishers_.find(topic);
  if (pos != publishers_.end()) {
    LOG_WARN << "Alread have this publisher with the topic \"" << topic.c_str() << "\".";
    return (true);
  }

  publishers_.insert(std::make_pair(topic,
      ros_node_handle_->advertise<MessageType>(topic, queue_size)));

  return (true);
}

template <typename MessageType>
bool RosNode::Publish(const std::string& topic,
    const MessageType& msg) {
  if (Nullptr_t == ros_node_handle_) {
    LOG_ERR << "Ros node is not ready.";
    return (false);
  }

  const auto& pos = publishers_.find(topic);
  if (pos == publishers_.end()) {
    LOG_WARN << "Have not such topic \"" << topic.c_str() << "\".";
    return (false);
  }

  pos->second.publish<MessageType>(msg);

  return (true);
}


}  // namespace framework
}  // namespace phoenix

#endif // #if (ENABLE_ROS_NODE)

#endif  // PHOENIX_FRAMEWORK_ROS_NODE_H_
