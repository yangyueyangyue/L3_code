/******************************************************************************
 ** lcm模块封装
 ******************************************************************************
 *
 *  对lcm通信模块的封装，用于LCM节点间通讯
 *
 *  @file       lcm_node.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_FRAMEWORK_LCM_NODE_H_
#define PHOENIX_FRAMEWORK_LCM_NODE_H_

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <string>

#include "utils/macros.h"

#if (ENABLE_LCM_NODE)

#include "boost/thread.hpp"

#include "lcm/lcm-cpp.hpp"
#include "lcm/lcm.h"
#include "lcm/lcm_coretypes.h"

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"


namespace phoenix {
namespace framework {


class LcmNode {
public:
  LcmNode(const std::string &url);
  LcmNode();
  ~LcmNode();

  bool Start();
  bool Stop();

  template <typename MessageHandlerClass>
  bool Subscribe(const std::string& channel,
                 void (MessageHandlerClass::*handlerMethod)(
                   const lcm::ReceiveBuffer* rbuf,
                   const std::string& channel),
                 MessageHandlerClass* handler);

  template <typename MessageType, typename MessageHandlerClass>
  bool Subscribe(const std::string& channel,
                 void (MessageHandlerClass::*handlerMethod)(
                   const lcm::ReceiveBuffer* rbuf,
                   const std::string& channel,
                   const MessageType* msg),
                 MessageHandlerClass* handler);

  inline Int32_t Publish(const std::string& channel,
                         void *data, unsigned int datalen);

  template<class MessageType>
  inline Int32_t Publish(const std::string& channel, const MessageType* msg);

private:
  void TheadReceivingMessages();

private:
  lcm::LCM lcm_;
  boost::atomic_bool thread_running_flag_;
  boost::thread thread_receiving_messages_;
};


template <typename MessageHandlerClass>
bool LcmNode::Subscribe(const std::string& channel,
                        void (MessageHandlerClass::*handlerMethod)(
                          const lcm::ReceiveBuffer* rbuf,
                          const std::string& channel),
                        MessageHandlerClass* handler) {
  if(!lcm_.good()) {
    LOG_ERR << "LCM is not good.";
    return (false);
  }

  lcm_.subscribe(channel, handlerMethod, handler);

  return (true);
}

template <typename MessageType, typename MessageHandlerClass>
bool LcmNode::Subscribe(const std::string& channel,
                        void (MessageHandlerClass::*handlerMethod)(
                          const lcm::ReceiveBuffer* rbuf,
                          const std::string& channel,
                          const MessageType* msg),
                        MessageHandlerClass* handler) {
  if(!lcm_.good()) {
    LOG_ERR << "LCM is not good.";
    return (false);
  }

  lcm_.subscribe(channel, handlerMethod, handler);

  return (true);
}

Int32_t LcmNode::Publish(const std::string& channel,
                         void *data, unsigned int datalen) {
  return (lcm_.publish(channel, data, datalen));
}

template<class MessageType>
Int32_t LcmNode::Publish(const std::string& channel,
                         const MessageType* msg) {
  return (lcm_.publish(channel, msg));
}


}  // namespace framework
}  // namespace phoenix


#endif // #if (ENABLE_LCM_NODE)


#endif  // PHOENIX_FRAMEWORK_LCM_NODE_H_

