//
#include "communication/lcm_node.h"


#if (ENABLE_LCM_NODE)


namespace phoenix {
namespace framework {


LcmNode::LcmNode(const std::string &url) : lcm_(url) {
  thread_running_flag_ = false;
}

LcmNode::LcmNode() {
  thread_running_flag_ = false;
}

LcmNode::~LcmNode() {
  Stop();
}

bool LcmNode::Start() {
  LOG_INFO(3) << "Start LCM node ...";

  if(!lcm_.good()) {
    LOG_ERR << "LCM is not good.";
    LOG_INFO(3) << "Start LCM node ... [NG]";
    return false;
  }

  if (!thread_running_flag_) {
    thread_running_flag_ = true;
    thread_receiving_messages_ =
        boost::thread(boost::bind(&LcmNode::TheadReceivingMessages, this));
  }

  LOG_INFO(3) << "Start LCM node ... [OK]";

  return (true);
}

bool LcmNode::Stop() {
  if (thread_running_flag_) {
    LOG_INFO(3) << "Stop LCM Receiving Thread ...";
    thread_running_flag_ = false;
    bool ret = thread_receiving_messages_.timed_join(
          boost::posix_time::seconds(2));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread \"LCM Receiving\" exiting.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop LCM Receiving Thread... [OK]";
    } else {
      LOG_INFO(3) << "Stop LCM Receiving Thread... [NG]";
    }
  }

  return true;
}

void LcmNode::TheadReceivingMessages() {
  LOG_INFO(3) << "LCM Receiving Thread ... [Started]";

  if(!lcm_.good()) {
    LOG_ERR << "LCM is not good.";
    return;
  }

  while (thread_running_flag_) {
    //lcm_.handle();
    lcm_.handleTimeout(200/*ms*/);
  }

  LOG_INFO(3) << "LCM Receiving Thread ... [Stopped]";
}


}  // namespace framework
}  // namespace phoenix


#endif // #if (ENABLE_LCM_NODE)

