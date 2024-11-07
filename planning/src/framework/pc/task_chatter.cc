/* Copyright 2018,2019 Kotei Co., Ltd.
 ******************************************************************************
 ** 其它任务处理模块
 ******************************************************************************
 *
 *  处理其它琐碎的任务
 *
 *  @file       task_chatter.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "task_chatter.h"
#include <boost/date_time.hpp>
#include "utils/macros.h"
#include "utils/log.h"
#include "communication/shared_data.h"

#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
#include "modules/map/proto/map.pb.h"
#include "modules/routing/proto/routing.pb.h"
#endif

namespace phoenix {
namespace framework {


/******************************************************************************/
/** 构造函数
 ******************************************************************************
 *  @param      manager         (in)           管理者
 *
 *  @retval     none
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       构造函数
 *
 *  <Attention>
 *       None
 *
 */
TaskChatter::TaskChatter(Task* manager) :
  Task(TASK_ID_CHATTER, "Chatter", manager) {
  thread_running_flag_ = false;
  msg_queue_.Clear();
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
 *       None
 *
 */
TaskChatter::~TaskChatter() {
  Stop();
}

/******************************************************************************/
/** 开启任务
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       开启任务
 *
 *  <Attention>
 *       none
 *
 */
bool TaskChatter::Start() {
  // Start Thread
  if (!thread_running_flag_) {
    LOG_INFO(3) << "Start Chatter Thread ...";
    msg_queue_.Clear();
    thread_running_flag_ = true;
    thread_do_task_ =
        boost::thread(boost::bind(&TaskChatter::TheadDoTask, this));
  }

  return (true);
}

/******************************************************************************/
/** 关闭任务
 ******************************************************************************
 *  @param      none
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       关闭任务
 *
 *  <Attention>
 *       none
 *
 */
bool TaskChatter::Stop() {
  // Stop Thread
  if (thread_running_flag_) {
    LOG_INFO(3) << "Stop Chatter Thread ...";
    msg_queue_.Clear();
    thread_running_flag_ = false;
    msg_ready_cond_.notify_one();
    bool ret = thread_do_task_.timed_join(boost::posix_time::seconds(2));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread \"Chatter\" exiting.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop Chatter Thread... [OK]";
    } else {
      LOG_INFO(3) << "Stop Chatter Thread... [NG]";
    }
  }

  return (true);
}

/******************************************************************************/
/** 处理模块消息
 ******************************************************************************
 *  @param      msg             (in)           消息
 *              sender          (in)           发送者
 *
 *  @retval     true             成功
 *              false            失败
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       处理模块消息
 *
 *  <Attention>
 *       None
 *
 */
bool TaskChatter::HandleMessage(const Message& msg, Task* sender) {
  // Must be reentrant function

  bool ret = true;

  switch (msg.id()) {
  case (MSG_ID_MAP_DATA):
  {
    const MessageMapData& msg_map_data =
        dynamic_cast<const MessageMapData&>(msg);
    if (msg_map_data.buff_size() > 1) {
      SaveMapDataBuff(msg_map_data.data_buff(),
                      msg_map_data.buff_size());

      MessageChatterTask req_msg(CHATTER_WORK_REQ_PARSE_MAP_DATA);
      msg_queue_.Put(req_msg);
      msg_ready_cond_.notify_one();
    } else {
      LOG_ERR << "Chatter task received invalid map data.";
    }
  }
    break;

  case (MSG_ID_ROUTING_DATA):
  {
    const MessageRoutingData& msg_routing_data =
        dynamic_cast<const MessageRoutingData&>(msg);
    if (msg_routing_data.buff_size() > 1) {
      SaveRoutingDataBuff(msg_routing_data.data_buff(),
                          msg_routing_data.buff_size());

      MessageChatterTask req_msg(CHATTER_WORK_REQ_PARSE_ROUTING_DATA);
      msg_queue_.Put(req_msg);
      msg_ready_cond_.notify_one();
    } else {
      LOG_ERR << "Chatter task received invalid routing data.";
    }
  }
    break;

  default:
    ret = false;
    break;
  }

  return (ret);
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
 *       线程函数
 *
 *  <Attention>
 *       None
 *
 */
void TaskChatter::TheadDoTask() {
  LOG_INFO(3) << "Chatter Thread ... [Started]";

  while (thread_running_flag_) {
    // waiting message
    bool received_msg = false;
    MessageChatterTask message;
    {
      Lock lock(msg_ready_lock_);
      while (thread_running_flag_) {
        received_msg = msg_queue_.Get(&message);
        if (received_msg) {
          // data ready
          break;
        } else {
          msg_ready_cond_.wait(lock);
        }
      }
    }
    if (!thread_running_flag_) {
      break;
    }
    if (!received_msg) {
      continue;
    }

    switch (message.work_id()) {
    case (CHATTER_WORK_REQ_PARSE_MAP_DATA): {
      std::shared_ptr<Char_t> map_data_array = Nullptr_t;
      Int32_t map_data_array_size = 0;
      GetMapDataBuff(&map_data_array, &map_data_array_size);

#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
      std::shared_ptr<apollo::hdmap::Map> map_data(new apollo::hdmap::Map());
      if (map_data->ParseFromArray(map_data_array.get(), map_data_array_size)) {
        SharedData::instance()->SetRawHDMap(map_data);
      } else {
        LOG_ERR << "Failed to parse map data from array.";
      }
#endif
    }
      break;

    case (CHATTER_WORK_REQ_PARSE_ROUTING_DATA): {
      std::shared_ptr<Char_t> routing_data_array = Nullptr_t;
      Int32_t routing_data_array_size = 0;
      GetRoutingDataBuff(&routing_data_array, &routing_data_array_size);
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
      std::shared_ptr<apollo::routing::RoutingResponse>
          routing_data(new apollo::routing::RoutingResponse());
      if (routing_data->ParseFromArray(routing_data_array.get(),
          routing_data_array_size)) {
        SharedData::instance()->SetRawRouting(routing_data);
      } else {
        LOG_ERR << "Failed to parse routing data from array.";
      }
#endif
    }
      break;

    default: {
      LOG_WARN << "Chatter Thread received invalid req.";
    }
      break;
    }
  }

  LOG_INFO(3) << "Chatter Thread ... [Stopped]";
}

/******************************************************************************/
/** 保存地图数据
 ******************************************************************************
 *  @param      buff            (in)          数据地址
 *              size            (in)          数据尺寸
 *
 *  @retval     none
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       保存地图数据
 *
 *  <Attention>
 *       None
 *
 */
void TaskChatter::SaveMapDataBuff(const Char_t* buff, Int32_t size) {
  Lock lock(map_data_array_lock_);

  std::shared_ptr<Char_t> map_data_array(new Char_t[size],
      std::default_delete<Char_t[]>());
  std::memcpy(map_data_array.get(), buff, size);

  map_data_array_ = map_data_array;
  map_data_array_size_ = size;
}

/******************************************************************************/
/** 读取地图数据
 ******************************************************************************
 *  @param      map_data_array        (out)          数据地址
 *              map_data_array_size   (out)          数据尺寸
 *
 *  @retval     none
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       读取地图数据
 *
 *  <Attention>
 *       None
 *
 */
void TaskChatter::GetMapDataBuff(
    std::shared_ptr<Char_t>* map_data_array, Int32_t* map_data_array_size) {
  Lock lock(map_data_array_lock_);

  *map_data_array = map_data_array_;
  *map_data_array_size = map_data_array_size_;
}

/******************************************************************************/
/** 保存routing数据
 ******************************************************************************
 *  @param      buff            (in)          数据地址
 *              size            (in)          数据尺寸
 *
 *  @retval     none
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       保存routing数据
 *
 *  <Attention>
 *       None
 *
 */
void TaskChatter::SaveRoutingDataBuff(const Char_t* buff, Int32_t size) {
  Lock lock(routing_data_array_lock_);

  std::shared_ptr<Char_t> routing_data_array(new Char_t[size],
      std::default_delete<Char_t[]>());
  std::memcpy(routing_data_array.get(), buff, size);

  routing_data_array_ = routing_data_array;
  routing_data_array_size_ = size;
}

/******************************************************************************/
/** 读取routing数据
 ******************************************************************************
 *  @param      routing_data_array        (out)          数据地址
 *              routing_data_array_size   (out)          数据尺寸
 *
 *  @retval     none
 *
 *  @version    001 2018.11.22   pengc         新规作成
 *
 *  <Function>
 *       读取touting数据
 *
 *  <Attention>
 *       None
 *
 */
void TaskChatter::GetRoutingDataBuff(
    std::shared_ptr<Char_t>* routing_data_array,
    Int32_t* routing_data_array_size) {
  Lock lock(routing_data_array_lock_);

  *routing_data_array = routing_data_array_;
  *routing_data_array_size = routing_data_array_size_;
}


}  // namespace framework
}  // namespace phoenix

