//
#ifndef PHOENIX_COMMON_COM_TIMER_H_
#define PHOENIX_COMMON_COM_TIMER_H_

#include "utils/com_utils.h"
#include "container/doubly_linked_list.h"


namespace phoenix {
namespace common {


/// TODO: 若使用自定义的时钟，则不能超过其最大允许的计时范围 (USER_CLOCK_US_MODULO)
class ComTimer {
public:
  enum { MAX_USER_DATA_NUM = 4 };

public:
  ComTimer();
  ComTimer(Int64_t timeout_ms);
  virtual ~ComTimer();

  void SetTimeout(Int64_t timeout_ms) {
    settings_timeout_ = timeout_ms;
  }
  bool IsActive() const {
    return active_flag_;
  }
  Int64_t Elapsed() const;
  void Restart();
  void Stop();
  bool Update();
  bool Update(Int64_t timestamp_ms);

  void SetUserDataInt32(Int32_t idx, Int32_t value) {
    bool valid_idx = (0 <= idx) && (idx < MAX_USER_DATA_NUM);
    COM_CHECK(valid_idx);
    if (valid_idx) {
      user_data_.int32_value[idx] = value;
    }
  }
  void SetUserDataFloat32(Int32_t idx, Float32_t value) {
    bool valid_idx = (0 <= idx) && (idx < MAX_USER_DATA_NUM);
    COM_CHECK(valid_idx);
    if (valid_idx) {
      user_data_.float32_value[idx] = value;
    }
  }

  Int32_t GetUserDataInt32(Int32_t idx) const {
    bool valid_idx = (0 <= idx) && (idx < MAX_USER_DATA_NUM);
    COM_CHECK(valid_idx);
    if (valid_idx) {
      return (user_data_.int32_value[idx]);
    }
    return (0);
  }
  Float32_t GetUserDataFloat32(Int32_t idx) const {
    bool valid_idx = (0 <= idx) && (idx < MAX_USER_DATA_NUM);
    COM_CHECK(valid_idx);
    if (valid_idx) {
      return (user_data_.float32_value[idx]);
    }
    return (0.0);
  }

protected:
  virtual void NotifyTimeout();

private:
  bool active_flag_;
  Int64_t settings_timeout_;
  Int64_t start_timestamp_;

  struct {
    Int32_t int32_value[MAX_USER_DATA_NUM];
    Float32_t float32_value[MAX_USER_DATA_NUM];
  } user_data_;
};


template <Int32_t MaxTimerNum>
class ComTimerList {
public:
  ComTimerList() {
    // nothing to do
  }
  ~ComTimerList() {
    // nothing to do
  }

  void Clear() {
    timer_list_.Clear(timer_pool_);
  }

  bool AddTimer(ComTimer* timer) {
    return (timer_list_.PushBack(timer, timer_pool_));
  }

  void Update();

private:
  typedef DoublyLinkedList<ComTimer*, MaxTimerNum> timer_list_type;
  typedef DataPool<DoublyLinkedListNode<ComTimer*>, MaxTimerNum> timer_pool_type;
  timer_list_type timer_list_;
  timer_pool_type timer_pool_;
};

template <Int32_t MaxTimerNum>
void ComTimerList<MaxTimerNum>::Update() {
  Int64_t timestamp = GetClockNowMs();

  typename timer_list_type::iterator it = timer_list_.begin(timer_pool_);
  for (; it != timer_list_.end(timer_pool_); ++it) {
    (*it)->Update(timestamp);
  }
}


}  // namespace common
}  // namespace phoenix


#endif // PHOENIX_COMMON_COM_TIMER_H_



