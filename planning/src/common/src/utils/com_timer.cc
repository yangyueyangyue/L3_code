//
#include "utils/com_timer.h"


namespace phoenix {
namespace common {


ComTimer::ComTimer() {
  active_flag_ = false;
  settings_timeout_ = 0;
  start_timestamp_ = GetClockNowMs();

  common::com_memset(&user_data_, 0, sizeof(user_data_));
}

ComTimer::ComTimer(Int64_t timeout_ms) {
  active_flag_ = false;
  settings_timeout_ = timeout_ms;
  start_timestamp_ = GetClockNowMs();

  common::com_memset(&user_data_, 0, sizeof(user_data_));
}

ComTimer::~ComTimer() {
}

Int64_t ComTimer::Elapsed() const {
  return (CalcElapsedClockMs(start_timestamp_, GetClockNowMs()));
}

void ComTimer::Restart() {
  active_flag_ = true;

  start_timestamp_ = GetClockNowMs();
}

void ComTimer::Stop() {
  active_flag_ = false;
}

bool ComTimer::Update() {
  return (Update(GetClockNowMs()));
}

bool ComTimer::Update(Int64_t timestamp_ms) {
  // printf("update timer, timestamp=%ld\n", timestamp_ms);
  bool timeout_flag = false;
  if (!active_flag_) {
    return (timeout_flag);
  }

  if (CalcElapsedClockMs(start_timestamp_, timestamp_ms) >= settings_timeout_) {
    NotifyTimeout();
    active_flag_ = false;
    timeout_flag = true;
  }

  return (timeout_flag);
}

void ComTimer::NotifyTimeout() {
  // nothing to do
}


}  // namespace common
}  // namespace phoenix
