#include "sensor/task_recv_mpu_data.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "vehicle_model_wrapper.h"
#include "communication/shared_data.h"




#define ARS_RADAR_LOG_FLAG 0
#define ARS_RADAR_CANFRAME_LOG_FLAG 0

#if (ENABLE_CAN_DEV_ZLGCANNET)
#include "can_dev/zlg_can_net/can_driver_zlgcannet_fd.h"
#endif


namespace phoenix {
namespace sensor {


void ReceiveUdpMsg(const Char_t *channel,Uint64_t timestamp,
                                    const void *buf, Int32_t buf_len,
                                    bool has_time, void *user)
{ 
    if(user == Nullptr_t){
      return;
    }

    // LOG_INFO(3) << "Begin Receive Udp Msg !! ";
  
    TaskRecvMpuData * task = (TaskRecvMpuData *)user;
    MpuState::MonitorMpuState mpu_state;
    const Uint8_t* buffer=(const Uint8_t*)buf;
    Int64_t ts_record;
    Int32_t size_time = sizeof(ts_record);
    common::com_memcpy(&ts_record, buffer, size_time);
    Int32_t len = buf_len;
    Uint8_t* p_buff = (Uint8_t*)buffer;
    if (has_time) {
        common::com_memcpy(&ts_record, buffer, size_time);
        len = buf_len - size_time;
        p_buff = (p_buff + size_time);
    }
    else {
        
    }
    
    if (!mpu_state.ParseFromArray(p_buff, len)) {
        LOG_ERR << "Failed to parse mpu state from array.";
        return;
    }
    mpu_state.set_timestamp(ts_record);
    framework::MessageMpuStateData message(framework::MSG_ID_RECV_MPU_STATE_DATA, &mpu_state);
    task->Notify(message);
    mpu_state.Clear();

}

/*
void MsgReceiver::HandleMonitorMpuState(const void *buf, Int32_t buf_len, bool has_time)
{
    phoenix::data_serial::serial_mpu_cass<MpuState::MonitorMpuState> SerialMpuCass;
    MpuState::MonitorMpuState MonitorMpuStateObj;

    const Uint8_t* buffer = (const Uint8_t*) buf;
    Int64_t ts_record;
    Int32_t size_time = sizeof(ts_record);
    common::com_memcpy(&ts_record, buffer, size_time);
    if (has_time) {
        common::com_memcpy(&ts_record, buffer, size_time);
        SerialMpuCass.ParseProcess(MonitorMpuStateObj, buffer + size_time, buf_len - size_time);
    }
    else {
        SerialMpuCass.ParseProcess(MonitorMpuStateObj, buffer, buf_len);
    }
    MessageMonitorMpuState msg(&MonitorMpuStateObj);
    msg.setRecordTimestamp(ts_record);

    Notify(msg);
}




*/



TaskRecvMpuData::TaskRecvMpuData(framework::Task *manager)
  : framework::Task(
      framework::TASK_ID_RECV_ARS_DATA_FRONT, "Recv ARS Front Data", manager) {
  running_flag_recv_ = false;

  
 /**
  * mpu udp通信参数配置初始化
 */
  udp_mpu_param_ = new framework::mpu::UdpParam();
  udp_mpu_param_->enable_recv = true;
  udp_mpu_param_->enable_send = true;
  udp_mpu_param_->rcv_port = 22010;
  udp_mpu_param_->snd_remote_port = 22011;
  udp_mpu_param_->mode = framework::mpu::UdpParam::MODE_GROUP;
  com_snprintf(udp_mpu_param_->snd_addr, 20, "224.10.10.2");
  udp_mpu_node_ = new framework::mpu::UdpMPUNode();
  mpu_data_ready_ = false;
}


TaskRecvMpuData::~TaskRecvMpuData() {
  Stop();
  
  delete udp_mpu_param_;
  delete udp_mpu_node_;
}

bool TaskRecvMpuData::Start() {
    if(running_flag_recv_){
        return (true);
    }
    bool ret = udp_mpu_node_->Start(*udp_mpu_param_);
    if(ret == false){
      udp_mpu_node_->Stop();
      LOG_ERR << "Start Receive  MPU State Failed!!";
      return false;
    }

    ret = udp_mpu_node_->Subscribe("mpu/State", ReceiveUdpMsg, this);
    if(ret == false){
      udp_mpu_node_->Stop();
      LOG_ERR << "Start Subscribe  MPU State Failed!!";
      return false;
    }

    ret = udp_mpu_node_->StartReceiving();
    if(ret == false){
      udp_mpu_node_->Stop();
      LOG_ERR<< "Start Receiving  MPU State Failed!!";
      return false;
    }
    LOG_INFO(3) << "Start Receive  MPU State data !! ...";
    running_flag_recv_ = true;
    return (true);
}

bool TaskRecvMpuData::Stop() {
  if (running_flag_recv_) {
    running_flag_recv_ = false;
    udp_mpu_node_ ->Stop();
    LOG_INFO(3) << "Stop thread of receiving mpu data data ...";
  }
  return (true);
}


} // namespace sensor
} // namespace phoenix

