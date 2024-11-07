//
#include "utils/com_utils.h"
#include "utils/log.h"
#include "vehicle_model_wrapper.h"
#include "sensor/task_recv_maxieye_data.h"
#include "communication/shared_data.h"
#include "parse_sensor_data_common.h"

#include <cmath>
#include <deque>
#include <fstream>

#define CANFRAME_LOG_FLAG 0
#define LANE_LOG_FLAG 0
#define OBSTACLE_LOG_FLAG 0
#define TRAFFICE_LOG_FLAG 0
#define VISUAL_CONTROL_LOG_FLAG 0
#define VISUAL_CONTROL_LANE_LOG_FLAG 0

#if (ENABLE_CAN_DEV_ZLGCANNET)
#include "can_dev/zlg_can_net/can_driver_zlgcannet_fd.h"
#endif


namespace phoenix {
namespace sensor {


#if Enable_Lane_Type_Filter_Flag
//Note: The memory increases automatically even these macros are disable.
//#define Enable_Lane_Type_Filter_Flag 1  // Enable the lane type filter
//#define Enalbe_Save_Lane_Type_Res 1     //Save the log file

int Max_Int(const int a, const int b) {
    return a > b ? a : b;
}

//Create the buff
/*[Input] buf_size, set the buff size. 1 - d array.FIFO
 *[Output] buffer, the array on buf_size.
 *         the initial values set - 1.
 */
std::deque<int> CreateBuff(const int buf_size = 5) {
    return std::deque<int>(buf_size, -1);
}

//Reset the buff
/*[Input] buf_size, set the buff size. 1 - d array.FIFO
 *[Output]buffer, the array on buf_size.
 *        Set the initial values to - 1.
 */
void ResetBuff(std::deque<int>& buf) {
    for (size_t i = 0; i < buf.size(); ++i)
        buf.at(i) = -1;
}

//Count the slack numbers on each type and output the Maximum result
/*Input: buff, the unchecked buff array
 *Output : y, the maximum type result; -2, no result; 1, solid line; 0, dash line;-1, the buff initial value
*/
int CntMaxNumDashSolidType(const std::deque<int>& buff) {
    int buff_size = buff.size();

    if (buff_size < 3) {
        printf("The filter buff size set error.\n");
        return -2;
    }
    else if (buff_size % 2 == 0) {
        printf("The filter buff size should be odd.\n");
        return -2;
    }

    //0, dash; 1, solid; 2, no result; -1, the buff initial value
    int	dash_cnt_num = 0;
    int solid_cnt_num = 0;
    int no_res_cnt_num = 0;
    int init_res_cnt_num = 0;

    for (int i = 0; i < buff_size; ++i) {
        switch (buff.at(i)) {
        case 0:
            ++dash_cnt_num;
            break;
        case 1:
            ++solid_cnt_num;
        case -1:
            ++init_res_cnt_num;
            break;
        default:
            ++no_res_cnt_num;
            break;
        }
    }

    int max_value = Max_Int(Max_Int(Max_Int(dash_cnt_num, solid_cnt_num), no_res_cnt_num), init_res_cnt_num);

    if (dash_cnt_num == max_value) {
        return 0;       // dash line
    }
    else if (solid_cnt_num == max_value) {
        return 1;       // solid line
    }
    else if (init_res_cnt_num == max_value) {
        return -1;      //initial value
    }
    else {
        return -2;      //no result
    }

}

//Check the buffer value
/*[Input]
 * buff, the unchecked array.
 *     buf_size, indicats the array size.
 *[Output]
 * res, 1, empty array; 0, full array.
*/
int IsBuffEmpty(const std::deque<int>& buff, const int buff_size) {
    int res = 0;
    if (buff.empty() || (buff.size() < buff_size))
        res = 1;
    else if (CntMaxNumDashSolidType(buff) == -1) {
        res = 1;
    }
    return res;
}


//The method to filter current type vibration
/*[Input]
 *   cur_ori_lane_type_res, the current orignal type. 0, dash line; 1, solid
 *        line; -1, initial value; -2, the -2 value;
 *   lane_type_buff, the buffer for caching the current result.Output
 *        the maximum type number.
 *   buff_size, set the lane_type_buff size
 *   max_obs_thrsh_size, the maximum frames that exceptional result remains
 *[Output]
 *   cur_filter_lane_res, the filtered result
 *   lane_type_buff, the updated buff while involving the new data
*/
int FilterCurLaneType(const int cur_ori_lane_type_res, std::deque<int>& lane_type_buff,
                      const int buff_size, const int max_obs_thrsh_size) {
    int cur_filter_lane_res = -2;
    static int cur_obs_cnt = 0;             // cur_obs_cnt, the type exceptional statistics counts

    if (IsBuffEmpty(lane_type_buff, buff_size)) {
        if ((0 == cur_ori_lane_type_res) || (1 == cur_ori_lane_type_res)) {
            lane_type_buff.pop_front();
            lane_type_buff.push_back(cur_ori_lane_type_res);
            cur_filter_lane_res = cur_ori_lane_type_res;
        }
    }
    else {
        int cur_lane_type_res = CntMaxNumDashSolidType(lane_type_buff);
        bool cur_cmp_res = (std::abs(cur_ori_lane_type_res - cur_lane_type_res) < 1e-7);

        if ( !cur_cmp_res
            && (cur_obs_cnt < max_obs_thrsh_size)){
            cur_filter_lane_res = cur_lane_type_res;
            ++cur_obs_cnt;
        }
        else if (cur_cmp_res
            && (cur_obs_cnt < max_obs_thrsh_size)) {
            cur_filter_lane_res = cur_lane_type_res;
            lane_type_buff.pop_front();
            lane_type_buff.push_back(cur_ori_lane_type_res);
            cur_obs_cnt = 0;
        }
        else if (cur_obs_cnt >= max_obs_thrsh_size) {
            ResetBuff(lane_type_buff);
            cur_obs_cnt = 0;
//			cur_filter_lane_res = -2;       //It can be cancelled.
        }
    }

    return cur_filter_lane_res;
}

int FilterCurLaneType(const int cur_ori_lane_type_res, std::deque<int>& lane_type_buff,
                      int& cur_obs_cnt, const int buff_size, const int max_obs_thrsh_size) {
    int cur_filter_lane_res = -2;
//    static int cur_obs_cnt = 0;             // cur_obs_cnt, the type exceptional statistics counts

    if (IsBuffEmpty(lane_type_buff, buff_size)) {
        if ((0 == cur_ori_lane_type_res) || (1 == cur_ori_lane_type_res)) {
            lane_type_buff.pop_front();
            lane_type_buff.push_back(cur_ori_lane_type_res);
            cur_filter_lane_res = cur_ori_lane_type_res;
        }
    }
    else {
        int cur_lane_type_res = CntMaxNumDashSolidType(lane_type_buff);
        bool cur_cmp_res = (std::abs(cur_ori_lane_type_res - cur_lane_type_res) < 1e-7);

        if ( !cur_cmp_res
            && (cur_obs_cnt < max_obs_thrsh_size)){
            cur_filter_lane_res = cur_lane_type_res;
            ++cur_obs_cnt;
        }
        else if (cur_cmp_res
            && (cur_obs_cnt < max_obs_thrsh_size)) {
            cur_filter_lane_res = cur_lane_type_res;
            lane_type_buff.pop_front();
            lane_type_buff.push_back(cur_ori_lane_type_res);
            cur_obs_cnt = 0;
        }
        else if (cur_obs_cnt >= max_obs_thrsh_size) {
            ResetBuff(lane_type_buff);
            cur_obs_cnt = 0;
//			cur_filter_lane_res = -2;       //It can be cancelled.
        }
    }

    return cur_filter_lane_res;
}

//Convert the lane type of msg to filter type
/*[Input]
 *  ori_lane_type_res,  lane mark type.
 *  enum LaneMarkType {
    LANE_MARK_TYPE_INVALID = 0,
    LANE_MARK_TYPE_UNKNOWN = 1,
    LANE_MARK_TYPE_DASHED = 2,
    LANE_MARK_TYPE_SOLID = 3,
    LANE_MARK_TYPE_DOUBLE_LANE_MARK = 4,
    LANE_MARK_TYPE_BOTTS_DOTS = 5,
    LANE_MARK_TYPE_ROAD_EDGE = 6
    };
 *
 *[Output]
 *  0, dash; 1, solid; -2, no result; -1, the buff initial value
**/
int CvtMsgLaneType2FilterType(const ad_msg::LaneMarkCamera::LaneMarkType& ori_lane_type_res ) {
    int filter_lane_res = -1;
    printf("[##Lane type res]%d\n", ori_lane_type_res);
    switch (ori_lane_type_res) {
     case ad_msg::LaneMarkCamera::LANE_MARK_TYPE_INVALID:
        filter_lane_res = -2;
        break;
     case ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED:
     case ad_msg::LaneMarkCamera::LANE_MARK_TYPE_BOTTS_DOTS:
        filter_lane_res = 0;
        break;
     case ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID:
     case ad_msg::LaneMarkCamera::LANE_MARK_TYPE_ROAD_EDGE:
     case ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DOUBLE_LANE_MARK:
        filter_lane_res = 1;
        break;
     default:
        filter_lane_res = -2;
        break;
     }

    return filter_lane_res;
}

//Convert the lane type of msg to filter type
/*[Input]
 *  ori_lane_type_res,  lane mark type.
 *  0, dash; 1, solid; 2, no result; -1, the buff initial value
 *
 *[Output]
 *  enum LaneMarkType {
    LANE_MARK_TYPE_INVALID = 0,
    LANE_MARK_TYPE_UNKNOWN = 1,
    LANE_MARK_TYPE_DASHED = 2,
    LANE_MARK_TYPE_SOLID = 3,
    LANE_MARK_TYPE_DOUBLE_LANE_MARK = 4,
    LANE_MARK_TYPE_BOTTS_DOTS = 5,
    LANE_MARK_TYPE_ROAD_EDGE = 6
  };
**/
ad_msg::LaneMarkCamera::LaneMarkType CvtFilterType2MsgLaneType(const int ori_lane_type_res ) {
    ad_msg::LaneMarkCamera::LaneMarkType filter_lane_res = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_INVALID;

    //0, dash; 1, solid; 2, no result; -1, the buff initial value
    switch (ori_lane_type_res) {
     case -2:
        filter_lane_res = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_INVALID;
        break;
     case 0:
        filter_lane_res = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
        break;
     case 1:
        filter_lane_res =  ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
        break;
     default:
        filter_lane_res = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_UNKNOWN;
        break;
     }

    return filter_lane_res;
}

#endif


TaskRecvMaxieyeData::TaskRecvMaxieyeData(framework::Task *manager)
  : framework::Task(
      framework::TASK_ID_RECV_MAXIEYE_DATA, "Recv Maxieye Data", manager) {
  running_flag_recv_ = false;

  can_channel_ = Nullptr_t;

#if (ENABLE_CAN_DEV_ZLGCANNET)
  can_channel_ = new can_dev::CanDriverZlgCanNetFd();
#endif

  veh_model::VehicleModelWrapper veh_model;
  const veh_model::SensorOnVehicle& sensor_param = veh_model.GetSensor();

  common::Matrix<Float32_t, 2, 1> rotate_center;
  rotate_center.SetZeros();
  maxieye_mat_calibration_.SetIdentity();
  visual_control_mat_calibration_.SetIdentity();

  // 前视一体机标定参数
  const veh_model::SensorOnVehicle::Calibration& maxieye_calibration =
      sensor_param.calibration_cam[0];
  common::Rotate_2D<Float32_t>(
        rotate_center, maxieye_calibration.yaw_offset, &maxieye_mat_calibration_);
  common::Translate_2D(
        maxieye_calibration.x_offset, maxieye_calibration.y_offset, &maxieye_mat_calibration_);

  // 环视相机标定参数
  const veh_model::SensorOnVehicle::Calibration& visual_control_calibration =
      sensor_param.calibration_cam[1];
  common::Rotate_2D<Float32_t>(
        rotate_center, visual_control_calibration.yaw_offset, &visual_control_mat_calibration_);
  common::Translate_2D(
        visual_control_calibration.x_offset, visual_control_calibration.y_offset, &visual_control_mat_calibration_);

  // 前视一体机
  lane_list_ready_ = false;
  lane_list_.Clear();
  lane_curb_list_ready_ = false;
  lane_curb_list_.Clear();

  traffic_signal_ready_ = false;
  traffic_signal_list_.Clear();

  obj_list_ready_ = false;
  obj_list_.Clear();

  // 视觉控制器
  visual_control_lane_list_ready_ = false;
  visual_control_lane_list_.Clear();

  front_obj_list_ready_ = false;
  front_obj_list_.Clear();

  front_left_obj_list_ready_ = false;
  front_left_obj_list_.Clear();

  front_right_obj_list_ready_ = false;
  front_right_obj_list_.Clear();

  rear_left_obj_list_ready_ = false;
  rear_left_obj_list_.Clear();

  rear_right_obj_list_ready_ = false;
  rear_right_obj_list_.Clear();

  rear_obj_list_ready_ = false;
  rear_obj_list_.Clear();
}

TaskRecvMaxieyeData::~TaskRecvMaxieyeData() {
  Stop();

  if (Nullptr_t != can_channel_) {
    delete can_channel_;
  }
}

bool TaskRecvMaxieyeData::Start() {
  if (Nullptr_t == can_channel_) {
    LOG_ERR << "Invalid Can channel.";
    return false;
  }

  if (running_flag_recv_) {
    return (true);
  }

  can_dev::CanChannelParam can_channel_param;
  can_channel_param.channel = 0;
  can_channel_param.bit_rate = can_dev::CAN_BIT_RATE_500K;
  com_snprintf(can_channel_param.can_net.ip_addr, 30, "192.168.1.178");
  can_channel_param.can_net.port = 8000;

  if (!can_channel_->OpenChannel(can_channel_param)) {
    LOG_ERR << "Falied to open CAN channel " << can_channel_param.channel;
    return false;
  }

  // 前视一体机
  lane_list_ready_ = false;
  lane_list_.Clear();
  lane_curb_list_ready_ = false;
  lane_curb_list_.Clear();

  traffic_signal_ready_ = false;
  traffic_signal_list_.Clear();

  obj_list_ready_ = false;
  obj_list_.Clear();

  // 视觉控制器
  visual_control_lane_list_ready_ = false;
  visual_control_lane_list_.Clear();

  front_obj_list_ready_ = false;
  front_obj_list_.Clear();

  front_left_obj_list_ready_ = false;
  front_left_obj_list_.Clear();

  front_right_obj_list_ready_ = false;
  front_right_obj_list_.Clear();

  rear_left_obj_list_ready_ = false;
  rear_left_obj_list_.Clear();

  rear_right_obj_list_ready_ = false;
  rear_right_obj_list_.Clear();

  rear_obj_list_ready_ = false;
  rear_obj_list_.Clear();

  LOG_INFO(3) << "Create thread of receiving maxieye data...";
  running_flag_recv_ = true;
  thread_recv_ = boost::thread(boost::bind(&TaskRecvMaxieyeData::ThreadReceiving, this));

  LOG_INFO(3) << "Create thread of receiving maxieye data... [OK]";

  return (true);
}

bool TaskRecvMaxieyeData::Stop() {
  if (running_flag_recv_) {
    running_flag_recv_ = false;

    LOG_INFO(3) << "Stop thread of receiving maxieye data ...";
    bool ret = thread_recv_.timed_join(boost::posix_time::seconds(1));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread of receiving maxieye data to stop.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop thread of receiving maxieye data ... [OK]";
    } else {
      LOG_INFO(3) << "Stop thread of receiving maxieye data ... [NG]";
    }

    LOG_INFO(3) << "Close CAN Channel.";
    if (Nullptr_t != can_channel_) {
      can_channel_->CloseChannel();
    }
  }

  return (true);
}

void TaskRecvMaxieyeData::ThreadReceiving() {
  LOG_INFO(3) << "Thread of receiving maxieye data ... [Started]";

  Int32_t frame_num = 0;
  can_dev::CanFdFrame can_frame[18];

  while (running_flag_recv_) {
    frame_num = can_channel_->ReadCanFdWait(&(can_frame[0]), 18, 50);

    if (frame_num > 0) {
      for (Int32_t i = 0; i < frame_num; ++i) {
        const can_dev::CanFdFrame& fm = can_frame[i];
        // 填充原始数据
        FullCanFdFrame(fm);

        #if 0
        //if(fm.id == 0x19D01AE8) {
        //if(fm.id == 0x19D000E8 || fm.id == 0x19D239E8) {
        //if(fm.id => 0x19D001E8 || fm.id <= 0x10D008E8) {
        //if(fm.id == 0x19D00E41 || fm.id == 0x19D00F41) {
        //if(fm.id == 0x19D01041 || fm.id == 0x19D01141) {
        //if(fm.id == 0x19D01241 || fm.id == 0x19D01341) {
        //if(fm.id == 0x19D01441 || fm.id == 0x19D01541) {
        //if(fm.id == 0x19D01641 || fm.id == 0x19D01741) {
        //if(fm.id == 0x19D01841 || fm.id == 0x19D01941) {
          //printf("Front Camera: id[%08X] data\n", fm.id);
          printf("Camera: id[%08X] data\n", fm.id);
          //printf("Rear Left Camera: id[%08X] data\n", fm.id);
          //printf("Rear Right Camera: id[%08X] data\n", fm.id);
          //printf("Rear Camera: id[%08X] data\n", fm.id);
          for(int i = 0; i< 64; i= i + 8){
            printf("data[%02X %02X %02X %02X %02X %02X %02X %02X]\n",
                  fm.data[i], fm.data[i+1], fm.data[i+2], fm.data[i+3],
                fm.data[i+4], fm.data[i+5], fm.data[i+6], fm.data[i+7]);
          }
        //}
        #endif
        
        ParseCanFrame(fm);
      }

      // 发送原始CanFdFrame数据
      framework::MessageCanFdFrameList message(framework::MSG_ID_RECV_CAMERA_CANFD_FRAME, &canfd_frame_list);
      Notify(message);

      canfd_frame_list.Clear();

      // 前视一体机
      if (lane_list_ready_) {

          LOG_INFO(3) << "### Notify Lane Mark Camera List.";

        //Add the ego lane type filter
        //Filter the lane type
#if Enable_Lane_Type_Filter_Flag
        const static int max_obs_thrsh_size = 180;
        const static int buff_size = 7;
        static int left_cur_obs_cnt = 0;
        static int right_cur_obs_cnt = 0;
        static std::deque<int> left_lane_type_buff = CreateBuff(buff_size);        // It need to initilize the buff.
        static std::deque<int> right_lane_type_buff = CreateBuff(buff_size);

        #if Enalbe_Save_Lane_Type_Res
            static int64_t frame_cnts = 0;
            static std::ofstream lane_out_file;
            if (!lane_out_file.is_open()){
                lane_out_file.open("lane_type.txt");
                lane_out_file << "frame ID, ori cur left lane, filter cur left lane, ori cur right lane, filter cur right lane " << std::endl;
            }
        #endif

        int tmp_ori_cur_left_lane_res = -2;
        int tmp_ori_cur_right_lane_res = -2;
        int tmp_filter_cur_left_lane_res = -2;
        int tmp_filter_cur_right_lane_res = -2;

        printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");

        for (uint i = 0; i < lane_list_.lane_mark_list_.lane_mark_num; ++i) {

            int cur_lane_id = lane_list_.lane_mark_list_.lane_marks[i].id;

                if (cur_lane_id == 1){
                  printf("#########[Left Ori-lane]lane type: %d; ", lane_list_.lane_mark_list_.lane_marks[i].lane_mark_type);
                  tmp_ori_cur_left_lane_res = lane_list_.lane_mark_list_.lane_marks[i].lane_mark_type;

                  int filter_lane_tyep_res = FilterCurLaneType(CvtMsgLaneType2FilterType(lane_list_.lane_mark_list_.lane_marks[i].lane_mark_type), left_lane_type_buff,
                              left_cur_obs_cnt, buff_size, max_obs_thrsh_size);

                  lane_list_.lane_mark_list_.lane_marks[i].lane_mark_type = CvtFilterType2MsgLaneType(filter_lane_tyep_res);
                  printf("[Left Filter-lane]lane type: %d\n", lane_list_.lane_mark_list_.lane_marks[i].lane_mark_type);

                  tmp_filter_cur_left_lane_res = lane_list_.lane_mark_list_.lane_marks[i].lane_mark_type;
               }
               if (cur_lane_id == -1) {
                  printf("#########[Right Ori-lane]lane type: %d; ", lane_list_.lane_mark_list_.lane_marks[i].lane_mark_type);

                  tmp_ori_cur_right_lane_res = lane_list_.lane_mark_list_.lane_marks[i].lane_mark_type;

                  int filter_lane_tyep_res = FilterCurLaneType(CvtMsgLaneType2FilterType(lane_list_.lane_mark_list_.lane_marks[i].lane_mark_type), right_lane_type_buff,
                              right_cur_obs_cnt, buff_size, max_obs_thrsh_size);
                  lane_list_.lane_mark_list_.lane_marks[i].lane_mark_type = CvtFilterType2MsgLaneType(filter_lane_tyep_res);
                  printf("[Right Filter-lane]lane type: %d\n", lane_list_.lane_mark_list_.lane_marks[i].lane_mark_type);

                  tmp_filter_cur_right_lane_res = lane_list_.lane_mark_list_.lane_marks[i].lane_mark_type;
               }
        }

        printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");

       #if Enalbe_Save_Lane_Type_Res
                  lane_out_file << frame_cnts << ","
                                << tmp_ori_cur_left_lane_res << ", "
                                << tmp_filter_cur_left_lane_res << ", "
                                << tmp_ori_cur_right_lane_res << ", "
                                << tmp_filter_cur_right_lane_res
                                << std::endl;

                  ++frame_cnts;
       #endif

      //  enum LaneMarkType {
      //    LANE_MARK_TYPE_INVALID = 0,
      //    LANE_MARK_TYPE_UNKNOWN = 1,
      //    LANE_MARK_TYPE_DASHED = 2,
      //    LANE_MARK_TYPE_SOLID = 3,
      //    LANE_MARK_TYPE_DOUBLE_LANE_MARK = 4,
      //    LANE_MARK_TYPE_BOTTS_DOTS = 5,
      //    LANE_MARK_TYPE_ROAD_EDGE = 6

#endif


        framework::MessageLaneMarkCameraList message(&lane_list_.GetLaneList());
        Notify(message);

        lane_list_ready_ = false;
        lane_list_.Clear();
      }

      // 前视一体机路沿
      if(lane_curb_list_ready_){
        LOG_INFO(3) << "### Notify Lane Curb Camera List.";
        framework::MessageLaneCurbCameraList message(&lane_curb_list_.GetLaneList());
        Notify(message);
        lane_curb_list_ready_ = false;
        lane_curb_list_.Clear();
      }

      if (traffic_signal_ready_) {
        LOG_INFO(3) << "### Notify Traffic Signal Camera List.";
        framework::MessageTrafficSignalList message(&traffic_signal_list_.GetTraSignList());
        Notify(message);

        traffic_signal_ready_ = false;
        traffic_signal_list_.Clear();
      }

      NotifyObjList(obj_list_ready_, 8);

      // 视觉控制器
      if (visual_control_lane_list_ready_) {
        LOG_INFO(3) << "### Notify Visual Control Lane Mark Camera List.";
        framework::MessageVisualControlLaneMarkCameraList message(&visual_control_lane_list_.GetLaneList());
        Notify(message);

        visual_control_lane_list_ready_ = false;
        visual_control_lane_list_.Clear();
      }
      NotifyObjList(front_obj_list_ready_, 0);
      NotifyObjList(front_left_obj_list_ready_, 1);
      NotifyObjList(front_right_obj_list_ready_, 2);
      NotifyObjList(rear_left_obj_list_ready_, 3);
      NotifyObjList(rear_right_obj_list_ready_, 4);
      NotifyObjList(rear_obj_list_ready_, 5);
    }
  }

  LOG_INFO(3) << "Thread of receiving mobileye data ... [Stopped]";
}

void TaskRecvMaxieyeData::NotifyObjList(bool obj_list_ready, Uint8_t camera_symbol) {
  if (obj_list_ready) {
    ad_msg::ObstacleCameraList *obstacle_list = NULL;
    switch(camera_symbol) {
      case (8):
        obstacle_list = &obj_list_.obstacle_list_;
        obstacle_list->cam_type = ad_msg::ObstacleCameraList::CAM_TYPE_MAXIEYE_D500;
        break;
      case (0):
        obstacle_list = &front_obj_list_.obstacle_list_;
        obstacle_list->cam_type = ad_msg::ObstacleCameraList::CAM_TYPE_VISUAL_CONTROL_FRONT;
        break;
      case (1):
        obstacle_list = &front_left_obj_list_.obstacle_list_;
        obstacle_list->cam_type = ad_msg::ObstacleCameraList::CAM_TYPE_VISUAL_CONTROL_FRONT_LEFT;
        break;
      case (2):
        obstacle_list = &front_right_obj_list_.obstacle_list_;
        obstacle_list->cam_type = ad_msg::ObstacleCameraList::CAM_TYPE_VISUAL_CONTROL_FRONT_RIGHT;
        break;
      case (3):
        obstacle_list = &rear_left_obj_list_.obstacle_list_;
        obstacle_list->cam_type = ad_msg::ObstacleCameraList::CAM_TYPE_VISUAL_CONTROL_REAR_LEFT;
        break;
      case (4):
        obstacle_list = &rear_right_obj_list_.obstacle_list_;
        obstacle_list->cam_type = ad_msg::ObstacleCameraList::CAM_TYPE_VISUAL_CONTROL_REAR_RIGHT;
        break;
      case (5):
        obstacle_list = &rear_obj_list_.obstacle_list_;
        obstacle_list->cam_type = ad_msg::ObstacleCameraList::CAM_TYPE_VISUAL_CONTROL_REAR;
        break;
      default:
        break;
    }

    //obstacle_list.cam_type = ad_msg::ObstacleCameraList::CAM_TYPE_VISUAL_CONTROL;
    // Convert to vehicle coordinate.
    framework::SharedData::instance()->GetChassis(&chassis_);
    common::Matrix<Float32_t, 2, 1> point_conv;
    for (Int32_t j = 0; j < obstacle_list->obstacle_num; ++j) {
      ad_msg::ObstacleCamera& obj = obstacle_list->obstacles[j];

      if (camera_symbol == 8) {
        point_conv(0) = obj.x;
        point_conv(1) = obj.y;
        common::TransformVert_2D(maxieye_mat_calibration_, &point_conv);
        obj.x = point_conv(0);
        obj.y = point_conv(1);
      } else {
        point_conv(0) = obj.x;
        point_conv(1) = obj.y;
        common::TransformVert_2D(visual_control_mat_calibration_, &point_conv);
        obj.x = point_conv(0);
        obj.y = point_conv(1);
      }

      // 锥形桶的速度默认为0，不需要加车身速度
      if ((camera_symbol == 8) && (obj.type == ad_msg::OBJ_TYPE_TRAFFIC_CONE)) {
        continue;
      }

      obj.v_x += chassis_.v;
    }

    if (8 == camera_symbol) {
        LOG_INFO(3) << "### Notify Obstacle Camera List (num="
                    << obstacle_list->obstacle_num << ").";
        framework::MessageObstacleCameraList message(
                framework::MSG_ID_RECV_MAXIRYR_CAMERA_OBJECTS_FRONT, &obj_list_.GetObjList());
        Notify(message);

        obj_list_ready_ = false;
        obj_list_.Clear();

    } else if (0 == camera_symbol) {
        LOG_INFO(3) << "### Notify Obstacle Front Camera List (num="
                    << obstacle_list->obstacle_num << ").";
        framework::MessageObstacleCameraList message(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT, &front_obj_list_.GetObjList());
        Notify(message);

        front_obj_list_ready_ = false;
        front_obj_list_.Clear();

    } else if (1 == camera_symbol) {
        LOG_INFO(3) << "### Notify Obstacle Front Left Camera List (num="
                    << obstacle_list->obstacle_num << ").";
        framework::MessageObstacleCameraList message(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_LEFT, &front_left_obj_list_.GetObjList());
        Notify(message);

        front_left_obj_list_ready_ = false;
        front_left_obj_list_.Clear();

    } else if (2 == camera_symbol) {
        LOG_INFO(3) << "### Notify Obstacle Front Right Camera List (num="
                    << obstacle_list->obstacle_num << ").";
        framework::MessageObstacleCameraList message(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_RIGHT, &front_right_obj_list_.GetObjList());
        Notify(message);

        front_right_obj_list_ready_ = false;
        front_right_obj_list_.Clear();

    } else if (3 == camera_symbol) {
        LOG_INFO(3) << "### Notify Obstacle Rear Left Camera List (num="
                    << obstacle_list->obstacle_num << ").";
        framework::MessageObstacleCameraList message(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_LEFT, &rear_left_obj_list_.GetObjList());
        Notify(message);

        rear_left_obj_list_ready_ = false;
        rear_left_obj_list_.Clear();

    } else if (4 == camera_symbol) {
        LOG_INFO(3) << "### Notify Obstacle Rear Right Camera List (num="
                    << obstacle_list->obstacle_num << ").";
        framework::MessageObstacleCameraList message(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_RIGHT, &rear_right_obj_list_.GetObjList());
        Notify(message);

        rear_right_obj_list_ready_ = false;
        rear_right_obj_list_.Clear();

    } else if (5 == camera_symbol) {
        LOG_INFO(3) << "### Notify Obstacle Rear Camera List (num="
                    << obstacle_list->obstacle_num << ").";
        framework::MessageObstacleCameraList message(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR, &rear_obj_list_.GetObjList());
        Notify(message);

        rear_obj_list_ready_ = false;
        rear_obj_list_.Clear();

    }
  }
}

void TaskRecvMaxieyeData::FullCanFdFrame(const can_dev::CanFdFrame& frame) {
  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].time_stamp = frame.time_stamp;

  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].id = frame.id;
  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].FD = frame.FD;
  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].RTR = frame.RTR;
  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].EXT = frame.EXT;
  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].BRS = frame.BRS;
  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].ESI = frame.ESI;
  canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].data_len = frame.data_len;
  for(Uint8_t i = 0; i < frame.data_len; i++) {
    canfd_frame_list.canfd_frame[canfd_frame_list.canfd_frame_num].data[i] = frame.data[i];
  }

  canfd_frame_list.canfd_frame_num++;
}

void TaskRecvMaxieyeData::ParseCanFrame(const can_dev::CanFdFrame &frame) {
  // 车道线数据
  static phoenix::can_dev::CanFdFrame lane_canfd_frame;
  phoenix::framework::SharedData* shared_data = phoenix::framework::SharedData::instance();
  switch (frame.id) {
  // Lanes
  // 车道线/道路边线（最多输出三车道（4条）车道线/道路边线，
  // 当车辆所在车道同时存在车道线和道路边缘时，只输出车道线的信息，
  // 若只有道路边缘则输出道路边缘信息）
  case (0x19D000E8):
    lane_list_ready_ = false;
    lane_list_.Clear();
    //获取车道线曲率参数(c0, c1, c2, c3)
    memcpy(&lane_canfd_frame, &frame, sizeof(phoenix::can_dev::CanFdFrame)); 
    ParseLaneFromCanFrame(frame);
    //lane_list_ready_ = true;
    break;
  case (0x19D239E8):
    //获取车道线的长度(range),将车道线的长度填充到车道线报文00e8空闲的字节中
    for(Uint8_t i = 0; i < phoenix::ad_msg::LaneMarkCameraList::MAX_LANE_MARK_NUM; i++) {
      lane_canfd_frame.data[14 + i * 16] = frame.data[1 + i * 3];
      lane_canfd_frame.data[15 + i * 16] = frame.data[2 + i * 3];
    }
    shared_data->SetLaneCanFdData(lane_canfd_frame);
    ParseLaneLengthFromCanFrame(frame);
    lane_list_ready_ = true;
    break;

  // Obstacles
  // 最多16个目标，每条Message放2个目标，需要分配8个ID
  case (0x19D001E8):
    obj_list_ready_ = false;
    obj_list_.Clear();
    ParseMaxieyeObjFromCanFrame(frame);
    break;
  case (0x19D002E8):
    ParseMaxieyeObjFromCanFrame(frame);
    break;
  case (0x19D003E8):
    ParseMaxieyeObjFromCanFrame(frame);
    break;
  case (0x19D004E8):
    ParseMaxieyeObjFromCanFrame(frame);
    break;
  case (0x19D005E8):
    ParseMaxieyeObjFromCanFrame(frame);
    break;
  case (0x19D006E8):
    ParseMaxieyeObjFromCanFrame(frame);
    break;
  case (0x19D007E8):
    ParseMaxieyeObjFromCanFrame(frame);
    break;
  case (0x19D008E8):
    ParseMaxieyeObjFromCanFrame(frame);
    break;
  //锥形桶
  case (0x19D01AE8):
    ParseTrafficConeFromCanFrame(frame);
    break;
  case (0x19D00FE8):
    ParseTrafficConeFromCanFrame(frame);
    obj_list_ready_ = true;
    break;

  /// TODO：添加融合障碍物目标 ???
  /// TODO：添加红绿灯
  // 添加限速牌
  case (0x19D00DE8):
    traffic_signal_ready_ = false;
    traffic_signal_list_.Clear();
    ParseTSRFromCanFrame(frame);
    traffic_signal_ready_ = true;
    break;

  //专门针对路沿的报文处理
  case (0x19D196E8):
  case (0x19D195E8):
    lane_curb_list_ready_ = false;
    lane_curb_list_.Clear();
    ParseLaneCurbFromCanFrame(frame);
    lane_curb_list_ready_ = true;
    break;

  // 前摄像头
  // 车道线
  case (0x19D02141):
    visual_control_lane_list_ready_ = false;
    visual_control_lane_list_.Clear();
    ParseVisualControlLaneFromCanFrame(frame);
    visual_control_lane_list_ready_ = true;
    break;
  case (0x19D23A41):
    ParseVisualControlLaneLengthFromCanFrame(frame);
    //visual_control_lane_list_ready_ = true;
    break;

  case (0x19D00E41):
    front_obj_list_ready_ = false;
    front_obj_list_.Clear();
    ParseVisualControlObjFromCanFrame(frame, 0);
    break;
  case (0x19D00F41):
    ParseVisualControlObjFromCanFrame(frame, 0);
    front_obj_list_ready_ = true;
    break;

  // 前左摄像头
  case (0x19D01041):
    front_left_obj_list_ready_ = false;
    front_left_obj_list_.Clear();
    ParseVisualControlObjFromCanFrame(frame, 1);
    break;
  case (0x19D01141):
    ParseVisualControlObjFromCanFrame(frame, 1);
    front_left_obj_list_ready_ = true;
    break;

  // 前右摄像头
  case (0x19D01241):
    front_right_obj_list_ready_ = false;
    front_right_obj_list_.Clear();
    ParseVisualControlObjFromCanFrame(frame, 2);
    break;
  case (0x19D01341):
    ParseVisualControlObjFromCanFrame(frame, 2);
    front_right_obj_list_ready_ = true;
    break;

  // 后左摄像头
  case (0x19D01441):
    rear_left_obj_list_ready_ = false;
    rear_left_obj_list_.Clear();
    ParseVisualControlObjFromCanFrame(frame, 3);
    break;
  case (0x19D01541):
    ParseVisualControlObjFromCanFrame(frame, 3);
    rear_left_obj_list_ready_ = true;
    break;
  
  // 后右摄像头
  case (0x19D01641):
    rear_right_obj_list_ready_ = false;
    rear_right_obj_list_.Clear();
    ParseVisualControlObjFromCanFrame(frame, 4);
    break;
  case (0x19D01741):
    ParseVisualControlObjFromCanFrame(frame, 4);
    rear_right_obj_list_ready_ = true;
    break;
  
  // 后摄像头
  case (0x19D01841):
    rear_obj_list_ready_ = false;
    rear_obj_list_.Clear();
    ParseVisualControlObjFromCanFrame(frame, 5);
    break;
  case (0x19D01941):
    ParseVisualControlObjFromCanFrame(frame, 5);
    rear_obj_list_ready_ = true;
    break;

  default:
    break;
  }
}

//解析锥形桶
void TaskRecvMaxieyeData::ParseTrafficConeFromCanFrame(
    const can_dev::CanFdFrame& frame) {
  // Traffic Cone 1 2
  ParseTwoTrafficConeFromData(&frame.data[0]);
  // Traffic Cone 3 4
  ParseTwoTrafficConeFromData(&frame.data[9]);
  // Traffic Cone 5 6
  ParseTwoTrafficConeFromData(&frame.data[18]);
  // Traffic Cone 7 8
  ParseTwoTrafficConeFromData(&frame.data[27]);
}

void TaskRecvMaxieyeData::ParseTwoTrafficConeFromData(const Uint8_t* data) {
  Uint8_t Traffic_Cone_ID[2];
  Float32_t Traffic_Cone_DistLgt[2];
  Float32_t Traffic_Cone_DistLat[2];

  // 锥形桶1
  // 目标ID, Factor:1 ｜ Offset: 0 ∣ Range:0 ～ 15
  Traffic_Cone_ID[0] = parse_sensor_data(data, 0, 0, 8);
  // 纵向距离, Factor:0.01 ｜ Offset: -80 ∣ Range:-80 ～ 80
  Traffic_Cone_DistLgt[0] = parse_sensor_data(data, 1, 8, 14) * 0.01 + (-80);
  // 横向距离, Factor:0.01 ｜ Offset: -80 ∣ Range:-80 ～ 80
  Traffic_Cone_DistLat[0] = parse_sensor_data(data, 2, 22, 14) * 0.01 + (-80);

  // 锥形桶2
  // 目标ID, Factor:1 ｜ Offset: 0 ∣ Range:0 ～ 15
  Traffic_Cone_ID[1] = parse_sensor_data(data, 4, 36, 8);
  // 纵向距离, Factor:0.01 ｜ Offset: -80 ∣ Range:-80 ～ 80
  Traffic_Cone_DistLgt[1] = parse_sensor_data(data, 5, 44, 14) * 0.01 + (-80);
  // 横向距离, Factor:0.01 ｜ Offset: -80 ∣ Range:-80 ～ 80
  Traffic_Cone_DistLat[1] = parse_sensor_data(data, 7, 58, 14) * 0.01 + (-80);


  for (Uint8_t i = 0; i < 2; i++) {    
    // 对锥形桶进行过滤
    if ((parse_sensor_data(data, 1, 8, 14) == 0) && (parse_sensor_data(data, 2, 22, 14) == 0)) {
      continue;
    }

    if ((parse_sensor_data(data, 5, 44, 14) == 0) && (parse_sensor_data(data, 7, 58, 14) == 0)) {
      continue;
    }

    if (Traffic_Cone_DistLgt[i] < 0.0F) {
      continue;
    }

    // Get back buffer from list
    ad_msg::ObstacleCamera* obj = obj_list_.Back();
    if (Nullptr_t == obj) {
      LOG_ERR << "Obj list is full.";
      return;
    }

    // 障碍物ID
    // 为了避免锥形桶和障碍物的id相同，所以在锥形桶的id基础上加18(障碍物id的最大值为17(通过前视一体机数据表格得到))
    obj->id = Traffic_Cone_ID[i] + 18;
    // 障碍物类型
    obj->type = ad_msg::OBJ_TYPE_TRAFFIC_CONE;
    // 障碍物的状态
    // 目标运动状态, 0x0:目标静止；0x1:目标运动
    obj->status = ad_msg::ObstacleCamera::OBJ_STATUS_STOPPED;
    // 障碍物长度 (m)
    obj->length = 0.37;
    // 障碍宽度 (m)
    obj->width = 0.37;
    // 障碍物高度（m）
    obj->height = 0.7;
    // 障碍物位置x坐标 (m)
    obj->x = Traffic_Cone_DistLgt[i];
    // 障碍物位置y坐标 (m)
    obj->y = Traffic_Cone_DistLat[i];

    // Push back to list
    obj_list_.PushBack();
  }
}

//解析路沿
void TaskRecvMaxieyeData::ParseLaneCurbFromCanFrame(
    const can_dev::CanFdFrame& frame) {
  // lane curb 01
  ParseOneLaneCurbFromData(&frame.data[0]);
  // lane curb 02
  ParseOneLaneCurbFromData(&frame.data[16]);
}

void TaskRecvMaxieyeData::ParseOneLaneCurbFromData(const Uint8_t* data) {
  // ID, 0x1:左一；0x2:右一；0x3:左二；0x4：右二；
  Uint8_t lane_id = data[0] & 0x07;
  // 类型（包括车道线和道路边线）, 0x0-Background (BK), 0x1- Single Solid White (SSW)
  // 0x2- Single Dashed White (SDW), 0x3- Single Solid Yellow (SSY)
  // 0x4- Single Dashed Yellow (SDY), 0x5- Double Solid Yellow (DSY)
  // 0x6- Dashed Solid (DS), 0x7- Solid Dashed (SD), 0x8- Deceleration Strip (DEC)
  // 0x9- Bold (BO), 0xA- RoadEdge, 0xB- Bott's Dots, 0xC-Barrier, 0xF - Invalid
  Uint8_t lane_type = (data[0] >> 3) & 0x1F;
  // 弯道半径, Factor:1 ｜ Offset:0 ∣ Range:0~65535, 直道设置弯道半径为5000m，无效值FFFF
  Uint32_t lane_curve_radius = ((Uint32_t)data[2] << 8) | (Uint32_t)data[1];
  // 横向位置/曲率参数C0, Factor:0.00390625 ｜ Offset: -128 ∣ Range:-128 ~ 128m
  Float64_t lane_curvature_0 =
      (((Uint32_t)data[4] << 8) | (Uint32_t)data[3]) * 0.00390625 - 128.0;
  // 航向角, Factor:0.1 ｜ Offset: -90 ∣ Range:-90~90°
  Float32_t lane_heading_angle =
      ((((Uint32_t)data[6] & 0x0F) << 8) | (Uint32_t)data[5]) * 0.1F - 90.0F;
  // 置信度, 0x0:不可信；0x1:＜25%；0x2:＜50%；0x3:＜75%；0x4：＜90%；0x5:＜99%；0x6:＜99.9%；0x7:≤100%
  // 现在只有：0x0:不可信；0x3:＜75%；0x7:≤100%
  Uint8_t lane_prob_of_exist = data[7] & 0x07;
  // 弯道方向, 0x0:直道；0x1:左偏；0x2:右偏；无效FF
  Uint8_t lane_curv_diret = (data[7] >> 3) & 0x03;
  // 曲率参数C1, Factor:1.08949*10^5 ｜ Offset: -0.357 ∣ Range:-0.357~0.357rad
  Float64_t lane_curvature_1 =
      (((Uint32_t)data[9] << 8) | (Uint32_t)data[8]) * 0.0000108949 - 0.357;
  // 曲率参数C2, Factor:9.76563*10^7 ｜ Offset: -0.031999 ∣ Range:-0.031999~0.032(1/m)
  Float64_t lane_curvature_2 =
      (((Uint32_t)data[11] << 8) | (Uint32_t)data[10]) * 0.000000976563 - 0.031999;
  // 曲率参数C3, Factor:3.72529*10^9 ｜ Offset: -0.000122 ∣ Range:-0.0001221~0.0001221(1/m^2)
  Float64_t lane_curvature_3 =
      (((Uint32_t)data[13] << 8) | (Uint32_t)data[12]) * 0.00000000372529 - 0.000122;

  // 车道线识别的质量(0,1 – low quality.The lane measurements are not valid
  // in low quality. The system will not give an LDW in that situation.
  // 2,3 – high quality)
  Int32_t quality;
  // 置信度, 0x0:不可信；0x1:＜25%；0x2:＜50%；0x3:＜75%；0x4：＜90%；
  // 0x5:＜99%；0x6:＜99.9%；0x7:≤100%
  switch (lane_prob_of_exist) {
  case (0):
    quality = 0;
    break;
  case (1):
    quality = 0;
    break;
  case (2):
    quality = 1;
    break;
  case (3):
    quality = 2;
    break;
  case (4):
    quality = 2;
    break;
  case (5):
    quality = 3;
    break;
  case (6):
    quality = 3;
    break;
  case (7):
    quality = 3;
    break;
  default:
    quality = 0;
    break;
  }
  if (lane_id < 1 || lane_id > 4) {
    quality = 0;
  }
  if ((lane_curvature_0 < -128.0) || (lane_curvature_0 > 128.0)) {
    quality = 0;
  }
  if ((lane_curvature_1 < -0.3571) || (lane_curvature_1 > 0.3571)) {
    quality = 0;
  }
  if ((lane_curvature_2 < -0.0321) || (lane_curvature_2 > 0.0321)) {
    quality = 0;
  }
  if ((lane_curvature_3 < -0.00012211) || (lane_curvature_3 > 0.00012211)) {
    quality = 0;
  }

  #if (LANE_LOG_FLAG == 1)
  printf("----------Lane Curb----------------------------\n");
  printf("lane_prob_of_exist:%d\n", lane_prob_of_exist);
  printf("lane_id:%d\n", lane_id);
  printf("lane_curvature_0:%f\n", lane_curvature_0);
  printf("lane_curvature_1:%f\n", lane_curvature_1);
  printf("lane_curvature_2:%f\n", lane_curvature_2);
  printf("lane_curvature_3:%f\n", lane_curvature_3);
  printf("quality:%d\n", quality);
  printf("--------------------------------------\n");
  #endif

  // Get back buffer from list
  ad_msg::LaneMarkCamera* lane = lane_curb_list_.Back();
  if (Nullptr_t == lane) {
    LOG_ERR << "Lane list is full.";
    return;
  }

  // 车道编号，左边车道线为正值，右边车道线为负数;
  // (数值的绝对值按照距离当前车道的远近依次增加，
  // 第一条左边的车道线为1，第一条右边的车道线为-1)
  switch (lane_id) {
  // ID, 0x1:左一；0x2:右一；0x3:左二；0x4：右二；
  case (1):
    lane->id = 1;
    break;
  case (2):
    lane->id = -1;
    break;
  case (3):
    lane->id = 2;
    break;
  case (4):
    lane->id = -2;
    break;
  default:
    lane->id = 99;
    break;
  }
  // 车道线类型
  switch (lane_type) {
  // 类型（包括车道线和道路边线）, 0x0-Background (BK), 0x1- Single Solid White (SSW)
  // 0x2- Single Dashed White (SDW), 0x3- Single Solid Yellow (SSY)
  // 0x4- Single Dashed Yellow (SDY), 0x5- Double Solid Yellow (DSY)
  // 0x6- Dashed Solid (DS), 0x7- Solid Dashed (SD), 0x8- Deceleration Strip (DEC)
  // 0x9- Bold (BO), 0xA- RoadEdge, 0xB- Bott's Dots, 0xC-Barrier, 0xF - Invalid
  case (0):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_UNKNOWN;
    break;
  case (1):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (2):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    break;
  case (3):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (4):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    break;
  case (5):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DOUBLE_LANE_MARK;
    break;
  case (6):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    break;
  case (7):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (8):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (9):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (10):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_ROAD_EDGE;
    break;
  case (11):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_BOTTS_DOTS;
    break;
  case (12):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_ROAD_EDGE;
    break;
  default:
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_INVALID;
    break;
  }
  // 车道线识别的质量(0,1 – low quality.The lane measurements are not valid
  // in low quality. The system will not give an LDW in that situation.
  // 2,3 – high quality)
  lane->quality = quality;
  // 车道线的纵向有效范围字段中的值是否有效
  lane->view_range_valid = 1;  // Dummy
  // 车道线的宽度
  lane->mark_width = 0.0F;  // Dummy
  /// TODO: 确认车道线的有效识别长度 !!!
  // 车道线的纵向有效范围的起始位置
  lane->view_range_start = 0.0F;  // Dummy
  // 车道线的纵向有效范围的结束位置
  lane->view_range_end = 60.0F;
  // 曲线参数c0
  lane->c0 = lane_curvature_0;
  // 曲线参数c1
  lane->c1 = lane_curvature_1;
  // 曲线参数c2
  lane->c2 = lane_curvature_2;
  // 曲线参数c3
  lane->c3 = lane_curvature_3;
  // 备注：
  // 车道线使用三阶多项式曲线定义 y = c0 + c1 * x + c2 * x^2 + c3 * x^3
  // 使用右手迪卡尔坐标系，x为车身纵轴方向，向前为正方向，y为车身横轴方向，向左为正方向;
  // 角度以x轴正方向为0度，逆时针方向为正。

  // Push back to list
  lane_curb_list_.PushBack();
}


void TaskRecvMaxieyeData::ParseLaneFromCanFrame(
    const can_dev::CanFdFrame& frame) {
  // lane 01
  ParseOneLaneFromData(&frame.data[0]);
  // lane 02
  ParseOneLaneFromData(&frame.data[16]);
  // lane 03
  ParseOneLaneFromData(&frame.data[32]);
  // lane 04
  ParseOneLaneFromData(&frame.data[48]);
}

void TaskRecvMaxieyeData::ParseOneLaneFromData(const Uint8_t* data) {
  // ID, 0x1:左一；0x2:右一；0x3:左二；0x4：右二；
  Uint8_t lane_id = data[0] & 0x07;
  // 类型（包括车道线和道路边线）, 0x0-Background (BK), 0x1- Single Solid White (SSW)
  // 0x2- Single Dashed White (SDW), 0x3- Single Solid Yellow (SSY)
  // 0x4- Single Dashed Yellow (SDY), 0x5- Double Solid Yellow (DSY)
  // 0x6- Dashed Solid (DS), 0x7- Solid Dashed (SD), 0x8- Deceleration Strip (DEC)
  // 0x9- Bold (BO), 0xA- RoadEdge, 0xB- Bott's Dots, 0xC-Barrier, 0xF - Invalid
  Uint8_t lane_type = (data[0] >> 3) & 0x1F;
  // 弯道半径, Factor:1 ｜ Offset:0 ∣ Range:0~65535, 直道设置弯道半径为5000m，无效值FFFF
  Uint32_t lane_curve_radius = ((Uint32_t)data[2] << 8) | (Uint32_t)data[1];
  // 横向位置/曲率参数C0, Factor:0.00390625 ｜ Offset: -128 ∣ Range:-128 ~ 128m
  Float64_t lane_curvature_0 =
      (((Uint32_t)data[4] << 8) | (Uint32_t)data[3]) * 0.00390625 - 128.0;
  // 航向角, Factor:0.1 ｜ Offset: -90 ∣ Range:-90~90°
  Float32_t lane_heading_angle =
      ((((Uint32_t)data[6] & 0x0F) << 8) | (Uint32_t)data[5]) * 0.1F - 90.0F;
  // 置信度, 0x0:不可信；0x1:＜25%；0x2:＜50%；0x3:＜75%；0x4：＜90%；0x5:＜99%；0x6:＜99.9%；0x7:≤100%
  // 现在只有：0x0:不可信；0x3:＜75%；0x7:≤100%
  Uint8_t lane_prob_of_exist = data[7] & 0x07;
  // 弯道方向, 0x0:直道；0x1:左偏；0x2:右偏；无效FF
  Uint8_t lane_curv_diret = (data[7] >> 3) & 0x03;
  // 曲率参数C1, Factor:1.08949*10^5 ｜ Offset: -0.357 ∣ Range:-0.357~0.357rad
  Float64_t lane_curvature_1 =
      (((Uint32_t)data[9] << 8) | (Uint32_t)data[8]) * 0.0000108949 - 0.357;
  // 曲率参数C2, Factor:9.76563*10^7 ｜ Offset: -0.031999 ∣ Range:-0.031999~0.032(1/m)
  Float64_t lane_curvature_2 =
      (((Uint32_t)data[11] << 8) | (Uint32_t)data[10]) * 0.000000976563 - 0.031999;
  // 曲率参数C3, Factor:3.72529*10^9 ｜ Offset: -0.000122 ∣ Range:-0.0001221~0.0001221(1/m^2)
  Float64_t lane_curvature_3 =
      (((Uint32_t)data[13] << 8) | (Uint32_t)data[12]) * 0.00000000372529 - 0.000122;

  // 车道线识别的质量(0,1 – low quality.The lane measurements are not valid
  // in low quality. The system will not give an LDW in that situation.
  // 2,3 – high quality)
  Int32_t quality;
  // 置信度, 0x0:不可信；0x1:＜25%；0x2:＜50%；0x3:＜75%；0x4：＜90%；
  // 0x5:＜99%；0x6:＜99.9%；0x7:≤100%
  switch (lane_prob_of_exist) {
  case (0):
    quality = 0;
    break;
  case (1):
    quality = 1;
    break;
  case (2):
    quality = 2;
    break;
  case (3):
    quality = 2;
    break;
  case (4):
    quality = 3;
    break;
  case (5):
    quality = 3;
    break;
  case (6):
    quality = 3;
    break;
  case (7):
    quality = 3;
    break;
  default:
    quality = 0;
    break;
  }
  if (lane_id < 1 || lane_id > 4) {
    quality = 0;
  }
  if ((lane_curvature_0 < -128.0) || (lane_curvature_0 > 128.0)) {
    quality = 0;
  }
  if ((lane_curvature_1 < -0.3571) || (lane_curvature_1 > 0.3571)) {
    quality = 0;
  }
  if ((lane_curvature_2 < -0.0321) || (lane_curvature_2 > 0.0321)) {
    quality = 0;
  }
  if ((lane_curvature_3 < -0.00012211) || (lane_curvature_3 > 0.00012211)) {
    quality = 0;
  }

  #if (LANE_LOG_FLAG == 1)
  printf("-------------Lane-------------------------\n");
  printf("lane_prob_of_exist:%d\n", lane_prob_of_exist);
  printf("lane_id:%d\n", lane_id);
  printf("lane_type:%x\n", lane_type);
  printf("lane_curvature_0:%f\n", lane_curvature_0);
  printf("lane_curvature_1:%f\n", lane_curvature_1);
  printf("lane_curvature_2:%f\n", lane_curvature_2);
  printf("lane_curvature_3:%f\n", lane_curvature_3);
  printf("quality:%d\n", quality);
  printf("--------------------------------------\n");
  #endif

  // Get back buffer from list
  ad_msg::LaneMarkCamera* lane = lane_list_.Back();
  if (Nullptr_t == lane) {
    LOG_ERR << "Lane list is full.";
    return;
  }

  // 车道编号，左边车道线为正值，右边车道线为负数;
  // (数值的绝对值按照距离当前车道的远近依次增加，
  // 第一条左边的车道线为1，第一条右边的车道线为-1)
  switch (lane_id) {
  // ID, 0x1:左一；0x2:右一；0x3:左二；0x4：右二；
  case (1):
    lane->id = 1;
    break;
  case (2):
    lane->id = -1;
    break;
  case (3):
    lane->id = 2;
    break;
  case (4):
    lane->id = -2;
    break;
  default:
    lane->id = 99;
    break;
  }
  // 车道线类型
  switch (lane_type) {
  // 类型（包括车道线和道路边线）, 0x0-Background (BK), 0x1- Single Solid White (SSW,单白实线)
  // 0x2- Single Dashed White (SDW,单白虚线), 0x3- Single Solid Yellow (SSY,单黄实线)
  // 0x4- Single Dashed Yellow (SDY,单黄虚线), 0x5- Double Solid Yellow (DSY,双黄实线)
  // 0x6- Dashed Solid (DS,左虚右实), 0x7- Solid Dashed (SD,左实右虚), 0x8- Deceleration Strip (DEC,减速带)
  // 0x9- Bold (BO), 0xA- RoadEdge (道路边缘), 0xB- Bott's Dots, 0xC-Barrier (障碍), 0xF - Invalid
  case (0):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_UNKNOWN;
    break;
  case (1):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (2):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    break;
  case (3):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (4):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    break;
  case (5):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DOUBLE_LANE_MARK;
    break;
  case (6):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    break;
  case (7):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (8):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (9):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (10):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_ROAD_EDGE;
    break;
  case (11):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_BOTTS_DOTS;
    break;
  case (12):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_ROAD_EDGE;
    break;
  default:
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_INVALID;
    break;
  }
  // 车道线识别的质量(0,1 – low quality.The lane measurements are not valid
  // in low quality. The system will not give an LDW in that situation.
  // 2,3 – high quality)
  lane->quality = quality;
  // 车道线的纵向有效范围字段中的值是否有效
  lane->view_range_valid = 1;  // Dummy
  // 车道线的宽度
  lane->mark_width = 0.0F;  // Dummy
  /// TODO: 确认车道线的有效识别长度 !!!
  // 车道线的纵向有效范围的起始位置
  lane->view_range_start = 0.0F;  // Dummy
  // 车道线的纵向有效范围的结束位置
  lane->view_range_end = 60.0F;
  // 曲线参数c0
  lane->c0 = lane_curvature_0;
  // 曲线参数c1
  lane->c1 = lane_curvature_1;
  // 曲线参数c2
  lane->c2 = lane_curvature_2;
  // 曲线参数c3
  lane->c3 = lane_curvature_3;
  // 备注：
  // 车道线使用三阶多项式曲线定义 y = c0 + c1 * x + c2 * x^2 + c3 * x^3
  // 使用右手迪卡尔坐标系，x为车身纵轴方向，向前为正方向，y为车身横轴方向，向左为正方向;
  // 角度以x轴正方向为0度，逆时针方向为正。

  // Push back to list
  lane_list_.PushBack();
}

void TaskRecvMaxieyeData::ParseVisualControlLaneFromCanFrame(
    const can_dev::CanFdFrame& frame) {
  // lane 01
  ParseVisualControlOneLaneFromData(&frame.data[0]);
  // lane 02
  ParseVisualControlOneLaneFromData(&frame.data[16]);
  // lane 03
  ParseVisualControlOneLaneFromData(&frame.data[32]);
  // lane 04
  ParseVisualControlOneLaneFromData(&frame.data[48]);
}

void TaskRecvMaxieyeData::ParseVisualControlOneLaneFromData(const Uint8_t* data) {
  // ID, 0x1:左一；0x2:右一；0x3:左二；0x4：右二；
  Uint8_t lane_id = data[0] & 0x07;
  // 类型（包括车道线和道路边线）, 0x0-Background (BK), 0x1- Single Solid White (SSW)
  // 0x2- Single Dashed White (SDW), 0x3- Single Solid Yellow (SSY)
  // 0x4- Single Dashed Yellow (SDY), 0x5- Double Solid Yellow (DSY)
  // 0x6- Dashed Solid (DS), 0x7- Solid Dashed (SD), 0x8- Deceleration Strip (DEC)
  // 0x9- Bold (BO), 0xA- RoadEdge, 0xB- Bott's Dots, 0xC-Barrier, 0xF - Invalid
  Uint8_t lane_type = (data[0] >> 3) & 0x1F;
  // 弯道半径, Factor:1 ｜ Offset:0 ∣ Range:0~65535, 直道设置弯道半径为5000m，无效值FFFF
  Uint32_t lane_curve_radius = ((Uint32_t)data[2] << 8) | (Uint32_t)data[1];
  // 横向位置/曲率参数C0, Factor:0.00390625 ｜ Offset: -128 ∣ Range:-128 ~ 128m
  Float64_t lane_curvature_0 =
      -((((Uint32_t)data[4] << 8) | (Uint32_t)data[3]) * 0.00390625 - 128.0);
  // 航向角, Factor:0.1 ｜ Offset: -90 ∣ Range:-90~90°
  Float32_t lane_heading_angle =
      ((((Uint32_t)data[6] & 0x0F) << 8) | (Uint32_t)data[5]) * 0.1F - 90.0F;
  // 置信度, 0x0:不可信；0x1:＜25%；0x2:＜50%；0x3:＜75%；0x4：＜90%；0x5:＜99%；0x6:＜99.9%；0x7:≤100%
  // 现在只有：0x0:不可信；0x3:＜75%；0x7:≤100%
  Uint8_t lane_prob_of_exist = data[7] & 0x07;
  // 弯道方向, 0x0:直道；0x1:左偏；0x2:右偏；无效FF
  Uint8_t lane_curv_diret = (data[7] >> 3) & 0x03;
  // 曲率参数C1, Factor:1.08949*10^5 ｜ Offset: -0.357 ∣ Range:-0.357~0.357rad
  Float64_t lane_curvature_1 =
      -((((Uint32_t)data[9] << 8) | (Uint32_t)data[8]) * 0.0000108949 - 0.357);
  // 曲率参数C2, Factor:1.08949*10^5 ｜ Offset: -0.357 ∣ Range:-0.357~0.357(1/m)
  Float64_t lane_curvature_2 =
      -((((Uint32_t)data[11] << 8) | (Uint32_t)data[10]) * 0.0000108949 - 0.357);
  // 曲率参数C3, Factor:1.08949*10^5 ｜ Offset: -0.357 ∣ Range:-0.357~0.357(1/m^2)
  Float64_t lane_curvature_3 =
      -((((Uint32_t)data[13] << 8) | (Uint32_t)data[12]) * 0.0000108949 - 0.357);

  // 车道线识别的质量(0,1 – low quality.The lane measurements are not valid
  // in low quality. The system will not give an LDW in that situation.
  // 2,3 – high quality)
  Int32_t quality;
  // 置信度, 0x0:不可信；0x1:＜25%；0x2:＜50%；0x3:＜75%；0x4：＜90%；
  // 0x5:＜99%；0x6:＜99.9%；0x7:≤100%
  switch (lane_prob_of_exist) {
  case (0):
    quality = 0;
    break;
  case (1):
    quality = 0;
    break;
  case (2):
    quality = 1;
    break;
  case (3):
    quality = 2;
    break;
  case (4):
    quality = 2;
    break;
  case (5):
    quality = 3;
    break;
  case (6):
    quality = 3;
    break;
  case (7):
    quality = 3;
    break;
  default:
    quality = 0;
    break;
  }
  if (lane_id < 1 || lane_id > 4) {
    quality = 0;
  }
  if ((lane_curvature_0 < -128.0) || (lane_curvature_0 > 128.0)) {
    quality = 0;
  }
  if ((lane_curvature_1 < -0.3571) || (lane_curvature_1 > 0.3571)) {
    quality = 0;
  }
  if ((lane_curvature_2 < -0.3571) || (lane_curvature_2 > 0.3571)) {
    quality = 0;
  }
  if ((lane_curvature_3 < -0.3571) || (lane_curvature_3 > 0.3571)) {
    quality = 0;
  }

//  quality = 3;

  #if (VISUAL_CONTROL_LANE_LOG_FLAG == 1)
  printf("--------------------------------------\n");
  printf("lane_id:%d\n", lane_id);
  printf("lane_prob_of_exist:%d\n", lane_prob_of_exist);
  printf("lane_curvature_0:%f\n", lane_curvature_0);
  printf("lane_curvature_1:%f\n", lane_curvature_1);
  printf("lane_curvature_2:%f\n", lane_curvature_2);
  printf("lane_curvature_3:%f\n", lane_curvature_3);
  printf("quality:%d\n", quality);
  printf("--------------------------------------\n");
  #endif

  // Get back buffer from list
  ad_msg::LaneMarkCamera* lane = visual_control_lane_list_.Back();
  if (Nullptr_t == lane) {
    LOG_ERR << "Lane list is full.";
    return;
  }

  // 车道编号，左边车道线为正值，右边车道线为负数;(环视相机左边车道线为负值，右边车道线为正值)
  // (数值的绝对值按照距离当前车道的远近依次增加，
  // 第一条左边的车道线为1，第一条右边的车道线为-1)
  switch (lane_id) {
  // ID, 0x1:左一；0x2:右一；0x3:左二；0x4：右二；
  case (1):
    lane->id = 1;
    break;
  case (2):
    lane->id = -1;
    break;
  case (3):
    lane->id = 2;
    break;
  case (4):
    lane->id = -2;
    break;
  default:
    lane->id = 99;
    break;
  }
  // 车道线类型
  switch (lane_type) {
  // 类型（包括车道线和道路边线）, 0x0-Background (BK), 0x1- Single Solid White (SSW)
  // 0x2- Single Dashed White (SDW), 0x3- Single Solid Yellow (SSY)
  // 0x4- Single Dashed Yellow (SDY), 0x5- Double Solid Yellow (DSY)
  // 0x6- Dashed Solid (DS), 0x7- Solid Dashed (SD), 0x8- Deceleration Strip (DEC)
  // 0x9- Bold (BO), 0xA- RoadEdge, 0xB- Bott's Dots, 0xC-Barrier, 0xF - Invalid
  case (0):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_UNKNOWN;
    break;
  case (1):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (2):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    break;
  case (3):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (4):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    break;
  case (5):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DOUBLE_LANE_MARK;
    break;
  case (6):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_DASHED;
    break;
  case (7):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (8):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (9):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_SOLID;
    break;
  case (10):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_ROAD_EDGE;
    break;
  case (11):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_BOTTS_DOTS;
    break;
  case (12):
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_ROAD_EDGE;
    break;
  default:
    lane->lane_mark_type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_INVALID;
    break;
  }
  // 车道线识别的质量(0,1 – low quality.The lane measurements are not valid
  // in low quality. The system will not give an LDW in that situation.
  // 2,3 – high quality)
  lane->quality = quality;
  // 车道线的纵向有效范围字段中的值是否有效
  lane->view_range_valid = 1;  // Dummy
  // 车道线的宽度
  lane->mark_width = 0.0F;  // Dummy
  /// TODO: 确认车道线的有效识别长度 !!!
  // 车道线的纵向有效范围的起始位置
  lane->view_range_start = 0.0F;  // Dummy
  // 车道线的纵向有效范围的结束位置
  lane->view_range_end = 40.0F;
  // 曲线参数c0
  lane->c0 = lane_curvature_0;
  // 曲线参数c1
  lane->c1 = lane_curvature_1;
  // 曲线参数c2
  lane->c2 = lane_curvature_2;
  // 曲线参数c3
  lane->c3 = lane_curvature_3;
  // 备注：
  // 车道线使用三阶多项式曲线定义 y = c0 + c1 * x + c2 * x^2 + c3 * x^3
  // 使用右手迪卡尔坐标系，x为车身纵轴方向，向前为正方向，y为车身横轴方向，向左为正方向;
  // 角度以x轴正方向为0度，逆时针方向为正。

  // Push back to list
  visual_control_lane_list_.PushBack();
}

void TaskRecvMaxieyeData::ParseLaneLengthFromCanFrame(const can_dev::CanFdFrame& frame) {
  Uint8_t Lane_ID_1 = parse_sensor_data(frame.data, 0, 0, 3);
  Float32_t Lane_Width_1 = parse_sensor_data(frame.data, 0, 3, 5) * 0.02;
  Float32_t Lane_Range_1 = parse_sensor_data(frame.data, 1, 8, 16) * 0.1;

  Uint8_t Lane_ID_2 = parse_sensor_data(frame.data, 3, 24, 3);
  Float32_t Lane_Width_2 = parse_sensor_data(frame.data, 3, 27, 5) * 0.02;
  Float32_t Lane_Range_2 = parse_sensor_data(frame.data, 4, 32, 16) * 0.1;

  Uint8_t Lane_ID_3 = parse_sensor_data(frame.data, 6, 48, 3);
  Float32_t Lane_Width_3 = parse_sensor_data(frame.data, 6, 51, 5) * 0.02;
  Float32_t Lane_Range_3 = parse_sensor_data(frame.data, 7, 56, 16) * 0.1;

  Uint8_t Lane_ID_4 = parse_sensor_data(frame.data, 9, 72, 3);
  Float32_t Lane_Width_4 = parse_sensor_data(frame.data, 9, 75, 5) * 0.02;
  Float32_t Lane_Range_4 = parse_sensor_data(frame.data, 10, 80, 16) * 0.1;

  #if (LANE_LOG_FLAG == 1)
  printf("--------------------------------------\n");
  printf("Lane_ID_1:%d\n", Lane_ID_1);
  printf("Lane_Width_1:%f\n", Lane_Width_1);
  printf("Lane_Range_1:%f\n", Lane_Range_1);
  printf("Lane_ID_2:%d\n", Lane_ID_2);
  printf("Lane_Width_2:%f\n", Lane_Width_2);
  printf("Lane_Range_2:%f\n", Lane_Range_2);
  printf("Lane_ID_3:%d\n", Lane_ID_3);
  printf("Lane_Width_3:%f\n", Lane_Width_3);
  printf("Lane_Range_3:%f\n", Lane_Range_3);
  printf("Lane_ID_4:%d\n", Lane_ID_4);
  printf("Lane_Width_4:%f\n", Lane_Width_4);
  printf("Lane_Range_4:%f\n", Lane_Range_4);
  printf("--------------------------------------\n");
  #endif

  ad_msg::LaneMarkCameraList lane_list = lane_list_.GetLaneList();
  for(Uint8_t i = 0; i < lane_list.lane_mark_num; i++) {
    Int32_t lane_id = lane_list.lane_marks[i].id;
    switch (lane_id) {
      case (1):
        lane_list.lane_marks[i].mark_width = Lane_Width_1;
        lane_list.lane_marks[i].view_range_start = 0.0F;
        lane_list.lane_marks[i].view_range_end = Lane_Range_1;
        break;

      case (-1):
        lane_list.lane_marks[i].mark_width = Lane_Width_2;
        lane_list.lane_marks[i].view_range_start = 0.0F;
        lane_list.lane_marks[i].view_range_end = Lane_Range_2;
        break;

      case (2):
        lane_list.lane_marks[i].mark_width = Lane_Width_3;
        lane_list.lane_marks[i].view_range_start = 0.0F;
        lane_list.lane_marks[i].view_range_end = Lane_Range_3;
        break;

      case (-2):
        lane_list.lane_marks[i].mark_width = Lane_Width_4;
        lane_list.lane_marks[i].view_range_start = 0.0F;
        lane_list.lane_marks[i].view_range_end = Lane_Range_4;
        break;

      default:
        lane_list.lane_marks[i].mark_width = 0.0F;
        lane_list.lane_marks[i].view_range_start = 0.0F;
        lane_list.lane_marks[i].view_range_end = 60.0F;
        break;
    }
  }
  lane_list_.SetLaneList(lane_list);
}

void TaskRecvMaxieyeData::ParseVisualControlLaneLengthFromCanFrame(const can_dev::CanFdFrame& frame) {
  Uint8_t Lane_ID_1 = parse_sensor_data(frame.data, 0, 0, 3);
  Float32_t Lane_Width_1 = parse_sensor_data(frame.data, 0, 3, 5) * 0.02;
  Float32_t Lane_Range_1 = parse_sensor_data(frame.data, 1, 8, 16) * 0.1;

  Uint8_t Lane_ID_2 = parse_sensor_data(frame.data, 3, 24, 3);
  Float32_t Lane_Width_2 = parse_sensor_data(frame.data, 3, 27, 5) * 0.02;
  Float32_t Lane_Range_2 = parse_sensor_data(frame.data, 4, 32, 16) * 0.1;

  Uint8_t Lane_ID_3 = parse_sensor_data(frame.data, 6, 48, 3);
  Float32_t Lane_Width_3 = parse_sensor_data(frame.data, 6, 51, 5) * 0.02;
  Float32_t Lane_Range_3 = parse_sensor_data(frame.data, 7, 56, 16) * 0.1;

  Uint8_t Lane_ID_4 = parse_sensor_data(frame.data, 9, 72, 3);
  Float32_t Lane_Width_4 = parse_sensor_data(frame.data, 9, 75, 5) * 0.02;
  Float32_t Lane_Range_4 = parse_sensor_data(frame.data, 10, 80, 16) * 0.1;

  #if 0
  printf("--------------------------------------\n");
  printf("Lane_ID_1:%d\n", Lane_ID_1);
  printf("Lane_Width_1:%f\n", Lane_Width_1);
  printf("Lane_Range_1:%f\n", Lane_Range_1);
  printf("Lane_ID_2:%d\n", Lane_ID_2);
  printf("Lane_Width_2:%f\n", Lane_Width_2);
  printf("Lane_Range_2:%f\n", Lane_Range_2);
  printf("Lane_ID_3:%d\n", Lane_ID_3);
  printf("Lane_Width_3:%f\n", Lane_Width_3);
  printf("Lane_Range_3:%f\n", Lane_Range_3);
  printf("Lane_ID_4:%d\n", Lane_ID_4);
  printf("Lane_Width_4:%f\n", Lane_Width_4);
  printf("Lane_Range_4:%f\n", Lane_Range_4);
  printf("--------------------------------------\n");
  #endif

  ad_msg::LaneMarkCameraList lane_list = visual_control_lane_list_.GetLaneList();
  for(Uint8_t i = 0; i < lane_list.lane_mark_num; i++) {
    Int32_t lane_id = lane_list.lane_marks[i].id;
    switch (lane_id) {
      case (1):
        lane_list.lane_marks[i].mark_width = Lane_Width_1;
        lane_list.lane_marks[i].view_range_start = 0.0F;
        lane_list.lane_marks[i].view_range_end = Lane_Range_1;
        break;

      case (-1):
        lane_list.lane_marks[i].mark_width = Lane_Width_2;
        lane_list.lane_marks[i].view_range_start = 0.0F;
        lane_list.lane_marks[i].view_range_end = Lane_Range_2;
        break;

      case (2):
        lane_list.lane_marks[i].mark_width = Lane_Width_3;
        lane_list.lane_marks[i].view_range_start = 0.0F;
        lane_list.lane_marks[i].view_range_end = Lane_Range_3;
        break;

      case (-2):
        lane_list.lane_marks[i].mark_width = Lane_Width_4;
        lane_list.lane_marks[i].view_range_start = 0.0F;
        lane_list.lane_marks[i].view_range_end = Lane_Range_4;
        break;

      default:
        lane_list.lane_marks[i].mark_width = 0.0F;
        lane_list.lane_marks[i].view_range_start = 0.0F;
        lane_list.lane_marks[i].view_range_end = 60.0F;
        break;
    }
  }
  visual_control_lane_list_.SetLaneList(lane_list);
}

void TaskRecvMaxieyeData::ParseMaxieyeObjFromCanFrame(
    const can_dev::CanFdFrame& frame) {
  // Obj 1
  ParseMaxieyeOneObjFromData(&frame.data[0]);
  // Obj 2
  ParseMaxieyeOneObjFromData(&frame.data[32]);
}

void TaskRecvMaxieyeData::ParseMaxieyeOneObjFromData(const Uint8_t* data) {
  // 目标数量, 0~250，无效值255
  Int32_t num_of_objects = data[0];
  // 目标ID, Factor:1 ｜ Offset: 0 ∣ Range:0 ~ 250，无效值255
  Int32_t obj_id = data[3];
  // 纵向距离, Factor:0.01 ｜ Offset: -200 ∣ Range:-200.00 ~ 200.00m
  Float32_t obj_dist_lgt =
      (((Uint32_t)data[5] << 8) | ((Uint32_t)data[4])) * 0.01F - 200.0F;
  // 横向距离, Factor:0.01 ｜ Offset: -200 ∣ Range:-200.00 ~ 200.00m
  Float32_t obj_dist_lat =
      (((Uint32_t)data[7] << 8) | ((Uint32_t)data[6])) * 0.01F - 200.0F;
  // 纵向相对速度, Factor:0.01 ｜ Offset: -100 ∣ Range:-100.00 ~ 100.00m/s
  Float32_t obj_vel_lgt =
      (((Uint32_t)data[9] << 8) | ((Uint32_t)data[8])) * 0.01F - 100.0F;
  // 横向相对速度, Factor:0.01 ｜ Offset: -100 ∣ Range:-100.00 ~ 100.00m/s
  Float32_t obj_vel_lat =
      (((Uint32_t)data[11] << 8) | ((Uint32_t)data[10])) * 0.01F - 100.0F;
  // 水平方位角, Factor:0.01 ｜ Offset: -180 ∣ Range:-180.00 ~ 180.00°
  Float32_t obj_hori_ang =
      (((Uint32_t)data[13] << 8) | ((Uint32_t)data[12])) * 0.01F - 180.0F;
  // 横向相对加速度, Factor:0.01 ｜ Offset: -15.00 ∣ Range:-15.00 ~ 15.00m/s^2
  Float32_t obj_acc_lat =
      (((Uint32_t)data[15] << 8) | ((Uint32_t)data[14])) * 0.01F - 15.0F;
  // 纵向相对加速度, Factor:0.01 ｜ Offset: -15.00 ∣ Range:-15.00 ~ 15.00m/s^2
  Float32_t obj_acc_lgt =
      (((Uint32_t)data[17] << 8) | ((Uint32_t)data[16])) * 0.01F - 15.0F;
  // 目标的角速率, Factor:0.01 ｜ Offset: -327.68 ∣ Range:-367.28 ~ 367.28 degree/sec
  Float32_t obj_angle_rate =
      (((Uint32_t)data[19] << 8) | ((Uint32_t)data[18])) * 0.01F - 327.68F;
  // 目标的尺寸变化, Factor:0.0002 ｜Offset: -6.5532 ∣ Range:-6.5532 ~ 6.5332 [pix/sec]
  Float32_t obj_scale_change =
      (((Uint32_t)data[21] << 8) | ((Uint32_t)data[20])) * 0.01F - 6.5532F;
  // 目标运动状态, 0x0:目标静止；0x1:目标运动
  Uint8_t obj_dyn_prop = data[22] & 0x03;
  // 目标种类, 0x0：特殊车；0x1:乘用车 0x2:商用车; 0x3:骑行者；0x4:行人;
  //          0x5：动物；0x6：遗撒物；0x7：其他/无法定义
  Uint8_t obj_class = (data[22] >> 2) & 0x07;
  // 目标来历(预留), 0x0:deleted, 0x1:new, 0x2:measured, 0x3:predicted
  //               0x4:deleted for merge, 0x5:new from merge
  Uint8_t obj_valid = (data[22] >> 5) & 0x07;
  // 目标所在车道, 0x0:本车道；0x1:左一车道；0x2:右一车道；0x3:左二车道；
  //             0x4：右二车道；0x5:≥左三；0x6:≥右三；0x7：无效
  Uint8_t obj_lane = data[23] & 0x07;
  // 目标置信度, 0x0:不可信；0x1:＜25%；0x2:＜50%；0x3:＜75%；0x4：＜90%；0x5:＜99%；
  //           0x6:＜99.9%；0x7:≤100%
  Uint8_t obj_prob_of_exist = (data[23] >> 3)&0x07;
  // 生命周期, Factor:1 ｜ Offset: 0 ∣ Range:0 ~ 2047
  Int32_t obj_age = (((Uint32_t)data[25] << 8) | ((Uint32_t)data[24]));
  // 目标宽度, Factor:0.1 ｜ Offset:0 ∣ Range:0 ~ 12.7m
  Float32_t obj_width = data[26] * 0.1F;
  // 目标长度, Factor:0.25 ｜ Offset:0 ∣ Range:0 ~ 67.25m
  Float32_t obj_length = data[27] * 0.25F; //m
  // 目标高度, Factor:0.1｜Offset:0 ∣ Range:0 ~ 25.0m
  Float32_t obj_height = data[28] * 0.1F; //m

  #if (OBSTACLE_LOG_FLAG == 1)
  printf("--------------------------------------\n");
  printf("num_of_objects:%d\n", num_of_objects);
  printf("obj_id:%d\n", obj_id);
  printf("obj_dist_lgt:%f\n", obj_dist_lgt);
  printf("obj_dist_lat:%f\n", obj_dist_lat);
  printf("obj_vel_lgt:%f\n", obj_vel_lgt);
  printf("obj_vel_lat:%f\n", obj_vel_lat);
  printf("obj_hori_ang:%f\n", obj_hori_ang);
  printf("obj_acc_lat:%f\n", obj_acc_lat);
  printf("obj_acc_lgt:%f\n", obj_acc_lgt);
  printf("obj_angle_rate:%f\n", obj_angle_rate);
  printf("obj_scale_change:%f\n", obj_scale_change);
  printf("obj_dyn_prop:%d\n", obj_dyn_prop);
  printf("obj_class:%d\n", obj_class);
  printf("obj_valid:%d\n", obj_valid);
  printf("obj_lane:%d\n", obj_lane);
  printf("obj_prob_of_exist:%d\n", obj_prob_of_exist);
  printf("obj_age:%d\n", obj_age);
  printf("obj_width:%f\n", obj_width);
  printf("obj_length:%f\n", obj_length);
  printf("obj_height:%f\n", obj_height);
  printf("----------------------------\n");
  #endif

  // 根据障碍物属性过滤障碍物
  if (num_of_objects < 1) {
    return;
  }
  if (obj_id < 0 | obj_id > 250) {
    return;
  }
  if (obj_prob_of_exist < 3) {
    return;
  }
  if((parse_sensor_data(data, 4, 32, 16) == 20000) | (parse_sensor_data(data, 4, 32, 16) == 0)) {
    return;
  }

  // 根据障碍物横向纵向距离过滤障碍物
  // 过滤掉纵向距离大于150m的障碍物
  if (obj_dist_lgt > 150) {
    return;
  }
  // 过滤掉横向距离大于三个车道的障碍物(1个车道按4m计算)
  if (abs(obj_dist_lat) > 10) {
    return;
  }

  // Get back buffer from list
  ad_msg::ObstacleCamera* obj = obj_list_.Back();
  if (Nullptr_t == obj) {
    LOG_ERR << "Obj list is full.";
    return;
  }

  obj->accel_x = obj_acc_lgt;
  obj->accel_y = obj_acc_lat;
  obj->age = obj_age;
  obj->id = obj_id;
  obj->type = obj_class;
  obj->lane = obj_lane;
  obj->length = obj_length;
  obj->width = obj_width;
  obj->height = obj_height;
  obj->scale_change = obj_scale_change;
  obj->v_x = obj_vel_lgt;
  obj->v_y = obj_vel_lat;
  obj->x = obj_dist_lgt;
  obj->y = obj_dist_lat;
  // 障碍物ID
  obj->id = obj_id;
  // 障碍物类型
  // 目标种类, 0x0：特殊车；0x1:乘用车 0x2:商用车; 0x3:骑行者；0x4:行人;
  //          0x5：动物；0x6：遗撒物；0x7：其他/无法定义
  switch (obj_class) {
  case (0):
    obj->type = ad_msg::OBJ_TYPE_SPECIAL_VEHICLE;
    break;
  case (1):
    obj->type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
    break;
  case (2):
    obj->type = ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE;
    break;
  case (3):
    obj->type = ad_msg::OBJ_TYPE_BICYCLE;
    break;
  case (4):
    obj->type = ad_msg::OBJ_TYPE_PEDESTRIAN;
    break;
  case (5):
    obj->type = ad_msg::OBJ_TYPE_ANIMAL;
    break;
  case (6):
    obj->type = ad_msg::OBJ_TYPE_DISCARD;
    break;
  default:
    obj->type = ad_msg::OBJ_TYPE_UNKNOWN;
    break;
  }
  // 障碍物的状态
  // 目标运动状态, 0x0:目标静止；0x1:目标运动
  switch (obj_dyn_prop) {
  case (0):
    obj->status = ad_msg::ObstacleCamera::OBJ_STATUS_STOPPED;
    break;
  case (1):
    obj->status = ad_msg::ObstacleCamera::OBJ_STATUS_MOVING;
    break;
  default:
    obj->status = ad_msg::ObstacleCamera::OBJ_STATUS_STOPPED;
    break;
  }
  // 切入/切出的类型
  obj->cut_in = ad_msg::ObstacleCamera::CUT_IN_TYPE_UNKNOWN;
  // 转向灯的状态
  obj->blinker = ad_msg::ObstacleCamera::BLINKER_UNKNOWN;
  // 制动灯状态
  obj->brake_lights = false;
  // The age of the obstacle (in frames). This value starts at 1 when the
  // obstacle is first detected, and increments in 1 each frame.
  obj->age = obj_age;
  // 指示障碍物在哪个车道
  // 目标所在车道, 0x0:本车道；0x1:左一车道；0x2:右一车道；0x3:左二车道；
  //             0x4：右二车道；0x5:≥左三；0x6:≥右三；0x7：无效
  switch (obj_lane) {
  case (0):
    obj->lane = 0;
    break;
  case (1):
    obj->lane = 1;
    break;
  case (2):
    obj->lane = -1;
    break;
  case (3):
    obj->lane = 2;
    break;
  case (4):
    obj->lane = -2;
    break;
  case (5):
    obj->lane = 3;
    break;
  case (6):
    obj->lane = -3;
    break;
  default:
    obj->lane = 0;
    break;
  }
  // 障碍物长度 (m)
  obj->length = obj_length;
  // 障碍宽度 (m)
  obj->width = obj_width;
  // 障碍物高度（m）
  obj->height = obj_height;
  // 障碍物位置x坐标 (m)
  obj->x = obj_dist_lgt;
  // 障碍物位置y坐标 (m)
  obj->y = obj_dist_lat;
  // 障碍物航向 (rad)
  obj->heading = common::NormalizeAngle(common::com_deg2rad(obj_hori_ang));
  // 障碍物的x向速度 (m/sec)
  obj->v_x = obj_vel_lgt;
  // 障碍物的y向速度 (m/sec)
  obj->v_y = obj_vel_lat;
  // 障碍物的加速度 (m/sec^2)
  obj->accel_x = obj_acc_lgt;
  // 障碍物的加速度 (m/sec^2)
  obj->accel_y = obj_acc_lat;
  // 障碍物的角速度 (rad/sec)
  obj->yaw_rate = common::com_deg2rad(obj_angle_rate);
  // 尺度的变化 (pix/sec)
  obj->scale_change = obj_scale_change;


#if  DEBUG_REL_VX
  //printf("-------------------DEBUG_REL_VX2-------------------\n\n\n");
  obj->scale_change = obj_vel_lgt;

#endif

#if  USE_MAIN_CAMERA_PROB_OF_EXIST
  //当需要将置信度传递给 融合模块
  obj->scale_change = obj_prob_of_exist;

#endif


  // Push back to list
  obj_list_.PushBack();
}

void TaskRecvMaxieyeData::ParseTSRFromCanFrame(const can_dev::CanFdFrame& frame) {
  // 交通标志ID, Factor:1｜Offset: 0∣Range:0~255
  Uint8_t TSR_ID = parse_sensor_data(frame.data, 0, 0, 8);
  // 交通标志牌类型
  Uint8_t TSR_Type = parse_sensor_data(frame.data, 1, 8, 8);
  // 交通标志置信度, 0x0:不可信；0x1:＜25%；0x2:＜50%；0x3:＜75%；0x4：＜90%；0x5:＜99%；0x6:＜99.9%；0x7:≤100%
  Uint8_t TSR_ProbOfExist = parse_sensor_data(frame.data, 2, 16, 3);
  // 交通标志纵向距离, Factor:0.01｜Offset: 0∣Range:0~200m
  Float32_t TSR_DistLgt = parse_sensor_data(frame.data, 3, 24, 16) * 0.01;
  // 交通标志横向距离, Factor:0.01｜Offset: -32∣Range:-32~31m
  Float32_t TSR_DistLat = parse_sensor_data(frame.data, 5, 40, 16) * 0.01 + (-32);
  // 交通标志高度信息, Factor:0.01｜Offset: -15∣Range:-15~15m
  Float32_t TSR_Height = parse_sensor_data(frame.data, 7, 56, 16) * 0.01 + (-15);
  // 地面标志ID, Factor:1｜Offset: 0∣Range:0~255
  Uint8_t GroundMark_ID = parse_sensor_data(frame.data, 15, 120, 8);
  // 地面标志类型
  Uint8_t GroundMark_Type = parse_sensor_data(frame.data, 16, 128, 8);
  // 地面标志置信度, 0x0:不可信；0x1:＜25%；0x2:＜50%；0x3:＜75%；0x4：＜90%；0x5:＜99%；0x6:＜99.9%；0x7:≤100%
  Uint8_t GroundMark_ProbOfExist = parse_sensor_data(frame.data, 17, 136, 3);
  // 地面标志纵向距离, Factor:0.01｜Offset: 0∣Range:0~200m
  Float32_t GroundMark_DistLgt = parse_sensor_data(frame.data, 18, 144, 16) * 0.01;
  // 地面标志横向距离, Factor:0.01｜Offset: -32∣Range:-32~31m
  Float32_t GroundMark_DistLat = parse_sensor_data(frame.data, 20, 160, 16) * 0.01 + (-32);

#if (TRAFFICE_LOG_FLAG == 1)
  printf("TSR_ID = %d, TSR_Type = %d, TSR_ProbOfExist = %d, TSR_DistLgt = %f, TSR_DistLat = %f, TSR_Height = %f\n", 
          TSR_ID, 
          TSR_Type, 
          TSR_ProbOfExist, 
          TSR_DistLgt, 
          TSR_DistLat, 
          TSR_Height);
#endif

  if (TSR_ProbOfExist < 3) {
    return;
  }

  // Get back buffer from list
  ad_msg::TrafficSignalSpeedRestriction *traffic_signal_speed_restriction = traffic_signal_list_.Back();
  if (Nullptr_t == traffic_signal_speed_restriction) {
    LOG_ERR << "Obj list is full.";
    return;
  }

  traffic_signal_speed_restriction->box.x = TSR_DistLgt;
  traffic_signal_speed_restriction->box.y = TSR_DistLat;
  traffic_signal_speed_restriction->box.z = TSR_Height;

  switch (TSR_Type) {
    case 81:  //限速10km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 10 / 3.6;
    break;

    case 82:  //限速100km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 100 / 3.6;
    break;
    
    case 83:  //限速105km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 105 / 3.6;
    break;

    case 84:  //限速110km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 110 / 3.6;
    break;

    case 85:  //限速115km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 115 / 3.6;
    break;

    case 86:  //限速120km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 120 / 3.6;
    break;

    case 87:  //限速15km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 15 / 3.6;
    break;

    case 88:  //限速20km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 20 / 3.6;
    break;

    case 89:  //限速25km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 25 / 3.6;
    break;

    case 90:  //限速30km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 30 / 3.6;
    break;

    case 91:  //限速35km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 35 / 3.6;
    break;

    case 92:  //限速40km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 40 / 3.6;
    break;

    case 93:  //限速45km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 45 / 3.6;
    break;

    case 94:  //限速5km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 5 / 3.6;
    break;

    case 95:  //限速50km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 50 / 3.6;
    break;

    case 96:  //限速55km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 55 / 3.6;
    break;

    case 97:  //限速60km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 60 / 3.6;
    break;

    case 98:  //限速65km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 65 / 3.6;
    break;

    case 99:  //限速70km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 70 / 3.6;
    break;

    case 100:  //限速75km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 75 / 3.6;
    break;

    case 101:  //限速80km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 80 / 3.6;
    break;

    case 102:  //限速85km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 85 / 3.6;
    break;

    case 103:  //限速90km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 90 / 3.6;
    break;

    case 104:  //限速95km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION;
      traffic_signal_speed_restriction->speed = 95 / 3.6;
    break;

    case 130:  //解除限速10km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::END_RESTRICTION;
      traffic_signal_speed_restriction->speed = 10 / 3.6;
    break;

    case 131:  //解除限速100km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::END_RESTRICTION;
      traffic_signal_speed_restriction->speed = 100 / 3.6;
    break;

    case 132:  //解除限速20km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::END_RESTRICTION;
      traffic_signal_speed_restriction->speed = 20 / 3.6;
    break;

    case 133:  //解除限速30km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::END_RESTRICTION;
      traffic_signal_speed_restriction->speed = 30 / 3.6;
    break;
    
    case 134:  //解除限速40km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::END_RESTRICTION;
      traffic_signal_speed_restriction->speed = 40 / 3.6;
    break;

    case 135:  //解除限速45km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::END_RESTRICTION;
      traffic_signal_speed_restriction->speed = 45 / 3.6;
    break;

    case 136:  //解除限速5km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::END_RESTRICTION;
      traffic_signal_speed_restriction->speed = 5 / 3.6;
    break;

    case 137:  //解除限速50km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::END_RESTRICTION;
      traffic_signal_speed_restriction->speed = 50 / 3.6;
    break;

    case 138:  //解除限速60km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::END_RESTRICTION;
      traffic_signal_speed_restriction->speed = 60 / 3.6;
    break;

    case 139:  //解除限速70km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::END_RESTRICTION;
      traffic_signal_speed_restriction->speed = 70 / 3.6;
    break;

    case 140:  //解除限速80km/h
      traffic_signal_speed_restriction->type = ad_msg::TrafficSignalSpeedRestriction::END_RESTRICTION;
      traffic_signal_speed_restriction->speed = 80 / 3.6;
    break;

    default:
    break;
  }

  // Push back to list
  traffic_signal_list_.PushBack();
}

void TaskRecvMaxieyeData::ParseVisualControlObjFromCanFrame(
    const can_dev::CanFdFrame& frame, Uint8_t camera_symbol) {
    // Obj 1
    ParseVisualControlObjFromData(&frame.data[0], camera_symbol);
    // Obj 2
    ParseVisualControlObjFromData(&frame.data[16], camera_symbol);
    // Obj 3
    ParseVisualControlObjFromData(&frame.data[32], camera_symbol);
    // Obj 4
    ParseVisualControlObjFromData(&frame.data[48], camera_symbol);
}

void TaskRecvMaxieyeData::ParseVisualControlObjFromData(const Uint8_t* data, Uint8_t camera_symbol) {
    // 目标ID, Factor:1｜Offset: 0∣Range:0~255
  Uint8_t Object_ID = parse_sensor_data(data, 0, 0, 8);

  // 纵向距离, Factor:0.1｜Offset: -200∣Range:-200~209.5m
  Float32_t Object_DistRange = parse_sensor_data(data, 1, 8, 12) * 0.1 + (-200);

  // 横向距离, Factor:0.1｜Offset: -100∣Range:-100~104.7m
  Float32_t Object_DistLat = parse_sensor_data(data, 3, 24, 11) * 0.1 + (-100);

  // 纵向相对速度, Factor:0.1｜Offset: -100∣Range:-100~104.7m/s
  Float32_t Object_VrelRange = parse_sensor_data(data, 5, 40, 11) * 0.1 + (-100);

  // 横向相对速度, Factor:0.1｜Offset: -100∣Range:-100~104.7m/s
  Float32_t Object_VrelLat = parse_sensor_data(data, 7, 56, 11) * 0.1 + (-100);

  // 水平方位角, Factor:0.1｜Offset: -90∣Range:-90~114.7°
  Float32_t Object_HoriAngel = parse_sensor_data(data, 9, 72, 11) * 0.1 + (-90);

  // 目标运动状态
  // 0x0:静止；0x1:运动；0x2:远离；0x3:靠近；0x4~0x7预留
  Uint8_t Object_DynProp = parse_sensor_data(data, 11, 88, 3);

  // 目标种类
  // 0x1:Vehicle；0x2:Truck;0x3:骑行者；0x4:Ped;0x5~0x7预留
  Uint8_t Object_Class = parse_sensor_data(data, 11, 91, 3);

  if (Object_ID >= 255) {
    return;
  }

  if (parse_sensor_data(data, 1, 8, 12) == 0) {
    return;
  }

  if (parse_sensor_data(data, 3, 24, 11) == 0) {
    return;
  }

  // 根据障碍物横向纵向距离过滤障碍物
  // 过滤掉纵向距离大于150m的障碍物,或小于-70m的障碍物
  if (Object_DistRange > 150 || Object_DistRange < -70) {
    return;
  }
  // 过滤掉横向距离大于三个车道的障碍物(1个车道按4m计算)
  if (abs(Object_DistLat) > 10) {
    return;
  }
  
  #if (1 == VISUAL_CONTROL_LOG_FLAG)
  printf("Object_ID:%d\n", Object_ID);
  printf("Object_DistRange:%f\n", Object_DistRange);
  printf("Object_DistLat:%f\n", Object_DistLat);
  printf("Object_VrelRange:%f\n", Object_VrelRange);
  printf("Object_VrelLat:%f\n", Object_VrelLat);
  printf("Object_HoriAngel:%f\n", Object_HoriAngel);
  printf("Object_DynProp:%d\n", Object_DynProp);
  printf("Object_Class:%d\n", Object_Class);
  #endif

  // Get back buffer from list
  ad_msg::ObstacleCamera* obj = Nullptr_t;
  switch (camera_symbol) {
    case (0):
      //obj = front_obj_list_.Back();
      obj = front_obj_list_.Back();
      break;
    case (1):
      obj = front_left_obj_list_.Back();
      break;
    case (2):
      obj = front_right_obj_list_.Back();
      break;
    case (3):
      obj = rear_left_obj_list_.Back();
      break;
    case (4):
      obj = rear_right_obj_list_.Back();
      break;
    case (5):
      obj = rear_obj_list_.Back();
      break;

    default:
      break;
  }

  if (Nullptr_t == obj) {
    LOG_ERR << "Obj list is full.";
    return;
  }

  /// 障碍物ID
  obj->id = Object_ID;

  /// 障碍物的类型
  // 0x1:Vehicle；0x2:Truck;0x3:骑行者；0x4:Ped;0x5~0x7预留
  switch (Object_Class) {
    case (1):
      obj->type = ad_msg::OBJ_TYPE_PASSENGER_VEHICLE;
      break;
    case (2):
      obj->type = ad_msg::OBJ_TYPE_COMMERCIAL_VEHICLE;
      break;
    case (3):
      obj->type = ad_msg::OBJ_TYPE_BICYCLE;
      break;
    case (4):
      obj->type = ad_msg::OBJ_TYPE_PEDESTRIAN;
      break;
    default:
      obj->type = ad_msg::OBJ_TYPE_UNKNOWN;
      break;
  }

  // 障碍物状态
  // 0x0:静止；0x1:运动；0x2:远离；0x3:靠近；0x4~0x7预留
  switch (Object_DynProp) {
    case (0):
      obj->status = ad_msg::ObstacleCamera::OBJ_STATUS_STOPPED;
      break;
    case (1):
      obj->status = ad_msg::ObstacleCamera::OBJ_STATUS_MOVING;
      break;
    case (3):
      obj->status = ad_msg::ObstacleCamera::OBJ_STATUS_ONCOMING;
      break;
    default:
      obj->status = ad_msg::ObstacleCamera::OBJ_STATUS_UNKNOWN;
      break;
  }

  /// 障碍物长度 (m)
  obj->length = 1.0;
  /// 障碍宽度 (m)
  obj->width = 1.0;
  /// 障碍物位置x坐标 (m)
  obj->x = Object_DistRange;
  /// 障碍物位置y坐标 (m)
  obj->y = Object_DistLat;
  /// 障碍物的x向速度 (m/sec)
  obj->v_x = Object_VrelRange;
  /// 障碍物的y向速度 (m/sec)
  obj->v_y = Object_VrelLat;
  /// 障碍物的x向加速度 (m/sec^2)
  obj->accel_x = 0;
  /// 障碍物的y向加速度 (m/sec^2)
  obj->accel_y = 0;
  // 障碍物航向 (rad)
  //obj->heading = common::NormalizeAngle(common::com_deg2rad(Object_HoriAngel));

#if  DEBUG_REL_VX
  //printf("-------------------DEBUG_REL_V4-------------------\n\n\n");
  obj->scale_change = Object_VrelRange;

#endif

  // Push back to list
  switch (camera_symbol) {
    case (0):
      front_obj_list_.PushBack();
      break;
    case (1):
      front_left_obj_list_.PushBack();
      break;
    case (2):
      front_right_obj_list_.PushBack();
      break;
    case (3):
      rear_left_obj_list_.PushBack();
      break;
    case (4):
      rear_right_obj_list_.PushBack();
      break;
    case (5):
      rear_obj_list_.PushBack();
      break;
    default:
      break;
  }
}

} // namespace sensor
} // namespace phoenix

