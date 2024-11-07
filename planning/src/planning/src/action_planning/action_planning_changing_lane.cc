/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       action_planning_base.h
 * @brief      行为规划基类
 * @details    行为规划的基类，规划车辆行为(变道、跟车、避让、巡航等)
 *
 * @author     sip (sip@goyu-ai.com)
 * @date       2021/05/07
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/05/07  <td>1.0      <td>sip       <td>First edition
 * </table>
 *
 ******************************************************************************/

#include "action_planning/action_planning_changing_lane.h"

#include "utils/macros.h"
#include "utils/log.h"
#include "geometry/geometry_utils.h"
#include "vehicle_model_wrapper.h"


#define ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE (1)


namespace phoenix {
namespace planning {


ActionPlanningChangingLane::ActionPlanningChangingLane() {
  veh_model::VehicleModelWrapper veh_model;
  // 车长，单位：米
  param_.vehicle_length = veh_model.GetVehicleLength();
  // 车宽，单位：米
  param_.vehicle_width = veh_model.GetVehicleWidth();
  // 车辆的定位点到 front of vehicle 的距离，单位：米
  param_.dist_of_localization_to_front =
      veh_model.GetDistOfLocalizationToFront();
  // 车辆的定位点到 rear of vehicle 的距离，单位：米
  param_.dist_of_localization_to_rear =
      veh_model.GetDistOfLocalizationToRear();
  // 车辆的定位点到中心点的距离，单位：米
  param_.dist_of_localization_to_center =
      veh_model.GetDistOfLocalizationToCenter();

  param_.dist_limit_table_for_changine_lane.Clear();
  param_.dist_limit_table_for_changine_lane.PushBack(
        common::LerpTableNodeType1(50.0F/3.6F, 20.0F));
  param_.dist_limit_table_for_changine_lane.PushBack(
        common::LerpTableNodeType1(65.0F/3.6F, 28.0F));
  param_.dist_limit_table_for_changine_lane.PushBack(
        common::LerpTableNodeType1(80.0F/3.6F, 35.0F));
  param_.dist_limit_table_for_changine_lane.PushBack(
        common::LerpTableNodeType1(90.0F/3.6F, 40.0F));

  driving_map_ = Nullptr_t;
  changing_lane_status_ = ChangingLaneRsp::STATUS_CHANGING_LANE_NONE;
  pre_changing_lane_status_ = ChangingLaneRsp::STATUS_CHANGING_LANE_NONE;
  prev_lat_offset_to_major_ref_ = 0.0F;

  changing_lane_req_.Clear();
  hmi_changing_lane_req_.Clear();
}

ActionPlanningChangingLane::~ActionPlanningChangingLane(){
  // nothing to do
}

void ActionPlanningChangingLane::Handle(
    const ActionPlanningDataSource& data_source,
    ActionPlanningResult* planning_result) {
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
  std::cout << "######### Action planning (Changing lane) >>>>>>>>>" << std::endl;
#endif

  driving_map_ = data_source.driving_map;

  planning_result->changing_lane_req.Clear();
  changing_lane_req_.set_allow_auto_changing_to_left(false);
  changing_lane_req_.set_allow_auto_changing_to_right(false);

  // 更新车身参数
  UpdateVehicleParameter();
  // 更新变道状态
  UpdateChangingLaneStatus(data_source);

#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
  LOG_INFO(5) << "$$$ Changing lane request from HMI = "
              << data_source.planning_settings->changing_lane_req;
#endif
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
  LOG_INFO(5) << "$$$ Initial changing lane request = ";
#endif
  Int32_t raw_req_type = GetChangingLaneReqType(data_source, planning_result);
  Int32_t request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
  if (ChangingLaneReq::REQ_CHANGING_LANE_LEFT == raw_req_type) {
    // Request to change to left lane
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "    Changing to left lane.";
#endif
    switch (changing_lane_status_) {
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_NONE):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_LEFT;
      break;
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_I):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_II):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_I):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
      break;
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_II):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
      break;
    case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_I):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_II):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
      break;
    case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
      break;
    default:
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    }
  } else if (ChangingLaneReq::REQ_CHANGING_LANE_RIGHT == raw_req_type) {
    // Request to change to right lane
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "    Changing to right lane.";
#endif
    switch (changing_lane_status_) {
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_NONE):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_RIGHT;
      break;
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_I):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
      break;
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_II):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
      break;
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_I):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_II):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_I):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
      break;
    case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_II):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
      break;
    case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    default:
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    }
  } else if (ChangingLaneReq::REQ_CHANGING_LANE_ABORT == raw_req_type) {
    // Request to abort changing lane
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "    Aborting changing lane.";
#endif
    switch (changing_lane_status_) {
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_NONE):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_I):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
      break;
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_II):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
      break;
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_I):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
      break;
    case (ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_II):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
      break;
    case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_I):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_II):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II):
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    default:
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      break;
    }
  } else {
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "    No request.";
#endif
    request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
  }

  // TODO: 判断车道条件是否允许变道(障碍物, 车道线类型, 车道线质量 etc.)
  Int32_t refuse_reason = REFUSE_CHANGINHG_LANE_REASON_NONE;
#if 1
  if (ChangingLaneReq::REQ_CHANGING_LANE_LEFT == request_type) {
    bool result = JudgeChangingLaneConditions(request_type, &refuse_reason);
    if (!result) {
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;

      StopHoldChangeLaneReqByRefuseReason(refuse_reason);

      // 若变道请求在保持期间，不报警，通知正在准备变道
      if (act_planning_shared_data_->timers.hold_changing_lane_req.IsActive()) {
        act_planning_shared_data_->AddEventReporting(
              ActionPlanningSharedData::EVENT_TYPE_PREPARE_FOR_CHANGING_LANE_LEFT,
              refuse_reason);
      } else {
        act_planning_shared_data_->AddEventReporting(
              ActionPlanningSharedData::EVENT_TYPE_REFUSE_CHANGING_LANE_LEFT,
              refuse_reason);
      }
    }
  } else if (ChangingLaneReq::REQ_CHANGING_LANE_RIGHT == request_type) {
    bool result = JudgeChangingLaneConditions(request_type, &refuse_reason);
    if (!result) {
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;

      StopHoldChangeLaneReqByRefuseReason(refuse_reason);

      // 若变道请求在保持期间，不报警，通知正在准备变道
      if (act_planning_shared_data_->timers.hold_changing_lane_req.IsActive()) {
        act_planning_shared_data_->AddEventReporting(
              ActionPlanningSharedData::EVENT_TYPE_PREPARE_FOR_CHANGING_LANE_RIGHT,
              refuse_reason);
      } else {
        act_planning_shared_data_->AddEventReporting(
              ActionPlanningSharedData::EVENT_TYPE_REFUSE_CHANGING_LANE_RIGHT,
              refuse_reason);
      }
    }
  } else {
    // 若保持变道请求超时，发送通知
    if (!act_planning_shared_data_->timers.hold_changing_lane_req.IsActive()) {
      Int32_t hold_req_type =
          act_planning_shared_data_->timers.hold_changing_lane_req.GetUserDataInt32(0);
      if (ChangingLaneReq::REQ_CHANGING_LANE_NONE != hold_req_type) {
        act_planning_shared_data_->timers.hold_changing_lane_req.SetUserDataInt32(
              0, ChangingLaneReq::REQ_CHANGING_LANE_NONE);
        if (ChangingLaneReq::REQ_CHANGING_LANE_LEFT == hold_req_type) {
          act_planning_shared_data_->AddEventReporting(
                ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_LEFT);
        } else if (ChangingLaneReq::REQ_CHANGING_LANE_RIGHT == hold_req_type) {
          act_planning_shared_data_->AddEventReporting(
                ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_RIGHT);
        } else {
          LOG_WARN << "Invalid changing lane request type in holding.";
        }
      }
    }
    /* D001 fuyuanyi 2023-6-30 (begin) */
    // 去掉换道抑制中的抑制
    if (!act_planning_shared_data_->config.enable_action_planning_force_changing_lane){
#if 1
    // 若目标车道有风险，则中止变道
    if (ChangingLaneRsp::STATUS_CHANGING_LANE_NONE != changing_lane_status_) {
      Int32_t event_type = ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_LEFT;
      bool result = JudgeChangingLaneConditionsInChangingLane(
            request_type, &refuse_reason, &event_type);
      if (!result) {
        request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
        act_planning_shared_data_->AddEventReporting(event_type, refuse_reason);
      } else {
        request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      }
    }
}
#endif
    /* D001 fuyuanyi 2023-6-30 (end) */
  }
#endif

  // 若开始变道了，则取消变道保持
  if (ChangingLaneRsp::STATUS_CHANGING_LANE_NONE != changing_lane_status_) {
    act_planning_shared_data_->timers.hold_changing_lane_req.Stop();
    act_planning_shared_data_->timers.hold_changing_lane_req.SetUserDataInt32(
          0, ChangingLaneReq::REQ_CHANGING_LANE_NONE);
  }

  if (ChangingLaneReq::REQ_CHANGING_LANE_ABORT == request_type) {
    ForbidAlcChangeLane();
  }
  if (act_planning_shared_data_->timers.forbid_alc_changing_lane.IsActive()) {
    if ((ChangingLaneReq::REQ_CHANGING_LANE_LEFT == request_type) ||
        (ChangingLaneReq::REQ_CHANGING_LANE_RIGHT == request_type)) {
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
    }
  }
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
  LOG_INFO(5) << "$$$ Finally, changing lane request = " << request_type;
#endif

  if (ChangingLaneReq::REQ_CHANGING_LANE_NONE != request_type) {
    if (changing_lane_req_.request() != request_type) {
      changing_lane_req_.set_request(request_type);
    }
    changing_lane_req_.set_refuse_reason(refuse_reason);
  }

  changing_lane_req_.set_sequence(changing_lane_req_.sequence() + 1);
  planning_result->changing_lane_req = changing_lane_req_;
  planning_result->changing_lane_req.set_request(request_type);

  // 根据变道状态设置转向灯的状态
  switch (changing_lane_status_) {
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_NONE):
    planning_result->turn_lamp = ad_msg::VEH_TURN_LAMP_OFF;
    break;
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_I):
    planning_result->turn_lamp = ad_msg::VEH_TURN_LAMP_LEFT;
    break;
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_II):
    planning_result->turn_lamp = ad_msg::VEH_TURN_LAMP_LEFT;
    break;
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_I):
    planning_result->turn_lamp = ad_msg::VEH_TURN_LAMP_RIGHT;
    break;
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_II):
    planning_result->turn_lamp = ad_msg::VEH_TURN_LAMP_RIGHT;
    break;
  case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_I):
    planning_result->turn_lamp = ad_msg::VEH_TURN_LAMP_LEFT;
    break;
  case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_II):
    planning_result->turn_lamp = ad_msg::VEH_TURN_LAMP_LEFT;
    break;
  case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I):
    planning_result->turn_lamp = ad_msg::VEH_TURN_LAMP_RIGHT;
    break;
  case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II):
    planning_result->turn_lamp = ad_msg::VEH_TURN_LAMP_RIGHT;
    break;
  default:
    planning_result->turn_lamp = ad_msg::VEH_TURN_LAMP_OFF;
    break;
  }

#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
  std::cout << "<<<<<<<< Action planning (Changing lane) (End) ####" << std::endl;
#endif
}

void ActionPlanningChangingLane::UpdateVehicleParameter() {
  veh_model::VehicleModelWrapper veh_model;

  // 车长，单位：米
  param_.vehicle_length = veh_model.GetVehicleLength();
  // 车宽，单位：米
  param_.vehicle_width = veh_model.GetVehicleWidth();
  // 车辆的定位点到 front of vehicle 的距离，单位：米
  param_.dist_of_localization_to_front =
      veh_model.GetDistOfLocalizationToFront();
  // 车辆的定位点到 rear of vehicle 的距离，单位：米
  param_.dist_of_localization_to_rear =
      veh_model.GetDistOfLocalizationToRear();
  // 车辆的定位点到中心点的距离，单位：米
  param_.dist_of_localization_to_center =
      veh_model.GetDistOfLocalizationToCenter();

  if (ad_msg::VEH_TRAILER_STATUS_CONNECTED ==
      act_planning_shared_data_->chassis.trailer_status) {
    param_.vehicle_width =
        common::Max(veh_model.GetVehicleWidth(), veh_model.GetTrailerWidth());
  }

  LOG_INFO(5) << "Vehicle Width = " << param_.vehicle_width
              << ", Trailer Status = " << act_planning_shared_data_->chassis.trailer_status;
}

void ActionPlanningChangingLane::UpdateChangingLaneStatus(
    const ActionPlanningDataSource& data_source) {
  Int32_t cur_chg_lane_sta = data_source.changing_lane_rsp->status();

  if (ChangingLaneRsp::RSP_CHANGING_LANE_REFUSE ==
      data_source.changing_lane_rsp->response()) {
    switch (changing_lane_req_.request()) {
    case (ChangingLaneReq::REQ_CHANGING_LANE_LEFT):
      act_planning_shared_data_->AddEventReporting(
            ActionPlanningSharedData::EVENT_TYPE_REFUSE_CHANGING_LANE_LEFT,
            data_source.changing_lane_rsp->refuse_reason());
      break;
    case (ChangingLaneReq::REQ_CHANGING_LANE_RIGHT):
      act_planning_shared_data_->AddEventReporting(
            ActionPlanningSharedData::EVENT_TYPE_REFUSE_CHANGING_LANE_RIGHT,
            data_source.changing_lane_rsp->refuse_reason());
      break;
    case (ChangingLaneReq::REQ_CHANGING_LANE_ABORT):
      act_planning_shared_data_->AddEventReporting(
            ActionPlanningSharedData::EVENT_TYPE_REFUSE_ABORT_CHANGING_LANE,
            data_source.changing_lane_rsp->refuse_reason());
      break;
    default:
      // nothing to do
      break;
    }
  }

  switch (cur_chg_lane_sta) {
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_NONE): {
    switch (changing_lane_req_.request()) {
    case (ChangingLaneReq::REQ_CHANGING_LANE_LEFT):
      // 20231218 longjiaoy start
      switch (changing_lane_status_) {
      case (ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_II):
        act_planning_shared_data_->AddEventReporting(
              ActionPlanningSharedData::EVENT_TYPE_COMPLETE_CHANGING_LANE_LEFT);
        break;
      default:
        // nothing to do
        break;
      }
      // 20231218 longjiaoy end
//      act_planning_shared_data_->AddEventReporting(
//            ActionPlanningSharedData::EVENT_TYPE_COMPLETE_CHANGING_LANE_LEFT);
        break;
    case (ChangingLaneReq::REQ_CHANGING_LANE_RIGHT):
//      act_planning_shared_data_->AddEventReporting(
//            ActionPlanningSharedData::EVENT_TYPE_COMPLETE_CHANGING_LANE_RIGHT);
      // 20231218 longjiaoy start
      switch (changing_lane_status_) {
      case (ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_II):
        act_planning_shared_data_->AddEventReporting(
              ActionPlanningSharedData::EVENT_TYPE_COMPLETE_CHANGING_LANE_RIGHT);
        break;
      default:
        // nothing to do
        break;
      }
      // 20231218 longjiaoy end
      break;
//    case (ChangingLaneReq::REQ_CHANGING_LANE_ABORT): {
//      // nothing to do
//      switch (changing_lane_status_) {
//      case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_I):
//      case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_II):
//        act_planning_shared_data_->AddEventReporting(
//              ActionPlanningSharedData::EVENT_TYPE_COMPLETE_CHANGING_LANE_RIGHT);
//        break;
//      case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I):
//      case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II):
//        act_planning_shared_data_->AddEventReporting(
//              ActionPlanningSharedData::EVENT_TYPE_COMPLETE_CHANGING_LANE_LEFT);
//        break;
//      default:
//        // nothing to do
//        break;
//      }
//    }
//      break;
    default:
      // nothing to do
      break;
    }
  }
    break;
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_I):
    act_planning_shared_data_->AddEventReporting(
          ActionPlanningSharedData::EVENT_TYPE_START_CHANGING_LANE_LEFT);
    break;
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_II):
    act_planning_shared_data_->AddEventReporting(
          ActionPlanningSharedData::EVENT_TYPE_START_CHANGING_LANE_LEFT);
    break;
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_I):
    act_planning_shared_data_->AddEventReporting(
          ActionPlanningSharedData::EVENT_TYPE_START_CHANGING_LANE_RIGHT);
    break;
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_II):
    act_planning_shared_data_->AddEventReporting(
          ActionPlanningSharedData::EVENT_TYPE_START_CHANGING_LANE_RIGHT);
    break;
  case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_I):
    act_planning_shared_data_->AddEventReporting(
          ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_RIGHT,
          data_source.changing_lane_rsp->refuse_reason());
    break;
  case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_II):
    act_planning_shared_data_->AddEventReporting(
          ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_RIGHT,
          data_source.changing_lane_rsp->refuse_reason());
    break;
  case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I):
    act_planning_shared_data_->AddEventReporting(
          ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_LEFT,
          data_source.changing_lane_rsp->refuse_reason());
    break;
  case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II):
    act_planning_shared_data_->AddEventReporting(
          ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_LEFT,
          data_source.changing_lane_rsp->refuse_reason());
    break;
  default:
    // nothing to do
    break;
  }

  if (ChangingLaneRsp::RSP_CHANGING_LANE_REFUSE ==
      data_source.changing_lane_rsp->response()) {
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "$$$ Received refusing changing lane response.";
#endif
    changing_lane_req_.set_request(ChangingLaneReq::REQ_CHANGING_LANE_NONE);
  }
  if (ChangingLaneRsp::STATUS_CHANGING_LANE_NONE == cur_chg_lane_sta) {
    changing_lane_req_.set_request(ChangingLaneReq::REQ_CHANGING_LANE_NONE);
  }
  pre_changing_lane_status_  = changing_lane_status_;
  changing_lane_status_ = cur_chg_lane_sta;

  UpdateChangingLaneStatusByRef(data_source);
}

void ActionPlanningChangingLane::UpdateChangingLaneStatusByRef(
    const ActionPlanningDataSource &data_source) {
  const driv_map::DrivingMapWrapper* driving_map = data_source.driving_map;
  if (Nullptr_t == driving_map) {
    LOG_ERR << "Invalid driving map.";
    return;
  }
  const ActionPlanningSharedData::LaneInfo& cur_lane =
      act_planning_shared_data_->lane_info_list[ActionPlanningSharedData::LANE_IDX_CURR];

  plan_var_t curr_l_ref = driving_map->GetProjPointOnMajorRefLine().l;
  plan_var_t prev_l_ref = prev_lat_offset_to_major_ref_;
  prev_lat_offset_to_major_ref_ = curr_l_ref;

  Int32_t ref_changed_flag = 0;
  if ((prev_l_ref > 0.0F) && (curr_l_ref < 0.0F) &&
      ((prev_l_ref - curr_l_ref) > (cur_lane.risk_info.boundary.left_width- 0.1F))) {
    // 参考线向左切换了, 使用之前的误差变化率
    ref_changed_flag = 1;
  } else if ((prev_l_ref < 0.0F) && (curr_l_ref > 0.0F) &&
             ((curr_l_ref - prev_l_ref) > (cur_lane.risk_info.boundary.right_width - 0.1F))) {
    // 参考线向右切换了, 使用之前的误差变化率
    ref_changed_flag = 2;
  } else {
    // 参考线未发生切换
    ref_changed_flag = 0;
  }

  if (changing_lane_status_ == ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_I) {
    if (ref_changed_flag == 1) {
      changing_lane_status_ = ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_II;
    }
  } else if (changing_lane_status_ == ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_I) {
    if (ref_changed_flag == 1) {
      changing_lane_status_ = ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_II;
    }
  } else if (changing_lane_status_ == ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_I) {
    if (ref_changed_flag == 2) {
      changing_lane_status_ = ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_II;
    }
  } else if (changing_lane_status_ == ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I) {
    if (ref_changed_flag == 2) {
      changing_lane_status_ = ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II;
    }
  } else if (changing_lane_status_ == ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_II) {
    if (ref_changed_flag == 2) {
      changing_lane_status_ = ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_I;
    }
  } else if (changing_lane_status_ == ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_II) {
    if (ref_changed_flag == 2) {
      changing_lane_status_ = ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_I;
    }
  }  else if (changing_lane_status_ == ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_II) {
    if (ref_changed_flag == 1) {
      changing_lane_status_ = ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_I;
    }
  }  else if (changing_lane_status_ == ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II) {
    if (ref_changed_flag == 1) {
      changing_lane_status_ = ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I;
    }
  } else {
    // nothig to do
  }
}

Int32_t ActionPlanningChangingLane::GetChangingLaneReqType(
    const ActionPlanningDataSource& data_source,
    ActionPlanningResult* planning_result) {
  Int32_t request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;


#if 1
  if (!planning_result->enable_alc) {
    // 设置项，不允许变道
    return (request_type);
  }
#else
  /// For test only
  if (planning_result->enable_alc) {
    request_type = EventChangingLaneReq::REQ_CHANGING_LANE_ABORT;
    return (request_type);
  }
#endif

#if 0
  Int32_t hmi_request_type = 0;
  if (hmi_changing_lane_req_.request() != data_source.planning_settings->changing_lane_req) {
    hmi_changing_lane_req_.set_request(data_source.planning_settings->changing_lane_req);
    hmi_changing_lane_req_.set_sequence(hmi_changing_lane_req_.sequence() + 1);
    hmi_request_type =  hmi_changing_lane_req_.request();
  } else {
    hmi_request_type = 0;
  }

#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
  LOG_INFO(5) << "$$$ Changing lane request from HMI = "
              << hmi_changing_lane_req_.request();
#endif
  /// 人机交互模块的请求
  if (1 == hmi_request_type) {
    // Request to change to left lane
    request_type = ChangingLaneReq::REQ_CHANGING_LANE_LEFT;
  } else if (2 == hmi_request_type) {
    // Request to change to right lane
    request_type = ChangingLaneReq::REQ_CHANGING_LANE_RIGHT;
  } else if (3 == hmi_request_type) {
    // Request to abort changing lane
    request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
  } else {
    request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
  }
#else
  #if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "$$$ Changing lane request from HMI = "
                << data_source.planning_settings->changing_lane_req;
  #endif
    /// 人机交互模块的请求
    if (1 == data_source.planning_settings->changing_lane_req) {
      // Request to change to left lane
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_LEFT;
    } else if (2 ==  data_source.planning_settings->changing_lane_req) {
      // Request to change to right lane
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_RIGHT;
    } else if (3 ==  data_source.planning_settings->changing_lane_req) {
      // Request to abort changing lane
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_ABORT;
    } else {
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
    }
#endif
  /* k006 pengc 2023-2-21 (begin) */
  // 非自动模式下不执行变道请求
  if ((ad_msg::VEH_DRIVING_MODE_ROBOTIC != data_source.chassis->driving_mode)) {
    request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
  }
  /* k006 pengc 2023-2-21 (end) */

  if((data_source.chassis->v < 40.0F/3.6F)||
       (data_source.chassis->v > 100.0F/3.6F)){
       /* D006 fuyuanyi 2023-11-16 (begin) */
       // 速度不满足换道条件，不响应左右换道请求，同时清除计时器
      if (ChangingLaneReq::REQ_CHANGING_LANE_LEFT == request_type) {
        act_planning_shared_data_->AddEventReporting(
              ActionPlanningSharedData::EVENT_TYPE_REFUSE_CHANGING_LANE_LEFT,
              REFUSE_CHANGINHG_LANE_REASON_OUT_OF_SPEEDLIMIT);
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      } else if (ChangingLaneReq::REQ_CHANGING_LANE_RIGHT == request_type) {
        act_planning_shared_data_->AddEventReporting(
              ActionPlanningSharedData::EVENT_TYPE_REFUSE_CHANGING_LANE_RIGHT,
              REFUSE_CHANGINHG_LANE_REASON_OUT_OF_SPEEDLIMIT);
      request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
      } else {
       // nothing to do;
      }
      act_planning_shared_data_->timers.hold_changing_lane_req.Stop();
      act_planning_shared_data_->timers.hold_changing_lane_req.SetUserDataInt32(
            0, ChangingLaneReq::REQ_CHANGING_LANE_NONE);
    }
     /* D006 fuyuanyi 2023-11-16 (end) */


#if 1
  // 保持一定时间的变道请求
  if (ChangingLaneReq::REQ_CHANGING_LANE_NONE != request_type) {
    if (ChangingLaneReq::REQ_CHANGING_LANE_ABORT != request_type) {
      act_planning_shared_data_->timers.hold_changing_lane_req.SetTimeout(30*1000);
      act_planning_shared_data_->timers.hold_changing_lane_req.SetUserDataInt32(0, request_type);
      act_planning_shared_data_->timers.hold_changing_lane_req.Restart();
    } else {
      if (ChangingLaneReq::REQ_CHANGING_LANE_ABORT == request_type) {
        /* k007 longjiaoy 2023-02-28 (start) */
        if (act_planning_shared_data_->timers.hold_changing_lane_req.IsActive()) {
          // 增加变道请求保持中，人工中断指令通知
          if (ChangingLaneReq::REQ_CHANGING_LANE_LEFT ==
              act_planning_shared_data_->timers.hold_changing_lane_req.GetUserDataInt32(0)) {
            act_planning_shared_data_->AddEventReporting(
                  ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_LEFT,
                  REFUSE_CHANGINHG_LANE_REASON_BY_DRIVER);
          } else if (ChangingLaneReq::REQ_CHANGING_LANE_RIGHT ==
                     act_planning_shared_data_->timers.hold_changing_lane_req.GetUserDataInt32(0)){
            act_planning_shared_data_->AddEventReporting(
                  ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_RIGHT,
                  REFUSE_CHANGINHG_LANE_REASON_BY_DRIVER);
          } else {
            // nothing to do
          }
        }
        /* k007 longjiaoy 2023-02-28 (end) */
      }
      act_planning_shared_data_->timers.hold_changing_lane_req.Stop();
      act_planning_shared_data_->timers.hold_changing_lane_req.SetUserDataInt32(
            0, ChangingLaneReq::REQ_CHANGING_LANE_NONE);
    }
  } else {
    if (act_planning_shared_data_->timers.hold_changing_lane_req.IsActive()) {
      request_type = act_planning_shared_data_->timers.hold_changing_lane_req.GetUserDataInt32(0);
    }
  }
#endif

  /* k006 pengc 2023-2-21 (begin) */
  // 非自动模式下不执行变道请求
  if ((ad_msg::VEH_DRIVING_MODE_ROBOTIC != data_source.chassis->driving_mode)) {
    act_planning_shared_data_->timers.hold_changing_lane_req.Stop();
    act_planning_shared_data_->timers.hold_changing_lane_req.SetUserDataInt32(
          0, ChangingLaneReq::REQ_CHANGING_LANE_NONE);
  }
  /* k006 pengc 2023-2-21 (end) */

  // 人机交互模块的请求 最优先
  if (ChangingLaneReq::REQ_CHANGING_LANE_NONE != request_type) {
    // 驾驶员触发的变道
    ForbidAutoChangingLane();

    return (request_type);
  }

  /// 若配置参数/设置项, 不允许自主变道
  if (!planning_result->enable_ngp) {
    return (request_type);
  }
  if (!act_planning_shared_data_->config.enable_auto_changing_lane) {
    return (request_type);
  }

  const ActionPlanningSharedData::LaneInfo& curr_lane =
      act_planning_shared_data_->lane_info_list[ActionPlanningSharedData::LANE_IDX_CURR];
  const ActionPlanningSharedData::LaneInfo& left_lane =
      act_planning_shared_data_->lane_info_list[ActionPlanningSharedData::LANE_IDX_LEFT_1];
  const ActionPlanningSharedData::LaneInfo& right_lane =
      act_planning_shared_data_->lane_info_list[ActionPlanningSharedData::LANE_IDX_RIGHT_1];


  if (left_lane.valid &&
      !left_lane.risk_info.has_risk &&
      !left_lane.risk_info.has_uncertain_obj &&
      (left_lane.risk_info.lane_quality >= 2) /*&&
      (left_lane.risk_info.cost < curr_lane.risk_info.cost)*/) {
    // 可以向左变道避让
    changing_lane_req_.set_allow_auto_changing_to_left(true);
  }
  if (right_lane.valid &&
      !right_lane.risk_info.has_risk &&
      !right_lane.risk_info.has_uncertain_obj &&
      (right_lane.risk_info.lane_quality >= 2) /*&&
      (right_lane.risk_info.cost < curr_lane.risk_info.cost)*/) {
    // 可以向右变道避让
    changing_lane_req_.set_allow_auto_changing_to_right(true);
  }

  /// 若正在变道，则不允许请求主动变道
  if (ChangingLaneRsp::STATUS_CHANGING_LANE_NONE != changing_lane_status_) {
    return (request_type);
  }

  /// 轨迹规划模块主动请求变道
  bool changing_lane_by_obstacles = false;
  switch (data_source.changing_lane_req_from_trj_planning->request()) {
  case (ChangingLaneReq::REQ_CHANGING_LANE_LEFT):
    changing_lane_by_obstacles = true;
    request_type = ChangingLaneReq::REQ_CHANGING_LANE_LEFT;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "    TrjPlanning request changing to left lane.";
#endif
    break;
  case (ChangingLaneReq::REQ_CHANGING_LANE_RIGHT):
    changing_lane_by_obstacles = true;
    request_type = ChangingLaneReq::REQ_CHANGING_LANE_RIGHT;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "    TrjPlanning request changing to right lane.";
#endif
    break;
  default:
    break;
  }
  if (ChangingLaneReq::REQ_CHANGING_LANE_NONE == request_type) {
    /// 变道锁定时间内不允许主动变道
    if (!act_planning_shared_data_->timers.forbid_auto_changing_lane.IsActive()) {
      /// 行为规划模块主动请求变道
      request_type = ReqChangingLaneAutomatically(data_source);
    }
  }

//  if ((curr_lane.valid) &&
//      (curr_lane.risk_info.forward_obj_info.valid) &&
//      ((curr_lane.risk_info.forward_obj_info.ttc < 5.0F)
//       || (common::com_abs(curr_lane.risk_info.forward_obj_info.t_gap) < 3.0F))) {
//    request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;
//  }

  if ((changing_lane_by_obstacles) &&
      (ChangingLaneReq::REQ_CHANGING_LANE_NONE != request_type)) {
    // 因为障碍物产生的变道
    ForbidAutoChangingLane();
  }

  return (request_type);
}

Int32_t ActionPlanningChangingLane::ReqChangingLaneAutomatically(
    const ActionPlanningDataSource& data_source) {
  Int32_t request_type = ChangingLaneReq::REQ_CHANGING_LANE_NONE;

  /// TODO: 根据routing信息进行主动变道

  Int32_t nearest_lane_index = -1;
  common::PathPoint nearest_point_on_lane;
  Int32_t nearest_idx_in_neighbor = 0;
  common::StaticVector<driv_map::NeighborsLaneInfo,
      driv_map::MAX_NEIGHBOR_LANE_NUM> neighbor_lanes;
  data_source.driving_map->GetNearestLaneToCurrPosition(
      &nearest_lane_index, &nearest_point_on_lane,
      &nearest_idx_in_neighbor, &neighbor_lanes);
  if (!data_source.driving_map->IsValidLaneIndex(nearest_lane_index)) {
    // 当前车道无效
    return (request_type);
  }
  Int32_t neighbor_lanes_num = neighbor_lanes.Size();
  if (neighbor_lanes_num < 2) {
    // 当前只有一条车道
    return (request_type);
  }
  if (data_source.driving_map->IsOnRoutingSegment(
        nearest_lane_index, nearest_point_on_lane.s)) {
    // 当前处于routing车道
    return (request_type);
  }

  Int32_t min_offset = neighbor_lanes_num;
  Int32_t nearest_routing_in_neighbor = nearest_idx_in_neighbor;
  for (Int32_t i = 0; i < neighbor_lanes_num; ++i) {
    const driv_map::NeighborsLaneInfo& neighbor = neighbor_lanes[i];
    if (!neighbor.valid_lat) {
      continue;
    }
    if (!neighbor.valid_lon) {
      continue;
    }
    if (i == nearest_idx_in_neighbor) {
      continue;
    }
    if (data_source.driving_map->IsOnRoutingSegment(
          neighbor.lane_index, neighbor.proj_point.s)) {
      Int32_t offset = common::com_abs(i - nearest_idx_in_neighbor);
      if (offset < min_offset) {
        min_offset = offset;
        nearest_routing_in_neighbor = i;
      }
    }
  }

  if (nearest_routing_in_neighbor > nearest_idx_in_neighbor) {
    request_type = ChangingLaneReq::REQ_CHANGING_LANE_LEFT;
  } else if (nearest_routing_in_neighbor < nearest_idx_in_neighbor) {
    request_type = ChangingLaneReq::REQ_CHANGING_LANE_RIGHT;
  } else {
    // nothing to do
  }

  return (request_type);
}

void ActionPlanningChangingLane::ForbidAutoChangingLane() {
  const ActionPlanningSharedData::LaneInfo& curr_lane =
      act_planning_shared_data_->lane_info_list[ActionPlanningSharedData::LANE_IDX_CURR];

  Int64_t hold_time = 10*1000;

  plan_var_t t_gap = 100.0F;
  const ActionPlanningLaneRiskAnalyser::RiskyObj& forward_obj =
      curr_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_FRONT];
  const ActionPlanningLaneRiskAnalyser::RiskyObj& left_front_obj =
      curr_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_LEFT_FRONT];
  const ActionPlanningLaneRiskAnalyser::RiskyObj& right_front_obj =
      curr_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_RIGHT_FRONT];

  if (forward_obj.valid && (forward_obj.t_gap < t_gap)) {
    t_gap = forward_obj.t_gap;
  }
  if (left_front_obj.valid && (left_front_obj.t_gap < t_gap)) {
    t_gap = left_front_obj.t_gap;
  }
  if (right_front_obj.valid && (right_front_obj.t_gap < t_gap)) {
    t_gap = right_front_obj.t_gap;
  }

  if (curr_lane.valid && (t_gap > 0.0F) && (t_gap < 20.0F)) {
    hold_time = 2 * 1000 * t_gap;
  }

  if (hold_time < 5*1000) {
    hold_time = 5 * 1000;
  }
  if (hold_time > 30*1000) {
    hold_time = 30*1000;
  }
  if (act_planning_shared_data_->chassis.v < 15.0F/3.6F) {
    if (hold_time < 60*1000) {
      hold_time = 60*1000;
    }
  }
  act_planning_shared_data_->timers.forbid_auto_changing_lane.SetTimeout(hold_time);
  act_planning_shared_data_->timers.forbid_auto_changing_lane.Restart();
}

void ActionPlanningChangingLane::ForbidAlcChangeLane() {

  Int64_t hold_time = 2*1000;

  act_planning_shared_data_->timers.forbid_alc_changing_lane.SetTimeout(hold_time);
  act_planning_shared_data_->timers.forbid_alc_changing_lane.Restart();
}

void ActionPlanningChangingLane::StopHoldChangeLaneReqByRefuseReason(
    Int32_t refuse_reason) {
  if ((REFUSE_CHANGINHG_LANE_REASON_IN_TUNNEL == refuse_reason) ||
      (REFUSE_CHANGINHG_LANE_REASON_IN_CURVE == refuse_reason) ||
      (REFUSE_CHANGINHG_LANE_REASON_NEAR_RAMP == refuse_reason)) {
    act_planning_shared_data_->timers.hold_changing_lane_req.Stop();
    act_planning_shared_data_->timers.hold_changing_lane_req.SetUserDataInt32(
          0, ChangingLaneReq::REQ_CHANGING_LANE_NONE);
  }
}

/*******************************************************************************
 * @brief 判断车道条件是否允许变道(障碍物, 车道线类型, 车道线质量 etc.)
 * @param[in]  data_source      行为规划相关数据
 * @param[in]  request_type     请求变道的方向
 * @return true:允许变道
 * @return false:不允许变道
 *******************************************************************************/
bool ActionPlanningChangingLane::JudgeChangingLaneConditions(
    Int32_t request_type, Int32_t* refuse_reason) {
  *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_NONE;

  Int32_t lane_idx = -1;
  if (ChangingLaneReq::REQ_CHANGING_LANE_LEFT == request_type) {
    lane_idx = ActionPlanningSharedData::LANE_IDX_LEFT_1;
  } else if (ChangingLaneReq::REQ_CHANGING_LANE_RIGHT == request_type) {
    lane_idx = ActionPlanningSharedData::LANE_IDX_RIGHT_1;
  } else {
    LOG_ERR << "Invalid type of changing lane request.";
  }

  if (lane_idx < 0) {
    return false;
  }

  /* k003 longjiaoy 2022-11-28 (start) */
  const ActionPlanningSharedData::LaneInfo& curr_lane =
      act_planning_shared_data_->lane_info_list[
      ActionPlanningSharedData::LANE_IDX_CURR];
  /* k003 longjiaoy 2022-11-28 (end) */
  const ActionPlanningSharedData::LaneInfo& tar_lane =
      act_planning_shared_data_->lane_info_list[lane_idx];

  if (!tar_lane.valid) {
    *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_INVALID_LANE;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "$$$ Refuse channging lane (Invalid lane).";
#endif
    return false;
  }

  if (tar_lane.risk_info.lane_quality < 2) {
    *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_LOW_QUALITY_LANE;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "$$$ Refuse changing lane (Low quality lane).";
#endif
    return false;
  }

  // 判断当前车道的风险
  if (curr_lane.risk_info.has_risk) {
    plan_var_t t = 0;
    Int32_t lower = common::LerpInOrderedTable(
          param_.dist_limit_table_for_changine_lane,
          act_planning_shared_data_->chassis.v, &t);
    Float32_t safe_change_dist =
         common::Lerp(
          param_.dist_limit_table_for_changine_lane[lower].value,
          param_.dist_limit_table_for_changine_lane[lower+1].value, t);

    // 距离前向障碍是否过近
    const ActionPlanningLaneRiskAnalyser::RiskyObj& forward_obj_1 =
        curr_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_FRONT];
    const ActionPlanningLaneRiskAnalyser::RiskyObj& forward_obj_2 =
        curr_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_FRONT];
    /* D002 fuyuanyi 2023-11-1 (begin) */
    // 纵向距离不减去到车定位点
    /* D002 fuyuanyi 2023-11-1 (end) */
    Float32_t forward_obj_1_dist = forward_obj_1.lon_dist ;

    if ((forward_obj_1.valid && (forward_obj_1_dist < safe_change_dist)) ||
        (forward_obj_2.valid && (forward_obj_2.ttc < 10.0F))) {
      *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_OBSTACLE;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
      LOG_INFO(5) << "$$$ Refuse changing lane (curr lane's front obj t_gap("
                  << forward_obj_1.t_gap
                  << ") or ttc("
                  << forward_obj_2.ttc
                  << ") is little).";
#endif
      return false;
    }

    /* D001 fuyuanyi 2023-6-30 (begin) */
  if (!act_planning_shared_data_->config.enable_action_planning_force_changing_lane){
    // 是否存在紧靠车身的障碍物
    const ActionPlanningLaneRiskAnalyser::RiskyObj& closed_obj_1 =
        curr_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_CLOSE];
    const ActionPlanningLaneRiskAnalyser::RiskyObj& closed_obj_2 =
        curr_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_CLOSE];
    if ((closed_obj_1.valid) ||
        (closed_obj_2.valid)) {
      *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_OBSTACLE;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
      LOG_INFO(5) << "$$$ Refuse changing lane (curr lane exist closed obj).";
#endif
      return false;
    }

    // 目标车道方向的侧方是否存在障碍物
    if (ChangingLaneReq::REQ_CHANGING_LANE_LEFT == request_type) {
      const ActionPlanningLaneRiskAnalyser::RiskyObj& left_obj_1 =
          curr_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_LEFT];
      const ActionPlanningLaneRiskAnalyser::RiskyObj& left_obj_2 =
          curr_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_LEFT];
      if ((left_obj_1.valid) ||
          (left_obj_2.valid)) {
        *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_OBSTACLE;
  #if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
        LOG_INFO(5) << "$$$ Refuse changing lane (curr lane exist lateral obj).";
  #endif
        return false;
      }
    } else if (ChangingLaneReq::REQ_CHANGING_LANE_RIGHT == request_type) {
      const ActionPlanningLaneRiskAnalyser::RiskyObj& right_obj_1 =
          curr_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_RIGHT];
      const ActionPlanningLaneRiskAnalyser::RiskyObj& right_obj_2 =
          curr_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_RIGHT];
      if ((right_obj_1.valid) ||
          (right_obj_2.valid)) {
        *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_OBSTACLE;
  #if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
        LOG_INFO(5) << "$$$ Refuse changing lane (curr lane exist lateral obj).";
  #endif
        return false;
      }
    } else {
      // nothing to do
    }
  }
  
     /* D001 fuyuanyi 2023-6-30 (end) */
 }


  /* D001 fuyuanyi 2023-7-03 (begin) */
  // 判断目标车道的风险
  if (tar_lane.risk_info.has_risk) {
    const ActionPlanningLaneRiskAnalyser::RiskyObj& forward_obj_1 =
        tar_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_FRONT];
    const ActionPlanningLaneRiskAnalyser::RiskyObj& forward_obj_2 =
        tar_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_FRONT]; 

   if (forward_obj_1.valid || forward_obj_2.valid) {
    // 目标车道前方障碍物引起的风险需要判断距离
     plan_var_t t = 0;
       Int32_t lower = common::LerpInOrderedTable(
           param_.dist_limit_table_for_changine_lane,
           act_planning_shared_data_->chassis.v, &t);
       Float32_t safe_change_dist =
          common::Lerp(
           param_.dist_limit_table_for_changine_lane[lower].value,
           param_.dist_limit_table_for_changine_lane[lower+1].value, t);
     Float32_t forward_obj_1_dist = forward_obj_1.lon_dist;
      if ((forward_obj_1.valid && (forward_obj_1_dist < safe_change_dist)) ||
          (forward_obj_2.valid && (forward_obj_2.ttc < 10.0F))){
    *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_OBSTACLE;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "$$$ Refuse changing lane (Has risk).";
#endif
    return false;
      }
   } else  {

     if (!act_planning_shared_data_->config.enable_action_planning_force_changing_lane){
       *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_OBSTACLE;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
       LOG_INFO(5) << "$$$ Refuse changing lane (Has risk).";
#endif
       // 目标车道后方位障碍物引起的风险
       return false;
     }
   }

  }
  /* D001 fuyuanyi 2023-7-03 (end) */

  if (tar_lane.risk_info.has_uncertain_obj) {
    *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_UNCERTAIN_OBSTACLE;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "$$$ Refuse changing lane (Has uncertain obj).";
#endif
    return false;
  }

  /* k003 longjiaoy 2022-11-28 (start) */
#if 1
  if ((ChangingLaneReq::REQ_CHANGING_LANE_LEFT == request_type) &&
      !curr_lane.risk_info.boundary.allow_to_change_left) {
    *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_SOLID_BOUNDARY;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "$$$ Refuse changing lane (Solid boundary).";
#endif
    return false;
  }
  if ((ChangingLaneReq::REQ_CHANGING_LANE_RIGHT == request_type) &&
      !curr_lane.risk_info.boundary.allow_to_change_right) {
    *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_SOLID_BOUNDARY;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "$$$ Refuse changing lane (Solid boundary).";
#endif
    return false;
  }
#endif
  /* k003 longjiaoy 2022-11-28 (end) */

  // 地理围栏
  const common::StaticVector<ad_msg::SceneStory,
      ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM>& story_list =
      driving_map_->GetSceneStoryList();

  for (Int32_t i = 0; i < story_list.Size(); ++i) {
    const ad_msg::SceneStory& story = story_list[i];

    // 在隧道内不变道
    if (ad_msg::SCENE_STORY_TYPE_CLOSE_TO_TUNNEL == story.type) {
      if (story.area.DistanceToArea() < 20.0F) {
        *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_IN_TUNNEL;
//#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
        LOG_INFO(5) << "$$$ Refuse changing lane (In tunel).";
//#endif
        return false;
      }
    }
    // 弯道半径 < 500m 不变道
    if (ad_msg::SCENE_STORY_TYPE_CLOSE_TO_CURVE == story.type) {
      if (story.area.DistanceToArea() < 20.0F) {
        *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_IN_CURVE;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
        LOG_INFO(5) << "$$$ Refuse changing lane (In curve).";
#endif
        return false;
      }
    }
    // 匝道，不向匝道变道
    if (ad_msg::SCENE_STORY_TYPE_CLOSE_TO_RAMP == story.type) {
     if (story.area.DistanceToArea() < 100.0F) {
        *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_NEAR_RAMP;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
        LOG_INFO(5) << "$$$ Refuse changing lane (in ramp).";
#endif
        /// TODO: 判断目标车道是否通向匝道
        return false;
     }
    }
    // 路过上/下匝道口，不向匝道变道
    if (ad_msg::SCENE_STORY_TYPE_PASSING_RAMP == story.type) {
     if (story.area.DistanceToArea() < 100.0F) {
        *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_NEAR_RAMP;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
        LOG_INFO(5) << "$$$ Refuse changing lane (near ramp).";
#endif
        /// TODO: 判断目标车道是否通向匝道
        return false;
     }
    }
  }

  // 弯道半径 < 500m 不变道
  Int32_t major_ref_idx = driving_map_->GetMajorReferenceLineIndex();
  if (driving_map_->IsValidReferenceLineIndex(major_ref_idx)) {
    const common::StaticVector<common::Path::CurveSegment,
        common::Path::kMaxCurveSegmentNum>& curvature_info =
        driving_map_->GetSmoothReferenceLineCurveInfo(major_ref_idx);
    const common::PathPoint& proj_on_path =
        driving_map_->GetProjPointOnMajorRefLine();
    Int32_t curve_seg_num = curvature_info.Size();
    for (Int32_t i = 0; i < curve_seg_num; ++i) {
      const common::Path::CurveSegment& curve_seg = curvature_info[i];
      plan_var_t abs_curvature = common::com_abs(curve_seg.max_curvature);

#if 0
      LOG_INFO(5) << "curve[" << i << "]: type=" << curve_seg.type
                  << ", start_s=" << curve_seg.start_s
                  << ", len=" << curve_seg.length
                  << ", curvature=" << curve_seg.max_curvature
                  << ", radius=" << ((abs_curvature > 0.0001F) ? 1.0F/curve_seg.max_curvature : 10000);
#endif
      if (common::Path::TYPE_STRAIGHT == curve_seg.type) {
        continue;
      }
      if ((curve_seg.start_s + curve_seg.length) < proj_on_path.s) {
        continue;
      }
      if ((curve_seg.start_s - 15.0F) > proj_on_path.s) {
        continue;
      }

      if (abs_curvature > 1.0F/500.0F) {
        *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_IN_CURVE;
        act_planning_shared_data_->timers.forbid_changing_lane_in_curve.SetTimeout(3*1000);
        act_planning_shared_data_->timers.forbid_changing_lane_in_curve.Restart();

        return false;
      }
    }
  }
  if (act_planning_shared_data_->timers.forbid_changing_lane_in_curve.IsActive()) {
    *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_IN_CURVE;
//#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "$$$ Refuse changing lane (in curve).";
//#endif
    return false;
  }

  /// TODO: 经过匝道，不向匝道变道

  // 目标车道宽度过窄，不变道
  if ((tar_lane.risk_info.boundary.left_width < 0.5F*param_.vehicle_width+0.1F) ||
      (tar_lane.risk_info.boundary.right_width < 0.5F*param_.vehicle_width+0.1F)) {
    *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_INVALID_LANE;
//#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
    LOG_INFO(5) << "$$$ Refuse changing lane (too small width of target lane)"
                << ", left_width=" << tar_lane.risk_info.boundary.left_width
                << ", right_width=" << tar_lane.risk_info.boundary.right_width;
//#endif
    return false;
  }

  return true;
}

bool ActionPlanningChangingLane::JudgeChangingLaneConditionsInChangingLane(
    Int32_t request_type, Int32_t* refuse_reason, Int32_t* event_type) {
  *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_NONE;
  *event_type = ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_LEFT;

  Int32_t ret_lane_idx = -1;
  const Int32_t cur_lane_idx = ActionPlanningSharedData::LANE_IDX_CURR;
  Int32_t tar_lane_idx = -1;
  // 1 ~ left, 2 ~ right
  Int32_t ret_direction = 0;
  Int32_t tar_direction = 0;

 if ((ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_II == changing_lane_status_) ||
     (ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_II == changing_lane_status_)) {
   //增加stage2人工取消的原因
   if (ChangingLaneReq::REQ_CHANGING_LANE_ABORT == request_type) {
     *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_BY_DRIVER;
   }
   return true;
 }

  switch (changing_lane_status_) {
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_NONE):
    break;
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_I):
    ret_lane_idx = ActionPlanningSharedData::LANE_IDX_CURR;
    tar_lane_idx = ActionPlanningSharedData::LANE_IDX_LEFT_1;
    ret_direction = 2;
    tar_direction = 1;
    *event_type = ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_LEFT;
    break;
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_II):
    ret_lane_idx = ActionPlanningSharedData::LANE_IDX_RIGHT_1;
    tar_lane_idx = ActionPlanningSharedData::LANE_IDX_CURR;
    ret_direction = 2;
    tar_direction = 1;
    *event_type = ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_LEFT;
    break;
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_I):
    ret_lane_idx = ActionPlanningSharedData::LANE_IDX_CURR;
    tar_lane_idx = ActionPlanningSharedData::LANE_IDX_RIGHT_1;
    ret_direction = 1;
    tar_direction = 2;
    *event_type = ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_RIGHT;
    break;
  case (ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_II):
    ret_lane_idx = ActionPlanningSharedData::LANE_IDX_LEFT_1;
    tar_lane_idx = ActionPlanningSharedData::LANE_IDX_CURR;
    ret_direction = 1;
    tar_direction = 2;
    *event_type = ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_RIGHT;
    break;
  case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I):
    ret_lane_idx = ActionPlanningSharedData::LANE_IDX_CURR;
    tar_lane_idx = ActionPlanningSharedData::LANE_IDX_RIGHT_1;
    ret_direction = 1;
    tar_direction = 2;
    *event_type = ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_LEFT;
    break;
  case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_II):
    if (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_RIGHT_I != pre_changing_lane_status_) {
      ret_lane_idx = ActionPlanningSharedData::LANE_IDX_CURR;
    } else {
      ret_lane_idx = ActionPlanningSharedData::LANE_IDX_LEFT_1;
    }
    tar_lane_idx = ActionPlanningSharedData::LANE_IDX_CURR;
    ret_direction = 1;
    tar_direction = 2;
    *event_type = ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_LEFT;
    break;
  case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_I):
    ret_lane_idx = ActionPlanningSharedData::LANE_IDX_CURR;
    tar_lane_idx = ActionPlanningSharedData::LANE_IDX_LEFT_1;
    ret_direction = 2;
    tar_direction = 1;
    *event_type = ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_RIGHT;
    break;
  case (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_II):
    if (ChangingLaneRsp::STATUS_ABORT_CHANGING_LANE_TO_LEFT_I != pre_changing_lane_status_) {
      ret_lane_idx = ActionPlanningSharedData::LANE_IDX_CURR;
    } else {
      ret_lane_idx = ActionPlanningSharedData::LANE_IDX_RIGHT_1;
    }
    tar_lane_idx = ActionPlanningSharedData::LANE_IDX_CURR;
    ret_direction = 2;
    tar_direction = 1;
    *event_type = ActionPlanningSharedData::EVENT_TYPE_ABORT_CHANGING_LANE_RIGHT;
    break;
  default:
    break;
  }

  if ((ret_lane_idx < 0) || (tar_lane_idx < 0)) {
    return true;
  }

  Int32_t other_lane_idx = -1;
  if (cur_lane_idx != ret_lane_idx) {
    other_lane_idx = ret_lane_idx;
  } else {
    other_lane_idx = tar_lane_idx;
  }

  const ActionPlanningSharedData::LaneInfo& cur_lane =
      act_planning_shared_data_->lane_info_list[cur_lane_idx];
  const ActionPlanningSharedData::LaneInfo& other_lane =
      act_planning_shared_data_->lane_info_list[other_lane_idx];

  if (!cur_lane.valid || !other_lane.valid) {
    return true;
  }

  bool tar_lane_has_risk = false;
  bool ret_lane_has_risk = false;
  bool risk_is_forward_obj = false;
  // 当前车道
  if (cur_lane.risk_info.has_risk) {
      plan_var_t t= 0;
      Int32_t lower = common::LerpInOrderedTable(
            param_.dist_limit_table_for_changine_lane,
            act_planning_shared_data_->chassis.v, &t);
      Float32_t safe_change_dist =
           common::Lerp(
            param_.dist_limit_table_for_changine_lane[lower].value,
            param_.dist_limit_table_for_changine_lane[lower+1].value, t);

      // 距离前向障碍是否过近
      const ActionPlanningLaneRiskAnalyser::RiskyObj& forward_obj_1 =
          cur_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_FRONT];
      const ActionPlanningLaneRiskAnalyser::RiskyObj& forward_obj_2 =
          cur_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_FRONT];

      Float32_t forward_obj_1_dist = forward_obj_1.lon_dist ;

      if ((forward_obj_1.valid && (forward_obj_1_dist< safe_change_dist)) ||
          (forward_obj_2.valid && (forward_obj_2.ttc < 8.0F))) {
  #if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
        LOG_INFO(5) << "$$$ Refuse changing lane (curr lane's front obj t_gap("
                    << forward_obj_1.t_gap
                    << ") or ttc("
                    << forward_obj_2.ttc
                    << ") is little).";
  #endif
        if (cur_lane_idx == tar_lane_idx) {
          tar_lane_has_risk = true;
        }
        if (cur_lane_idx == ret_lane_idx) {
          risk_is_forward_obj = true;
          ret_lane_has_risk = true;
        }
      }

    // 是否存在紧靠车身的障碍物
    const ActionPlanningLaneRiskAnalyser::RiskyObj& closed_obj_1 =
        cur_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_CLOSE];
    const ActionPlanningLaneRiskAnalyser::RiskyObj& closed_obj_2 =
        cur_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_CLOSE];
    if ((closed_obj_1.valid) || (closed_obj_2.valid)) {
      if (cur_lane_idx == tar_lane_idx) {
        tar_lane_has_risk = true;
      }
      if (cur_lane_idx == ret_lane_idx) {
        ret_lane_has_risk = true;
      }
      risk_is_forward_obj = false;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
      LOG_INFO(5) << "$$$ Refuse changing lane (curr lane exist closed obj).";
#endif
    }

    // 目标车道方向的侧方是否存在障碍物
    const ActionPlanningLaneRiskAnalyser::RiskyObj& left_obj_1 =
        cur_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_LEFT];
    const ActionPlanningLaneRiskAnalyser::RiskyObj& left_obj_2 =
        cur_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_LEFT];
    if ((left_obj_1.valid) || (left_obj_2.valid)) {
      if ((cur_lane_idx == tar_lane_idx) && (1 == tar_direction)) {
        tar_lane_has_risk = true;
      }
      if ((cur_lane_idx == ret_lane_idx) && (1 == ret_direction)) {
        ret_lane_has_risk = true;
      }
      risk_is_forward_obj = false;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
      LOG_INFO(5) << "$$$ Refuse changing lane (curr lane exist left lateral obj).";
#endif
    }
    const ActionPlanningLaneRiskAnalyser::RiskyObj& right_obj_1 =
        cur_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_RIGHT];
    const ActionPlanningLaneRiskAnalyser::RiskyObj& right_obj_2 =
        cur_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_RIGHT];
    if ((right_obj_1.valid) || (right_obj_2.valid)) {
      if ((cur_lane_idx == tar_lane_idx) && (2 == tar_direction)) {
        tar_lane_has_risk = true;
      }
      if ((cur_lane_idx == ret_lane_idx) && (2 == ret_direction)) {
        ret_lane_has_risk = true;
      }
      risk_is_forward_obj = false;
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
      LOG_INFO(5) << "$$$ Refuse changing lane (curr lane exist right lateral obj).";
#endif
    }
  }

  // 另一个车道
  if (other_lane.risk_info.has_risk) {
      // 距离前向障碍是否过近
      plan_var_t t= 0;
      Int32_t lower = common::LerpInOrderedTable(
            param_.dist_limit_table_for_changine_lane,
            act_planning_shared_data_->chassis.v, &t);
      Float32_t safe_change_dist =
           common::Lerp(
            param_.dist_limit_table_for_changine_lane[lower].value,
            param_.dist_limit_table_for_changine_lane[lower+1].value, t);

      const ActionPlanningLaneRiskAnalyser::RiskyObj& forward_obj_1 =
          other_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_FRONT];
      const ActionPlanningLaneRiskAnalyser::RiskyObj& forward_obj_2 =
          other_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_FRONT];
    Float32_t forward_obj_1_dist = forward_obj_1.lon_dist ;

      if ((forward_obj_1.valid && (forward_obj_1_dist < safe_change_dist)) ||
          (forward_obj_2.valid && (forward_obj_2.ttc < 8.0F))) {
        if (cur_lane_idx != tar_lane_idx) {
          tar_lane_has_risk = true;
        }
        if (cur_lane_idx != ret_lane_idx) {
          ret_lane_has_risk = true;
        }
  #if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
        LOG_INFO(5) << "$$$ Refuse changing lane (tar lane's front obj t_gap("
                    << forward_obj_1.t_gap
                    << ") or ttc("
                    << forward_obj_2.ttc
                    << ") is little).";
  #endif
      }

    // 是否存在紧靠车身的障碍物
    const ActionPlanningLaneRiskAnalyser::RiskyObj& closed_obj_1 =
        other_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_CLOSE];
    const ActionPlanningLaneRiskAnalyser::RiskyObj& closed_obj_2 =
        other_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_CLOSE];
    if ((closed_obj_1.valid) || (closed_obj_2.valid)) {
      if (cur_lane_idx != tar_lane_idx) {
        tar_lane_has_risk = true;
      }
      if (cur_lane_idx != ret_lane_idx) {
        ret_lane_has_risk = true;
      }
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
      LOG_INFO(5) << "$$$ Refuse changing lane (tar lane exist closed obj).";
#endif
    }

    // 目标车道方向的左侧方是否存在障碍物
    const ActionPlanningLaneRiskAnalyser::RiskyObj& left_obj_1 =
        other_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_LEFT];
    const ActionPlanningLaneRiskAnalyser::RiskyObj& left_obj_2 =
        other_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_LEFT];
    if ((left_obj_1.valid) || (left_obj_2.valid)) {
      if (cur_lane_idx != tar_lane_idx) {
        tar_lane_has_risk = true;
      }
      if (cur_lane_idx != ret_lane_idx) {
        ret_lane_has_risk = true;
      }
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
      LOG_INFO(5) << "$$$ Refuse changing lane (curr lane exist left lateral obj).";
#endif
    }

    // 目标车道方向的右侧方是否存在障碍物
    const ActionPlanningLaneRiskAnalyser::RiskyObj& right_obj_1 =
        other_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_RIGHT];
    const ActionPlanningLaneRiskAnalyser::RiskyObj& right_obj_2 =
        other_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_RIGHT];
    if ((right_obj_1.valid) || (right_obj_2.valid)) {
      if (cur_lane_idx != tar_lane_idx) {
        tar_lane_has_risk = true;
      }
      if (cur_lane_idx != ret_lane_idx) {
        ret_lane_has_risk = true;
      }
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
      LOG_INFO(5) << "$$$ Refuse changing lane (curr lane exist right lateral obj).";
#endif
    }

    // 目标车道方向的左前方是否存在障碍物
    const ActionPlanningLaneRiskAnalyser::RiskyObj& left_front_obj_1 =
        other_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_LEFT_FRONT];
    const ActionPlanningLaneRiskAnalyser::RiskyObj& left_front_obj_2 =
        other_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_LEFT_FRONT];
    if ((left_front_obj_1.valid && left_front_obj_1.t_gap < 1.0F)||
        (left_front_obj_2.valid && left_front_obj_2.ttc < 8.0F)) {
      if (cur_lane_idx != tar_lane_idx) {
        tar_lane_has_risk = true;
      }
      if (cur_lane_idx != ret_lane_idx) {
        ret_lane_has_risk = true;
      }
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
      LOG_INFO(5) << "$$$ Refuse changing lane (tar lane's left front obj t_gap("
                  << left_front_obj_1.t_gap
                  << ") or ttc("
                  << left_front_obj_2.ttc
                  << ") is little).";
#endif
    }

    // 目标车道方向的右前方是否存在障碍物
    const ActionPlanningLaneRiskAnalyser::RiskyObj& right_front_obj_1 =
        other_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_RIGHT_FRONT];
    const ActionPlanningLaneRiskAnalyser::RiskyObj& right_front_obj_2 =
        other_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_RIGHT_FRONT];
    if ((right_front_obj_1.valid && right_front_obj_1.t_gap < 1.0F)||
        (right_front_obj_2.valid && right_front_obj_2.ttc < 8.0F)) {
      if (cur_lane_idx != tar_lane_idx) {
        tar_lane_has_risk = true;
      }
      if (cur_lane_idx != ret_lane_idx) {
        ret_lane_has_risk = true;
      }
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
      LOG_INFO(5) << "$$$ Refuse changing lane (tar lane's right front obj t_gap("
                  << right_front_obj_1.t_gap
                  << ") or ttc("
                  << right_front_obj_2.ttc
                  << ") is little).";
#endif
    }

    // 目标车道方向的左后是否存在障碍物
    const ActionPlanningLaneRiskAnalyser::RiskyObj& left_back_obj_1 =
        other_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_LEFT_BACK];
    const ActionPlanningLaneRiskAnalyser::RiskyObj& left_back_obj_2 =
        other_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_LEFT_BACK];
    if ((left_back_obj_1.valid && left_back_obj_1.t_gap < 0.8F)||
        (left_back_obj_2.valid && left_back_obj_2.ttc < 4.0F)) {
      if (cur_lane_idx != tar_lane_idx) {
        tar_lane_has_risk = true;
      }
      if (cur_lane_idx != ret_lane_idx) {
        ret_lane_has_risk = true;
      }
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
      LOG_INFO(5) << "$$$ Refuse changing lane (tar lane's left back obj t_gap("
                  << left_back_obj_1.t_gap
                  << ") or ttc("
                  << left_back_obj_2.ttc
                  << ") is little).";
#endif
    }

    // 目标车道方向的右后是否存在障碍物
    const ActionPlanningLaneRiskAnalyser::RiskyObj& right_back_obj_1 =
        other_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_RIGHT_BACK];
    const ActionPlanningLaneRiskAnalyser::RiskyObj& right_back_obj_2 =
        other_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_RIGHT_BACK];
    if ((right_back_obj_1.valid && right_back_obj_1.t_gap < 0.8F)||
        (right_back_obj_2.valid && right_back_obj_2.ttc < 4.0F)) {
      if (cur_lane_idx != tar_lane_idx) {
        tar_lane_has_risk = true;
      }
      if (cur_lane_idx != ret_lane_idx) {
        ret_lane_has_risk = true;
      }
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
      LOG_INFO(5) << "$$$ Refuse changing lane (tar lane's right back obj t_gap("
                  << right_back_obj_1.t_gap
                  << ") or ttc("
                  << right_back_obj_2.ttc
                  << ") is little).";
#endif
    }

    // 目标车道方向的正后方是否存在障碍物
    const ActionPlanningLaneRiskAnalyser::RiskyObj& back_obj_1 =
        other_lane.risk_info.nearest_obj[driv_map::OBJ_POSITION_BACK];
    const ActionPlanningLaneRiskAnalyser::RiskyObj& back_obj_2 =
        other_lane.risk_info.min_ttc_obj[driv_map::OBJ_POSITION_BACK];
    if ((back_obj_1.valid && back_obj_1.t_gap < 0.8F)||
        (back_obj_2.valid && back_obj_2.ttc < 4.0F)) {
      if (cur_lane_idx != tar_lane_idx) {
        tar_lane_has_risk = true;
      }
      if (cur_lane_idx != ret_lane_idx) {
        ret_lane_has_risk = true;
      }
#if ENABLE_ACTION_PLANNING_CHANGING_LANE_TRACE
      LOG_INFO(5) << "$$$ Refuse changing lane (tar lane's back obj t_gap("
                  << back_obj_1.t_gap
                  << ") or ttc("
                  << back_obj_2.ttc
                  << ") is little).";
#endif
    }
  }

  bool ret = true;
  if (tar_lane_has_risk) {
    *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_OBSTACLE;
  }
  if (ChangingLaneReq::REQ_CHANGING_LANE_ABORT == request_type) {
    *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_BY_DRIVER;
  }
  if ((ChangingLaneReq::REQ_CHANGING_LANE_ABORT == request_type) ||
      tar_lane_has_risk) {
    // 驾驶员请求中止变道，或者目标车道存在风险
    if (!ret_lane_has_risk) {
      // 原车道无风险, 返回原车道
      ret = false;
    } else {
      // 原车道有风险, 保持之前的变道状态
      ret = true;
    }
  }
	  
  // 获取相机识别的车道线数据
    ad_msg::LaneMarkCameraList lane_mark_camera_list_;	
	plan_var_t c0_left,c1_left,c2_left,c3_left;
	plan_var_t c0_right,c1_right,c2_right,c3_right;
	  for (Int32_t i = 0; i < lane_mark_camera_list_.lane_mark_num; ++i) {//4条车道线
		const ad_msg::LaneMarkCamera& lane = lane_mark_camera_list_.lane_marks[i];//LaneMarkCamera
		if (1 == lane.id) {
		  // left
		  c0_left = lane.c0;
		  c1_left = lane.c1;
		  c2_left = lane.c2;
		  c3_left = lane.c3;
		} else if (-1 == lane.id) {
		  // right
		  c0_right = lane.c0;
		  c1_right = lane.c1;
		  c2_right = lane.c2;
		  c3_right = lane.c3;
  
		} else {
		  //TODO : set default value
	//		LOG_INFO(5) << "非正左右车道线";
		}
	  }
  
	bool flag ;
	flag = (common::com_abs(c0_left ) < 0.01) || (common::com_abs(c0_right)< 0.01));
	if ((ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_I == changing_lane_status_) ||
		(ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_I == changing_lane_status_)) {
	  if (ret_lane_has_risk && (!risk_is_forward_obj)) { 
		if(flag){
		  ret = true;
		  }else{
		  ret = false;
		  *refuse_reason = REFUSE_CHANGINHG_LANE_REASON_OBSTACLE;
		  }
	  }
	  
  } else if ((ChangingLaneRsp::STATUS_CHANGING_LANE_RIGHT_STAGE_II == changing_lane_status_) ||
             (ChangingLaneRsp::STATUS_CHANGING_LANE_LEFT_STAGE_II == changing_lane_status_)) {
    if (tar_lane_has_risk) {
      ret = true;
    }
  }

  return (ret);
}


}  // namespace planning
}  // namespace phoenix

