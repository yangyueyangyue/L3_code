//
#include "task_manager.h"

#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/xml_parser.hpp"

#include "util.h"
#include "communication/parse_proto_msg.h"
#include "communication/msg_receiver.h"
#include "task_planning.h"

#include "planning_story.pb.h"


namespace phoenix {
namespace framework {


static void ReadPlanningStorys(
    const std::string& file_name,
    ad_msg::PlanningStoryList* story_list) {
  msg::planning::PlanningStoryList msg_planning_storys;
  if (GetProtoFromASCIIFile(file_name, &msg_planning_storys)) {
    ParseProtoMsg parser;
    parser.DecodePlanningStoryList(msg_planning_storys, story_list);

    std::cout << "\n###### Success to read planning storys from file."
              << std::endl;
    std::cout << "   >>> Decode " << story_list->story_num
              << " storys from protoc, these storys are:"
              << std::endl;
    for (Int32_t i = 0; i < story_list->story_num; ++i) {
      const ad_msg::PlanningStory& story = story_list->storys[i];
      std::cout << "        story[" << i
                << "]: id=" << story.id << ", type=" << story.type
                << ", condition{area[left=" << story.condition.area.left_width
                << ", right=" << story.condition.area.right_width
                << ", start=" << story.condition.area.start_s
                << ", end=" << story.condition.area.end_s
                << "], speed_l=" << story.condition.speed_low*3.6F
                << ", speed_h=" << story.condition.speed_high*3.6F
                << ", gear=" << (Int32_t)story.condition.gear
                << "}, action{cmd=" << story.action.cmd
                << "}"
                << std::endl;
    }
  }
}

static void ReadObjFilterConfig(
    const boost::property_tree::ptree& pt, PlanningConfig* config) {
  // 是否使用外部障碍物列表
  config->obj_filter_config.using_outside_obj_list = pt.get<bool>(
        "config.obj_filter.using_outside_obj_list");
  // 激光雷达
  config->obj_filter_config.lidar_usage = pt.get<bool>(
        "config.obj_filter.lidar_usage");

  std::cout << "\n    >>> config.obj_filter:" << std::endl;
  std::cout << "        using_outside_obj_list="
            << config->obj_filter_config.using_outside_obj_list
            << std::endl;
  std::cout << "        lidar_usage="
            << config->obj_filter_config.lidar_usage
            << std::endl;
}

static void ReadDrivingMapConfig(
    const boost::property_tree::ptree& pt, PlanningConfig* config) {
  Int32_t inputted_map_type =
      pt.get<int>("config.driving_map.inputted_map_type");

  switch (inputted_map_type) {
  case (0):
    config->driving_map_config.inputted_map_type =
        driv_map::DrivingMapConfig::INPUTTED_MAP_TYPE_CAMERA;
    break;
  case (1):
    config->driving_map_config.inputted_map_type =
        driv_map::DrivingMapConfig::INPUTTED_MAP_TYPE_HDMAP;
    break;
  case (2):
    config->driving_map_config.inputted_map_type =
        driv_map::DrivingMapConfig::INPUTTED_MAP_TYPE_MIXED;
    break;
  default:
    config->driving_map_config.inputted_map_type =
        driv_map::DrivingMapConfig::INPUTTED_MAP_TYPE_CAMERA;
    LOG_ERR << "Invalid inputted map type.";
    break;
  }

  std::cout << "\n    >>> config.driving_map:" << std::endl;
  std::cout << "        inputted_map_type="
            << config->driving_map_config.inputted_map_type
            << std::endl;
}

static void ReadActionPlanningConfig(
    const boost::property_tree::ptree& pt, PlanningConfig* config) {
  // 允许变道
  config->act_planning_config.enable_auto_changing_lane =
      pt.get<bool>("config.action_planning.enable_auto_changing_lane");
  //允许强制变道
  config->act_planning_config.enable_action_planning_force_changing_lane =
      pt.get<bool>("config.action_planning.enable_action_planning_force_changing_lane");


  std::cout << "\n    >>> config.action_planning:" << std::endl;
  std::cout << "        enable_auto_changing_lane="
            << config->act_planning_config.enable_auto_changing_lane
            <<"        enable_action_planning_force_changing_lane="
            <<config->act_planning_config.enable_action_planning_force_changing_lane
            << std::endl;
}

static void ReadTrajectoryPlanningConfig(
    const boost::property_tree::ptree& pt, PlanningConfig* config) {
  // 允许变道
  config->trj_planning_config.enable_changing_lane =
      pt.get<bool>("config.trajectory_planning.enable_changing_lane");
  // 允许车道内避让
  config->trj_planning_config.enable_avoiding_collision_in_lane =
      pt.get<bool>("config.trajectory_planning.enable_avoiding_collision_in_lane");
  // 允许跨线避让
  config->trj_planning_config.enable_avoiding_collision_over_line =
      pt.get<bool>("config.trajectory_planning.enable_avoiding_collision_over_line");
  // 允许强制变道
  config->trj_planning_config.enable_trajectory_planning_force_changing_lane =
      pt.get<bool>("config.trajectory_planning.enable_trajectory_planning_force_changing_lane");


  std::cout << "\n    >>> config.trajectory_planning:" << std::endl;
  std::cout << "        enable_changing_lane="
            << config->trj_planning_config.enable_changing_lane
            << std::endl;
  std::cout << "        enable_avoiding_collision_in_lane="
            << config->trj_planning_config.enable_avoiding_collision_in_lane
            << std::endl;
  std::cout << "        enable_avoiding_collision_over_line="
            << config->trj_planning_config.enable_avoiding_collision_over_line
            << std::endl;
  std::cout << "        enable_trajectory_planning_force_changing_lane="
            << config->trj_planning_config.enable_trajectory_planning_force_changing_lane
            << std::endl;
}

static void ReadVelocityPlanningConfig(
    const boost::property_tree::ptree& pt, PlanningConfig* config) {
  config->vel_planning_config.enable_aeb_pre_dec =
      pt.get<bool>("config.velocity_planning.enable_aeb_pre_dec");
  config->vel_planning_config.safe_dist_to_static_obj =
      pt.get<double>("config.velocity_planning.safe_dist_to_static_obj");
  config->vel_planning_config.safe_dist_for_low_speed_following =
      pt.get<double>("config.velocity_planning.safe_dist_for_low_speed_following");
  config->vel_planning_config.safe_dist_to_dynamic_obj =
      pt.get<double>("config.velocity_planning.safe_dist_to_dynamic_obj");
  config->vel_planning_config.safe_distance_in_backing_mode =
      pt.get<double>("config.velocity_planning.safe_distance_in_backing_mode");
  config->vel_planning_config.enable_stop_by_traffic_light =
      pt.get<bool>("config.velocity_planning.enable_stop_by_traffic_light");

  // 精准停车
  config->vel_planning_config.stop_accurately.enable =
      pt.get<bool>("config.velocity_planning.stop_accurately.enable");
  config->vel_planning_config.stop_accurately.braking_distance =
      pt.get<double>("config.velocity_planning.stop_accurately.braking_distance");
  config->vel_planning_config.stop_accurately.proposed_decelaration =
      pt.get<double>("config.velocity_planning.stop_accurately.proposed_decelaration");

  config->vel_planning_config.lat_acc_limit =
      pt.get<double>("config.velocity_planning.lat_acc_limit");

  std::cout << "\n    >>> config.velocity_planning:" << std::endl;
  std::cout << "        enable_aeb_pre_dec="
            << config->vel_planning_config.enable_aeb_pre_dec
            << std::endl;
  std::cout << "        safe_dist_to_static_obj="
            << config->vel_planning_config.safe_dist_to_static_obj
            << std::endl;
  std::cout << "        safe_dist_for_low_speed_following="
            << config->vel_planning_config.safe_dist_for_low_speed_following
            << std::endl;
  std::cout << "        safe_dist_to_dynamic_obj="
            << config->vel_planning_config.safe_dist_to_dynamic_obj
            << std::endl;
  std::cout << "        safe_dist_in_backing_mode="
            << config->vel_planning_config.safe_distance_in_backing_mode
            << std::endl;
  std::cout << "        stop_accurately:"<< std::endl;
  std::cout << "            enable="
            << config->vel_planning_config.stop_accurately.enable
            << std::endl;
  std::cout << "            braking_distance="
            << config->vel_planning_config.stop_accurately.braking_distance
            << std::endl;
  std::cout << "            proposed_decelaration="
            << config->vel_planning_config.stop_accurately.proposed_decelaration
            << std::endl;
  std::cout << "        lat_acc_limit="
            << config->vel_planning_config.lat_acc_limit
            << std::endl;
  std::cout << "        enable_stop_by_traffic_light="
            << config->vel_planning_config.enable_stop_by_traffic_light
            << std::endl;
}

bool TaskManager::ReadConfigFromFile() {
  std::string config_file = work_space_ + "/conf/planning_config.xml";

  std::cout << "\n###### Read configuration from file: (Begin) ######\n"
            << "  file_name= \"" << config_file << "\"." << std::endl;

  try {
    // Create empty property tree object
    boost::property_tree::ptree pt;
    // Load XML file and put its contects in property tree
    boost::property_tree::read_xml(config_file, pt);

    // 配置障碍物过滤模块
    ReadObjFilterConfig(pt, &config_);

    // 配置驾驶地图模块
    ReadDrivingMapConfig(pt, &config_);

    // 配置行为规划模块
    ReadActionPlanningConfig(pt, &config_);

    // 配置轨迹规划模块
    ReadTrajectoryPlanningConfig(pt, &config_);

    // 配置速度规划模块
    ReadVelocityPlanningConfig(pt, &config_);

    // 配置
    msg_receiver_->Configurate(config_);
    task_planning_->Configurate(config_);
    work_monitor_.Configurate(config_);
  } catch (std::exception &e) {
    LOG_ERR << "Failed to read config from file (" << e.what() << ").";
    return false;
  }

  std::string storys_file = work_space_ + "/conf/planning_storys.story";
  ad_msg::PlanningStoryList planning_story_list;
  ReadPlanningStorys(storys_file, &planning_story_list);
  task_planning_->SetPlanningStorys(planning_story_list);

  std::cout << "\n###### Read configuration from file. (End) ######\n"
            << std::endl;

  return true;
}


}
}
