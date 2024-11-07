//
#include "task_manager.h"

#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/xml_parser.hpp"

#include "math/math_utils.h"
#include "util.h"
#include "communication/parse_proto_msg.h"
#include "communication/msg_receiver.h"

#if(VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
#include "pc/qin_ev/task_chassis_control_qin_ev.h"
#elif(VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
#include "pc/ft_auman/task_chassis_control_ft_auman.h"
#elif(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
#include "pc/df_x320/task_chassis_control_df_x320.h"
#elif(VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
#include "pc/xd_eant/task_chassis_control_xd_eant.h"
#elif(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
#include "pc/df_d17_b1/task_chassis_control_df_d17_b1.h"
#elif(VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
#include "pc/df_d17_b2/task_chassis_control_df_d17_b2.h"
#else
  Error: Invalid vehicle platform
#endif


namespace phoenix {
namespace framework {


static void ReadChassisControlConfig(
    const boost::property_tree::ptree& pt, ControlConfig_t* config) {
  // 方向盘角度偏置 (弧度，左正右负)
  config->chassis_control_config.steering_wheel_angle_offset =
      common::com_deg2rad(
        pt.get<double>("config.chassis_control.steering_wheel_angle_offset"));

  std::cout << "\n    >>> config.chassis_control:" << std::endl;
  std::cout << "        steering_wheel_angle_offset="
            << common::com_rad2deg(config->chassis_control_config.steering_wheel_angle_offset)
            << std::endl;
}

bool TaskManager::ReadConfigFromFile() {
  std::string config_file = work_space_ + "/conf/control_config.xml";

  std::cout << "\n###### Read configuration from file: (Begin) ######\n"
            << "  file_name= \"" << config_file << "\"." << std::endl;

  try {
    // Create empty property tree object
    boost::property_tree::ptree pt;
    // Load XML file and put its contects in property tree
    boost::property_tree::read_xml(config_file, pt);

    // 配置底盘控制模块
    ReadChassisControlConfig(pt, &config_);

    // 配置
    task_chassis_control_->Configurate(config_.chassis_control_config);
  } catch (std::exception &e) {
    LOG_ERR << "Failed to read config from file (" << e.what() << ").";
    return false;
  }

  std::cout << "\n###### Read configuration from file. (End) ######\n"
            << std::endl;

  return true;
}


}
}
