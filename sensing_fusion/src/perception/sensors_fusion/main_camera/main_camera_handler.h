#ifndef MAIN_CAMERA_HANDLER_H_
#define MAIN_CAMERA_HANDLER_H_

#include "msg_common.h"
#include "msg_obstacle.h"
#include "radar_tracker.h"
#include "ad_msg.h"
#include "pos_filter/include/pos_filter_wrapper.h"
#include "utils/com_utils.h"
#include "math/math_utils.h"
#include "curve/cubic_polynomial_curve1d.h"
#include "geometry/vec2d.h"
#include "curve/path.h"
#include "framework/communication/shared_data.h"
#include <vector>
#include <map>
#include "ExpWrapperComm.h"

/*
是否启用预处理
*/
#define ENABLE_MAIN_CAMERA_PRE_PROCESS  (1)

#define CAMERA_PRE_LOG_LEVEL    (7)

using namespace std;

namespace phoenix{
namespace perception{
namespace main_camera{

struct ObstacleCameraSpeedList {
    Float32_t chassis_v;
    Int64_t timestamp;
    ad_msg::ObstacleCamera camera_obstacle;
};

class MainCameraHandler{
public:
    MainCameraHandler();
    ~MainCameraHandler();
    void UpdateMainCameraList(ad_msg::ObstacleCameraList& camera_list, ad_msg::LaneMarkCameraList& lane_curb_list);
    void SetRelativePosList(ad_msg::RelativePosList* rel_pos_list);
    void SetMapGeofenceInfo(exp_map_t::stMapGeofenceInfo * geofence_info);
    void CheckCameraDataStatus(ad_msg::ObstacleCameraList& camera_list);

    // 更新前视一体机障碍物速度(根据前视一体机上下帧移动距离和时间差，计算出纵向速度，并与传感器数据进行比较)
    void UpdateMainCameraObjectSpeed(ad_msg::ObstacleCameraList& camera_list, ad_msg::Chassis chassis);

    // 根据纵向距离拆解前视一体机数据
    Uint8_t DisMainCameraObject(ad_msg::ObstacleCameraList& camera_list, vector<Int32_t> dis_param, vector<ad_msg::ObstacleCameraList>& dis_camera_obstacle_vector_);

private:
    bool ConstructPath(ad_msg::LaneMarkCamera& lane_curb, common::Path& path);
    bool IsInTunnel();

    ad_msg::RelativePosList* rel_pos_list_;
    exp_map_t::stMapGeofenceInfo * geofence_info_;
    uint64_t  last_geofence_timestamp_;

    // 存储上一帧的前视一体机数据
    vector<ObstacleCameraSpeedList> last_camera_obstacle_vec_;

    framework::PerceptionModuleStatus perception_module_status_;
};



} //end of namespace phoenix
} //end of namespace perception
} //end of namespace main_camera



#endif
