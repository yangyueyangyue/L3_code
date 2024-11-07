#ifndef MAIN_RADAR_HANDLER_H_
#define MAIN_RADAR_HANDLER_H_

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

#include "geometry/aabbox2d.h"
#include "geometry/geometry_utils.h"

#include "sensors_fusion/multi_constructure_classes_repo.h"
#include "framework/communication/shared_data.h"
#include <vector>
#include <map>
#include "ExpWrapperComm.h"

/*
是否启用预处理
*/
#define ENABLE_MAIN_RADAR_PRE_PROCESS  (1)

using namespace std;

namespace phoenix{
namespace perception{
namespace main_radar{

class MainRadarHandler{
public:
    MainRadarHandler();
    ~MainRadarHandler();
    void UpdateMainRadarList(const Int64_t timestamp, driv_map::DrivingMapWrapper* driving_map , ad_msg::ObstacleRadarList& radar_list);
    void CheckRadarDataStatus(ad_msg::ObstacleRadarList& radar_list);
private:
    void CheckRadarCurbType(const Int64_t timestamp, driv_map::DrivingMapWrapper* driving_map , ad_msg::ObstacleRadarList& radar_list);
    void FindRoadBoundaryWidth(const common::StaticVector<driv_map::RoadBoundary, common::Path::kMaxPathPointNum> &road_boundary, const Float32_t s_ref, Float32_t *left_width, Float32_t *right_width);
    framework::PerceptionModuleStatus perception_module_status_;
};



} //end of namespace phoenix
} //end of namespace perception
} //end of namespace main_radar



#endif
