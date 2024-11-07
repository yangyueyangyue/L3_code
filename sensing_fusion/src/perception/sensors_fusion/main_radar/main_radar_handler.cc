#include "main_radar_handler.h"
#include "sensors_fusion/sensors_fusion_macros.h"
#include <unordered_map>

namespace phoenix{
namespace perception{
namespace main_radar{

MainRadarHandler::MainRadarHandler()
{

}


MainRadarHandler::~MainRadarHandler()
{

}


/**
 * @brief 检测前毫米波雷达数据是否有异常，并记录状态，不改变原有数据。
 * 
 */
void MainRadarHandler::CheckRadarDataStatus(ad_msg::ObstacleRadarList& radar_list)
{
    // 获取感知数据
    framework::SharedData* shared_data = framework::SharedData::instance();
    // 获取感知模块状态值
    shared_data->GetPerceptionModuleStatus(&perception_module_status_);

    for (int i = 0; i < radar_list.obstacle_num; i++)
    {
        ad_msg::ObstacleRadar & obstacle = radar_list.obstacles[i];
        if(obstacle.x< -10|| 
           obstacle.y< -200||
           obstacle.y> 200){ 
           perception_module_status_.fw_radar_data_status |= framework::PERCEPTION_MODULE_DATA_POS_BEYOND_RANGE;
        }
        if(common::com_abs(obstacle.v_x) > 60|| 
           common::com_abs(obstacle.v_y) > 60){ 
           perception_module_status_.fw_radar_data_status |= framework::PERCEPTION_MODULE_DATA_VELOCITY_BEYOND_RANGE;
        }
    }
    shared_data->SetPerceptionModuleStatus(perception_module_status_);
}




void MainRadarHandler::CheckRadarCurbType(const Int64_t timestamp, driv_map::DrivingMapWrapper* driving_map , ad_msg::ObstacleRadarList& radar_list)
{

  if (Nullptr_t == driving_map) {
    return;
  }
  if (driving_map->GetReferenceLinesNum() < 1) {
    return;
  }
  // Get major reference line
  Int32_t major_ref_line_index = driving_map->GetMajorReferenceLineIndex();
  if (!driving_map->IsValidReferenceLineIndex(major_ref_line_index)) {
    return;
  }

  ad_msg::RelativePos current_rel_pos;
  if (!phoenix::pos_filter::PosFilterWrapper::FindRelPosFromListByTimestamp(
        timestamp, driving_map->GetRelativePosList(),
        &current_rel_pos)) {
    LOG_ERR << "Failed to get current posistion from relative position list.";
    return;
  }
  common::Matrix<Float32_t, 2, 1> rotate_center;
  rotate_center.SetZeros();
  common::Matrix<Float32_t, 3, 3> mat_conv;
  mat_conv.SetIdentity();
  common::Rotate_2D<Float32_t>(rotate_center, current_rel_pos.heading, &mat_conv);
  common::Translate_2D(current_rel_pos.x, current_rel_pos.y, &mat_conv);
  common::Matrix<Float32_t, 2, 1> point_conv;

  const common::Path& major_ref_line =
      driving_map->GetSmoothReferenceLine(major_ref_line_index);

  const common::StaticVector<driv_map::RoadBoundary,
      common::Path::kMaxPathPointNum>& road_boundary =
      driving_map->GetRoadBoundary();

  Int32_t count = 0;
  Int32_t obj_num = radar_list.obstacle_num;
  for (Int32_t i = 0; i < obj_num; ++i) {

    ad_msg::ObstacleRadar & obj =  radar_list.obstacles[i];

    point_conv(0) = obj.x;
    point_conv(1) = obj.y;

    common::TransformVert_2D(mat_conv, &point_conv);

    sensorsfusion::MainRefLineRelation main_ref;


    common::Vec2d pos(point_conv(0), point_conv(1));
    if (!major_ref_line.FindProjection(
          pos, &main_ref.proj_on_major_ref_line_)) {
      LOG_ERR << "[Error]Failed to find projecton on major reference line.";
      break;
    }
    main_ref.valid_ = true;

    // 判断是否是路沿
    if ((common::com_abs(obj.v_x) < 15.0F/3.6F)) {
      // 仅ESR感知的静态障碍物
      // 判断障碍物体在道路中的位置
      Float32_t left_boundary_width = 0;
      Float32_t right_boundary_width = 0;
      FindRoadBoundaryWidth(
            road_boundary,
            main_ref.proj_on_major_ref_line_.s,
            &left_boundary_width, &right_boundary_width);

      Float32_t lat_offset = main_ref.proj_on_major_ref_line_.l;

      if ((lat_offset > (left_boundary_width - 1.0F)) &&
          (lat_offset < (left_boundary_width + 2.0F))) {
        // left road boundary curb
        obj.type = ad_msg::OBJ_TYPE_CURB;

      } else if ((lat_offset < -(right_boundary_width - 1.0F)) &&
                 (lat_offset > -(right_boundary_width + 2.0F))) {
        // right road boundary curb
        obj.type = ad_msg::OBJ_TYPE_CURB;

      } else {
        // nothing to do
      }
    } else {
      if (ad_msg::OBJ_TYPE_CURB == obj.type ) {
        obj.type = ad_msg::OBJ_TYPE_UNKNOWN;
      }
    }

    count++;
  }

}

void MainRadarHandler::FindRoadBoundaryWidth(const common::StaticVector<driv_map::RoadBoundary, common::Path::kMaxPathPointNum> &road_boundary, const Float32_t s_ref, Float32_t *left_width, Float32_t *right_width)
{
    if (road_boundary.Empty()) {
      *left_width = 1.5F;
      *right_width = 1.5F;
      return;
    }
    if (s_ref <= road_boundary[0].ref_point.s) {
      *left_width = road_boundary[0].left_width;
      *right_width = road_boundary[0].right_width;
      return;
    }
    if (s_ref >= road_boundary.Back().ref_point.s) {
      *left_width = road_boundary.Back().left_width;
      *right_width = road_boundary.Back().right_width;
      return;
    }
    Int32_t low = 0;
    Int32_t high = static_cast<Int32_t>(road_boundary.Size());
    while (low + 1 < high) {
      const Int32_t mid = (low + high) / 2;
      if (road_boundary[mid].ref_point.s <= s_ref) {
        low = mid;
      } else {
        high = mid;
      }
    }
    const driv_map::RoadBoundary &sample1 = road_boundary[low];
    const driv_map::RoadBoundary &sample2 = road_boundary[high];
    Float32_t ratio = 0.0F;
    if (sample2.ref_point.s - sample1.ref_point.s > 0.1F) {
      ratio = (sample2.ref_point.s - s_ref) /
          (sample2.ref_point.s - sample1.ref_point.s);
    }
    if (ratio > 1.0F) {
      ratio = 1.0F;
    } else if (ratio < 0.0F) {
      ratio = 0.0F;
    } else {
      // nothing to do
    }

    *left_width = sample1.left_width * ratio +
        sample2.left_width * (1.0F - ratio);
    *right_width = sample1.right_width * ratio +
        sample2.right_width * (1.0F - ratio);
}




/**
 * @brief 后续会加入是否在隧道的判断！！！！！！！
 * 
 * @param camera_list 
 * @param lane_curb_list 
 */
void MainRadarHandler::UpdateMainRadarList(const Int64_t timestamp, driv_map::DrivingMapWrapper* driving_map , ad_msg::ObstacleRadarList& radar_list)
{
    CheckRadarCurbType(timestamp, driving_map, radar_list);
    CheckRadarDataStatus(radar_list);
}


} //end of namespace phoenix
} //end of namespace perception
} //end of namespace main_radar


