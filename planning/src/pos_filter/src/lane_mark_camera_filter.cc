/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       lane_mark_camera_filter.cc
 * @brief      使用相机识别的车道线进行辅助定位
 * @details    使用相机识别的车道线进行估计当前车辆的相对位置
 *
 * @author     pengc
 * @date       2020.12.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/12/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/
#include "lane_mark_camera_filter.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "geometry/icp_core.h"
#include "curve/cubic_polynomial_curve1d.h"


#define ENABLE_LANE_MARK_CAMERA_FILTER_TRACE (0)


namespace phoenix {
namespace pos_filter {


/// TODO: 确认车身坐标系(后轴中心)在相机车道线(3次多项式)上的位置？是否需要旋转及平移？


/**
 * @date       2020.11.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/11/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
LaneMarkCameraFilter::LaneMarkCameraFilter() {
  common::Matrix<Scalar, 2, 1> rotate_center;
  rotate_center.SetZeros();
  mat_camera_calibration_.SetIdentity();
  common::Rotate_2D<Scalar>(rotate_center, common::com_deg2rad(0.0F), &mat_camera_calibration_);
  common::Translate_2D(0.0F, 0.00F, &mat_camera_calibration_);

  filter_quality_ = FILTER_QUALITY_INVALID;
  raw_lane_list_idx_ = 0;

  curr_lane_info_list_index_ = 0;

  /* k006 pengc 2023-02-16 (begin) */
  // 平滑车道线，检测异常跳变的车道线
  internal_lane_info_.Clear();
  /* k006 pengc 2023-02-16 (end) */
}

/**
 * @date       2020.11.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/11/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
LaneMarkCameraFilter::~LaneMarkCameraFilter() {
  // nothing to do
}

void LaneMarkCameraFilter::Configurate(const PosFilterConfig& conf) {
  common::Matrix<Scalar, 2, 1> rotate_center;
  rotate_center.SetZeros();
  mat_camera_calibration_.SetIdentity();
  common::Rotate_2D<Scalar>(rotate_center, conf.cam_lane.h_offset, &mat_camera_calibration_);
  common::Translate_2D(conf.cam_lane.x_offset, conf.cam_lane.y_offset, &mat_camera_calibration_);

  printf("#### Configurate camera lane: x_offset=%0.3fm, y_offset=%0.3fm, h_offset=%0.3fdeg\n",
      conf.cam_lane.x_offset, conf.cam_lane.y_offset, common::com_rad2deg(conf.cam_lane.h_offset));
}

void LaneMarkCameraFilter::Reset() {
  filter_quality_ = FILTER_QUALITY_INVALID;
  raw_lane_list_idx_ = 0;
  raw_lane_list_[0].Clear();
  raw_lane_list_[1].Clear();
  raw_lane_mark_seg_kdtree_.Clear();

  curr_lane_info_list_index_ = 0;
  lane_info_list_[0].Clear();
  lane_info_list_[1].Clear();
  lane_boundary_line_seg_kdtree_.Clear();

  /* k006 pengc 2023-02-16 (begin) */
  // 平滑车道线，检测异常跳变的车道线
  internal_lane_info_.Clear();
  /* k006 pengc 2023-02-16 (end) */
}

/**
 * @date       2020.11.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/11/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
bool LaneMarkCameraFilter::Update(
    const ad_msg::LaneMarkCameraList& lane_mark_list,
    const Scalar delta_pos[3], Scalar filtered_delta_pos[3]) {
  // 对摄像头识别的车道线的参数曲线的采样步长
  // 由于摄像头识别的车道线曲率都不会太大，所以可以将采样步长加长些
  static const Scalar kSampleStepLen = 2.0F;

  filtered_delta_pos[0] = delta_pos[0];
  filtered_delta_pos[1] = delta_pos[1];
  filtered_delta_pos[2] = delta_pos[2];

  // 之前的车道线
  const ad_msg::LaneInfoCameraList& prev_lane_list =
      raw_lane_list_[raw_lane_list_idx_];
  // 用来保存当前的车道线
  Int32_t new_lane_list_idx = raw_lane_list_idx_ + 1;
  if (new_lane_list_idx > 1) {
    new_lane_list_idx = 0;
  }
  ad_msg::LaneInfoCameraList& new_lane_list = raw_lane_list_[new_lane_list_idx];
  new_lane_list.Clear();

  // 遍历摄像头识别的车道线列表
  common::Matrix<Scalar, 2, 1> point_conv;
  // 新的车道线列表的起始索引
  Int32_t new_lane_idx = 0;
  for (Int32_t i = 0; i < lane_mark_list.lane_mark_num; ++i) {
    const ad_msg::LaneMarkCamera& lane_mark = lane_mark_list.lane_marks[i];

    if (lane_mark.quality < 2) {
      // 识别效果差，丢弃此车道线
      continue;
    }
    if ((1 != lane_mark.id) && (-1 != lane_mark.id)) {
      // 目前只有当前车道两侧的车道线识别效果好，其它更远处的相邻车道识别效果差
      continue;
    }

    // 识别的车道线的有效距离
    Scalar lane_mark_len = lane_mark.view_range_end - lane_mark.view_range_start;
    if (lane_mark_len < 3.0F) {
      // 识别的车道线的有效距离太短
      continue;
    }

    // 指向需要生成的新的车道线
    ad_msg::LaneBoundaryLineCamera& new_lane =
        new_lane_list.boundary_lines[new_lane_idx];
    new_lane_idx++;
    if (new_lane_idx > ad_msg::LaneInfoCameraList::MAX_LANE_BOUNDARY_LINE_NUM) {
      LOG_ERR << "The index of lane line exceed the max number.";
      break;
    }

    // 需要采样的车道线的点的数量
    Int32_t sample_size = lane_mark_len / kSampleStepLen;
    if (sample_size < 2) {
      sample_size = 2;
    }

    // 生成车道线的参数化方程
    common::CubicPolynomialCurve1d<Scalar> curve;
    curve.SetCoefficient(lane_mark.c0, lane_mark.c1, lane_mark.c2, lane_mark.c3);

    Int32_t new_point_idx = 0;
    for (Int32_t j = 0; j < sample_size; ++j) {
      if (new_point_idx >= ad_msg::LaneBoundaryLineCamera::MAX_CURVE_POINT_NUM) {
        break;
      }

      point_conv(0) = j * kSampleStepLen;
      if (point_conv(0) > lane_mark_len) {
        // 采样点不能超过有效距离
        point_conv(0) = lane_mark_len;
      }
      // 将新的车道线的曲线点转换到之前帧的坐标系下
      point_conv(0) += lane_mark.view_range_start;
      point_conv(1) = curve.Evaluate(0, point_conv(0));
      common::TransformVert_2D(mat_camera_calibration_, &point_conv);

      new_lane.curve[new_point_idx].fake = false;
      new_lane.curve[new_point_idx].x = point_conv(0);
      new_lane.curve[new_point_idx].y = point_conv(1);
      new_point_idx++;
    }
    new_lane.curve_point_num = new_point_idx;
    new_lane.id = lane_mark.id;
    new_lane.type = lane_mark.lane_mark_type;
  }
  new_lane_list.boundary_line_num = new_lane_idx;

  phoenix::common::AABBoxKDTreeParams params;
  params.max_leaf_dimension = 5.0f;  // meters.
  params.max_leaf_size = 16;
  raw_lane_mark_seg_kdtree_.SetKDTreeParams(params);
  raw_lane_mark_seg_kdtree_.Clear();
  temp_data_.tree_nodes_stack.Clear();

  LaneLineSegObj obj;
  for (Int32_t lane_index = 0;
       lane_index < prev_lane_list.boundary_line_num; ++lane_index) {
    const ad_msg::LaneBoundaryLineCamera&
        lane_line = prev_lane_list.boundary_lines[lane_index];
    for (Int32_t point_index = 0;
         point_index < (lane_line.curve_point_num-1); ++point_index) {
      if (lane_line.curve[point_index].fake ||
          lane_line.curve[point_index + 1].fake) {
        break;
      }
      obj.lane_id = lane_line.id;
      obj.lane_line_index = lane_index;
      obj.point_index = point_index;
      phoenix::common::AABBox2d box(
            lane_line.curve[point_index].x, lane_line.curve[point_index].y,
            lane_line.curve[point_index + 1].x, lane_line.curve[point_index + 1].y);
      raw_lane_mark_seg_kdtree_.AddObject(obj, box);
    }
  }
  raw_lane_mark_seg_kdtree_.Partition(temp_data_.tree_nodes_stack);

  // 对两帧的车道线进行配准，获取相对精确的相对位置的变换
  bool register_ret = false;
  if ((prev_lane_list.boundary_line_num > 0) &&
      (raw_lane_mark_seg_kdtree_.GetObjectsSize() > 0)) {
    if (RegisterFramesOfLaneLines(lane_mark_list, filtered_delta_pos)) {
      register_ret = true;
    } else {
      LOG_ERR << "Failed to register lane mark.";
    }
  }

  // 切换当前的车道线列表
  raw_lane_list_idx_ = new_lane_list_idx;

  return (register_ret);
}

/**
 * @brief 将相机识别的车道线数据转换为车道中心线及车道边界
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/11/08  <td>1.0      <td>pengc     <td>First edition
 * <tr><td>2020/12/28  <td>1.1      <td>sip       <td>增加相邻的左右车道的解析
 * </table>
 */
bool LaneMarkCameraFilter::UpdateLaneLineList(
    const ad_msg::LaneMarkCameraList& lane_mark_list,
    const Scalar delta_pos[3], Scalar heading_corrected, Scalar veh_velocity,
    bool match_prev_lane_mark) {
  // 对摄像头识别的车道线的参数曲线的采样步长
  // 由于摄像头识别的车道线曲率都不会太大，所以可以将采样步长加长些
  const Scalar sample_step_len = 2.0F;

#if ENABLE_LANE_MARK_CAMERA_FILTER_TRACE
  std::cout << "### UpdateLaneLineList (Begin) ###" << std::endl;
  std::cout << "delta_pos = " << delta_pos[0] << ", " << delta_pos[1]
            << ", " << common::com_rad2deg(delta_pos[2]) << ")" << std::endl;
  std::cout << "!!!!! UpdateLaneLineList: lane_mark_num="
            << (Int32_t)lane_mark_list.lane_mark_num
            << std::endl;
#endif

  bool update_success = false;
  // 新的车道线列表的起始索引
  Int32_t new_lane_line_index = 0;

  // 需要将旧的车道线列表更新为新的车道线列表，
  // 所以需要在两个车道线列表的缓存之间做切换
  const ad_msg::LaneInfoCameraList& curr_lane_info_list =
      lane_info_list_[curr_lane_info_list_index_];
  Int32_t next_lane_info_list_index = curr_lane_info_list_index_ + 1;
  if (next_lane_info_list_index > 1) {
    next_lane_info_list_index = 0;
  }
  ad_msg::LaneInfoCameraList& next_lane_info_list =
      lane_info_list_[next_lane_info_list_index];

  // 根据两个车道线检测帧之间的转换关系，定义正向的平移/旋转矩阵
  common::Matrix<Scalar, 3, 3> mat_convert;
  common::Matrix<Scalar, 2, 1> rotate_center;
  common::Matrix<Scalar, 2, 1> point_conv;
  mat_convert.SetIdentity();
  rotate_center.SetZeros();
  common::Rotate_2D<Scalar>(rotate_center, delta_pos[2], &mat_convert);
  common::Rotate_2D<Scalar>(rotate_center, heading_corrected, &mat_convert);
  common::Translate_2D(delta_pos[0], delta_pos[1], &mat_convert);
  // 根据两个车道线检测帧之间的转换关系，定义逆向的平移/旋转矩阵
  common::Matrix<Scalar, 3, 3> mat_convert_invr;
  mat_convert_invr.SetIdentity();
  rotate_center.SetZeros();
  common::Rotate_2D<Scalar>(rotate_center, -delta_pos[2], &mat_convert_invr);
  common::Translate_2D(-delta_pos[0], -delta_pos[1], &mat_convert_invr);

  /* k001 sip 2020-12-28 (begin) */
  // 遍历摄像头识别的车道线列表
  bool valid_left_lane_mark = false;
  bool valid_right_lane_mark = false;
  bool valid_next_left_lane_mark = false;
  bool valid_next_right_lane_mark = false;
  Int32_t left_lane_mark_index =-1;
  Int32_t right_lane_mark_index = -1;
  // Parse left lane and right lane from camera
  Scalar curr_lane_mark_len = 0.0; //当前车道最长长度
  for (Int32_t i = 0; i < lane_mark_list.lane_mark_num; ++i) {
    const ad_msg::LaneMarkCamera& lane_mark =
        lane_mark_list.lane_marks[i];
    if (lane_mark.quality < 2) {
      // 识别效果差，丢弃此车道线
      continue;
    }
    if ((1 != lane_mark.id) && (-1 != lane_mark.id)) {
      // 目前只有当前车道两侧的车道线识别效果好，其它更远处的相邻车道识别效果差
      continue;
    }
    // 识别的车道线的有效距离
    Scalar lane_mark_len =
        lane_mark.view_range_end - lane_mark.view_range_start;
    if (lane_mark_len < (common::Max(2.0F*sample_step_len, 5.0F))) {
    //if (lane_mark_len < (common::Max(2.0F*sample_step_len, common::Max(common::Min(1.0F*veh_velocity, 30.0F), 5.0F)))) {
      // 识别的车道线的有效距离太短
      continue;
    }
    curr_lane_mark_len = common::Max(curr_lane_mark_len, lane_mark_len);

    if (1 == lane_mark.id) {
      left_lane_mark_index = i;
    } else if (-1 == lane_mark.id) {
      right_lane_mark_index = i;
    } else {
      // nothing to do
    }
  }
  /* k001 sip 2020-12-28 (end) */

  //common::Matrix<Scalar, 3, 3> mat_convert_tmp;
  //mat_convert_tmp.SetIdentity();
  SmoothLaneMark(lane_mark_list, mat_convert);

  // 对当前车道的左右车道线进行处理
  for (Int32_t i = 0; i < lane_mark_list.lane_mark_num; ++i) {
    const ad_msg::LaneMarkCamera& lane_mark = lane_mark_list.lane_marks[i];
    if (lane_mark.quality < 2) {
      // 识别效果差，丢弃此车道线
      continue;
    }

    // 识别的车道线的有效距离
    Scalar lane_mark_len =
        lane_mark.view_range_end - lane_mark.view_range_start;
    if (lane_mark_len < (common::Max(2.0F*sample_step_len, 5.0F))) {
      // 识别的车道线的有效距离太短
      continue;
    }

    if (1 == lane_mark.id) {
      valid_left_lane_mark = true;
    }
    if (-1 == lane_mark.id) {
      valid_right_lane_mark = true;
    }
    if ((2 == lane_mark.id) && (lane_mark_len > 20.0F)) {
      valid_next_left_lane_mark = true;
    }
    if ((-2 == lane_mark.id) && (lane_mark_len > 20.0F)) {
      valid_next_right_lane_mark = true;
    }
    /* k003 longjiaoy 2022-11-28 (start) */
    // 之前不填充相邻车道的车道边线, 现在修改为填充所有的车道边线
    // if ((1 != lane_mark.id) && (-1 != lane_mark.id)) {
    //   // 目前只有当前车道两侧的车道线识别效果好，其它更远处的相邻车道识别效果差
    //   continue;
    // }
    /* k003 longjiaoy 2022-11-28 (end) */
    // 使用当前车道的左右边线的最大长度
    lane_mark_len = curr_lane_mark_len;
    // 需要采样的车道线的点的数量
    Int32_t sample_size = common::com_round(lane_mark_len / sample_step_len);
    // 根据车速，延长车道
    Int32_t sample_size_ext = sample_size;
    if (lane_mark_len < (4.0F*common::com_abs(veh_velocity))) {
      sample_size_ext = common::com_round(4.0F*common::com_abs(veh_velocity) / sample_step_len);
#if ENABLE_LANE_MARK_CAMERA_FILTER_TRACE
      std::cout << "Exten lane line[" << lane_mark.id
                << "] length from " << lane_mark_len
                << "[" << sample_size
                << "] to " << 4.0F*common::com_abs(veh_velocity)
                << "[" << sample_size_ext << "]" << std::endl;
#endif
    }

    /* k006 pengc 2023-02-16 (begin) */
    // 平滑车道线，检测异常跳变的车道线
    common::CubicPolynomialCurve1d<Scalar> curve;
    if (internal_lane_info_.valid) {
      if (1 == lane_mark.id) {
        const InternalLaneInfo::BoundaryLine& boundary =
            internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_LEFT);
        if (boundary.valid) {
          curve.SetCoefficient(boundary.coef[0], boundary.coef[1],
                               boundary.coef[2], boundary.coef[3]);
        } else {
          curve.SetCoefficient(lane_mark.c0, lane_mark.c1,
                               lane_mark.c2, lane_mark.c3);
        }
      } else if (-1 == lane_mark.id) {
        const InternalLaneInfo::BoundaryLine& boundary =
            internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_RIGHT);
        if (boundary.valid) {
          curve.SetCoefficient(boundary.coef[0], boundary.coef[1],
                               boundary.coef[2], boundary.coef[3]);
        } else {
          curve.SetCoefficient(lane_mark.c0, lane_mark.c1,
                               lane_mark.c2, lane_mark.c3);
        }
      } else if (2 == lane_mark.id) {
        const InternalLaneInfo::BoundaryLine& boundary =
            internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_LEFT_2);
        if (boundary.valid) {
          curve.SetCoefficient(boundary.coef[0], boundary.coef[1],
                               boundary.coef[2], boundary.coef[3]);
        } else {
          curve.SetCoefficient(lane_mark.c0, lane_mark.c1,
                               lane_mark.c2, lane_mark.c3);
        }
      } else if (-2 == lane_mark.id) {
        const InternalLaneInfo::BoundaryLine& boundary =
            internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_RIGHT_2);
        if (boundary.valid) {
          curve.SetCoefficient(boundary.coef[0], boundary.coef[1],
                               boundary.coef[2], boundary.coef[3]);
        } else {
          curve.SetCoefficient(lane_mark.c0, lane_mark.c1,
                               lane_mark.c2, lane_mark.c3);
        }
      } else {
        curve.SetCoefficient(lane_mark.c0, lane_mark.c1,
                             lane_mark.c2, lane_mark.c3);
      }
    } else {
      curve.SetCoefficient(lane_mark.c0, lane_mark.c1,
                           lane_mark.c2, lane_mark.c3);
    }
    /* k006 pengc 2023-02-16 (end) */

    // 新的车道线列表中曲线点的起始索引
    Int32_t new_lane_line_point_index = 0;
    // 车道线id
    Int32_t lane_id = lane_mark.id;
    // 指向需要生成的新的车道线
    ad_msg::LaneBoundaryLineCamera* new_lane_line =
        &(next_lane_info_list.boundary_lines[new_lane_line_index]);
    new_lane_line_index++;
    /* k003 longjiaoy 2022-11-28 (start) */
    // 目前只有当前车道两侧的车道线识别效果好，其它更远处的相邻车道识别效果差,
    // 所以对于相邻的车道, 其边线不跟之前保存的车道边线进行合并
    if ((lane_boundary_line_seg_kdtree_.GetObjectsSize() < 1) ||
        !match_prev_lane_mark ||
        ((1 != lane_mark.id) && (-1 != lane_mark.id))) {
    /* k003 longjiaoy 2022-11-28 (end) */
      // 第一次构造
      if (new_lane_line_index >
          ad_msg::LaneInfoCameraList::MAX_LANE_BOUNDARY_LINE_NUM) {
        LOG_ERR << "The index of lane line exceed the max number.";
        break;
      }

      point_conv(0) = 0.0F + lane_mark.view_range_start;
      point_conv(1) = curve.Evaluate(0, point_conv(0));
      common::TransformVert_2D(mat_camera_calibration_, &point_conv);
      new_lane_line->curve[new_lane_line_point_index].fake = false;
      new_lane_line->curve[new_lane_line_point_index].x = point_conv(0);
      new_lane_line->curve[new_lane_line_point_index].y = point_conv(1);
      new_lane_line_point_index++;

      // printf("a: Add new point[%d]: (%0.1f, %0.1f)\n",
      //        new_lane_line_point_index-1, point_conv(0), point_conv(1));
    } else {
      // 不是第一次构造，需要跟之间保存的车道线进行合并
      // 使用新的车道线替换之前保存的车道线的重合部分

      // 新的车道线的起点，需要转换到之前保存的车道线的坐标系下
      common::Vec2d start_point(
            0.0F + lane_mark.view_range_start,
            curve.Evaluate(0.0F, 0.0F + lane_mark.view_range_start));
      point_conv(0) = start_point.x();
      point_conv(1) = start_point.y();
      common::TransformVert_2D(mat_camera_calibration_, &point_conv);
      start_point.set_x(point_conv(0));
      start_point.set_y(point_conv(1));
      common::TransformVert_2D(mat_convert, &point_conv);
      common::Vec2d start_point_conv(point_conv(0), point_conv(1));

      // 查找新的车道线与之前保存的车道线的重合部分的起点
      Int32_t obj_index = 0;
      Scalar min_sq_distance = 0.0F;
      temp_data_.tree_nodes_stack.Clear();
      const LaneLineSegObj* obj = lane_boundary_line_seg_kdtree_.FindNearest(
            temp_data_.tree_nodes_stack, start_point_conv,
            CalcSquareDistToPt(start_point_conv, curr_lane_info_list),
            &obj_index, &min_sq_distance);
      if (Nullptr_t == obj) {
        LOG_ERR << "Failed to find nearest point from lane line list.";
      } else {
        if ((obj->lane_line_index < 0) ||
            (obj->lane_line_index >= curr_lane_info_list.boundary_line_num)) {
          LOG_ERR << "Detected invalid lane index " << obj->lane_line_index;
        } else {
          const ad_msg::LaneBoundaryLineCamera& old_lane_line =
              curr_lane_info_list.boundary_lines[obj->lane_line_index];
          if ((obj->point_index < 0) ||
              (obj->point_index >= (old_lane_line.curve_point_num-1))) {
            LOG_ERR << "Detected invalid point index " << obj->point_index;
          } else {
            const ad_msg::LaneBoundaryLineCamera::CurvePoint& point_ref1 =
                old_lane_line.curve[obj->point_index];
            const ad_msg::LaneBoundaryLineCamera::CurvePoint& point_ref2 =
                old_lane_line.curve[obj->point_index+1];
            common::Vec2d point1(point_ref1.x, point_ref1.y);
            common::Vec2d point2(point_ref2.x, point_ref2.y);
            common::Vec2d nearest_point;
            Scalar t = 0.0F;
            common::ClosestPtPointToLine_2D(point1, point2, start_point_conv,
                                            &(nearest_point), &t);
            Scalar squared_dist =
                start_point_conv.DistanceSquareTo(nearest_point);
            // Int32_t orientation_on_path = 0;
            // orientation_on_path = common::Orient_2D(
            //       point1, point2, start_point_conv);
            // if (orientation_on_path < 0) {
            //   // right side
            // } else {
            //   // left side
            // }
#if ENABLE_LANE_MARK_CAMERA_FILTER_TRACE
            std::cout << "squared_dist = " << squared_dist
                      << ", t = " << t
                      << ", point_index = " << obj->point_index
                      << std::endl;
#endif
            if (squared_dist < 0.1F*0.1F) {
              // 找到了之前保存的车道线与新的车道线的重合部分的起点
              // 只保留之前保存的车道线的未重合部分的部分点
              if ((0 == obj->point_index) &&
                  (nearest_point.DistanceSquareTo(point1) > 0.1F*0.1F) &&
                  ((-1e-6F < t) && (t < 1.0F+1e-6F))) {
                point_conv(0) = old_lane_line.curve[0].x;
                point_conv(1) = old_lane_line.curve[0].y;
                common::TransformVert_2D(mat_convert_invr, &point_conv);

                new_lane_line->curve[new_lane_line_point_index].fake =
                    old_lane_line.curve[0].fake;
                new_lane_line->curve[new_lane_line_point_index].x =
                    point_conv(0);
                new_lane_line->curve[new_lane_line_point_index].y =
                    point_conv(1);
                new_lane_line_point_index++;

                // printf("b: Add new point[%d]: (%0.1f, %0.1f)\n",
                //        new_lane_line_point_index-1, point_conv(0), point_conv(1));
              } else {
                Int32_t start_index = obj->point_index - 60;
                if (start_index < 0) {
                  start_index = 0;
                }
                for (Int32_t j = start_index; j < obj->point_index; ++j) {
                  // 将旧的车道线保存到新的坐标系下
                  // std::cout << "add old point[" << j << "]: ";
                  point_conv(0) = old_lane_line.curve[j].x;
                  point_conv(1) = old_lane_line.curve[j].y;
                  // std::cout << "From(" << point_conv(0) << "," << point_conv(1)
                  //          << ") ";
                  common::TransformVert_2D(mat_convert_invr, &point_conv);
                  // std::cout << "To(" << point_conv(0) << "," << point_conv(1)
                  //          << ")" << std::endl;

                  new_lane_line->curve[new_lane_line_point_index].fake =
                      old_lane_line.curve[j].fake;
                  new_lane_line->curve[new_lane_line_point_index].x =
                      point_conv(0);
                  new_lane_line->curve[new_lane_line_point_index].y =
                      point_conv(1);
                  new_lane_line_point_index++;

                  // printf("c: Add new point[%d]: (%0.1f, %0.1f)\n",
                  //        new_lane_line_point_index-1, point_conv(0), point_conv(1));

                  if (new_lane_line_point_index >=
                      ad_msg::LaneBoundaryLineCamera::MAX_CURVE_POINT_NUM) {
                    break;
                  }

                  if ((obj->point_index-1) == j) {
                    // 若两个点之间的距离过大，则在中间插值一个点
                    point1.set_x(point_conv(0));
                    point1.set_y(point_conv(1));
                    point_conv(0) = old_lane_line.curve[j+1].x;
                    point_conv(1) = old_lane_line.curve[j+1].y;
                    common::TransformVert_2D(mat_convert_invr, &point_conv);
                    point2.set_x(point_conv(0));
                    point2.set_y(point_conv(1));
                    if (point1.DistanceTo(point2) > sample_step_len) {
                      nearest_point = 0.2F * point1 + 0.8F * point2;
                      new_lane_line->curve[new_lane_line_point_index].fake =
                          old_lane_line.curve[j].fake;
                      new_lane_line->curve[new_lane_line_point_index].x =
                          nearest_point.x();
                      new_lane_line->curve[new_lane_line_point_index].y =
                          nearest_point.y();
                      new_lane_line_point_index++;

                      // printf("d: Add new point[%d]: (%0.1f, %0.1f)\n",
                      //        new_lane_line_point_index-1, point_conv(0), point_conv(1));
                    }
                  }
                }
              }
            }
          }
        }
      }

      if (new_lane_line_point_index <
          ad_msg::LaneBoundaryLineCamera::MAX_CURVE_POINT_NUM) {
        // 保存新数据的起点
        new_lane_line->curve[new_lane_line_point_index].fake = false;
        new_lane_line->curve[new_lane_line_point_index].x = start_point.x();
        new_lane_line->curve[new_lane_line_point_index].y = start_point.y();
        new_lane_line_point_index++;

        // printf("e: Add new point[%d]: (%0.1f, %0.1f)\n",
        //        new_lane_line_point_index-1, point_conv(0), point_conv(1));
      }
    }

    // 将新的车道线保存到车道线列表中
    for (Int32_t j = 1; j < sample_size_ext; ++j) {
      if (new_lane_line_point_index >=
          ad_msg::LaneBoundaryLineCamera::MAX_CURVE_POINT_NUM) {
        break;
      }
      point_conv(0) = j * sample_step_len + lane_mark.view_range_start;
      point_conv(1) = curve.Evaluate(0, point_conv(0));
      common::TransformVert_2D(mat_camera_calibration_, &point_conv);
      if (j < sample_size) {
        new_lane_line->curve[new_lane_line_point_index].fake = false;
      } else {
        new_lane_line->curve[new_lane_line_point_index].fake = true;
      }
      new_lane_line->curve[new_lane_line_point_index].x = point_conv(0);
      new_lane_line->curve[new_lane_line_point_index].y = point_conv(1);
      new_lane_line_point_index++;

      // printf("f: Add new point[%d]: (%0.1f, %0.1f)\n",
      //        new_lane_line_point_index-1, point_conv(0), point_conv(1));
    }
    new_lane_line->curve_point_num = new_lane_line_point_index;
    new_lane_line->id = lane_id;
    /* k003 longjiaoy 2022-11-28 (start) */
    // 保存车道边线类型
    new_lane_line->type = lane_mark.lane_mark_type;
    /* k003 longjiaoy 2022-11-28 (end) */
  }
  
  next_lane_info_list.boundary_line_num = new_lane_line_index;

  bool next_lane_info_list_valid = false;
  for (Int32_t lane_index = 0;
       lane_index < next_lane_info_list.boundary_line_num; ++lane_index) {
    if (next_lane_info_list.boundary_lines[lane_index].curve_point_num > 1) {
      next_lane_info_list_valid = true;
      break;
    }
  }

  if (next_lane_info_list_valid &&
      (valid_left_lane_mark || valid_right_lane_mark)) {
    next_lane_info_list.msg_head = lane_mark_list.msg_head;

    // 使用更新后的车道线构建kd-tree
    BuildLaneLineSegKDTree(next_lane_info_list);

    // Construct centre line of lanes
    ConstructLaneCenterLines(next_lane_info_list);

    // 切换当前的车道线列表
    curr_lane_info_list_index_ = next_lane_info_list_index;

    update_success = true;
  }

#if ENABLE_LANE_MARK_CAMERA_FILTER_TRACE
  std::cout << "### UpdateLaneLineList (End) ###" << std::endl;
#endif

  return (update_success);
}

/* k006 pengc 2023-02-16 (begin) */
// 平滑车道线，检测异常跳变的车道线
bool LaneMarkCameraFilter::SmoothLaneMark(
    const ad_msg::LaneMarkCameraList& lane_mark_list,
    const common::Matrix<Scalar, 3, 3>& mat_conv_new_to_old) {

  // LOG_INFO(5) << "### SmoothLaneMark (begin) ###";

  if (internal_lane_info_.valid) {
    Int64_t time_elapsed = common::CalcElapsedClockMs(
          internal_lane_info_.timestamp, lane_mark_list.msg_head.timestamp);
    if (time_elapsed > 500 || time_elapsed < 0) {
      // 旧的数据过时了
      internal_lane_info_.Clear();
    }
  }

  Int32_t left_mark_idx =-1;
  Int32_t right_mark_idx = -1;
  Int32_t left_2_mark_idx =-1;
  Int32_t right_2_mark_idx = -1;
  Scalar left_mark_len = 0.0F;
  Scalar right_mark_len = 0.0F;
  Scalar left_2_mark_len = 0.0F;
  Scalar right_2_mark_len = 0.0F;
  Scalar curr_lane_len = 0.0;
  // Parse left lane and right lane from camera
  for (Int32_t i = 0; i < lane_mark_list.lane_mark_num; ++i) {
    const ad_msg::LaneMarkCamera& lane_mark =
        lane_mark_list.lane_marks[i];
    if (lane_mark.quality < 2) {
      // 识别效果差，丢弃此车道线
      continue;
    }
    // 识别的车道线的有效距离
    Scalar mark_len = lane_mark.view_range_end - lane_mark.view_range_start;
    if (mark_len < 20.0F) {
      // 识别的车道线的有效距离太短
      continue;
    }
    if (1 == lane_mark.id) {
      left_mark_idx = i;
      left_mark_len = mark_len;
    } else if (-1 == lane_mark.id) {
      right_mark_idx = i;
      right_mark_len = mark_len;
    } else if (2 == lane_mark.id) {
      left_2_mark_idx = i;
      left_2_mark_len = mark_len;
    } else if (-2 == lane_mark.id) {
      right_2_mark_idx = i;
      right_2_mark_len = mark_len;
    } else {
      // nothing to do
    }
  }
  curr_lane_len = common::Max(left_mark_len, right_mark_len);

  if ((left_mark_idx < 0) && (right_mark_idx < 0)) {
    return false;
  }

  SmoothFirstCoefOfLaneMark(
      lane_mark_list, mat_conv_new_to_old,
      left_mark_idx, right_mark_idx, left_2_mark_idx, right_2_mark_idx);

  Float64_t corrected_c1 = 0.0F;
  Float64_t corrected_c2 = 0.0F;
  Float64_t corrected_c3 = 0.0F;
  if (!SmoothCommonCoefOfLaneMark(lane_mark_list, left_mark_idx, right_mark_idx,
                                  &corrected_c1, &corrected_c2, &corrected_c3)) {
    LOG_ERR << "Failed to smooth common coefficient of lane mark.";
    return false;
  }

  internal_lane_info_.valid = true;
  internal_lane_info_.timestamp = lane_mark_list.msg_head.timestamp;
  internal_lane_info_.common_curve_c1 = corrected_c1;
  internal_lane_info_.common_curve_c2 = corrected_c2;
  internal_lane_info_.common_curve_c3 = corrected_c3;

  InternalLaneInfo::BoundaryLine& left_boundary =
      internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_LEFT);
  InternalLaneInfo::BoundaryLine& right_boundary =
      internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_RIGHT);
  InternalLaneInfo::BoundaryLine& left_2_boundary =
      internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_LEFT_2);
  InternalLaneInfo::BoundaryLine& right_2_boundary =
      internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_RIGHT_2);

  if (left_boundary.valid) {
    left_boundary.coef[1] = internal_lane_info_.common_curve_c1;
    left_boundary.coef[2] = internal_lane_info_.common_curve_c2;
    left_boundary.coef[3] = internal_lane_info_.common_curve_c3;

    left_boundary.forward_len = left_mark_len;
  }
  if (right_boundary.valid) {
    right_boundary.coef[1] = internal_lane_info_.common_curve_c1;
    right_boundary.coef[2] = internal_lane_info_.common_curve_c2;
    right_boundary.coef[3] = internal_lane_info_.common_curve_c3;

    right_boundary.forward_len = right_mark_len;
  }
  if (left_2_boundary.valid) {
    left_2_boundary.coef[1] = internal_lane_info_.common_curve_c1;
    left_2_boundary.coef[2] = internal_lane_info_.common_curve_c2;
    left_2_boundary.coef[3] = internal_lane_info_.common_curve_c3;

    left_2_boundary.forward_len = left_2_mark_len;
  }
  if (right_2_boundary.valid) {
    right_2_boundary.coef[1] = internal_lane_info_.common_curve_c1;
    right_2_boundary.coef[2] = internal_lane_info_.common_curve_c2;
    right_2_boundary.coef[3] = internal_lane_info_.common_curve_c3;

    right_2_boundary.forward_len = right_2_mark_len;
  }

  // LOG_INFO(5) << "### SmoothLaneMark (end) ###";

  return true;
}

void LaneMarkCameraFilter::SmoothFirstCoefOfLaneMark(
    const ad_msg::LaneMarkCameraList& lane_mark_list,
    const common::Matrix<Scalar, 3, 3>& mat_conv_new_to_old,
    Int32_t left_mark_idx, Int32_t right_mark_idx,
    Int32_t left_2_mark_idx, Int32_t right_2_mark_idx) {

  common::Matrix<Scalar, 2, 1> point_conv;
  common::CubicPolynomialCurve1d<Scalar> curve;

  InternalLaneInfo::BoundaryLine* old_left_mark =
      &internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_LEFT);
  InternalLaneInfo::BoundaryLine* old_right_mark =
      &internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_RIGHT);
  InternalLaneInfo::BoundaryLine* old_left_2_mark =
      &internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_LEFT_2);
  InternalLaneInfo::BoundaryLine* old_right_2_mark =
      &internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_RIGHT_2);

  const ad_msg::LaneMarkCamera* left_mark = Nullptr_t;
  const ad_msg::LaneMarkCamera* right_mark = Nullptr_t;
  const ad_msg::LaneMarkCamera* left_2_mark = Nullptr_t;
  const ad_msg::LaneMarkCamera* right_2_mark = Nullptr_t;
  common::Vec2d start_of_left_mark;
  common::Vec2d start_of_right_mark;
  common::Vec2d start_of_left_2_mark;
  common::Vec2d start_of_right_2_mark;

  common::PathPoint left_mark_proj_on_old_left_mark;
  common::PathPoint left_mark_proj_on_old_right_mark;
  common::PathPoint right_mark_proj_on_old_left_mark;
  common::PathPoint right_mark_proj_on_old_right_mark;
  common::PathPoint left_2_mark_proj_on_old_left_2_mark;
  common::PathPoint right_2_mark_proj_on_old_right_2_mark;

  if (left_mark_idx >= 0) {
    left_mark = &lane_mark_list.lane_marks[left_mark_idx];

    curve.SetCoefficient(
          left_mark->c0, left_mark->c1, left_mark->c2, left_mark->c3);
    point_conv(0) = 0.0F + left_mark->view_range_start;
    point_conv(1) = curve.Evaluate(0.0F, 0.0F + left_mark->view_range_start);
    common::TransformVert_2D(mat_camera_calibration_, &point_conv);
    common::TransformVert_2D(mat_conv_new_to_old, &point_conv);
    start_of_left_mark.set_x(point_conv(0));
    start_of_left_mark.set_y(point_conv(1));
  }
  if (right_mark_idx >= 0) {
    right_mark = &lane_mark_list.lane_marks[right_mark_idx];

    curve.SetCoefficient(
          right_mark->c0, right_mark->c1, right_mark->c2, right_mark->c3);
    point_conv(0) = 0.0F + right_mark->view_range_start;
    point_conv(1) = curve.Evaluate(0.0F, 0.0F + right_mark->view_range_start);
    common::TransformVert_2D(mat_camera_calibration_, &point_conv);
    common::TransformVert_2D(mat_conv_new_to_old, &point_conv);
    start_of_right_mark.set_x(point_conv(0));
    start_of_right_mark.set_y(point_conv(1));
  }
  if (left_2_mark_idx >= 0) {
    left_2_mark = &lane_mark_list.lane_marks[left_2_mark_idx];

    curve.SetCoefficient(
          left_2_mark->c0, left_2_mark->c1, left_2_mark->c2, left_2_mark->c3);
    point_conv(0) = 0.0F + left_2_mark->view_range_start;
    point_conv(1) = curve.Evaluate(0.0F, 0.0F + left_2_mark->view_range_start);
    common::TransformVert_2D(mat_camera_calibration_, &point_conv);
    common::TransformVert_2D(mat_conv_new_to_old, &point_conv);
    start_of_left_2_mark.set_x(point_conv(0));
    start_of_left_2_mark.set_y(point_conv(1));
  }
  if (right_2_mark_idx >= 0) {
    right_2_mark = &lane_mark_list.lane_marks[right_2_mark_idx];

    curve.SetCoefficient(
          right_2_mark->c0, right_2_mark->c1, right_2_mark->c2, right_2_mark->c3);
    point_conv(0) = 0.0F + right_2_mark->view_range_start;
    point_conv(1) = curve.Evaluate(0.0F, 0.0F + right_2_mark->view_range_start);
    common::TransformVert_2D(mat_camera_calibration_, &point_conv);
    common::TransformVert_2D(mat_conv_new_to_old, &point_conv);
    start_of_right_2_mark.set_x(point_conv(0));
    start_of_right_2_mark.set_y(point_conv(1));
  }

  if (left_mark_idx >= 0) {
    if (old_left_mark->valid) {
      if (!old_left_mark->curve.FindProjection(
            start_of_left_mark, &left_mark_proj_on_old_left_mark)) {
        old_left_mark->valid = false;
        left_mark_proj_on_old_left_mark.Clear();

        LOG_ERR << "Failed to find projection on old left lane mark.";
      }
      // printf("left_mark_proj_on_old_left_mark.l=%f\n", left_mark_proj_on_old_left_mark.l);
    }
    if (old_right_mark->valid) {
      if (!old_right_mark->curve.FindProjection(
            start_of_left_mark, &left_mark_proj_on_old_right_mark)) {
        old_right_mark->valid = false;
        left_mark_proj_on_old_right_mark.Clear();

        LOG_ERR << "Failed to find projection on old right lane mark.";
      }
      // printf("left_mark_proj_on_old_right_mark.l=%f\n", left_mark_proj_on_old_right_mark.l);
    }
  }
  if (right_mark_idx >= 0) {
    if (old_left_mark->valid) {
      if (!old_left_mark->curve.FindProjection(
            start_of_right_mark, &right_mark_proj_on_old_left_mark)) {
        old_left_mark->valid = false;
        right_mark_proj_on_old_left_mark.Clear();

        LOG_ERR << "Failed to find projection on old left lane mark.";
      }
      // printf("right_mark_proj_on_old_left_mark.l=%f\n", right_mark_proj_on_old_left_mark.l);
    }
    if (old_right_mark->valid) {
      if (!old_right_mark->curve.FindProjection(
            start_of_right_mark, &right_mark_proj_on_old_right_mark)) {
        old_right_mark->valid = false;
        right_mark_proj_on_old_right_mark.Clear();

        LOG_ERR << "Failed to find projection on old right lane mark.";
      }
      // printf("right_mark_proj_on_old_right_mark.l=%f\n", right_mark_proj_on_old_right_mark.l);
    }
  }
  if (left_2_mark_idx >= 0) {
    if (old_left_2_mark->valid) {
      if (!old_left_2_mark->curve.FindProjection(
            start_of_left_2_mark, &left_2_mark_proj_on_old_left_2_mark)) {
        old_left_2_mark->valid = false;
        left_2_mark_proj_on_old_left_2_mark.Clear();

        LOG_ERR << "Failed to find projection on old left 2 lane mark.";
      }
      // printf("left_2_mark_proj_on_old_left_2_mark.l=%f\n", left_2_mark_proj_on_old_left_2_mark.l);
    }
  }
  if (right_2_mark_idx >= 0) {
    if (old_right_2_mark->valid) {
      if (!old_right_2_mark->curve.FindProjection(
            start_of_right_2_mark, &right_2_mark_proj_on_old_right_2_mark)) {
        old_right_2_mark->valid = false;
        right_2_mark_proj_on_old_right_2_mark.Clear();

        LOG_ERR << "Failed to find projection on old right 2 lane mark.";
      }
      // printf("right_2_mark_proj_on_old_right_2_mark.l=%f\n", right_2_mark_proj_on_old_right_2_mark.l);
    }
  }

  Int32_t changing_lane_flag = DetectLaneChangedEvent(
        left_mark, right_mark, old_left_mark, old_right_mark,
        left_mark_proj_on_old_left_mark, left_mark_proj_on_old_right_mark,
        right_mark_proj_on_old_left_mark, right_mark_proj_on_old_right_mark);
  if (1 == changing_lane_flag) {
    // 检测到向左变道了
    internal_lane_info_.ShiftBoundaryLineList(true);
    internal_lane_info_.ShiftCenterLineList(true);

    LOG_INFO(5) << "Detected lane changed by lane mark (-->left).";
  } else if (2 == changing_lane_flag) {
    // 检测到向右变道了
    internal_lane_info_.ShiftBoundaryLineList(false);
    internal_lane_info_.ShiftCenterLineList(false);

    LOG_INFO(5) << "Detected lane changed by lane mark (-->right).";
  } else {
    // 未变道
    // LOG_INFO(5) << "Undetected lane changed.";
  }
  if (0 != changing_lane_flag) {
    old_left_mark = &internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_LEFT);
    old_right_mark = &internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_RIGHT);
    old_left_2_mark = &internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_LEFT_2);
    old_right_2_mark = &internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_RIGHT_2);
  }

  // Smooth c0
  /// TODO: 若发生了变道，暂时不平滑c0的值，未来根据情况考虑是否平滑c0的值
  if ((1 == changing_lane_flag) || (2 == changing_lane_flag)) {
    // 左车道线
    SmoothLatOffsetOfLaneMark(
          "left mark", 0.0F, left_mark, old_left_mark);
    // 右车道线
    SmoothLatOffsetOfLaneMark(
          "right mark", 0.0F, right_mark, old_right_mark);
    // 左2车道线
    SmoothLatOffsetOfLaneMark(
          "left 2 mark", 0.0F, left_2_mark, old_left_2_mark);
    // 右2车道线
    SmoothLatOffsetOfLaneMark(
          "right 2 mark", 0.0F, right_2_mark, old_right_2_mark);
  } else {
    // 左车道线
    SmoothLatOffsetOfLaneMark(
          "left mark", left_mark_proj_on_old_left_mark.l,
          left_mark, old_left_mark);
    // 右车道线
    SmoothLatOffsetOfLaneMark(
          "right mark", right_mark_proj_on_old_right_mark.l,
          right_mark, old_right_mark);
    // 左2车道线
    SmoothLatOffsetOfLaneMark(
          "left 2 mark", left_2_mark_proj_on_old_left_2_mark.l,
          left_2_mark, old_left_2_mark);
    // 右2车道线
    SmoothLatOffsetOfLaneMark(
          "right 2 mark", right_2_mark_proj_on_old_right_2_mark.l,
          right_2_mark, old_right_2_mark);
  }

  CalcLaneWidthByLaneMark();
}

bool LaneMarkCameraFilter::SmoothCommonCoefOfLaneMark(
    const ad_msg::LaneMarkCameraList& lane_mark_list,
    Int32_t left_mark_idx,
    Int32_t right_mark_idx,
    Float64_t* corrected_c1,
    Float64_t* corrected_c2,
    Float64_t* corrected_c3) {
  bool corrected_valid = false;

  *corrected_c1 = 0.0F;
  *corrected_c2 = 0.0F;
  *corrected_c3 = 0.0F;
  if ((left_mark_idx >= 0) && (right_mark_idx >= 0)) {
    // 左右两条车道边线都识别了, 使用平均的参数，平滑某些异常识别的车道线的情况
    const ad_msg::LaneMarkCamera& left_mark =
        lane_mark_list.lane_marks[left_mark_idx];
    const ad_msg::LaneMarkCamera& right_mark =
        lane_mark_list.lane_marks[right_mark_idx];
    Float32_t left_mark_len =
        left_mark.view_range_end - left_mark.view_range_start;
    Float32_t right_mark_len =
        right_mark.view_range_end - right_mark.view_range_start;
    Int32_t prio_mark_flag = 0;

    *corrected_c1 = 0.5 * (left_mark.c1 + right_mark.c1);
    *corrected_c2 = 0.5 * (left_mark.c2 + right_mark.c2);
    *corrected_c3 = 0.5 * (left_mark.c3 + right_mark.c3);

    if ((left_mark_len > 1.8F*right_mark_len) || 
        ((left_mark_len > 40.0F) && (right_mark_len < 30.0F))) {
      prio_mark_flag = 1;
    } else if ((right_mark_len > 1.8F*left_mark_len) ||
               ((right_mark_len > 40.0F) && (left_mark_len < 30.0F))) {
      prio_mark_flag = 2;
    } else {
      if (internal_lane_info_.valid) {
        Float64_t abs_left_mark_diff_c1 =
            common::com_abs(left_mark.c1 - internal_lane_info_.common_curve_c1);
        Float64_t abs_left_mark_diff_c2 =
            common::com_abs(left_mark.c2 - internal_lane_info_.common_curve_c2);
        Float64_t abs_left_mark_diff_c3 =
            common::com_abs(left_mark.c3 - internal_lane_info_.common_curve_c3);
        Float64_t abs_right_mark_diff_c1 =
            common::com_abs(right_mark.c1 - internal_lane_info_.common_curve_c1);
        Float64_t abs_right_mark_diff_c2 =
            common::com_abs(right_mark.c2 - internal_lane_info_.common_curve_c2);
        Float64_t abs_right_mark_diff_c3 =
            common::com_abs(right_mark.c3 - internal_lane_info_.common_curve_c3);

        Float64_t left_mark_cost =
            1000.0F * abs_left_mark_diff_c1 +
            3.5F * 10000.0F * abs_left_mark_diff_c2 +
            10000.0F * abs_left_mark_diff_c3;
        Float64_t right_mark_cost =
            1000.0F * abs_right_mark_diff_c1 +
            3.5F * 10000.0F * abs_right_mark_diff_c2 +
            10000.0F * abs_right_mark_diff_c3;
        if (common::com_abs(left_mark_cost - right_mark_cost) < 10.0F) {
          // nothing to do
        } else if (left_mark_cost < right_mark_cost) {
          prio_mark_flag = 1;
        } else {
          prio_mark_flag = 2;
        }
      } else {
        Float64_t abs_left_mark_c1 = common::com_abs(left_mark.c1);
        Float64_t abs_left_mark_c2 = common::com_abs(left_mark.c2);
        Float64_t abs_left_mark_c3 = common::com_abs(left_mark.c3);
        Float64_t abs_right_mark_c1 = common::com_abs(right_mark.c1);
        Float64_t abs_right_mark_c2 = common::com_abs(right_mark.c2);
        Float64_t abs_right_mark_c3 = common::com_abs(right_mark.c3);

        Float64_t left_mark_cost =
            1000.0F * abs_left_mark_c1 +
            3.5F * 10000.0F * abs_left_mark_c2 +
            10000.0F * abs_left_mark_c3;
        Float64_t right_mark_cost =
            1000.0F * abs_right_mark_c1 +
            3.5F * 10000.0F * abs_right_mark_c2 +
            10000.0F * abs_right_mark_c3;
        if (common::com_abs(left_mark_cost - right_mark_cost) < 10.0F) {
          // nothing to do
        } else if (left_mark_cost < right_mark_cost) {
          prio_mark_flag = 1;
        } else {
          prio_mark_flag = 2;
        }
      }
    }

    if (1 == prio_mark_flag) {
      *corrected_c1 = left_mark.c1;
      *corrected_c2 = left_mark.c2;
      *corrected_c3 = left_mark.c3;
    } else if (2 == prio_mark_flag) {
      *corrected_c1 = right_mark.c1;
      *corrected_c2 = right_mark.c2;
      *corrected_c3 = right_mark.c3;
    } else {
      // nothing to do
    }

#if 0
    if ((left_mark_len < 20.0F) && (right_mark_len < 20.0F)) {
      *corrected_c2 = 0.0F;
      *corrected_c3 = 0.0F;
    }
#endif

    corrected_valid = true;

    //printf("### left_lane: len=%0.1f, c0=%0.4f, c1=%0.4f, c2=%0.4f, c3=%0.4f\n",
    //       left_mark_len,
    //       left_mark.c0,
    //       left_mark.c1,
    //       left_mark.c2,
    //       left_mark.c3);
    //printf("### right_lane: len=%0.1f, c0=%0.4f, c1=%0.4f, c2=%0.4f, c3=%0.4f\n",
    //       right_mark_len,
    //       right_mark.c0,
    //       right_mark.c1,
    //       right_mark.c2,
    //       right_mark.c3);
    //printf("### prio=%d, expected_lane: c1=%0.4f, c2=%0.4f, c3=%0.4f\n",
    //       prio_lane_flag, expected_c1, expected_c2, expected_c3);
  } else if ((left_mark_idx >= 0) && (right_mark_idx < 0)) {
    const ad_msg::LaneMarkCamera& left_mark =
        lane_mark_list.lane_marks[left_mark_idx];
    Float32_t left_mark_len =
        left_mark.view_range_end - left_mark.view_range_start;

    *corrected_c1 = left_mark.c1;
    *corrected_c2 = left_mark.c2;
    *corrected_c3 = left_mark.c3;

#if 0
    if (left_mark_len < 20.0F) {
      *corrected_c2 = 0.0F;
      *corrected_c3 = 0.0F;
    }
#endif

    corrected_valid = true;

    //printf("### left_lane: len=%0.1f, c0=%0.4f, c1=%0.4f, c2=%0.4f, c3=%0.4f\n",
    //       lane_mark_list.lane_marks[left_mark_idx].view_range_end -
    //       lane_mark_list.lane_marks[left_mark_idx].view_range_start,
    //       lane_mark_list.lane_marks[left_mark_idx].c0,
    //       lane_mark_list.lane_marks[left_mark_idx].c1,
    //       lane_mark_list.lane_marks[left_mark_idx].c2,
    //       lane_mark_list.lane_marks[left_mark_idx].c3);
  } else if ((left_mark_idx < 0) && (right_mark_idx >= 0)) {
    const ad_msg::LaneMarkCamera& right_mark =
        lane_mark_list.lane_marks[right_mark_idx];
    Float32_t right_mark_len =
        right_mark.view_range_end - right_mark.view_range_start;

    *corrected_c1 = right_mark.c1;
    *corrected_c2 = right_mark.c2;
    *corrected_c3 = right_mark.c3;

#if 0
    if (right_mark_len < 20.0F) {
      *corrected_c2 = 0.0F;
      *corrected_c3 = 0.0F;
    }
#endif

    corrected_valid = true;

    //printf("### right_lane: len=%0.1f, c0=%0.4f, c1=%0.4f, c2=%0.4f, c3=%0.4f\n",
    //       lane_mark_list.lane_marks[right_mark_idx].view_range_end -
    //       lane_mark_list.lane_marks[right_mark_idx].view_range_start,
    //       lane_mark_list.lane_marks[right_mark_idx].c0,
    //       lane_mark_list.lane_marks[right_mark_idx].c1,
    //       lane_mark_list.lane_marks[right_mark_idx].c2,
    //       lane_mark_list.lane_marks[right_mark_idx].c3);
  } else {
    // nothing to do
  }

  if (corrected_valid) {
    if (internal_lane_info_.valid) {
      Float64_t max_step_c1 = 0.01;
      Float64_t max_step_c2 = 0.00005*4.0;
      Float64_t max_step_c3 = 0.00001;
      Float64_t diff_c1 = *corrected_c1 - internal_lane_info_.common_curve_c1;
      Float64_t diff_c2 = *corrected_c2 - internal_lane_info_.common_curve_c2;
      Float64_t diff_c3 = *corrected_c3 - internal_lane_info_.common_curve_c3;

      if ((left_mark_idx >= 0) && (right_mark_idx >= 0)) {
        // nothing to do
      } else {
        if (common::com_abs(diff_c2) > 0.001) {
          max_step_c2 /= 8.0;
          max_step_c3 /= 8.0;
        } else if (common::com_abs(diff_c2) > 0.0005) {
          max_step_c2 /= 4.0;
          max_step_c3 /= 4.0;
        } else if (common::com_abs(diff_c2) > 0.0002) {
          max_step_c2 /= 2.0;
          max_step_c3 /= 2.0;
        } else {
          // nothing to do
        }
      }

      if (common::com_abs(*corrected_c1) >
          common::com_abs(internal_lane_info_.common_curve_c1)) {
        if (diff_c1 > max_step_c1) {
          *corrected_c1 = internal_lane_info_.common_curve_c1 + max_step_c1;
        } else if (diff_c1 < -max_step_c1) {
          *corrected_c1 = internal_lane_info_.common_curve_c1 - max_step_c1;
        } else {
          // nothing to do
        }
      }
      if (common::com_abs(*corrected_c2) >
          common::com_abs(internal_lane_info_.common_curve_c2)) {
        if (diff_c2 > max_step_c2) {
          *corrected_c2 = internal_lane_info_.common_curve_c2 + max_step_c2;
        } else if (diff_c2 < -max_step_c2) {
          *corrected_c2 = internal_lane_info_.common_curve_c2 - max_step_c2;
        } else {
          // nothing to do
        }
      }
      if (common::com_abs(*corrected_c3) >
          common::com_abs(internal_lane_info_.common_curve_c3)) {
        if (diff_c3 > max_step_c3) {
          *corrected_c3 = internal_lane_info_.common_curve_c3 + max_step_c3;
        } else if (diff_c3 < -max_step_c3) {
          *corrected_c3 = internal_lane_info_.common_curve_c3 - max_step_c3;
        } else {
          // nothing to do
        }
      }
    }
  }

  return (corrected_valid);
}

Int32_t LaneMarkCameraFilter::DetectLaneChangedEvent(
    const ad_msg::LaneMarkCamera* left_mark,
    const ad_msg::LaneMarkCamera* right_mark,
    const InternalLaneInfo::BoundaryLine* old_left_mark,
    const InternalLaneInfo::BoundaryLine* old_right_mark,
    const common::PathPoint& left_mark_proj_on_old_left_mark,
    const common::PathPoint& left_mark_proj_on_old_right_mark,
    const common::PathPoint& right_mark_proj_on_old_left_mark,
    const common::PathPoint& right_mark_proj_on_old_right_mark) const {
  Int32_t changing_lane_flag = 0;
  if ((Nullptr_t != left_mark) && (Nullptr_t != right_mark) &&
      old_left_mark->valid && old_right_mark->valid) {
    if ((common::com_abs(right_mark_proj_on_old_left_mark.l) <
         common::com_abs(left_mark_proj_on_old_left_mark.l)) &&
        (left_mark_proj_on_old_left_mark.l >
         common::Min(0.8F*(left_mark->c0 - right_mark->c0), 2.0))) {
      // 检测到向左变道了
      /* k001 longjiaoy 2023-10-15 (begin) */
      // changing_lane_flag = 1;
      // 还要判断right_mark是否向左移动一个车道
      if (right_mark_proj_on_old_right_mark.l >
          common::Min(0.8F*(left_mark->c0 - right_mark->c0), 2.0)) {
        changing_lane_flag = 1;
      }
      /* k001 longjiaoy 2023-10-15 (end) */
    } else if ((common::com_abs(left_mark_proj_on_old_right_mark.l) <
                common::com_abs(right_mark_proj_on_old_right_mark.l)) &&
               (right_mark_proj_on_old_right_mark.l <
                -common::Min(0.8F*(left_mark->c0 - right_mark->c0), 2.0))) {
      // 检测到向右变道了
      /* k001 longjiaoy 2023-10-15 (begin) */
      // changing_lane_flag = 2;
      // 还要判断left_mark是否向右移动一个车道
      if (left_mark_proj_on_old_left_mark.l  <
          -common::Min(0.8F*(left_mark->c0 - right_mark->c0), 2.0)) {
        changing_lane_flag = 2;
      }
      /* k001 longjiaoy 2023-10-15 (end) */
    } else {
      // nothing to do
    }
  } else if ((Nullptr_t != left_mark) && (Nullptr_t != right_mark) &&
             old_left_mark->valid && !old_right_mark->valid) {
    if ((right_mark_proj_on_old_left_mark.l > -1.5F) &&
        (left_mark_proj_on_old_left_mark.l >
         common::Min(0.8F*(left_mark->c0 - right_mark->c0), 2.0))) {
      // 检测到向左变道了
      changing_lane_flag = 1;
    } else if ((left_mark_proj_on_old_left_mark.l <
                -common::Min(0.8F*(left_mark->c0 - right_mark->c0), 2.0))) {
      // 检测到向右变道了
      changing_lane_flag = 2;
    } else {
      // nothing to do
    }
  } else if ((Nullptr_t != left_mark) && (Nullptr_t != right_mark) &&
             !old_left_mark->valid && old_right_mark->valid) {
    if ((right_mark_proj_on_old_right_mark.l >
         common::Min(0.8F*(left_mark->c0 - right_mark->c0), 2.0))) {
      // 检测到向左变道了
      changing_lane_flag = 1;
    } else if ((left_mark_proj_on_old_right_mark.l < 1.5) &&
               (right_mark_proj_on_old_right_mark.l <
                -common::Min(0.8F*(left_mark->c0 - right_mark->c0), 2.0))) {
      // 检测到向右变道了
      changing_lane_flag = 2;
    } else {
      // nothing to do
    }
  } else if ((Nullptr_t != left_mark) && (Nullptr_t == right_mark) &&
             old_left_mark->valid && old_right_mark->valid) {
    if (left_mark_proj_on_old_left_mark.l > 2.0F) {
      // 检测到向左变道了
      changing_lane_flag = 1;
    } else if ((left_mark_proj_on_old_right_mark.l < 1.5F) &&
               (left_mark_proj_on_old_left_mark.l < -2.0F)) {
      // 检测到向右变道了
      changing_lane_flag = 2;
    } else {
      // nothing to do
    }
  } else if ((Nullptr_t != left_mark) && (Nullptr_t == right_mark) &&
             old_left_mark->valid && !old_right_mark->valid) {
    if (left_mark_proj_on_old_left_mark.l > 2.0F) {
      // 检测到向左变道了
      changing_lane_flag = 1;
    } else if (left_mark_proj_on_old_left_mark.l < -2.0F) {
      // 检测到向右变道了
      changing_lane_flag = 2;
    } else {
      // nothing to do
    }
  } else if ((Nullptr_t != left_mark) && (Nullptr_t == right_mark) &&
             !old_left_mark->valid && old_right_mark->valid) {
    if (left_mark_proj_on_old_right_mark.l > 5.0F) {
      // 检测到向左变道了
      changing_lane_flag = 1;
    } else if (left_mark_proj_on_old_right_mark.l < 1.0F) {
      // 检测到向右变道了
      changing_lane_flag = 2;
    } else {
      // nothing to do
    }
  } else if ((Nullptr_t == left_mark) && (Nullptr_t != right_mark) &&
             old_left_mark->valid && old_right_mark->valid) {
    if ((right_mark_proj_on_old_left_mark.l > -1.5F) &&
        (right_mark_proj_on_old_right_mark.l > 2.0F)) {
      // 检测到向左变道了
      changing_lane_flag = 1;
    } else if (right_mark_proj_on_old_right_mark.l < -2.0F) {
      // 检测到向右变道了
      changing_lane_flag = 2;
    } else {
      // nothing to do
    }
  } else if ((Nullptr_t == left_mark) && (Nullptr_t != right_mark) &&
             old_left_mark->valid && !old_right_mark->valid) {
    if (right_mark_proj_on_old_left_mark.l > -1.0F) {
      // 检测到向左变道了
      changing_lane_flag = 1;
    } else if (right_mark_proj_on_old_left_mark.l < -5.0F) {
      // 检测到向右变道了
      changing_lane_flag = 2;
    } else {
      // nothing to do
    }
  } else if ((Nullptr_t == left_mark) && (Nullptr_t != right_mark) &&
             !old_left_mark->valid && old_right_mark->valid) {
    if (right_mark_proj_on_old_right_mark.l > 2.0F) {
      // 检测到向左变道了
      changing_lane_flag = 1;
    } else if (right_mark_proj_on_old_right_mark.l < -2.0F) {
      // 检测到向右变道了
      changing_lane_flag = 2;
    } else {
      // nothing to do
    }
  } else {
    // nothing to do
  }

  return (changing_lane_flag);
}

void LaneMarkCameraFilter::SmoothLatOffsetOfLaneMark(
    const Char_t* mark_description,
    Scalar lane_mark_offset,
    const ad_msg::LaneMarkCamera* new_lane_mark,
    InternalLaneInfo::BoundaryLine* old_lane_mark) {
  static const Scalar kMaxDeltaLatOffset = 0.1F;

  if (Nullptr_t != new_lane_mark) {
    if (old_lane_mark->valid) {
      if (lane_mark_offset > kMaxDeltaLatOffset) {
        old_lane_mark->coef[0] =
            new_lane_mark->c0 - lane_mark_offset + kMaxDeltaLatOffset;

        LOG_INFO(5) << "!!! Detected " << mark_description
                    << " is jumped to left (diff=" << lane_mark_offset
                    << "), correct c0 from " << new_lane_mark->c0
                    << " to " << old_lane_mark->coef[0];
      } else if (lane_mark_offset < -kMaxDeltaLatOffset) {
        old_lane_mark->coef[0] =
            new_lane_mark->c0 - lane_mark_offset - kMaxDeltaLatOffset;

        LOG_INFO(5) << "!!! Detected " << mark_description
                    << " is jumped to right (diff=" << lane_mark_offset
                    << "), correct c0 from " << new_lane_mark->c0
                    << " to " << old_lane_mark->coef[0];
      } else {
        old_lane_mark->coef[0] = new_lane_mark->c0;
      }
    } else {
      old_lane_mark->valid = true;
      old_lane_mark->coef[0] = new_lane_mark->c0;
    }
    old_lane_mark->age++;
    if (old_lane_mark->age > 100) {
      old_lane_mark->age = 100;
    }
  } else {
    old_lane_mark->valid = false;
    old_lane_mark->Clear();
  }
}

void LaneMarkCameraFilter::CalcLaneWidthByLaneMark() {
  static const Scalar kStandardLaneHalfWidth = 3.3F*0.5F;
  static const Scalar kMaxLaneHalfWidth = 3.75F*0.5F;
  static const Scalar kMaxDeltaLaneHalfWidthDoubleBoundary = 0.10F;
  static const Scalar kMaxDeltaLaneHalfWidthSingleBoundary = 0.02F;

  const InternalLaneInfo::BoundaryLine* left_mark =
      &internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_LEFT);
  const InternalLaneInfo::BoundaryLine* right_mark =
      &internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_RIGHT);
  const InternalLaneInfo::BoundaryLine* left_2_mark =
      &internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_LEFT_2);
  const InternalLaneInfo::BoundaryLine* right_2_mark =
      &internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_RIGHT_2);

  InternalLaneInfo::CenterLine* curr_center_line =
      &internal_lane_info_.center_line(InternalLaneInfo::CENTER_CURR);
  InternalLaneInfo::CenterLine* left_center_line =
      &internal_lane_info_.center_line(InternalLaneInfo::CENTER_LEFT);
  InternalLaneInfo::CenterLine* right_center_line =
      &internal_lane_info_.center_line(InternalLaneInfo::CENTER_RIGHT);

  // current lane
  if (left_mark->valid && right_mark->valid) {
    Scalar half_width = 0.5F * (left_mark->coef[0] - right_mark->coef[0]);
    Scalar left_width = half_width;
    Scalar right_width = half_width;
    if (half_width > kMaxLaneHalfWidth) {
      if (common::com_abs(left_mark->coef[0]) < common::com_abs(right_mark->coef[0])) {
        left_width = kMaxLaneHalfWidth;
        right_width = half_width - kMaxLaneHalfWidth + half_width;
      } else {
        left_width = half_width - kMaxLaneHalfWidth + half_width;
        right_width = kMaxLaneHalfWidth;
      }
    }

    if (curr_center_line->valid) {
      Scalar delta_half_width = left_width - curr_center_line->left_width;
      if (delta_half_width > kMaxDeltaLaneHalfWidthDoubleBoundary) {
        curr_center_line->left_width += kMaxDeltaLaneHalfWidthDoubleBoundary;
      } else if (delta_half_width < -kMaxDeltaLaneHalfWidthDoubleBoundary) {
        curr_center_line->left_width -= kMaxDeltaLaneHalfWidthDoubleBoundary;
      } else {
        curr_center_line->left_width = left_width;
      }

      delta_half_width = right_width - curr_center_line->right_width;
      if (delta_half_width > kMaxDeltaLaneHalfWidthDoubleBoundary) {
        curr_center_line->right_width += kMaxDeltaLaneHalfWidthDoubleBoundary;
      } else if (delta_half_width < -kMaxDeltaLaneHalfWidthDoubleBoundary) {
        curr_center_line->right_width -= kMaxDeltaLaneHalfWidthDoubleBoundary;
      } else {
        curr_center_line->right_width = right_width;
      }
    } else {
      curr_center_line->valid = true;
      curr_center_line->left_width = left_width;
      curr_center_line->right_width = right_width;
    }
  } else if (left_mark->valid && !right_mark->valid) {
    if (curr_center_line->valid) {
      if (curr_center_line->left_width > kMaxLaneHalfWidth) {
        if ((curr_center_line->left_width - kMaxLaneHalfWidth) >
            kMaxDeltaLaneHalfWidthSingleBoundary) {
          curr_center_line->left_width -= kMaxDeltaLaneHalfWidthSingleBoundary;
        } else {
          curr_center_line->left_width = kMaxLaneHalfWidth;
        }
      }
    } else {
      curr_center_line->valid = true;
      curr_center_line->left_width = kStandardLaneHalfWidth;
      curr_center_line->right_width = kStandardLaneHalfWidth;
    }
  } else if (!left_mark->valid && right_mark->valid) {
    if (curr_center_line->valid) {
      if (curr_center_line->right_width > kMaxLaneHalfWidth) {
        if ((curr_center_line->right_width - kMaxLaneHalfWidth) >
            kMaxDeltaLaneHalfWidthSingleBoundary) {
          curr_center_line->right_width -= kMaxDeltaLaneHalfWidthSingleBoundary;
        } else {
          curr_center_line->right_width = kMaxLaneHalfWidth;
        }
      }
    } else {
      curr_center_line->valid = true;
      curr_center_line->left_width = kStandardLaneHalfWidth;
      curr_center_line->right_width = kStandardLaneHalfWidth;
    }
  } else {
    curr_center_line->valid = false;
  }

  // left lane
  if (left_2_mark->valid && left_mark->valid) {
    Scalar half_width = 0.5F * (left_2_mark->coef[0] - left_mark->coef[0]);
    Scalar left_width = half_width;
    Scalar right_width = half_width;
    if (half_width > kMaxLaneHalfWidth) {
      left_width = half_width - kMaxLaneHalfWidth + half_width;
      right_width = kMaxLaneHalfWidth;
    }
    if (left_center_line->valid) {
      Scalar delta_half_width = left_width - left_center_line->left_width;
      if (delta_half_width > kMaxDeltaLaneHalfWidthDoubleBoundary) {
        left_center_line->left_width += kMaxDeltaLaneHalfWidthDoubleBoundary;
      } else if (delta_half_width < -kMaxDeltaLaneHalfWidthDoubleBoundary) {
        left_center_line->left_width -= kMaxDeltaLaneHalfWidthDoubleBoundary;
      } else {
        left_center_line->left_width = left_width;
      }

      delta_half_width = right_width - left_center_line->right_width;
      if (delta_half_width > kMaxDeltaLaneHalfWidthDoubleBoundary) {
        left_center_line->right_width += kMaxDeltaLaneHalfWidthDoubleBoundary;
      } else if (delta_half_width < -kMaxDeltaLaneHalfWidthDoubleBoundary) {
        left_center_line->right_width -= kMaxDeltaLaneHalfWidthDoubleBoundary;
      } else {
        left_center_line->right_width = right_width;
      }
    } else {
      left_center_line->valid = true;
      left_center_line->left_width = left_width;
      left_center_line->right_width = right_width;
    }
  } else {
    left_center_line->valid = false;
  }

  // right lane
  if (right_2_mark->valid && right_mark->valid) {
    Scalar half_width = 0.5F * (right_mark->coef[0] - right_2_mark->coef[0]);
    Scalar left_width = half_width;
    Scalar right_width = half_width;
    if (half_width > kMaxLaneHalfWidth) {
      left_width = kMaxLaneHalfWidth;
      right_width = half_width - kMaxLaneHalfWidth + half_width;
    }
    if (right_center_line->valid) {
      Scalar delta_half_width = left_width - right_center_line->left_width;
      if (delta_half_width > kMaxDeltaLaneHalfWidthDoubleBoundary) {
        right_center_line->left_width += kMaxDeltaLaneHalfWidthDoubleBoundary;
      } else if (delta_half_width < -kMaxDeltaLaneHalfWidthDoubleBoundary) {
        right_center_line->left_width -= kMaxDeltaLaneHalfWidthDoubleBoundary;
      } else {
        right_center_line->left_width = left_width;
      }

      delta_half_width = right_width - right_center_line->right_width;
      if (delta_half_width > kMaxDeltaLaneHalfWidthDoubleBoundary) {
        right_center_line->right_width += kMaxDeltaLaneHalfWidthDoubleBoundary;
      } else if (delta_half_width < -kMaxDeltaLaneHalfWidthDoubleBoundary) {
        right_center_line->right_width -= kMaxDeltaLaneHalfWidthDoubleBoundary;
      } else {
        right_center_line->right_width = right_width;
      }
    } else {
      right_center_line->valid = true;
      right_center_line->left_width = left_width;
      right_center_line->right_width = right_width;
    }
  } else {
    right_center_line->valid = false;
  }
}

/* k006 pengc 2023-02-16 (end) */


/**
 * @date       2020.11.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/11/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
bool LaneMarkCameraFilter::RegisterFramesOfLaneLines(
    const ad_msg::LaneMarkCameraList& lane_mark_list,
    Scalar filtered_delta_pos[3]) {
  const Int32_t max_sample_num = 7;
  const Scalar sample_step_len = 5.0F;

#if ENABLE_LANE_MARK_CAMERA_FILTER_TRACE
  std::cout << "### RegisterFramesOfLaneLines (Begin) ###" << std::endl;
#endif


  bool register_ret = true;
  FuncGetPoints get_src_points(temp_data_.sample_points[0]);
  FuncGetPoints get_dst_points(temp_data_.sample_points[1]);
  phoenix::common::Matrix<Scalar, 3, 3> mat_rot;
  phoenix::common::Matrix<Scalar, 3, 1> vec_tran;

  for (Int32_t iter = 0; iter < 3; ++iter) {
    /// TODO：需要增加迭代停止条件(误差小于某个值，或者误差不再继续减小)
    // 根据两个车道线检测帧之间的转换关系，定义正向的平移/旋转矩阵
    common::Matrix<Scalar, 3, 3> mat_convert;
    common::Matrix<Scalar, 2, 1> rotate_center;
    common::Matrix<Scalar, 2, 1> point_conv;
    mat_convert.SetIdentity();
    rotate_center.SetZeros();
    common::Rotate_2D<Scalar>(rotate_center,
                              filtered_delta_pos[2], &mat_convert);
    common::Translate_2D(filtered_delta_pos[0],
                         filtered_delta_pos[1], &mat_convert);

    ad_msg::LaneInfoCameraList& prev_lane_mark_list =
        raw_lane_list_[raw_lane_list_idx_];
    temp_data_.sample_points[0].Clear();
    temp_data_.sample_points[1].Clear();

    Scalar sq_residual_error = 0.0F;
    // 遍历摄像头识别的车道线列表
    Int32_t lane_mark_num = lane_mark_list.lane_mark_num;
    for (Int32_t i = 0; i < lane_mark_num; ++i) {
      const ad_msg::LaneMarkCamera& lane_mark =
          lane_mark_list.lane_marks[i];

      if (lane_mark.quality < 2) {
        // 识别效果差，丢弃此车道线
        continue;
      }
      if ((1 != lane_mark.id) && (-1 != lane_mark.id)) {
        // 目前只有当前车道两侧的车道线识别效果好，其它更远处的相邻车道识别效果差
        continue;
      }

      // 识别的车道线的有效距离
      Scalar lane_mark_len =
          lane_mark.view_range_end - lane_mark.view_range_start;
      if (lane_mark_len < 1.0F) {
        // 识别的车道线的有效距离太短
        continue;
      }

      // 需要采样的车道线的点的数量
      Int32_t sample_size = lane_mark_len / sample_step_len;
      if (sample_size < 2) {
        sample_size = 2;
      } if (sample_size > max_sample_num) {
        sample_size = max_sample_num;
      }

#if ENABLE_LANE_MARK_CAMERA_FILTER_TRACE
      std::cout << "lane_mark_len = " << lane_mark_len
                << ", sample_size = " << sample_size
                << std::endl;
#endif
      // 生成车道线的参数化方程
      common::CubicPolynomialCurve1d<Scalar> curve;
      curve.SetCoefficient(lane_mark.c0, lane_mark.c1,
                           lane_mark.c2, lane_mark.c3);

      for (Int32_t j = 0; j < sample_size; ++j) {
        point_conv(0) = j * sample_step_len;
        if (point_conv(0) > lane_mark_len) {
          // 采样点不能超过有效距离
          point_conv(0) = lane_mark_len;
        }
        // 将新的车道线的曲线点转换到之前帧的坐标系下
        point_conv(0) += lane_mark.view_range_start;
        point_conv(1) = curve.Evaluate(0, point_conv(0));
        common::TransformVert_2D(mat_camera_calibration_, &point_conv);
        common::TransformVert_2D(mat_convert, &point_conv);
        common::Vec2d new_point(point_conv(0), point_conv(1));

        // 在之前帧的车道线中找到匹配点（点到线的距离最近的点，投影点）
        Int32_t obj_index = 0;
        Scalar min_sq_distance = 0.0F;
        temp_data_.tree_nodes_stack.Clear();
        const LaneLineSegObj* obj = raw_lane_mark_seg_kdtree_.FindNearest(
              temp_data_.tree_nodes_stack, new_point,
              CalcSquareDistToPt(new_point, prev_lane_mark_list),
              &obj_index, &min_sq_distance);
        if (Nullptr_t == obj) {
          LOG_ERR << "Failed to find nearest point from lane line list.";
        } else {
          if ((obj->lane_line_index < 0) ||
              (obj->lane_line_index >= prev_lane_mark_list.boundary_line_num)) {
            LOG_ERR << "Detected invalid lane index " << obj->lane_line_index;
          } else {
            const ad_msg::LaneBoundaryLineCamera& old_lane_line =
                prev_lane_mark_list.boundary_lines[obj->lane_line_index];
            if ((obj->point_index < 0) ||
                (obj->point_index >= (old_lane_line.curve_point_num-1))) {
              LOG_ERR << "Detected invalid point index " << obj->point_index;
            } else {
              const ad_msg::LaneBoundaryLineCamera::CurvePoint& point_ref1 =
                  old_lane_line.curve[obj->point_index];
              const ad_msg::LaneBoundaryLineCamera::CurvePoint& point_ref2 =
                  old_lane_line.curve[obj->point_index+1];
              common::Vec2d point1(point_ref1.x, point_ref1.y);
              common::Vec2d point2(point_ref2.x, point_ref2.y);
              common::Vec2d nearest_point;
              Scalar t = 0.0F;
              common::ClosestPtPointToLine_2D(point1, point2, new_point,
                                              &(nearest_point), &t);
              Scalar squared_dist =
                  new_point.DistanceSquareTo(nearest_point);

              if (squared_dist < 0.5F*0.5F &&
                  ((-1e-6F < t) && (t < 1.0F+1e-6F))) {
                // 只有在之前帧的车道线范围内的点，并且距离小于一定值
                //（过滤掉异常点）才可以算作匹配点

                // 计算残差
                sq_residual_error += squared_dist;

                // 保存匹配点到列表中
                common::Vec2d* src_point =
                    temp_data_.sample_points[0].Allocate();
                if (Nullptr_t == src_point) {
                  break;
                }
                common::Vec2d* dst_point =
                    temp_data_.sample_points[1].Allocate();
                if (Nullptr_t == dst_point) {
                  temp_data_.sample_points[0].PopBack();
                  break;
                }
                *src_point = nearest_point;
                *dst_point = new_point;
              }
            }
          }
        }
      }
    }

    Int32_t src_points_size = temp_data_.sample_points[0].Size();
    Int32_t dst_points_size = temp_data_.sample_points[1].Size();

    if (src_points_size != dst_points_size) {
      register_ret = false;
      break;
    }
    if (src_points_size < 2) {
      // 匹配点过少
      register_ret = false;
      filter_quality_ = FILTER_QUALITY_INVALID;
      break;
    }

    if (src_points_size > max_sample_num) {
      filter_quality_ = FILTER_QUALITY_GOOD;
    } else if (src_points_size < 4) {
      filter_quality_ = FILTER_QUALITY_BAD;
    } else {
      filter_quality_ = FILTER_QUALITY_NOT_SO_GOOD;
    }

#if ENABLE_LANE_MARK_CAMERA_FILTER_TRACE
    std::cout << "iter = " << iter << std::endl;
    std::cout << "sq_residual_error = " << sq_residual_error << std::endl;
    std::cout << "src_points_size = "
              << temp_data_.sample_points_1.Size()
              << ", dst_points_size = "
              << temp_data_.sample_points_2.Size()
              << std::endl;
#endif

    // 计算两组匹配点之间的转换关系
    bool ret = phoenix::common::CalcTfBtwCorrespondedPtsSetsQuaternionsBase_2D(
          src_points_size, get_src_points, get_dst_points, &mat_rot, &vec_tran);
    if (false == ret) {
      LOG_ERR << "Failed to calculate the transformation "
                 "matrix between the lane lines.";
      register_ret = false;
      break;
    }

    // 更新位置变换关系
    Scalar rot_angle = common::com_atan2(-mat_rot(0, 1), mat_rot(1, 1));
    filtered_delta_pos[0] -= vec_tran(0);
    filtered_delta_pos[1] -= vec_tran(1);
    filtered_delta_pos[2] -= rot_angle;

#if ENABLE_LANE_MARK_CAMERA_FILTER_TRACE
    std::cout << "mat_rot=\n" << mat_rot << std::endl;
    std::cout << "vec_tran= " << vec_tran(0) << ", " << vec_tran(1)
              << ", " << vec_tran(2) << std::endl;
    std::cout << "rotation_angle = " << common::com_rad2deg(rot_angle)
              << std::endl;
#endif
  }

#if ENABLE_LANE_MARK_CAMERA_FILTER_TRACE
  std::cout << "### RegisterFramesOfLaneLines (End) ###" << std::endl;
#endif

  return (register_ret);
}

/**
 * @date       2020.11.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/11/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void LaneMarkCameraFilter::BuildLaneLineSegKDTree(
    const ad_msg::LaneInfoCameraList& lane_info_list) {
  phoenix::common::AABBoxKDTreeParams params;
  params.max_leaf_dimension = 5.0f;  // meters.
  params.max_leaf_size = 16;

  lane_boundary_line_seg_kdtree_.SetKDTreeParams(params);
  lane_boundary_line_seg_kdtree_.Clear();
  temp_data_.tree_nodes_stack.Clear();

  LaneLineSegObj obj;
  for (Int32_t lane_index = 0;
       lane_index < lane_info_list.boundary_line_num; ++lane_index) {
    const ad_msg::LaneBoundaryLineCamera&
        lane_line = lane_info_list.boundary_lines[lane_index];
    /* k003 longjiaoy 2022-11-28 (start) */
    // 目前只有当前车道两侧的车道线识别效果好，其它更远处的相邻车道识别效果差,
    // 所以对于相邻的车道, 其边线不跟之前保存的车道边线进行比较
    if ((-1 == lane_line.id) || (1 == lane_line.id)) {
      for (Int32_t point_index = 0;
           point_index < (lane_line.curve_point_num-1); ++point_index) {
        if (lane_line.curve[point_index].fake ||
            lane_line.curve[point_index + 1].fake) {
          break;
        }
        obj.lane_id = lane_line.id;
        obj.lane_line_index = lane_index;
        obj.point_index = point_index;
        phoenix::common::AABBox2d box(lane_line.curve[point_index].x,
                                      lane_line.curve[point_index].y,
                                      lane_line.curve[point_index + 1].x,
            lane_line.curve[point_index + 1].y);
        lane_boundary_line_seg_kdtree_.AddObject(obj, box);
      }
    }
    /* k003 longjiaoy 2022-11-28 (end) */
  }

  lane_boundary_line_seg_kdtree_.Partition(temp_data_.tree_nodes_stack);
}

/**
 * @date       2020.11.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/11/08  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
Int32_t LaneMarkCameraFilter::FindLaneLineById(
    Int32_t id, const ad_msg::LaneInfoCameraList& lane_info_list) const {
  for (Int32_t i = 0; i < lane_info_list.boundary_line_num; ++i) {
    if (id == lane_info_list.boundary_lines[i].id) {
      return (i);
    }
  }

  return (-1);
}

/**
 * @date       2020.11.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/11/08  <td>1.0      <td>pengc     <td>First edition
 * <tr><td>2020/12/28  <td>1.0      <td>sip       <td>增加左右相邻车道的解析
 * </table>
 */
void LaneMarkCameraFilter::ConstructLaneCenterLines(
    ad_msg::LaneInfoCameraList& lane_info_list) {
  lane_info_list.center_line_num = 0;
  lane_info_list.center_lines[0].Clear();
  lane_info_list.center_lines[1].Clear();
  lane_info_list.center_lines[2].Clear();

  Int32_t curr_central_line_idx = -1;
  Int32_t left_central_line_idx = -1;
  Int32_t right_central_line_idx = -1;

  Int32_t left_line_index = FindLaneLineById(1, lane_info_list);
  Int32_t right_line_index = FindLaneLineById(-1, lane_info_list);
  /* k003 longjiaoy 2022-11-28 (start) */
  Int32_t left_2_line_index = FindLaneLineById(2, lane_info_list);
  Int32_t right_2_line_index = FindLaneLineById(-2, lane_info_list);
  /* k003 longjiaoy 2022-11-28 (end) */

  common::Path& left_path = temp_data_.path[0];
  common::Path& right_path = temp_data_.path[1];
  common::Path& left_2_path = temp_data_.path[2];
  common::Path& right_2_path = temp_data_.path[3];
  Int32_t left_path_points_num = 0;
  Int32_t right_path_points_num = 0;
  Int32_t dst_point_index = 0;
  ad_msg::LaneCenterLineCamera& curr_central_line =
      lane_info_list.center_lines[0];

  const InternalLaneInfo::BoundaryLine& left_boundary =
      internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_LEFT);
  const InternalLaneInfo::BoundaryLine& right_boundary =
      internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_RIGHT);
  const InternalLaneInfo::BoundaryLine& left_2_boundary =
      internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_LEFT_2);
  const InternalLaneInfo::BoundaryLine& right_2_boundary =
      internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_RIGHT_2);

  const InternalLaneInfo::CenterLine& curr_center =
      internal_lane_info_.center_line(InternalLaneInfo::CENTER_CURR);
  const InternalLaneInfo::CenterLine& left_center =
      internal_lane_info_.center_line(InternalLaneInfo::CENTER_LEFT);
  const InternalLaneInfo::CenterLine& right_center =
      internal_lane_info_.center_line(InternalLaneInfo::CENTER_RIGHT);

  // 构造左车道线曲线
  if (left_line_index >= 0) {
    common::StaticVector<common::Vec2d, MAX_LANE_LINE_SAMPLE_POINTS_NUM>&
        left_line_points = temp_data_.sample_points[0];
    left_line_points.Clear();
    const ad_msg::LaneBoundaryLineCamera& left_line =
        lane_info_list.boundary_lines[left_line_index];
    for (Int32_t i = 0; i < left_line.curve_point_num; ++i) {
      common::Vec2d* p = left_line_points.Allocate();
      if (Nullptr_t == p) {
        LOG_ERR << "Can't add points, storage is full.";
        break;
      }
      p->set_x(left_line.curve[i].x);
      p->set_y(left_line.curve[i].y);
    }
    left_path.Construct(left_line_points);
    left_path_points_num = left_path.points().Size();

    internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_LEFT).curve = left_path;
  }
  // 构造右车道线曲线
  if (right_line_index >= 0) {
    common::StaticVector<common::Vec2d, MAX_LANE_LINE_SAMPLE_POINTS_NUM>&
        right_line_points = temp_data_.sample_points[1];
    right_line_points.Clear();
    const ad_msg::LaneBoundaryLineCamera& right_line =
        lane_info_list.boundary_lines[right_line_index];
    for (Int32_t i = 0; i < right_line.curve_point_num; ++i) {
      common::Vec2d* p = right_line_points.Allocate();
      if (Nullptr_t == p) {
        LOG_ERR << "Can't add points, storage is full.";
        break;
      }
      p->set_x(right_line.curve[i].x);
      p->set_y(right_line.curve[i].y);
    }
    right_path.Construct(right_line_points);
    right_path_points_num = right_path.points().Size();

    internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_RIGHT).curve = right_path;
  }

  // 构造左2车道线曲线
  if (left_2_line_index >= 0) {
    common::StaticVector<common::Vec2d, MAX_LANE_LINE_SAMPLE_POINTS_NUM>&
        left_2_line_points = temp_data_.sample_points[0];
    left_2_line_points.Clear();
    const ad_msg::LaneBoundaryLineCamera& left_2_line =
        lane_info_list.boundary_lines[left_2_line_index];
    for (Int32_t i = 0; i < left_2_line.curve_point_num; ++i) {
      common::Vec2d* p = left_2_line_points.Allocate();
      if (Nullptr_t == p) {
        LOG_ERR << "Can't add points, storage is full.";
        break;
      }
      p->set_x(left_2_line.curve[i].x);
      p->set_y(left_2_line.curve[i].y);
    }
    left_2_path.Construct(left_2_line_points);

    internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_LEFT_2).curve = left_2_path;
  }
  // 构造右2车道线曲线
  if (right_2_line_index >= 0) {
    common::StaticVector<common::Vec2d, MAX_LANE_LINE_SAMPLE_POINTS_NUM>&
        right_2_line_points = temp_data_.sample_points[1];
    right_2_line_points.Clear();
    const ad_msg::LaneBoundaryLineCamera& right_2_line =
        lane_info_list.boundary_lines[right_2_line_index];
    for (Int32_t i = 0; i < right_2_line.curve_point_num; ++i) {
      common::Vec2d* p = right_2_line_points.Allocate();
      if (Nullptr_t == p) {
        LOG_ERR << "Can't add points, storage is full.";
        break;
      }
      p->set_x(right_2_line.curve[i].x);
      p->set_y(right_2_line.curve[i].y);
    }
    right_2_path.Construct(right_2_line_points);

    internal_lane_info_.boundary_line(InternalLaneInfo::BOUNDARY_RIGHT_2).curve = right_2_path;
  }

  curr_central_line.forward_len = common::Max(
        left_boundary.forward_len, right_boundary.forward_len);
  if (left_boundary.valid && right_boundary.valid) {
    curr_central_line.quality = 3;
  } else if (!left_boundary.valid && !right_boundary.valid) {
    curr_central_line.quality = 0;
  } else {
    curr_central_line.quality = 2;
  }
  curr_central_line.age = 0;

  /* k003 longjiaoy 2022-11-28 (start) */
  curr_central_line.left_boundary_index = left_line_index;
  curr_central_line.right_boundary_index = right_line_index;
  /* k003 longjiaoy 2022-11-28 (end) */

  // 当前车道左右两侧车道线都存在
  Scalar left_width = 3.3F*0.5F;
  Scalar right_width = left_width;
  if (curr_center.valid) {
    left_width = curr_center.left_width;
    right_width = curr_center.right_width;
  }
  if ((left_line_index >= 0) && (right_line_index >= 0)) {
    if ((left_path_points_num < 2) || (right_path_points_num < 2)) {
      LOG_ERR << "Invalid left or right path.";
    } else {
      // 补齐左右车道线长短不一致
      ConstructCurrentLaneCenterLines(
            left_path, right_path, &dst_point_index, &curr_central_line);
      lane_info_list.center_line_num = 1;
      curr_central_line_idx = 0;
    }
  } else if ((left_line_index >= 0) && (right_line_index < 0)) {
    //只有左边有车道线
    ConstructCenterLinesFromPath(
          -left_width, left_path, &dst_point_index, &curr_central_line);
    lane_info_list.center_line_num = 1;
    curr_central_line_idx = 0;
  } else if ((left_line_index < 0) && (right_line_index >= 0)) {
    //只有右边有车道线
    ConstructCenterLinesFromPath(
          right_width, right_path, &dst_point_index, &curr_central_line);
    lane_info_list.center_line_num = 1;
    curr_central_line_idx = 0;
  } else {
    //没有车道
    curr_central_line.Clear();
    lane_info_list.center_line_num = 0;
    curr_central_line_idx = -1;
  }

#if 0
  printf("Camera center line (In Camera Filter)=\n");
  for (Int32_t i = 0; i < curr_central_line.curve_point_num; ++i) {
    printf("p[%d]: (%0.1f, %0.1f)\n", i,
           curr_central_line.curve[i].x,
           curr_central_line.curve[i].y);
  }
#endif

  lane_info_list.quality = curr_central_line.quality;
  lane_info_list.forward_len = curr_central_line.forward_len;
  lane_info_list.left_width = left_width;
  lane_info_list.right_width = right_width;

  if (curr_central_line_idx >= 0) {
    //处理相邻车道
    if (left_2_boundary.valid) {
      //左边存在车道
      left_width = 3.3F*0.5F;
      right_width = left_width;
      if (left_center.valid) {
        left_width = left_center.left_width;
        right_width = left_center.right_width;
      }

      left_central_line_idx = lane_info_list.center_line_num;
      ad_msg::LaneCenterLineCamera& left_central_line =
          lane_info_list.center_lines[left_central_line_idx];
      left_central_line.id = 1;
      Int32_t point_index = 0;
      ConstructCenterLinesFromPath(
            right_width, left_path, &point_index, &left_central_line);

      left_central_line.forward_len = left_2_boundary.forward_len;
      if (left_2_boundary.age > 8) {
        if (left_2_boundary.forward_len > 50.0F) {
          left_central_line.quality = 3;
        } else {
          left_central_line.quality = 2;
        }
      } else {
        left_central_line.quality = 1;
      }
      left_central_line.age = 0;

      /* k003 longjiaoy 2022-11-28 (start) */
      left_central_line.left_boundary_index = left_2_line_index;
      left_central_line.right_boundary_index = left_line_index;
      /* k003 longjiaoy 2022-11-28 (end) */

      lane_info_list.center_line_num++;
    }

    if (right_2_boundary.valid) {
      //右边存在车道
      left_width = 3.3F*0.5F;
      right_width = left_width;
      if (right_center.valid) {
        left_width = right_center.left_width;
        right_width = right_center.right_width;
      }

      right_central_line_idx = lane_info_list.center_line_num;
      ad_msg::LaneCenterLineCamera& right_central_line =
          lane_info_list.center_lines[right_central_line_idx];
      right_central_line.id = -1;
      Int32_t point_index = 0;
      ConstructCenterLinesFromPath(
            -left_width, right_path, &point_index, &right_central_line);

      right_central_line.forward_len = right_2_boundary.forward_len;
      if (right_2_boundary.age > 8) {
        if (right_2_boundary.forward_len > 50.0F) {
          right_central_line.quality = 3;
        } else {
          right_central_line.quality = 2;
        }
      } else {
        right_central_line.quality = 1;
      }
      right_central_line.age = 0;

      /* k003 longjiaoy 2022-11-28 (start) */
      right_central_line.left_boundary_index = right_line_index;
      right_central_line.right_boundary_index = right_2_line_index;
      /* k003 longjiaoy 2022-11-28 (end) */

      lane_info_list.center_line_num++;
    }
  }
}


/**
 * @date       2020.12.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/12/28  <td>1.0      <td>sip       <td>First edition
 * </table>
 */
void LaneMarkCameraFilter::ConstructCurrentLaneCenterLines(
    const common::Path& left_path, const common::Path& right_path,
    Int32_t* point_index, ad_msg::LaneCenterLineCamera* central_line){
  const InternalLaneInfo::CenterLine& center_line =
      internal_lane_info_.center_line(InternalLaneInfo::CENTER_CURR);

  // 车辆在右边线的投影点
  common::PathPoint veh_on_right_path_projectiont;
  // 左边线终点在右边线的投影点
  common::PathPoint left_end_on_right_projectiont;
  // 车辆在左边线的投影点
  common::PathPoint veh_on_left_path_projectiont;
  // 右边线终点在左边线的投影点
  common::PathPoint right_end_on_left_projectiont;

  Int32_t proj_idx_veh_on_right_path = 0;
  Int32_t proj_idx_left_end_on_right_path = 0;
  Int32_t proj_idx_veh_on_left_path = 0;
  Int32_t proj_idx_right_end_on_left_path = 0;
  common::Vec2d veh_point(0.0F,0.0F);

  right_path.FindProjection(
        veh_point,
        &proj_idx_veh_on_right_path, &veh_on_right_path_projectiont);
  right_path.FindProjection(
        left_path.points().Back(),
        &proj_idx_left_end_on_right_path, &left_end_on_right_projectiont);


  left_path.FindProjection(
        veh_point,
        &proj_idx_veh_on_left_path, &veh_on_left_path_projectiont);
  left_path.FindProjection(
        right_path.points().Back(),
        &proj_idx_right_end_on_left_path, &right_end_on_left_projectiont);

  Scalar half_width = 0.5F * (common::com_abs(veh_on_left_path_projectiont.l) +
                              common::com_abs(veh_on_right_path_projectiont.l));
  Scalar left_width = half_width;
  Scalar right_width = half_width;
  if (center_line.valid) {
    left_width = center_line.left_width;
    right_width = center_line.right_width;
  }

  Int8_t construct_direction = 1;
  if (veh_on_left_path_projectiont.s > 0.5F) {
    construct_direction = 1;
  } else {
    if (veh_on_right_path_projectiont.s > 0.5F) {
      construct_direction = 2;
    }
  }

  if (1 == construct_direction) {
    CalaCenterLinesPoints(
          0, left_path, -left_width,
          left_width, right_width, point_index, central_line);
    if (left_end_on_right_projectiont.s < right_path.total_length()){
      // 右边终点比左边长
      CalaCenterLinesPoints(
            proj_idx_left_end_on_right_path+1, right_path, right_width,
            left_width, right_width, point_index, central_line);
    }
  } else {
    CalaCenterLinesPoints(
          0, right_path, right_width,
          left_width, right_width, point_index, central_line);
    if (left_end_on_right_projectiont.s > right_path.total_length()) {
      //左边终点比右边长
      CalaCenterLinesPoints(
            proj_idx_right_end_on_left_path+1, left_path, -left_width,
            left_width, right_width, point_index, central_line);
    }
  }
  central_line->curve_point_num = *point_index;
}

/**
 * @date       2020.12.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/12/28  <td>1.0      <td>sip       <td>First edition
 * </table>
 */
void LaneMarkCameraFilter::ConstructCenterLinesFromPath(
    const Float32_t sample_l, const common::Path &path,
    Int32_t* point_index, ad_msg::LaneCenterLineCamera* central_line){
  if (path.points().Size() < 2) {
    LOG_ERR << "The number of path points is too small.";
    return;
  }

  common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>&
      sample_points = temp_data_.sample_path_points[0];
  sample_points.Clear();
  path.UniformlySamplePathForward(
        0.0F, path.points().Size(), path.total_length() / path.points().Size(),
        sample_l, &sample_points);

  for (Int32_t i = 0; i < sample_points.Size(); ++i) {
    central_line->curve[*point_index].x = sample_points[i].point.x();
    central_line->curve[*point_index].y = sample_points[i].point.y();
    central_line->curve[*point_index].left_width =
        common::com_abs(sample_points[i].l);
    central_line->curve[*point_index].right_width =
        common::com_abs(sample_points[i].l);
    (*point_index)++;
  }
  central_line->curve_point_num = *point_index;
}


/**
 * @date       2020.12.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/12/28  <td>1.0      <td>sip       <td>根据路径的长短，计算中心点
 * </table>
 */
void LaneMarkCameraFilter::CalaCenterLinesPoints(
    Int32_t start_index, const common::Path& long_path, const Float32_t sample_l,
    const Float32_t left_width, const Float32_t right_width,
    Int32_t* point_index, ad_msg::LaneCenterLineCamera* central_line) {

  common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>&
      sample_points = temp_data_.sample_path_points[0];
  sample_points.Clear();
  common::PathPoint path_point;
  if (long_path.points().Size() > start_index){
    const common::Vec2d &temp_point = long_path.points()[start_index];
    long_path.FindNearest(temp_point, &path_point);
    Int32_t sample_num = long_path.points().Size()-start_index;
    if(sample_num<3){
      return;
    }
    long_path.UniformlySamplePathForward(
          path_point.s, sample_num,
          long_path.total_length() / long_path.points().Size(),
          sample_l, &sample_points);
    for (Int32_t i = 0; i < sample_points.Size(); ++i) {
      central_line->curve[*point_index].x = sample_points[i].point.x();
      central_line->curve[*point_index].y = sample_points[i].point.y();
#if 0
      central_line->curve[*point_index].left_width =
          common::com_abs(sample_points[i].l);
      central_line->curve[*point_index].right_width =
          common::com_abs(sample_points[i].l);
#else
      central_line->curve[*point_index].left_width = left_width;
      central_line->curve[*point_index].right_width = right_width;
#endif
      (*point_index)++;
    }
    central_line->curve_point_num = *point_index;
  }
}


}  // namespace pos_filter
}  // namespace phoenix
