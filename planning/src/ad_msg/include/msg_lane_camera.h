/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       msg_lane_camera.h
 * @brief      相机识别的车道线消息定义
 * @details    定义了相机识别的车道线消息类型
 *
 * @author     pengc
 * @date       2021.01.22
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2021/01/22  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_AD_MSG_MSG_LANE_CAMERA_H_
#define PHOENIX_AD_MSG_MSG_LANE_CAMERA_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "msg_common.h"


namespace phoenix {
namespace ad_msg {


/**
 * @struct LaneMarkCamera
 * @brief 车道线信息（摄像头识别的）
 */
struct LaneMarkCamera {
  /// 车道线的类型定义
  enum LaneMarkType {
    LANE_MARK_TYPE_INVALID = 0,
    LANE_MARK_TYPE_UNKNOWN = 1,
    LANE_MARK_TYPE_DASHED = 2,
    LANE_MARK_TYPE_SOLID = 3,
    LANE_MARK_TYPE_DOUBLE_LANE_MARK = 4,
    LANE_MARK_TYPE_BOTTS_DOTS = 5,
    LANE_MARK_TYPE_ROAD_EDGE = 6
  };

  /// 车道编号，左边车道线为正值，右边车道线为负数;
  /// (数值的绝对值按照距离当前车道的远近依次增加，
  /// 第一条左边的车道线为1，第一条右边的车道线为-1)
  Int32_t id;
  /// 车道线类型
  LaneMarkType lane_mark_type;
  /// 车道线识别的质量(0,1 – low quality.The lane measurements are not valid
  /// in low quality. The system will not give an LDW in that situation.
  /// 2,3 – high quality)
  Int32_t quality;
  /// 车道线的纵向有效范围字段中的值是否有效
  bool view_range_valid;
  /// 车道线的宽度
  Float32_t mark_width;
  /// 车道线的纵向有效范围的起始位置
  Float32_t view_range_start;
  /// 车道线的纵向有效范围的结束位置
  Float32_t view_range_end;
  /// 曲线参数c0
  Float64_t c0;
  /// 曲线参数c1
  Float64_t c1;
  /// 曲线参数c2
  Float64_t c2;
  /// 曲线参数c3
  Float64_t c3;
  /// 备注：
  /// 车道线使用三阶多项式曲线定义 y = c0 + c1 * x + c2 * x^2 + c3 * x^3
  /// 使用右手迪卡尔坐标系，x为车身纵轴方向，向前为正方向，y为车身横轴方向，向左为正方向;
  /// 角度以x轴正方向为0度，逆时针方向为正。

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    id = 0;
    lane_mark_type = LANE_MARK_TYPE_INVALID;
    quality = 0;
    view_range_valid = false;
    mark_width = 0.0F;
    view_range_start = 0.0F;
    view_range_end = 0.0F;
    c0 = 0.0F;
    c1 = 0.0F;
    c2 = 0.0F;
    c3 = 0.0F;
  }

  /**
   * @brief 构造函数
   */
  LaneMarkCamera() {
    Clear();
  }
};

/**
 * @struct LaneInfoCameraList
 * @brief 感知识别的车道线列表
 */
struct LaneMarkCameraList {
  /**
   * @enum MAX_LANE_MARK_NUM
   * @brief 列表中最大车道线的数量
   */
  enum { MAX_LANE_MARK_NUM = 4 };

  /// 消息头
  MsgHead msg_head;
  /// 车道线的数量
  Int8_t lane_mark_num;
  /// 感知识别的车道线列表
  LaneMarkCamera lane_marks[MAX_LANE_MARK_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();
    lane_mark_num = 0;
    for (Int32_t i = 0; i < MAX_LANE_MARK_NUM; ++i) {
      lane_marks[i].Clear();
    }
  }

  /**
   * @brief 构造函数
   */
  LaneMarkCameraList() {
    Clear();
  }
};

/**
 * @struct LaneBoundaryLineCamera
 * @brief 车道信息（摄像头识别的）
 */
struct LaneBoundaryLineCamera {
  /**
   * @enum MAX_CURVE_POINT_NUM
   * @brief 车道线中点的最大数量
   */
  enum { MAX_CURVE_POINT_NUM = 500 };

  /// 车道编号，左边车道线为正值，右边车道线为负数;
  /// (数值的绝对值按照距离当前车道的远近依次增加，
  /// 第一条左边的车道线为1，第一条右边的车道线为-1)
  Int32_t id;
  /// 边线类型
  Int32_t type;
  /// 车道线中点的数量
  Int32_t curve_point_num;
  /// 车道线
  struct CurvePoint {
    bool fake;
    /// 形点坐标x
    Float32_t x;
    /// 形点坐标y
    Float32_t y;
  } curve[MAX_CURVE_POINT_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    id = 0;
    type = ad_msg::LaneMarkCamera::LANE_MARK_TYPE_INVALID;
    curve_point_num = 0;
    common::com_memset(curve, 0, sizeof(curve));
  }

  /**
   * @brief 构造函数
   */
  LaneBoundaryLineCamera() {
    Clear();
  }
};

/**
 * @struct LaneCenterLineCamera
 * @brief 车道中心线信息（摄像头识别的）
 */
struct LaneCenterLineCamera {
  /**
   * @enum MAX_CURVE_POINT_NUM
   * @brief 车道线中点的最大数量
   */
  enum { MAX_CURVE_POINT_NUM = 500 };

  /// 车道线识别的质量, 3 - very good, 2 - not good, 1 - bad, 0 - invalid
  Int8_t quality;
  /// 历史数据存在的次数
  Int32_t age;
  /// 从车辆当前位置开始，车道线的前向长度
  Float32_t forward_len;
  /// 车道中心线编号，左边车道中心线为正值，右边车道中心线为负数;
  /// (数值的绝对值按照距离当前车道的远近依次增加，
  /// 当前车道中心线为0,第一条左边的车道中心线为1，第一条右边的车道中心线为-1)
  Int32_t id;
  /// 车道左边线的索引
  Int32_t left_boundary_index;
  /// 车道右边线的索引
  Int32_t right_boundary_index;
  /// 车道中心线点的数量
  Int32_t curve_point_num;
  /// 车道中心线
  struct CurvePoint {
    /// 形点坐标x
    Float32_t x;
    /// 形点坐标y
    Float32_t y;
    /// 车道中心线到左边线的宽度
    Float32_t left_width;
    /// 车道中心线到右边线的宽度
    Float32_t right_width;
  } curve[MAX_CURVE_POINT_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    quality = 0;
    age = 0;
    forward_len = 0.0F;
    id = 0;
    left_boundary_index = -1;
    right_boundary_index = -1;
    curve_point_num = 0;
    common::com_memset(curve, 0, sizeof(curve));
  }

  /**
   * @brief 构造函数
   */
  LaneCenterLineCamera() {
    Clear();
  }
};


/**
 * @struct LaneInfoCameraList
 * @brief Camera识别的车道线列表
 */
struct LaneInfoCameraList {
  /**
   * @enum MAX_LANE_MARK_NUM
   * @brief 列表中最大车道线的数量
   */
  enum { MAX_LANE_BOUNDARY_LINE_NUM = 4 };
  /**
   * @enum MAX_LANE_CENTRAL_LINE_NUM
   * @brief 列表中最大车道中心线的数量
   */
  enum { MAX_LANE_CENTER_LINE_NUM = 3 };

  /// 消息头
  MsgHead msg_head;
  /// 车道线识别的质量, 3 - very good, 2 - not good, 1 - bad, 0 - invalid
  Int8_t quality;
  /// 从车辆当前位置开始，车道线的前向长度
  Float32_t forward_len;
  /// 车辆当前位置左边车道宽度
  Float32_t left_width;
  /// 车辆当前位置右边车道宽度
  Float32_t right_width;
  /// 车道线的数量
  Int32_t boundary_line_num;
  /// 感知识别的车道线列表
  LaneBoundaryLineCamera boundary_lines[MAX_LANE_BOUNDARY_LINE_NUM];
  /// 车道中心线的数量
  Int32_t center_line_num;
  /// 车道中心线列表
  LaneCenterLineCamera center_lines[MAX_LANE_CENTER_LINE_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear() {
    msg_head.Clear();

    quality = 0;
    forward_len = 0.0F;
    left_width = 0.0F;
    right_width = 0.0F;

    boundary_line_num = 0;
    for (Int32_t i = 0; i < MAX_LANE_BOUNDARY_LINE_NUM; ++i) {
      boundary_lines[i].Clear();
    }

    center_line_num = 0;
    for (Int32_t i = 0; i < MAX_LANE_CENTER_LINE_NUM; ++i) {
      center_lines[i].Clear();
    }
  }

  /**
   * @brief 构造函数
   */
  LaneInfoCameraList() {
    Clear();
  }

  /**
   * @brief 根据ID查找车道中心线
   * @param[in] id 车道中心线ID
   * @return 车道中心的索引 (-1 ~ 没有找到)
   */
  Int32_t FindCenterLineById(const Int32_t id) const {
    for (Int32_t i = 0; i < center_line_num; ++i) {
      if (id == center_lines[i].id) {
        return (i);
      }
    }

    return (-1);
  }
};


} // namespace ad_msg
} // namespace phoenix


#endif // PHOENIX_AD_MSG_MSG_LANE_CAMERA_H_


