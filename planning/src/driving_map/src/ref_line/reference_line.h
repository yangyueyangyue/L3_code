/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       reference_line.h
 * @brief      参考线
 * @details    与单条参考线相关的一些变量和函数
 *
 * @author     boc
 * @date       2020.06.13
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/13  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/
#ifndef PHOENIX_DRIV_MAP_REFERENCE_LINE_H_
#define PHOENIX_DRIV_MAP_REFERENCE_LINE_H_

#include "container/static_vector.h"
#include "curve/path.h"
#include "driving_map.h"


namespace phoenix {
namespace driv_map {


/**
 * @class ReferenceLine
 * @brief 参考线。参考线指的是当前车辆附近的可以行驶的路径，由一些前后相连的车道组成。
 */
class ReferenceLine {
public:
  /**
   * @brief 构造函数。
   */
  ReferenceLine();
  /**
   * @brief 析构函数。
   */
  ~ReferenceLine();

  /**
   * @brief 清空内部成员变量。
   */
  void Clear();

  /**
   * @brief 获取参考线的左右相邻标记。
   * @return 参考线的左右相邻标记
   */
  Int32_t neighbor_flag() const {
    return (neighbor_flag_);
  }

  /**
   * @brief 设置参考线的左右相邻标记。
   */
  void set_neighbor_flag(Int32_t flag) {
    neighbor_flag_ = flag;
  }

  /**
   * @brief 获取车道的质量。
   */
  inline Int32_t quality() const { return quality_; }
  /**
   * @brief 设置车道的质量。
   * @param[in] q    车道的质量。
   */
  inline void set_quality(Int32_t q) { quality_ = q; }

  /**
   * @brief 获取参考线的车道段列表。
   * @return 参考线的车道段列表
   */
  const common::StaticVector<LaneSegment, MAX_LANE_SEGMENT_NUM>&
      lane_segments() const {
    return (lane_segments_);
  }

  /**
   * @brief 获取参考线的车道段列表。
   * @return 参考线的车道段列表
   */
  common::StaticVector<LaneSegment, MAX_LANE_SEGMENT_NUM>&
      lane_segments() {
    return (lane_segments_);
  }

  /**
   * @brief 获取参考线的路径。
   * @return 参考线的路径
   */
  const common::Path& curve() const {
    return (curve_);
  }

  /**
   * @brief 获取参考线的路径。
   * @return 参考线的路径
   */
  common::Path& curve() {
    return (curve_);
  }

  /**
   * @brief 获取参考线的平滑后的路径。
   * @return 参考线的平滑后的路径
   */
  const common::Path& smooth_curve() const {
    return (smooth_curve_);
  }

  /**
   * @brief 获取参考线的平滑后的路径。
   * @return 参考线的平滑后的路径
   */
  common::Path& smooth_curve() {
    return (smooth_curve_);
  }

  /**
   * @brief 获取参考线的曲线段信息。
   * @return 参考线的曲线段信息
   */
  const common::StaticVector<common::Path::CurveSegment,
      common::Path::kMaxCurveSegmentNum>& curve_curvature_info() const {
    return (curve_curvature_info_);
  }

  /**
   * @brief 获取参考线的曲线段信息。
   * @return 参考线的曲线段信息
   */
  common::StaticVector<common::Path::CurveSegment,
      common::Path::kMaxCurveSegmentNum>& curve_curvature_info() {
    return (curve_curvature_info_);
  }

  /**
   * @brief 获取参考线的平滑后的曲线段信息。
   * @return 参考线的平滑后的曲线段信息
   */
  const common::StaticVector<common::Path::CurveSegment,
      common::Path::kMaxCurveSegmentNum>& smooth_curve_curvature_info() const {
    return (smooth_curve_curvature_info_);
  }

  /**
   * @brief 获取参考线的平滑后的曲线段信息。
   * @return 参考线的平滑后的曲线段信息
   */
  common::StaticVector<common::Path::CurveSegment,
      common::Path::kMaxCurveSegmentNum>& smooth_curve_curvature_info() {
    return (smooth_curve_curvature_info_);
  }

  /**
   * @brief 查找指定车道在参考线的车道段列表中的索引
   * @param[in] lane_index 车道的索引
   * @return 车道在参考线的车道段列表中的索引
   */
  Int32_t FindLaneSegment(Int32_t lane_index) const;

  /**
   * @brief 通过参考线上的点查找其位于参考线上的哪个车道段
   * @param[in] s_on_ref_line 参考线上的点
   * @param[out] lane_index 这个点所位于的车道索引
   * @return 参考线上的车道段的索引
   */
  Int32_t FindLaneSegment(
      map_var_t s_on_ref_line, Int32_t * const lane_index, Float32_t* const s_on_lane) const;

  inline void SetContinuousSegment(map_var_t start_s, map_var_t end_s) {
    continuous_seg_.start_s = start_s;
    continuous_seg_.end_s = end_s;
  }
  inline void GetContinuousSegment(map_var_t* start_s, map_var_t* end_s) const {
    *start_s = continuous_seg_.start_s;
    *end_s = continuous_seg_.end_s;
  }

private:
  // 参考线的左右相邻标记。具体的值为参考线上当前位置处的车道在左右相邻车道中的相邻标记的值。
  // 当前参考线的左右相邻标记为0，右侧的为负，左侧的为正。
  Int32_t neighbor_flag_;
  // 车道的质量
  Int32_t quality_;
  // 参考线的车道段列表
  common::StaticVector<LaneSegment, MAX_LANE_SEGMENT_NUM> lane_segments_;
  // 参考线的路径
  common::Path curve_;
  // 参考线的平滑后的路径
  common::Path smooth_curve_;
  // 参考线的曲线段信息
  common::StaticVector<common::Path::CurveSegment,
      common::Path::kMaxCurveSegmentNum> curve_curvature_info_;
  // 参考线的平滑后曲线段信息
  common::StaticVector<common::Path::CurveSegment,
      common::Path::kMaxCurveSegmentNum> smooth_curve_curvature_info_;
  //
  struct ContinuousSeg {
    map_var_t start_s;
    map_var_t end_s;

    ContinuousSeg() {
      start_s = 0.0F;
      end_s = 0.0F;
    }
  } continuous_seg_;
};


}  // namespace driv_map
}  // namespace phoenix


#endif  // PHOENIX_DRIV_MAP_REFERENCE_LINE_H_


