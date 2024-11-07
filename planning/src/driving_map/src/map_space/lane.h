/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       lane.h
 * @brief      内部车道定义
 * @details    定义了内部车道的数据格式及相关访问接口
 *
 * @author     boc
 * @date       2020.06.03
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/03  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/
#ifndef PHOENIX_DRIVING_MAP_LANE_H_
#define PHOENIX_DRIVING_MAP_LANE_H_

#include "container/static_vector.h"
#include "geometry/vec2d.h"
#include "curve/path.h"
#include "driving_map.h"

namespace phoenix {
namespace driv_map {

/**
 * @class Lane
 * @brief 内部车道
 */
class Lane {
public:
  /// 单侧左右相邻车道的最大个数
  enum { MAX_TOPOLOGY_NEIGHBOR_LANE_NUM = 1 };
  /// 前后相连车道的最大个数
  enum { MAX_TOPOLOGY_CONNECTED_LANE_NUM = 5 };
  /// 车道上坡道信息采样点的最大个数
  enum { MAX_LANE_SLOPE_SAMPLE_NUM = 500 };
  /// 车道边界线形点的最大个数
  enum { MAX_LANE_BOUNDARY_CURVE_POINT_NUM = 500 };
  /// 车道边界宽度采样点最大个数
  enum { MAX_LANE_BOUNDARY_SAMPLE_NUM = 500 };

  /**
   * @struct TopologyId
   * @brief 车道的拓扑ID，包含车道在地图中的全局唯一ID和内部地图中车道列表中对应的索引。
   */
  struct TopologyId {
    /// 车道在地图中的全局唯一ID
    ID id;
    /// 内部地图中车道列表中对应的索引
    Int32_t index;

    /**
     * @brief 将全局唯一ID和索引的值清空。
     */
    void Clear() {
      id.Clear();
      index = -1;
    }

    /**
     * @brief 构造函数。
     */
    TopologyId() {
      index = -1;
    }
  };

  /**
   * @struct BoundaryWidthAssociation
   * @brief 车道边线关联信息，包含车道的单侧宽度。
   */
  struct BoundaryWidthAssociation {
    /// 沿着车道中心线的长度
    map_var_t s;
    /// 通过s指定的车道中心线位置处的车道宽度(车道中心线左边的宽度，
    /// 或右边的宽度)
    /// （车道中心线左边或右边的宽度可以是不相等的）
    map_var_t width;

    /**
     * @brief 将成员变量的值清空。
     */
    void Clear() {
      s = 0.0F;
      width = 0.0F;
    }

    /**
     * @brief 构造函数。
     */
    BoundaryWidthAssociation() {
      Clear();
    }
  };

  /**
   * @struct BoundaryTypeAssociation
   * @brief 车道边线关联信息，包含车道的车道边界类型。
   */
  struct BoundaryTypeAssociation {
    /// 沿着车道中心线的长度
    map_var_t s;
    /// 通过s指定的车道中心线位置处的车道边线类型
    /// （未知/黄虚线/白虚线/黄实线/白实线/双黄线/路沿）
    Int32_t type;

    /**
     * @brief 将成员变量的值清空。
     */
    void Clear() {
      s = 0.0F;
      type = static_cast<Int32_t>(LANE_BOUNDARY_TYPE_UNKNOWN);
    }

    /**
     * @brief 构造函数。
     */
    BoundaryTypeAssociation() {
      Clear();
    }
  };

  /**
   * @struct SlopeAssociation
   * @brief 车道的坡度信息。
   */
  struct SlopeAssociation {
    /// 沿着车道中心线的长度
    map_var_t s;
    /// 通过s指定的车道中心线位置处的坡度，单位：dz/ds
    map_var_t slope;

    /**
     * @brief 将成员变量的值清空。
     */
    void Clear() {
      s = 0.0F;
      slope = 0.0F;
    }

    /**
     * @brief 构造函数。
     */
    SlopeAssociation() {
      Clear();
    }
  };

  /**
   * @struct Boundary
   * @brief 车道边线。
   */
  struct Boundary {
    /// 车道边线是否真实在路面上出现
    bool is_virtual;
    /// 车道边线形点
    common::StaticVector<common::Vec2d,
        MAX_LANE_BOUNDARY_CURVE_POINT_NUM> curve;
    /// 车道宽度信息
    common::StaticVector<BoundaryWidthAssociation,
        MAX_LANE_BOUNDARY_SAMPLE_NUM> boundary_width_samples;
    /// 车道边线类型信息
    common::StaticVector<BoundaryTypeAssociation,
        MAX_LANE_BOUNDARY_SAMPLE_NUM> boundary_type_samples;

    /**
     * @brief 将成员变量的值清空。
     */
    void Clear() {
      is_virtual = false;
      curve.Clear();
      boundary_width_samples.Clear();
      boundary_type_samples.Clear();
    }

    /**
     * @brief 构造函数。
     */
    Boundary() {
      Clear();
    }
  };

public:
  /**
   * @brief 构造函数，设置成员变量的初始值。
   */
  Lane();

  /**
   * @brief 将成员变量的值清空。
   */
  void Clear();

  /**
   * @brief 获取车道的拓扑ID。
   */
  inline const TopologyId& topology_id() const { return id_; }
  /**
   * @brief 获取车道的拓扑ID。
   */
  inline TopologyId& topology_id() { return id_; }
  /**
   * @brief 设置车道的拓扑ID。
   * @param[in] id    车道的拓扑ID。
   */
  inline void set_topology_id(const TopologyId& id) { id_ = id; }

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
   * @brief 获取车道类型。
   */
  inline Int32_t lane_type() const { return lane_type_; }
  /**
   * @brief 设置车道类型。
   * @param[in] type    车道类型。
   */
  inline void set_lane_type(Int32_t type) { lane_type_ = type; }

  /**
   * @brief 获取车道转弯类型。
   */
  inline Int32_t turn_type() const { return turn_type_; }
  /**
   * @brief 设置车道转弯类型。
   * @param[in] type    车道转弯类型。
   */
  inline void set_turn_type(Int32_t type) { turn_type_ = type; }

  /**
   * @brief 获取车道方向。
   */
  inline Int32_t direction() const { return direction_; }
  /**
   * @brief 设置车道方向。
   * @param[in] direction    车道方向。
   */
  inline void set_direction(Int32_t direction) { direction_ = direction; }

  /**
   * @brief 获取车道限速值(high)。
   */
  inline map_var_t speed_limit_high() const { return speed_limit_high_; }
  /**
   * @brief 设置车道限速值(high)。
   * @param[in] direction    车道限速值。
   */
  inline void set_speed_limit_high(map_var_t speed) { speed_limit_high_ = speed; }

  /**
   * @brief 获取车道限速值(low)。
   */
  inline map_var_t speed_limit_low() const { return speed_limit_low_; }
  /**
   * @brief 设置车道限速值(low)。
   * @param[in] direction    车道限速值。
   */
  inline void set_speed_limit_low(map_var_t speed) { speed_limit_low_ = speed; }

  /**
   * @brief 获取车道中心线。
   */
  inline const common::Path& central_curve() const { return central_curve_; }
  /**
   * @brief 获取车道中心线。
   */
  inline common::Path& central_curve() { return central_curve_; }

  /**
   * @brief 获取车道左侧边线。
   */
  inline const Boundary& left_boundary() const { return left_boundary_; }
  /**
   * @brief 获取车道左侧边线。
   */
  inline Boundary& left_boundary() { return left_boundary_; }

  /**
   * @brief 获取车道右侧边线。
   */
  inline const Boundary& right_boundary() const { return right_boundary_; }
  /**
   * @brief 获取车道右侧边线。
   */
  inline Boundary& right_boundary() { return right_boundary_; }

  /**
   * @brief 获取车道坡度。
   */
  inline const common::StaticVector<SlopeAssociation,
      MAX_LANE_SLOPE_SAMPLE_NUM>& slope_samples() const {
    return slope_samples_;
  }
  /**
   * @brief 获取车道坡度。
   */
  inline common::StaticVector<SlopeAssociation,
      MAX_LANE_SLOPE_SAMPLE_NUM>& slope_samples() {
    return slope_samples_;
  }

  /**
   * @brief 获取相邻的左侧的且同向的车道的拓扑ID。
   */
  inline const common::StaticVector<TopologyId,
      MAX_TOPOLOGY_NEIGHBOR_LANE_NUM>& left_neighbor_forward_lanes() const {
    return left_neighbor_forward_lanes_;
  }
  /**
   * @brief 获取相邻的左侧的且同向的车道的拓扑ID。
   */
  inline common::StaticVector<TopologyId,
      MAX_TOPOLOGY_NEIGHBOR_LANE_NUM>& left_neighbor_forward_lanes() {
    return left_neighbor_forward_lanes_;
  }

  /**
   * @brief 获取相邻的右侧的且同向的车道的拓扑ID。
   */
  inline const common::StaticVector<TopologyId,
      MAX_TOPOLOGY_NEIGHBOR_LANE_NUM>& right_neighbor_forward_lanes() const {
    return right_neighbor_forward_lanes_;
  }
  /**
   * @brief 获取相邻的右侧的且同向的车道的拓扑ID。
   */
  inline common::StaticVector<TopologyId,
      MAX_TOPOLOGY_NEIGHBOR_LANE_NUM>& right_neighbor_forward_lanes() {
    return right_neighbor_forward_lanes_;
  }

  /**
   * @brief 获取相邻的左侧的且逆向的车道的拓扑ID。
   */
  inline const common::StaticVector<TopologyId,
      MAX_TOPOLOGY_NEIGHBOR_LANE_NUM>& left_neighbor_reverse_lanes() const {
    return left_neighbor_reverse_lanes_;
  }
  /**
   * @brief 获取相邻的左侧的且逆向的车道的拓扑ID。
   */
  inline common::StaticVector<TopologyId,
      MAX_TOPOLOGY_NEIGHBOR_LANE_NUM>& left_neighbor_reverse_lanes() {
    return left_neighbor_reverse_lanes_;
  }

  /**
   * @brief 获取相邻的右侧的且逆向的车道的拓扑ID。
   */
  inline const common::StaticVector<TopologyId,
      MAX_TOPOLOGY_NEIGHBOR_LANE_NUM>& right_neighbor_reverse_lanes() const {
    return right_neighbor_reverse_lanes_;
  }
  /**
   * @brief 获取相邻的右侧的且逆向的车道的拓扑ID。
   */
  inline common::StaticVector<TopologyId,
      MAX_TOPOLOGY_NEIGHBOR_LANE_NUM>& right_neighbor_reverse_lanes() {
    return right_neighbor_reverse_lanes_;
  }

  /**
   * @brief 获取与此车道相连的后方的车道的拓扑ID。
   */
  inline const common::StaticVector<TopologyId,
      MAX_TOPOLOGY_CONNECTED_LANE_NUM>& predecessor_lanes() const {
    return predecessor_lanes_;
  }
  /**
   * @brief 获取与此车道相连的后方的车道的拓扑ID。
   */
  inline common::StaticVector<TopologyId,
      MAX_TOPOLOGY_CONNECTED_LANE_NUM>& predecessor_lanes() {
    return predecessor_lanes_;
  }

  /**
   * @brief 获取与此车道相连的前方的车道的拓扑ID。
   */
  inline const common::StaticVector<TopologyId,
      MAX_TOPOLOGY_CONNECTED_LANE_NUM>& successor_lanes() const {
    return successor_lanes_;
  }
  /**
   * @brief 获取与此车道相连的前方的车道的拓扑ID。
   */
  inline common::StaticVector<TopologyId, MAX_TOPOLOGY_CONNECTED_LANE_NUM>&
      successor_lanes() {
    return successor_lanes_;
  }

  /**
   * @brief 获取与此车道相连的前方的车道起点。
   */
  inline const common::StaticVector<common::PathPoint,
      MAX_TOPOLOGY_CONNECTED_LANE_NUM>& successor_lanes_start_point() const {
    return successor_lanes_start_point_;
  }
  /**
   * @brief 获取与此车道相连的前方的车道起点。
   */
  inline common::StaticVector<common::PathPoint,
      MAX_TOPOLOGY_CONNECTED_LANE_NUM>& successor_lanes_start_point() {
    return successor_lanes_start_point_;
  }

  /* longjiaoy 20230428 (start) */
  /**
   * @brief 获取与此车道在后方相连的车道上的起点。
   */
  inline const common::StaticVector<common::PathPoint,
      MAX_TOPOLOGY_CONNECTED_LANE_NUM>& predecessor_lanes_start_point() const {
    return predecessor_lanes_start_point_;
  }
  /**
   * @brief 获取与此车道在后方相连的车道上的起点。
   * ******************A**********
   *                   |**************************
   *                   |投影点
   */
  inline common::StaticVector<common::PathPoint,
      MAX_TOPOLOGY_CONNECTED_LANE_NUM>& predecessor_lanes_start_point() {
    return predecessor_lanes_start_point_;
  }
 /* longjiaoy 20230428 (start) */

  /**
   * @brief 获取车道的宽度。
   * @param[in] s               沿着车道中心线的长度。
   * @param[out] left_width     车道中心线左边的宽度。
   * @param[out] right_width    车道中心线左边的宽度。
   */
  void GetWidth(
      map_var_t s, map_var_t * const left_width, map_var_t * const right_width) const;

  /* k003 longjiaoy 2022-11-28 (start) */
  /**
   * @brief 获取车道边线的类型
   * @param[in] s              沿着车道中心线的长度。
   * @param[out] left_type     左边界类型
   * @param[out] left_type     右边界类型
   */
  void GetBoundaryType(
      map_var_t s, Int32_t* const left_type, Int32_t* const right_type) const;
  /* k003 longjiaoy 2022-11-28 (end) */

  /**
   * @brief 获取车道坡度
   * @param[in] s 沿着车道中心线的长度
   * @return 车道中此处位置的坡度
   */
  map_var_t GetLaneSlope(map_var_t s) const;

private:
  /// 比较函数类
  class FuncCmpSlopeArcLen {
  public:
    inline bool operator ()(const Lane::SlopeAssociation slope_a,
                     const Lane::SlopeAssociation slope_b) const {
      if (slope_a.s > slope_b.s) {
        return true;
      }
      return false;
    }
  };

private:
  /*
   * @brief 获取车道中心线到车道边线的宽度。
   * @param[in] boundary        车道边线。
   * @param[in] s               沿着车道中心线的长度。
   * @param[out] type           车道边线的类型。
   * @return 与s对应位置处的车道中心线到车道边线的宽度。
   */
  map_var_t GetBoundaryWidth(const Boundary& boundary, const map_var_t s) const;

  /* k003 longjiaoy 2022-11-28 (start) */
  /*
   * @brief 获取车道边线的类型
   * @param[in] boundary        车道边线。
   * @param[in] s               沿着车道中心线的长度。
   * @return 车道边线的类型。
   */
  Int32_t GetBoundaryType(const Boundary& boundary, const map_var_t s) const;
  /* k003 longjiaoy 2022-11-28 (end) */

private:
  // 车道的拓扑ID，包括为车道分配的唯一ID以及内部使用的索引
  TopologyId id_;
  // 车道的质量
  Int32_t quality_;
  // 车道类型 （未知车道/机动车道/自行车道/人行道/停车车道）
  Int32_t lane_type_;
  // 弯道类型 （未知转弯类型车道/直行车道/左转车道/U型弯车道）
  Int32_t turn_type_;
  // 车道方向 （前进/后退/双向行驶）
  Int32_t direction_;
  // 车道限速值
  map_var_t speed_limit_high_;
  map_var_t speed_limit_low_;

  // 车道中心线
  common::Path central_curve_;
  // 车道左侧边线
  Boundary left_boundary_;
  // 车道右侧边线
  Boundary right_boundary_;

  // 相邻的左侧的且同向的车道的拓扑ID
  common::StaticVector<TopologyId, MAX_TOPOLOGY_NEIGHBOR_LANE_NUM>
      left_neighbor_forward_lanes_;
  // 相邻的右侧的且同向的车道的拓扑ID
  common::StaticVector<TopologyId, MAX_TOPOLOGY_NEIGHBOR_LANE_NUM>
      right_neighbor_forward_lanes_;
  // 相邻的左侧的且逆向的车道的拓扑ID
  common::StaticVector<TopologyId, MAX_TOPOLOGY_NEIGHBOR_LANE_NUM>
      left_neighbor_reverse_lanes_;
  // 相邻的右侧的且逆向的车道的拓扑ID
  common::StaticVector<TopologyId, MAX_TOPOLOGY_NEIGHBOR_LANE_NUM>
      right_neighbor_reverse_lanes_;

  // 与此车道相连的后方的车道的拓扑ID
  common::StaticVector<TopologyId, MAX_TOPOLOGY_CONNECTED_LANE_NUM>
      predecessor_lanes_;
  // 与此车道相连的前方的车道的拓扑ID
  common::StaticVector<TopologyId, MAX_TOPOLOGY_CONNECTED_LANE_NUM>
      successor_lanes_;
  // 与此道路相连的前方的车道起点信息。
  // 此数据为安全冗余项，主要是为了确保相连车道连接处的一定程度上的连续性，
  // 避免不良地图造成轨迹规划的异常。通过找到此车道终点坐标在相连车道中心线
  // 上的最近点来实现该功能。同时也需要通过这两点之间的距离来校验车道是否
  // 真实相连。
  // 注意事项：
  //    这些点的存储顺序应当与successor_id_中对应的车道id的顺序一致。
  common::StaticVector<common::PathPoint, MAX_TOPOLOGY_CONNECTED_LANE_NUM>
      successor_lanes_start_point_;

  common::StaticVector<common::PathPoint, MAX_TOPOLOGY_CONNECTED_LANE_NUM>
      predecessor_lanes_start_point_;

  // 车道坡度
  common::StaticVector<SlopeAssociation, MAX_LANE_SLOPE_SAMPLE_NUM>
      slope_samples_;
};  // Lane


}  // namespace driv_map
}  // namespace phoenix


#endif  // PHOENIX_DRIVING_MAP_LANE_H_


