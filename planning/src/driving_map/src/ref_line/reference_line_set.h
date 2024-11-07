/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       reference_line_set.h
 * @brief      处理参考线列表
 * @details    实现了获取参考线集合的相关方法
 *
 * @author     boc
 * @date       2020.06.11
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/11  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/
#ifndef PHOENIX_DRIV_MAP_REFERENCE_LINE_SET_H_
#define PHOENIX_DRIV_MAP_REFERENCE_LINE_SET_H_

#include "container/static_vector.h"
#include "curve/path.h"
#include "curve/segmented_quintic_polynomial_fitting.h"
#include "driving_map.h"
#include "map_space/hd_map.h"
#include "ref_line/reference_line.h"

namespace phoenix {
namespace driv_map {


/**
 * @class ReferenceLineSet
 * @brief 参考线列表处理类。
 */
class ReferenceLineSet {
public:
  /**
   * @brief 构造函数。
   */
  ReferenceLineSet();
  /**
   * @brief 析构函数。
   */
  ~ReferenceLineSet();

  /**
   * @brief 清理函数，对内部成员变量分配的存储空间进行清理。
   */
  void Clear();

  /**
   * @brief 构建所有的参考线
   * @param[in] point    车辆当前位置
   * @param[in] heading  车辆航向角
   * @param[in] map      地图数据
   * @return true-成功；false-失败。
   */
  bool Construct(
      const common::Vec2d& point,
      map_var_t heading,
      const HDMap& map);

  bool TrimReferenceLine(
      Int32_t ref_line_index,
      const common::Path& tar_curve,
      const common::Vec2d& start_point,
      map_var_t* trim_start_s, map_var_t* trim_end_s);

  /**
   * @brief 获取车辆当前位置最近车道的车道索引和最近点
   * @param[out] nearest_lane_index       车辆当前位置最近车道的车道索引
   * @param[out] nearest_point_on_lane    最近车道上离车辆当前位置的最近点
   */
  inline void GetNearestLaneToCurrPosition(
      Int32_t* const nearest_lane_index,
      common::PathPoint* const nearest_point_on_lane,
      Int32_t* const nearest_idx_in_neighbor,
      common::StaticVector<NeighborsLaneInfo, MAX_NEIGHBOR_LANE_NUM>*
      const neighbor_lanes) const {
    if (Nullptr_t != nearest_lane_index) {
      *nearest_lane_index = nearest_lane_index_;
    }
    if (Nullptr_t != nearest_point_on_lane) {
      *nearest_point_on_lane = nearest_point_on_lane_;
    }
    if (Nullptr_t != nearest_idx_in_neighbor) {
      *nearest_idx_in_neighbor = nearest_index_in_neighbor_lanes_of_curr_pos_;
    }
    if (Nullptr_t != neighbor_lanes) {
      *neighbor_lanes = neighbor_lanes_of_curr_pos_;
    }
  }

  /**
   * @brief 获取所有参考线的个数
   * @return 所有参考线的个数
   */
  inline Int32_t GetReferenceLinesNum() const {
    return (reference_lines_.Size());
  }

  /**
   * @brief 判断指定的参考线索引是否有效
   * @return true-指定的参考线索引有效；false-指定的参考线索引无效。
   */
  inline bool IsValidReferenceLineIndex(Int32_t index) const {
    return ((0 <= index) && (index < reference_lines_.Size()));
  }

  /**
   * @brief 获取参考线的左右相邻标记。离车辆最近的参考线的左右相邻标记为0，\n
   *        过右边的第i条车道的参考线为-i，过左边的第i条车道的参考线为i。
   * @return 参考线的左右相邻标记
   */
  inline Int32_t GetNeighborFlag(Int32_t index) const {
    COM_CHECK(IsValidReferenceLineIndex(index));
    return (reference_lines_[index].neighbor_flag());
  }

  /**
   * @brief 获取序号为index参考线的车道线质量。
   * @return 参考线的车道线质量
   */
  inline Int32_t GetLaneQuality(Int32_t index) const {
    COM_CHECK(IsValidReferenceLineIndex(index));
    return (reference_lines_[index].quality());
  }

  /**
   * @brief 获取参考线的车道段列表。
   * @param[in] index 参考线的索引
   * @return 参考线的车道段列表
   */
  inline const common::StaticVector<LaneSegment, MAX_LANE_SEGMENT_NUM>&
      GetLaneSegments(Int32_t index) const {
    COM_CHECK(IsValidReferenceLineIndex(index));
    return (reference_lines_[index].lane_segments());
  }

  /**
   * @brief 获取当前车辆位置在此参考线上的投影点到参考线起点的距离
   * @param[in] index 参考线的索引
   * @return 当前车辆位置在此参考线上的投影点到参考线起点的距离
   */
  inline map_var_t GetProjDistOfCurrPositionOnRef(Int32_t index) const {
    COM_CHECK(IsValidReferenceLineIndex(index));
    return (-reference_lines_[index].lane_segments().Front().start_s_on_ref);
  }

  /**
   * @brief 获取参考线的路径。
   * @param[in] index 参考线的索引
   * @return 参考线的路径
   */
  inline const common::Path& GetCurve(Int32_t index) const {
    COM_CHECK(IsValidReferenceLineIndex(index));
    return (reference_lines_[index].curve());
  }

  /**
   * @brief 获取参考线的平滑后的路径。
   * @param[in] index 参考线的索引
   * @return 参考线的平滑后的路径
   */
  inline const common::Path& GetSmoothCurve(Int32_t index) const {
    COM_CHECK(IsValidReferenceLineIndex(index));
    return (reference_lines_[index].smooth_curve());
  }

  /**
   * @brief 获取参考线的曲线段信息。
   * @param[in] index 参考线的索引
   * @return 参考线的曲线段信息
   */
  inline const common::StaticVector<common::Path::CurveSegment,
      common::Path::kMaxCurveSegmentNum>& GetCurveCurvatureInfo(
      Int32_t index) const {
    COM_CHECK(IsValidReferenceLineIndex(index));
    return (reference_lines_[index].curve_curvature_info());
  }

  /**
   * @brief 获取参考线的平滑后的曲线段信息。
   * @param[in] index 参考线的索引
   * @return 参考线的平滑后的曲线段信息
   */
  inline const common::StaticVector<common::Path::CurveSegment,
      common::Path::kMaxCurveSegmentNum>& GetSmoothCurveCurvatureInfo(
      Int32_t index) const {
    COM_CHECK(IsValidReferenceLineIndex(index));
    return (reference_lines_[index].smooth_curve_curvature_info());
  }

  /**
   * @brief 获取主参考线的索引。
   * @return 主参考线的索引
   */
  inline Int32_t GetMajorRefLineIndex() const {
    return (major_refefrence_line_index_);
  }

  /**
   * @brief 获取车辆位置在主参考线上的投影点。
   * @return 车辆位置在主参考线上的投影点
   */
  inline const common::PathPoint& GetProjPointOnMajorRefLine() const {
    return (proj_point_on_major_ref_line_);
  }

  /**
   * @brief 根据起点位置、起始车道和终点车道的索引查找对应的参考线。
   * @param[in] start_point         起点位置
   * @param[in] start_lane_index    起始车道
   * @param[in] end_lane_index      终点车道
   * @return 大于或者等于0时    -  经过起点位置、起始车道和终点车道的参考线的索引；\n
   *         -1时             -  没有查找到合适的参考线。
   * @note   1、当查找不到同时经过起始车道和终点车道的参考线时，\n
   *            会在所有经过终点车道的参考线中，以离起点位置\n
   *            最近的参考线作为对应的参考线。
   */
  Int32_t FindReferenceLine(
      const common::Vec2d& start_point,
      Int32_t start_lane_index,
      Int32_t end_lane_index) const;

  /**
   * @brief 根据起始车道和终点车道的索引查找对应的参考线。
   * @param[in] start_lane_index    起始车道
   * @param[in] end_lane_index      终点车道
   * @return 大于或者等于0时    -  经过起始车道和终点车道的参考线的索引；\n
   *         -1时             -  没有查找到合适的参考线。
   */
  Int32_t FindReferenceLine(
      Int32_t start_lane_index, Int32_t end_lane_index) const;

  /**
   * @brief 通过参考线上的点查找其位于参考线上的哪个车道段
   * @param[in] ref_line_index 参考线索引
   * @param[in] s_on_ref_line 参考线上的点到参考线起点的路径长
   * @param[out] lane_index 这个点所位于的车道索引
   * @return 参考线上的车道段的索引
   */
  Int32_t FindLaneSegmentByProjOnRef(Int32_t ref_line_index,
      map_var_t s_on_ref_line,
      Int32_t * const lane_index, Float32_t* const s_on_lane) const;

  Int32_t FindLaneSegmentById(
      const common::StaticVector<LaneSegment, MAX_LANE_SEGMENT_NUM>& lane_segments,
      const ID& lane_id) const {
    for (Int32_t i = 0; i < lane_segments.Size(); ++i) {
      if (lane_segments[i].lane_id == lane_id) {
        return (i);
      }
    }

    return (-1);
  }

  inline void GetContinuousSegment(
      Int32_t index, map_var_t* start_s, map_var_t* end_s) const {
    COM_CHECK(IsValidReferenceLineIndex(index));
    reference_lines_[index].GetContinuousSegment(start_s, end_s);
  }

private:
  /*
   * @struct ListForFindingCurrRefLine
   * @brief 查找当前参考线时需要的列表结构体。
   */
  struct ListForFindingCurrRefLine {
    struct RefLineInfo {
      Int32_t ref_line_idx;
      Int32_t routing_count;

      RefLineInfo() {
        Clear();
      }

      void Clear() {
        ref_line_idx = -1;
        routing_count = 0;
      }
    };
    // 参考线索引的列表
    common::StaticVector<RefLineInfo, MAX_REFERENCE_LINE_NUM> list[2];

    // 当前正在查找的参考线索引列表的索引
    Int32_t curr_index;
    // 下一个循环遍历时将要查找的参考线索引列表的索引
    Int32_t next_index;

    /*
     * @brief 将当前正在查找的参考线索引列表的索引和下一个循环遍历时将要查找的\n
     *        参考线索引列表的索引的值进行交换。
     */
    inline void Swap() {
      const Int32_t tmp = curr_index;
      curr_index = next_index;
      next_index = tmp;
    }

    /*
     * @brief 将成员变量的值清空。
     */
    inline void Clear() {
      curr_index = 0;
      next_index = 1;
      list[0].Clear();
      list[1].Clear();
    }

    /*
     * @brief 构造函数。
     */
    ListForFindingCurrRefLine() {
      curr_index = 0;
      next_index = 1;
    }

    /*
     * @brief 获取当前正在查找的参考线索引列表的索引。
     * @return 当前正在查找的参考线索引列表的索引
     */
    inline common::StaticVector<RefLineInfo, MAX_REFERENCE_LINE_NUM>& Curr() {
      return list[curr_index];
    }

    /*
     * @brief 获取下一个循环遍历时将要查找的参考线索引列表的索引。
     * @return 下一个循环遍历时将要查找的参考线索引列表的索引
     */
    inline common::StaticVector<RefLineInfo, MAX_REFERENCE_LINE_NUM>& Next() {
      return list[next_index];
    }
  };

private:
  /*
   * @brief 查找当前车道。
   * @param[in] point    车辆当前位置
   * @param[in] heading  车辆的航向角
   * @param[in] map      内部地图数据
   * @return true-查找成功；false-查找失败。
   */
  bool FindCurrentLane(
      const common::Vec2d& point, map_var_t heading, const HDMap& map);
  /*
   * @brief 查找所有参考线的车道段列表。
   * @param[in] map    内部地图数据
   * @return true-查找成功；false-查找失败。
   */
  bool FindAllReferenceLaneSegments(const HDMap& map);
  /*
   * @brief 查找参考线的起始车道列表。
   * @param[in] map            内部地图数据
   * @param[out] s_offset      车辆当前时刻所在位置到构建点的有向距离\n
   *                          （向前为正，向后为负）
   * @param[out] nearest_index 参考线的起始车道列表中，与当前车道起点航向角度差最小的\n
   *                           车道在参考线的起始车道列表中的索引
   * @param[out] start_neighbor_lanes   参考线的起始车道列表
   * @return true-查找成功；false-查找失败。
   * @note 1. start_neighbor_lanes中车道的排列顺序为从右向左进行排列。\n
   *       2. 当起始车道为当前车道时，构建点为车辆当前位置；\n
   *          当起始车道为当前车道的后方相连车道时，构建点为\n
   *          后方相连车道终点后面2米处的点。
   */
  bool SelectReferenceLineStartLanes(
      const HDMap& map,
      map_var_t * const s_offset,
      Int32_t * const nearest_index,
      common::StaticVector<NeighborsLaneInfo,
      MAX_NEIGHBOR_LANE_NUM> * const start_neighbor_lanes);
  /*
   * @brief 创建所有参考线。\n
   * 创建的具体内容包括路径、曲线段、平滑后的路径、平滑后的曲线段等信息。
   * @param[in] map            内部地图数据
   * @return true-创建成功；false-创建失败。
   */
  bool CreateAllReferenceLines(const HDMap& map);
  /*
   * @brief 根据参考线的车道段列表，创建单条参考线的路径等信息。\n
   * 创建的具体内容包括路径、曲线段、平滑后的路径、平滑后的曲线段。
   * @param[in] map            内部地图数据
   * @param[in&out] ref_line   单条参考线
   * @return true-创建成功；false-创建失败。
   */
  bool CreateReferenceLine(const HDMap& map, ReferenceLine * const ref_line);
  /*
   * @brief 查找当前参考线。以离当前车道最近的参考线作为当前参考线。
   * @param[in] map            内部地图数据
   * @return true-查找成功；false-查找失败。
   */
  bool FindCurrentReferenceLine(const HDMap& map);

private:
  // 车辆当前位置最近车道的车道索引
  Int32_t nearest_lane_index_;
  // 最近车道上离车辆当前位置的最近点
  common::PathPoint nearest_point_on_lane_;
  // 车辆当前位置最近车道在左右相邻车道列表中的索引
  Int32_t nearest_index_in_neighbor_lanes_of_curr_pos_;
  // 车辆当前位置的左右相邻车道列表
  common::StaticVector<NeighborsLaneInfo, MAX_NEIGHBOR_LANE_NUM>
      neighbor_lanes_of_curr_pos_;

  // 当前车道在参考线列表车道段中的索引
  Int32_t ref_lane_segment_start_index_;
  // 主参考线在参考线列表中的索引
  Int32_t major_refefrence_line_index_;
  // 参考线列表
  common::StaticVector<ReferenceLine, MAX_REFERENCE_LINE_NUM> reference_lines_;
  // 车辆当前位置在当前参考线上的投影点
  common::PathPoint proj_point_on_major_ref_line_;

  /*
   * @struct
   * @brief 与参考线相关的参数。
   */
  struct {
    // 参考线起点到车辆当前位置的路径长
    map_var_t backward_len;
    // 车辆当前位置到参考线终点的路径长
    map_var_t forward_len;
    // 参考线点列采样长度
    map_var_t anchor_point_sample_interval;
    // 参考线的中间车道段采样点离车道段起点和终点的最小路径长度
    map_var_t anchor_point_sample_cutoff_endpoint_dist;
  } param_;

  /*
   * @struct TemporaryData
   * @brief 构建参考线时需要的一些内部数据。
   */
  struct TemporaryData {
    /*
     * @struct StackForFindingRefLanes
     * @brief 查找所有参考线的车道段列表时需要的栈结构体。
     */
    struct StackForFindingRefLanes {
      /*
       * @struct StackNode
       * @brief 查找所有参考线的车道段列表时需要的单个栈结点。
       */
      struct StackNode {
        // 已经存储的车道段列表
        common::StaticVector<LaneSegment, MAX_LANE_SEGMENT_NUM> lanes;
        // 已累计的路径长
        map_var_t accumulated_s;
        // 当前正在处理的前向车道在前向车道列表中的索引
        Int32_t access_index;

        /*
         * @brief 清空所有成员变量的值。
         */
        void Clear() {
          accumulated_s = 0.0F;
          access_index = 0;
          lanes.Clear();
        }

        /*
         * @brief 构造函数。
         */
        StackNode() {
          accumulated_s = 0.0F;
          access_index = 0;
        }
      };

      // 栈中的结点列表
      common::StaticVector<StackNode, MAX_LANE_SEGMENT_NUM> nodes;

      /*
       * @brief 将栈中的结点信息清空。
       */
      void Clear() {
        for (Int32_t i = 0; i < nodes.Size(); ++i) {
          nodes[i].Clear();
        }
        nodes.Clear();
      }

      /*
       * @brief 构造函数。
       */
      StackForFindingRefLanes() {
        Clear();
      }

      /*
       * @brief   分配一个新的栈结点。
       * @return  分配的新的栈结点。
       */
      inline StackNode* Allocate() {
        StackNode * const node = nodes.Allocate();
        if (Nullptr_t != node) {
          node->Clear();
        }
        return (node);
      }

      /*
       * @brief 判断栈是否为空。
       * @return true-栈为空；false-栈不为空。
       */
      bool Empty() {
        return nodes.Empty();
      }

      /*
       * @brief 从栈中取顶上的结点。
       * @return 栈中顶上的结点。
       */
      inline StackNode& Top() {
        return nodes.Back();
      }

      /*
       * @brief 弹出栈顶的结点。
       */
      inline void Pop() {
        if (!nodes.Empty()) {
          nodes.Back().Clear();
          nodes.PopBack();
        }
      }
    } stack_for_finding_ref_lanes;

    // 在参考线的车道段列表的车道的中心线上进行采样时，需要的路径点列临时变量
    common::StaticVector<common::PathPoint, common::Path::kMaxPathPointNum>
        sample_points;

    common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>
        sample_2d_points;

    enum { MAX_PATH_LIST_SIZE = 1 };
    common::Path path_list[MAX_PATH_LIST_SIZE];

    // 曲线平滑工具类
    common::SegmentedQuinticPolynomialFitting curve_fitting;

    /*
     * @brief 将成员变量的值清空。
     */
    void Clear() {
      stack_for_finding_ref_lanes.Clear();
      sample_points.Clear();
    }
  } temp_data_;
};


}  // namespace driv_map
}  // namespace phoenix


#endif  // PHOENIX_DRIV_MAP_REFERENCE_LINE_SET_H_
