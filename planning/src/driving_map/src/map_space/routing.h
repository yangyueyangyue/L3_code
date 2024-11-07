/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       routing.h
 * @brief      内部导航路径
 * @details    定义了内部导航路径的数据格式及相关访问接口
 *
 * @author     boc
 * @date       2020.06.08
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/08  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/
#ifndef PHOENIX_DRIVING_MAP_ROUTING_H_
#define PHOENIX_DRIVING_MAP_ROUTING_H_

#include "container/static_vector.h"
#include "driving_map.h"

namespace phoenix {
namespace driv_map {


/**
 * @struct Routing
 * @brief 内部导航路径
 * @details 一条导航路径由多个(10个)RoutingPiece组成，
 *          这些RoutingSection为前后相连的道路。一个RoutingSection由
 *          多个RoutingSegment组成，这些RoutingSegment为左右相邻
 *          的车道。
 */
class Routing {
public:
  /**
   * @brief 构造函数
   */
  Routing();
  /**
   * @brief 析构函数
   */
  ~Routing();

  /**
   * @brief 清除内部数据
   */
  void Clear();

  /**
   * @brief 获取导航道路段列表
   * @return 导航道路段列表
   */
  const common::StaticVector<RoutingSection,
      MAX_ROUTING_SECTION_NUM>& routing_sections() const {
    return (routing_sections_);
  }

  /**
   * @brief 获取导航道路段列表
   * @return 导航道路段列表
   */
  common::StaticVector<RoutingSection,
      MAX_ROUTING_SECTION_NUM>& routing_sections() {
    return (routing_sections_);
  }

  /**
   * @brief 判断指定车道的某一段与导航路径是否有重叠
   * @param[in] lane_index 车道的索引
   * @param[in] start_s    起点的路径长
   * @param[in] end_s      终点的路径长
   * @return true-有重叠；false-没有重叠。
   */
  bool IsOverlappedWithRoutingSegment(
      Int32_t lane_index, map_var_t start_s, map_var_t end_s) const;

  /**
   * @brief 判断指定车道的某一段与导航路径是否有重叠。
   *        当有重叠时，获取重叠部分对应导航路径上的导航道路段索引和
   *        左右相邻导航车道索引。
   * @param[in] lane_index      车道的索引
   * @param[in] start_s         起点的路径长
   * @param[in] end_s           终点的路径长
   * @param[out] segment_index  重叠部分对应导航路径上的导航道路段索引
   * @param[out] neighbor_index 重叠部分对应导航路径上的左右相邻导航车道索引
   * @return true-有重叠；false-没有重叠。
   */
  bool IsOverlappedWithRoutingSegment(
      Int32_t lane_index, map_var_t start_s, map_var_t end_s,
      Int32_t * const segment_index, Int32_t * const neighbor_index) const;

  /**
   * @brief 判断指定车道的某一个点是否在导航路径上
   * @param[in] lane_index 车道的索引
   * @param[in] s          点的路径长
   * @return true-在导航路径上；false-不在导航路径上。
   */
  bool IsOnRoutingSegment(Int32_t lane_index, map_var_t s) const;

private:
  // 组成内部导航路径的道路段列表
  common::StaticVector<RoutingSection, MAX_ROUTING_SECTION_NUM>
      routing_sections_;
};


}  // namespace driv_map
}  // namespace phoenix


#endif  // PHOENIX_DRIVING_MAP_ROUTING_H_


