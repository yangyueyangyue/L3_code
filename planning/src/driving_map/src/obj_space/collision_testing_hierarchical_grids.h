/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       collision_testing_hierarchical_grids.h
 * @brief      分层网格空间分割
 * @details    使用分层的二维网格来分割某个二维空间，用于加快二维碰撞分析的速度
 *
 * @author     pengc
 * @date       2020.05.12
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/12  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_DRIVING_MAP_COLLISION_TESTING_HIERARCHICAL_GRIDS_H_
#define PHOENIX_DRIVING_MAP_COLLISION_TESTING_HIERARCHICAL_GRIDS_H_

#include "utils/macros.h"
#include "utils/log.h"
#include "curve/path.h"
#include "geometry/obbox2d.h"
#include "container/static_vector.h"
#include "driving_map.h"
#include "obj_space/object_map.h"
#include "obj_space/collision_testing_base.h"

namespace phoenix {
namespace driv_map {


/**
 * @class CollisionTestingHierarchicalGrids
 * @brief 实现二维分层网格空间分割算法，并提供碰撞分析的接口
 */
class CollisionTestingHierarchicalGrids : public CollisionTestingBase {
public:
  /**
   * @brief 构造函数
   * @param[in] min_cell_size 最底层的网格的尺寸
   * @param[in] sphere_to_cell_ratio 此网格中能存储的最大障碍物的外接圆的直径的比例
   * @param[in] cell_to_cell_ratio 网格层次上升时，网格尺寸变大的比率
   */
  CollisionTestingHierarchicalGrids(
      map_var_t min_cell_size = 5.0f,
      map_var_t sphere_to_cell_ratio = 1.0f/4.0f,
      map_var_t cell_to_cell_ratio = 2.0f);

  /**
   * @brief 清除内部数据
   */
  void Clear();

  /**
   * @brief 向网格中添加一个障碍物
   * @param[in] new_obj 要添加的新的障碍物的信息
   * @param[in] new_obj_list_index 新的障碍物在用户障碍物列表中的索引
   * @return true 成功 \n
   *         false 失败（通常是内部存储空间不足）
   */
  bool AddObject(const Object& new_obj, Int32_t new_obj_list_index);

  /**
   * @brief 告知已经完成了所有障碍物的添加，用于碰撞分析模块的进一步处理
   */
  void CompleteAddingObject() {
    // nothing to do
  }

  /**
   * @brief 根据索引返回障碍物关联信息
   * @param[in] obj_index 障碍物索引
   * @return 障碍物关联信息
   */
  const CollisionTestingBase::ObjectAssociation* GetObjectAssociation(
      Int32_t obj_index) const;

  /**
   * @brief 测试输入的目标物与网格中的障碍物的碰撞风险
   * @param[in] test_obj 测试的输入目标物
   * @param[out] risky_obj_list 碰撞测试时有风险的障碍物的列表
   * @return 碰撞的风险值
   */
  Int32_t TestCollision(
      const CollisionTestObj& test_obj,
      const CallbackGetObject& callback_get_obj,
      CallbackIfObjHasRisk* callback_if_has_risk) const;

private:
  /*
   * @class CellKey
   * @brief 使用哈希表来存储分层网格，这个类定义了哈希表的键值
   */
  class CellKey {
  public:
    /*
     * @brief 构造函数
     */
    CellKey() {
      x_ = 0;
      y_ = 0;
      level_ = 0;
    }

    /*
     * @brief 构造函数(有参数)
     * @param[in] x 网格坐标x
     * @param[in] y 网格坐标y
     * @param[in] level 网格所在的层次
     */
    CellKey(const Int32_t x, const Int32_t y, const Int32_t level) {
      x_ = x;
      y_ = y;
      level_ = level;
    }

    /*
     * @brief 清除内部数据
     */
    void Clear() {
      x_ = 0;
      y_ = 0;
      level_ = 0;
    }

    /*
     * @brief 获取网格坐标x
     * @return 网格坐标x
     */
    Int32_t x() const { return (x_); }
    /*
     * @brief 获取网格坐标y
     * @return 网格坐标y
     */
    Int32_t y() const { return (y_); }
    /*
     * @brief 获取网格所在的层次
     * @return 网格所在的层次
     */
    Int32_t level() const { return (level_); }

    /*
     * @brief 拷贝
     * @param[in] right 另一个键值
     */
    void operator =(const CellKey& right) {
      x_ = right.x_;
      y_ = right.y_;
      level_ = right.level_;
    }

    /*
     * @brief 判断键值是否相等
     * @param[in] right 另一个键值
     * @return true 相等 \n
     *         false 不相等
     */
    bool operator ==(const CellKey& right) const {
      return ((x_ == right.x_) && (y_ == right.y_) && (level_ == right.level_));
    }
    /*
     * @brief 判断键值是否不相等
     * @param[in] right 另一个键值
     * @return true 不相等 \n
     *         false 相等
     */
    bool operator !=(const CellKey& right) const {
      return ((x_ != right.x_) || (y_ != right.y_) || (level_ != right.level_));
    }

  private:
    // 网格坐标x
    Int32_t x_;
    // 网格坐标y
    Int32_t y_;
    // 网格所在层次
    Int32_t level_;
  };

  /*
   * @struct Obj
   * @brief 内部使用的障碍物信息
   */
  struct Obj {
    // 障碍物基础信息
    CollisionTestingBase::ObjInfoBase base;
    // 内部哈希散列表的索引
    CellKey cell_key;
    // 内部哈希散列表的index
    Int32_t bucket;

    /*
     * @brief 清除内部数据
     */
    void Clear() {
      base.Clear();
      cell_key.Clear();
      bucket = -1;
    }

    /*
     * @brief 构造函数
     */
    Obj() {
      Clear();
    }
  };

  /*
   * @class Cell
   * @brief 定义了分层网格的单元信息
   */
  class Cell {
  public:
    /*
     * @enum MAX_CELL_ITEM_NUM
     * @brief 每个单元可以存储的最大障碍物的数量
     */
    enum { MAX_CELL_ITEM_NUM = 64 };

    /*
     * @brief 构造函数
     */
    Cell() {
      // nothing to do
    }

    /*
     * @brief 清除内部数据
     */
    void Clear() {
      items_.Clear();
    }

    /*
     * @brief 向网格单元中添加一个障碍物
     * @param[in] new_obj_index 障碍物在列表中的索引
     * @return true 成功 \n
     *         false 失败（存储空间不足)
     */
    bool PushBack(Int32_t new_obj_index) {
      if (items_.Full()) {
        LOG_ERR << "Storage is full and cann't save objects anymore.";
        return false;
      }

      return (items_.PushBack(new_obj_index));
    }

    /*
     * @brief 获取网格单元中的障碍物列表
     * @return 网格单元中的障碍物列表
     */
    const common::StaticVector<Int32_t, MAX_CELL_ITEM_NUM>& items() const {
      return (items_);
    }

  private:
    // 存储在网格单元中的障碍物列表
    common::StaticVector<Int32_t, MAX_CELL_ITEM_NUM> items_;
  };

  /*
   * @enum HGRID_MAX_LEVEL
   * @brief 网格的最大层次
   */
  enum { HGRID_MAX_LEVEL = 16 };
  /*
   * @enum MAX_GRID_OBJ_NUM
   * @brief 最大可以保存在分层网格中的障碍物数量
   */
  enum { MAX_GRID_OBJ_NUM = 1024 };
  /*
   * @enum MAX_GRID_BUCKET_NUM
   * @brief 最大哈希散列表的尺寸
   */
  enum { MAX_GRID_BUCKET_NUM = 512 };

private:
  /*
   * @brief 通过键值获取哈希散列表的索引
   * @param[in] key 键值
   * @return 哈希散列表的索引
   */
  Int32_t CalcBucketIndex(const CellKey& key) const {
    // Computes hash bucket index in range [0, MAX_GRID_BUCKET_NUM-1]
    static const Int32_t h1 = 0x8da6b343;  // Large multiplicative constants;
    static const Int32_t h2 = 0xd8163841;  // here arbitrarily chosen primes
    static const Int32_t h3 = 0xcb1ab31f;
    Int32_t n = h1 * key.x() + h2 * key.y() + h3 * key.level();
    n = n % MAX_GRID_BUCKET_NUM;
    if (n < 0) n += MAX_GRID_BUCKET_NUM;

    return (n);
  }

  /*
   * @brief 计算障碍物覆盖的网格的范围
   * @param[in] obj 障碍物的矩形包围盒
   * @param[in] obj_radius 障碍物的矩形包围盒的外接圆半径
   * @param[in] cell_size 网格的尺寸
   * @param[out] x_min 障碍物覆盖的网格的范围的左下角坐标x
   * @param[out] y_min 障碍物覆盖的网格的范围的左下角坐标y
   * @param[out] x_max 障碍物覆盖的网格的范围的右上角坐标x
   * @param[out] y_max 障碍物覆盖的网格的范围的右上角坐标y
   */
  void CalcCellsOverlapped(
      const common::OBBox2d& obj, map_var_t obj_radius, map_var_t cell_size,
      Int32_t* x_min, Int32_t* y_min,
      Int32_t* x_max, Int32_t* y_max) const;

   /*
    * @brief 向网格中添加一个障碍物
    * @param[in] new_obj 障碍物信息
    * @return true 成功 \n
    *         false 失败（通常是存储空间不足)
    */
  bool AddObjectInternal(Obj* new_obj);

private:
  // 此网格中能存储的最大障碍物的外接圆的直径的比例
  map_var_t sphere_to_cell_ratio_;
  // 网格层次上升时，网格尺寸变大的比率
  map_var_t cell_to_cell_ratio_;
  // 最底层的网格的尺寸
  map_var_t min_cell_size_;

  // 标记某个网格层次是否有障碍物，用来加快碰撞分析的速度
  Uint32_t occupied_levels_mask_;
  // 某个网格层次包含的障碍物的数量
  Int32_t objects_at_level_[HGRID_MAX_LEVEL];

  // 分层网格中的障碍物列表
  common::StaticVector<Obj, MAX_GRID_OBJ_NUM> obj_list_;
  // 分层网格中哈希散列表
  common::StaticVector<Cell, MAX_GRID_BUCKET_NUM> grid_bucket_;
};


}  // namespace driv_map
}  // namespace phoenix


#endif  // PHOENIX_DRIVING_MAP_COLLISION_TESTING_HIERARCHICAL_GRIDS_H_
