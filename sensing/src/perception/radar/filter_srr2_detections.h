//
#ifndef PHOENIX_PERCEPTION_FILTER_SRR2_DETECTIONS_H_
#define PHOENIX_PERCEPTION_FILTER_SRR2_DETECTIONS_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "geometry/vec2d.h"
#include "geometry/aabbox2d.h"
#include "geometry/obbox2d.h"
#include "geometry/geometry_utils.h"
#include "geometry/aabboxkdtree2d.h"
#include "ad_msg.h"


namespace phoenix {
namespace perception {


class FilterSrr2Detections {
public:
  typedef Float32_t Scalar;

public:
  FilterSrr2Detections();
  ~FilterSrr2Detections();

  bool Filter(
      Int32_t radar_idx,
      const ad_msg::ObstacleRadarList& detection_list,
      const ad_msg::Chassis& chassis,
      ad_msg::ObstacleRadarList* tracked_obj_list);

private:
  enum { MAX_RADAR_NUM = 4 };
  enum { MAX_TRACKED_OBJ_NUM = 128 };
  enum { MAX_PARTNER_NUM = 40 };

  struct ObjIndex {
    Int32_t obj_idx;

    ObjIndex() {
      obj_idx = -1;
    }

    ObjIndex(Int32_t obj) {
      obj_idx = obj;
    }

    void Clear() {
      obj_idx = -1;
    }

    bool IsValid() const {
      return (obj_idx >= 0);
    }
  };

  struct ObjTracked {
    Int32_t raw_obj_idx;

    bool erase;

    Int32_t obj_id;

    Scalar length;
    Scalar width;
    struct {
      Scalar x;
      Scalar y;

      void Clear() {
        x = 0.0F;
        y = 0.0F;
      }
    } pos;
    common::AABBox2d aabb;

    Float32_t v_x;
    Float32_t v_y;

    void Clear() {
      raw_obj_idx = -1;
      erase = false;

      obj_id = 0;

      length = 0.0F;
      width = 0.0F;
      pos.Clear();
      aabb.Clear();

      v_x = 0.0F;
      v_y = 0.0F;
    }
  };

  struct ObjTrackedList {
    ad_msg::MsgHead msg_head;
    common::StaticVector<ObjTracked, MAX_TRACKED_OBJ_NUM> objects;

    ObjTrackedList() {
    }

    void Clear() {
      msg_head.Clear();
      Int32_t obj_num = objects.Size();
      for (Int32_t i = 0; i < obj_num; ++i) {
        objects[i].Clear();
      }
      objects.Clear();
    }
  };

  class CalcSquareDistToObjTracked;
  friend class CalcSquareDistToObjTracked;
  /*
   * @class CalcSquareDistToObjTracked
   * @brief 计算OBJ到OBJ的平方距离
   */
  class CalcSquareDistToObjTracked {
  public:
    /*
     * @brief 构造函数
     * @param[in]  point   待查询点
     * @param[in]  lane_lines  车道线
     */
    CalcSquareDistToObjTracked(
        const common::AABBox2d& aabb,
        const common::StaticVector<ObjTracked, MAX_TRACKED_OBJ_NUM>& objects)
      : obj_aabbox_(aabb), tracked_objects_(objects) {
      // nothing to do
    }
    /*
     * @brief 重载的'()'运算符
     * @param[in]  aabb            待查询的目标
     * @param[in]  tree_obj_index  待查询的目标在k-d树中对应的索引
     * @return 点到目标的平方距离。
     */
    Scalar operator ()(
        const ObjIndex& index, Int32_t tree_obj_index) const {
      return (common::Square(common::DistAABBToAABB_2D(
                             tracked_objects_[index.obj_idx].aabb,
                             obj_aabbox_)));
    }

  private:
    // 待查询Bounding box of target obj
    const common::AABBox2d& obj_aabbox_;
    const common::StaticVector<ObjTracked, MAX_TRACKED_OBJ_NUM>&
        tracked_objects_;
  };

private:
  ObjTrackedList tracked_objects_;

  // 用于加快最近OBJ查询等功能的k-d树
  common::AABBoxKDTree2d<ObjIndex,
      MAX_TRACKED_OBJ_NUM, 16> tracked_objects_kdtree_;
  // used for k-d tree
  common::StaticVector<common::AABBoxKDTreeNodeAssociation, 64>
      kdtree_nodes_stack_;
};


}  // namespace perception
}  // namespace phoenix


#endif // PHOENIX_PERCEPTION_FILTER_SRR2_DETECTIONS_H_

