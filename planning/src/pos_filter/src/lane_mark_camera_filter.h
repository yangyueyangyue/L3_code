/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       lane_mark_camera_filter.h
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
#ifndef PHOENIX_POS_FILTER_LANE_MARK_CAMERA_FILTER_H_
#define PHOENIX_POS_FILTER_LANE_MARK_CAMERA_FILTER_H_

#include "utils/macros.h"
#include "math/matrix.h"
#include "geometry/aabboxkdtree2d.h"
#include "geometry/geometry_utils.h"
#include "curve/path.h"
#include "ad_msg.h"
#include "pos_filter.h"


namespace phoenix {
namespace pos_filter {


/**
 * @class LaneMarkCameraFilter
 * @brief 将相机识别的车道线数据转换为车道中心线，\n
 *        并使用相机识别的车道线进行估计当前车辆的相对位置
 */
class LaneMarkCameraFilter {
public:
  typedef Float32_t Scalar;

  /**
   * @enum 位置估计的质量
   */
  enum {
    FILTER_QUALITY_INVALID = 0,
    FILTER_QUALITY_BAD,
    FILTER_QUALITY_NOT_SO_GOOD,
    FILTER_QUALITY_GOOD
  };

public:
  /**
   * @brief 构造函数
   */
  LaneMarkCameraFilter();

  /**
   * @brief 析构函数
   */
  ~LaneMarkCameraFilter();

  void Configurate(const PosFilterConfig& conf);

  /**
   * @brief 清除内部数据
   */
  void Reset();

  /**
   * @brief 是否初始化好
   * @return true - 已初始化，false - 未初始化
   */
  bool IsInitialized() const {
    return (lane_boundary_line_seg_kdtree_.GetObjectsSize() > 0);
  }

  /**
   * @brief 使用车道线估计相对位置(相对于上一帧数据)
   * @param[in] lane_mark_list 相机识别的车道线数据
   * @param[in] delta_pos 相对于上一帧数据，估计的位置增量(通常使用车速及角速度估计)
   * @param[out] filtered_delta_pos 使用车道线数据修正后的相对位置(主要修正航向角的变换) 
   * @return true - 成功，false - 识别
   */
  bool Update(
      const ad_msg::LaneMarkCameraList& lane_mark_list,
      const Scalar delta_pos[3], Scalar filtered_delta_pos[3]);

  /**
   * @brief 将相机识别的车道线数据转换为车道中心线及车道边界
   * @param[in] lane_mark_list 相机识别的车道线数据
   * @param[in] delta_pos 相对于上一帧数据的位置增量
   * @return true - 成功，false - 识别
   */
  bool UpdateLaneLineList(const ad_msg::LaneMarkCameraList& lane_mark_list,
      const Scalar delta_pos[3], Scalar heading_corrected, Scalar veh_velocity,
      bool match_prev_lane_mark);

  /**
   * @brief 获取位置估计的质量
   * @return 位置估计的质量
   */
  inline Int32_t GetFilterQuality() const { return (filter_quality_); }

  /**
   * @brief 获取车道中心线列表(从相机识别的车道线中提取的)
   * @return 车道中心线列表
   */
  const ad_msg::LaneInfoCameraList& GetLaneInfoCameraList() const {
    return (lane_info_list_[curr_lane_info_list_index_]);
  }

private:
  /*
   * @struct LaneLineSegObj
   * @brief 由车道段构成的目标，供k-d树使用。\n
   *        lane segment object for k-d tree
   */
  struct LaneLineSegObj {
    // 车道编号，左边车道线为正值，右边车道线为负数;
    // (数值的绝对值按照距离当前车道的远近依次增加，
    // 第一条左边的车道线为1，第一条右边的车道线为-1)
    Int32_t lane_id;
    // 车道索引
    Int32_t lane_line_index;
    // 目标索引
    Int32_t point_index;

    /*
     * @brief 清除成员变量的值
     */
    void Clear() {
      lane_id = 0;
      lane_line_index = -1;
      point_index = -1;
    }

    /*
     * @brief 构造函数
     */
    LaneLineSegObj() {
      Clear();
    }
  };

  class CalcSquareDistToPt;
  friend class CalcSquareDistToPt;
  /*
   * @class CalcSquareDistToPt
   * @brief 计算点到车道线目标的平方距离
   */
  class CalcSquareDistToPt {
  public:
    /*
     * @brief 构造函数
     * @param[in]  point   待查询点
     * @param[in]  lane_lines  车道线
     */
    CalcSquareDistToPt(const common::Vec2d& point,
                       const ad_msg::LaneInfoCameraList& lanes)
      : point_(point), lane_info_list_(lanes) {
      // nothing to do
    }
    /*
     * @brief 重载的'()'运算符
     * @param[in]  obj             待查询的车道段目标
     * @param[in]  tree_obj_index  待查询的车道段目标在k-d树中对应的索引
     * @return 点到车道段目标对应线段的平方最近距离。车道段目标对应线段为\n
     *         索引为obj.lane_index的车道上第obj.object_index+1个点到\n
     *         第obj.object_index+2个点所组成的线段。
     */
    Scalar operator ()(
        const LaneLineSegObj& obj, Int32_t tree_obj_index) const {
      const ad_msg::LaneBoundaryLineCamera::CurvePoint& start_point_ref =
          lane_info_list_.boundary_lines[obj.lane_line_index].
          curve[obj.point_index];
      const ad_msg::LaneBoundaryLineCamera::CurvePoint& end_point_ref =
          lane_info_list_.boundary_lines[obj.lane_line_index].
          curve[obj.point_index+1];
      common::Vec2d start_point(start_point_ref.x, start_point_ref.y);
      common::Vec2d end_point(end_point_ref.x, end_point_ref.y);

      return (common::SqDistPointToSeg_2D(start_point, end_point, point_));
    }

  private:
    // 待查询点
    const common::Vec2d point_;
    const ad_msg::LaneInfoCameraList& lane_info_list_;
  };

  enum { MAX_LANE_LINE_SAMPLE_POINTS_NUM = common::Path::kMaxPathPointNum };
  class FuncGetPoints {
  public:
    FuncGetPoints(const common::StaticVector<common::Vec2d,
                  MAX_LANE_LINE_SAMPLE_POINTS_NUM>& pts)
      : points_(pts) {
    }

    const Scalar x(const Int32_t index) const { return (points_[index].x()); }
    const Scalar y(const Int32_t index) const { return (points_[index].y()); }
    const Scalar z(const Int32_t index) const { return (0.0F); }

  private:
    const common::StaticVector<common::Vec2d,
        MAX_LANE_LINE_SAMPLE_POINTS_NUM>& points_;
  };

  /* k006 pengc 2023-02-16 (begin) */
  // 平滑车道线，检测异常跳变的车道线
  struct InternalLaneInfo {
    enum { MAX_BOUNDARY_LINE_NUM = 4 };
    enum { MAX_CENTER_LINE_NUM = 3 };
    enum { MAX_CURVE_POINT_NUM = 500 };

    enum BoundaryLineIdx {
      BOUNDARY_LEFT = 0,
      BOUNDARY_RIGHT,
      BOUNDARY_LEFT_2,
      BOUNDARY_RIGHT_2
    };

    enum CenterLineIdx {
      CENTER_CURR = 0,
      CENTER_LEFT,
      CENTER_RIGHT
    };

    struct BoundaryLine {
      bool valid;

      Int32_t age;
      Float32_t forward_len;

      Float64_t coef[4];
      common::Path curve;

      void Clear() {
        valid = false;

        age = 0;
        forward_len = 0.0F;

        common::com_memset(coef, 0, sizeof(coef));
        curve.Clear();
      }
    };

    struct CenterLine {
      bool valid;

      Float32_t left_width;
      Float32_t right_width;

      void Clear() {
        valid = 0;

        left_width = 0.0F;
        right_width = 0.0F;
      }
    };


    bool valid;
    Int32_t timestamp;

    Float64_t common_curve_c1;
    Float64_t common_curve_c2;
    Float64_t common_curve_c3;

    Int32_t boundary_line_list_map[MAX_BOUNDARY_LINE_NUM];
    BoundaryLine boundary_line_list[MAX_BOUNDARY_LINE_NUM];
    Int32_t center_line_list_map[MAX_CENTER_LINE_NUM];
    CenterLine center_line_list[MAX_CENTER_LINE_NUM];

    InternalLaneInfo() {
      Clear();
    }

    void Clear() {
      valid = false;
      timestamp = 0;

      common_curve_c1 = 0.0;
      common_curve_c2 = 0.0;
      common_curve_c3 = 0.0;

      for (Int32_t i = 0; i < MAX_BOUNDARY_LINE_NUM; ++i) {
        boundary_line_list_map[i] = i;
        boundary_line_list[i].Clear();
      }
      for (Int32_t i = 0; i < MAX_CENTER_LINE_NUM; ++i) {
        center_line_list_map[i] = i;
        center_line_list[i].Clear();
      }
    }

    inline BoundaryLine& boundary_line(BoundaryLineIdx idx) {
      return (boundary_line_list[boundary_line_list_map[idx]]);
    }
    void ShiftBoundaryLineList(bool direction) {
      if (direction) {
        // move to left
        Int32_t idx = boundary_line_list_map[BOUNDARY_LEFT_2];
        boundary_line_list_map[BOUNDARY_LEFT_2] =
            boundary_line_list_map[BOUNDARY_LEFT];
        boundary_line_list_map[BOUNDARY_LEFT] =
            boundary_line_list_map[BOUNDARY_RIGHT];
        boundary_line_list_map[BOUNDARY_RIGHT] =
            boundary_line_list_map[BOUNDARY_RIGHT_2];
        boundary_line_list_map[BOUNDARY_RIGHT_2] = idx;
        boundary_line_list[idx].valid = false;
      } else {
        // move to right
        Int32_t idx = boundary_line_list_map[BOUNDARY_RIGHT_2];
        boundary_line_list_map[BOUNDARY_RIGHT_2] =
            boundary_line_list_map[BOUNDARY_RIGHT];
        boundary_line_list_map[BOUNDARY_RIGHT] =
            boundary_line_list_map[BOUNDARY_LEFT];
        boundary_line_list_map[BOUNDARY_LEFT] =
            boundary_line_list_map[BOUNDARY_LEFT_2];
        boundary_line_list_map[BOUNDARY_LEFT_2] = idx;
        boundary_line_list[idx].valid = false;
      }
    }

    inline CenterLine& center_line(CenterLineIdx idx) {
      return (center_line_list[center_line_list_map[idx]]);
    }
    void ShiftCenterLineList(bool direction) {
      if (direction) {
        // move to left
        Int32_t idx = center_line_list_map[CENTER_LEFT];
        center_line_list_map[CENTER_LEFT] =
            center_line_list_map[CENTER_CURR];
        center_line_list_map[CENTER_CURR] =
            center_line_list_map[CENTER_RIGHT];
        center_line_list_map[CENTER_RIGHT] = idx;
        center_line_list[idx].valid = false;
        center_line(CENTER_CURR).valid = false;
      } else {
        // move to right
        Int32_t idx = center_line_list_map[CENTER_RIGHT];
        center_line_list_map[CENTER_RIGHT] =
            center_line_list_map[CENTER_CURR];
        center_line_list_map[CENTER_CURR] =
            center_line_list_map[CENTER_LEFT];
        center_line_list_map[CENTER_LEFT] = idx;
        center_line_list[idx].valid = false;
        center_line(CENTER_CURR).valid = false;
      }
    }
  };
  /* k006 pengc 2023-02-16 (end) */

private:
  bool SmoothLaneMark(
      const ad_msg::LaneMarkCameraList& lane_mark_list,
      const common::Matrix<Scalar, 3, 3>& mat_conv_new_to_old);

  void SmoothFirstCoefOfLaneMark(
      const ad_msg::LaneMarkCameraList& lane_mark_list,
      const common::Matrix<Scalar, 3, 3>& mat_conv_new_to_old,
      Int32_t left_mark_idx, Int32_t right_mark_idx,
      Int32_t left_2_mark_idx, Int32_t right_2_mark_idx);

  bool SmoothCommonCoefOfLaneMark(
      const ad_msg::LaneMarkCameraList& lane_mark_list,
      Int32_t left_mark_idx,
      Int32_t right_mark_idx,
      Float64_t* corrected_c1,
      Float64_t* corrected_c2,
      Float64_t* corrected_c3);

  Int32_t DetectLaneChangedEvent(
      const ad_msg::LaneMarkCamera* left_mark,
      const ad_msg::LaneMarkCamera* right_mark,
      const InternalLaneInfo::BoundaryLine* old_left_mark,
      const InternalLaneInfo::BoundaryLine* old_right_mark,
      const common::PathPoint& left_mark_proj_on_old_left_mark,
      const common::PathPoint& left_mark_proj_on_old_right_mark,
      const common::PathPoint& right_mark_proj_on_old_left_mark,
      const common::PathPoint& right_mark_proj_on_old_right_mark) const;

  void SmoothLatOffsetOfLaneMark(
      const Char_t* mark_description,
      Scalar lane_mark_offset,
      const ad_msg::LaneMarkCamera* new_lane_mark,
      InternalLaneInfo::BoundaryLine* old_lane_mark);

  void CalcLaneWidthByLaneMark();

  /*
   * @brief 将当前车道线数据与前一帧的车道线数据进行对齐
   * @param[in] lane_mark_list 当前相机识别的车道线数据
   * @param[out] filtered_delta_pos 对齐后的前后两帧之间的位置增量
   * @return true - 成功, false - 失败
   */
  bool RegisterFramesOfLaneLines(
      const ad_msg::LaneMarkCameraList& lane_mark_list,
      Scalar filtered_delta_pos[3]);

  /*
   * @brief 使用所有车道线构建k-d树（用来加快最近点的查找速度）
   */
  void BuildLaneLineSegKDTree(const ad_msg::LaneInfoCameraList& lane_info_list);

  /*
   * @brief 根据车道边线ID查找对应的车道边线
   * @param[in] id 车道边线ID
   * @param[in] lane_line_list 车道边线列表
   * @return 找到的车道边线在边线列表中的索引(-1 表示没有找到)
   */
  Int32_t FindLaneLineById(Int32_t id,
      const ad_msg::LaneInfoCameraList& lane_info_list) const;

  /*
   * @brief 使用相机识别的车道线数据构建车道中心线
   * @param[in] lane_line_list 相机识别的车道线数据
   */
  void ConstructLaneCenterLines(
      ad_msg::LaneInfoCameraList& lane_info_list);

  void ConstructCurrentLaneCenterLines(
      const common::Path& left_path,const common::Path& right_path,
      Int32_t* point_index, ad_msg::LaneCenterLineCamera* central_line);

  void ConstructCenterLinesFromPath(
      const Float32_t sample_l, const common::Path &path,
      Int32_t* point_index, ad_msg::LaneCenterLineCamera* central_line);

  void CalaCenterLinesPoints(
      Int32_t start_index, const common::Path& long_path, const Float32_t sample_l,
      const Float32_t left_width, const Float32_t right_width,
      Int32_t* point_index, ad_msg::LaneCenterLineCamera* central_line);

private:
  common::Matrix<Scalar, 3, 3> mat_camera_calibration_;

  Int32_t filter_quality_;
  Int32_t raw_lane_list_idx_;
  ad_msg::LaneInfoCameraList raw_lane_list_[2];
  common::AABBoxKDTree2d<LaneLineSegObj,
      ad_msg::LaneBoundaryLineCamera::MAX_CURVE_POINT_NUM *
      ad_msg::LaneInfoCameraList::MAX_LANE_BOUNDARY_LINE_NUM, 16>
      raw_lane_mark_seg_kdtree_;

  Int32_t curr_lane_info_list_index_;
  ad_msg::LaneInfoCameraList lane_info_list_[2];
  // 用于加快最近点查询等功能的k-d树
  common::AABBoxKDTree2d<LaneLineSegObj,
      ad_msg::LaneBoundaryLineCamera::MAX_CURVE_POINT_NUM *
      ad_msg::LaneInfoCameraList::MAX_LANE_BOUNDARY_LINE_NUM, 16>
      lane_boundary_line_seg_kdtree_;

  /* k006 pengc 2023-02-16 (begin) */
  // 平滑车道线，检测异常跳变的车道线
  InternalLaneInfo internal_lane_info_;
  /* k006 pengc 2023-02-16 (end) */

  /*
   * @struct TemporaryData
   * @brief 临时数据(程序运行过程中需要大块的临时数据)
   */
  struct TemporaryData {
    // 车道栈存储结点列表，在构建车道k-d树时需要用到的k-d树的内部变量。
    // used for k-d tree
    common::StaticVector<common::AABBoxKDTreeNodeAssociation, 64>
        tree_nodes_stack;

    common::StaticVector<common::Vec2d,
        MAX_LANE_LINE_SAMPLE_POINTS_NUM> sample_points[2];

    common::StaticVector<common::PathPoint,
        common::Path::kMaxPathPointNum> sample_path_points[1];

    common::Path path[4];
  } temp_data_;
};


}  // namespace pos_filter
}  // namespace phoenix


#endif // PHOENIX_POS_FILTER_LANE_MARK_CAMERA_FILTER_H_

