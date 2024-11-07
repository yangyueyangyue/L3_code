/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       path.cc
 * @brief      路径处理
 * @details    实现了部分路径处理方法（最近点查询、相交判断，采样等）
 *
 * @author     boc
 * @date       2020.05.14
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/14  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/

#include "curve/path.h"
#include "geometry/geometry_utils.h"
#include "utils/linear_interpolation.h"

namespace phoenix {
namespace common {

/// 认为是弯道时的最小曲率 （单位: m^(-1)）
/// 最小曲率0.003米^(-1) 等价于 最大转弯半径333米
// const geo_var_t Path::kCriticalCurvature = 0.003F;
/// 最小曲率0.002米^(-1) 等价于 最大转弯半径500米
const geo_var_t Path::kCriticalCurvature = 0.002F;

class CalcSquareDistToPt {
public:
  CalcSquareDistToPt(
      const Vec2d& point_in,
      const StaticVector<Vec2d, Path::kMaxPathPointNum>& point_list) :
    point_(point_in), points_(point_list) {}
  geo_var_t operator ()(Int32_t index, Int32_t tree_obj_index) const {
    return SqDistPointToSeg_2D(points_[index],
                               points_[index + 1], point_);
  }
private:
  const Vec2d point_;
  const StaticVector<Vec2d, Path::kMaxPathPointNum>& points_;
};

/**
 * @date       2020.05.14
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/14  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Path::Path(geo_var_t min_valid_seg_dist_in,
           geo_var_t max_valid_delta_heading_in) :
    min_valid_seg_dist_(min_valid_seg_dist_in),
    max_valid_delta_heading_(max_valid_delta_heading_in) {
  total_length_ = 0.0F;
}

/**
 * @date       2020.05.14
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/14  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Path::~Path() {
}

/**
 * @date       2020.05.14
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/14  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool Path::Construct(const StaticVector<Vec2d, kMaxPathPointNum>& points_in) {
  const Int32_t points_size_tmp = points_in.Size();
  if (points_size_tmp < 2) {
    return (false);
  }

  // 清理内部成员变量
  Clear();

  // 添加的路径点的索引
  Int32_t index = 0;
  // 当前构建点与上一个路径点的距离
  geo_var_t dist = 0.0F;
  // 当前构建点的航向角（单位：弧度）
  geo_var_t heading = 0.0F;
  // 当前构建点与上一个路径点航向角的差值
  geo_var_t delta_heading = 0.0F;
  for (Int32_t i = 0; i < points_size_tmp; ++i) {
    const Vec2d& point = points_in[i];
    if (0 == index) {
      points_.PushBack(point);
      headings_.PushBack(0.0F);
      curvatures_.PushBack(0.0F);
      accumulated_s_.PushBack(0.0F);
      ++index;
    } else  {
      dist = point.DistanceTo(points_[index-1]);
      if (dist < min_valid_seg_dist_) {
        // 舍弃过近的点
#if 0
        LOG_INFO(5) << "distance(" << dist << ") < min_valid_seg_dist("
                    << min_valid_seg_dist_ << ").";
#endif
        continue;
      }
      // 计算航向角
      heading = com_atan2(point.y() - points_[index-1].y(),
          point.x() - points_[index - 1].x());
      if (index > 1) {
        delta_heading = phoenix::common::NormalizeAngle(
            heading - headings_[index - 2]);
      } else {
        delta_heading = 0.0F;
      }
      if (com_abs(delta_heading) > max_valid_delta_heading_) {
        // 舍弃航向角变化过大的点
#if 0
        LOG_INFO(5) << "abs delta_heading("
                    << phoenix::common::com_rad2deg(delta_heading)
                    << ") > max_valid_delta_heading("
                    << phoenix::common::com_rad2deg(max_valid_delta_heading_)
                    << ").";
#endif
        continue;
      }
      points_.PushBack(point);
      headings_[index - 1] = heading;
      accumulated_s_.PushBack(dist + accumulated_s_[index - 1]);
      headings_.PushBack(0.0F);
      curvatures_.PushBack(0.0F);
      ++index;
    }
  }
  if (index < 1) {
    return (false);
  }

  if (index > 1) {
    // 设置最后一个点的航向角
    headings_[index - 1] = headings_[index - 2];
  }
  total_length_ = accumulated_s_.Back();

  if (points_.Size() < 2) {
    LOG_ERR << "Points size " << points_.Size() << " too less.";
    return (false);
  }

  // 计算曲率
  for (Int32_t i = 0; i < index; ++i) {
    curvatures_[i] = SmoothCurvature(i);
  }
//  for (Int32_t i = 0; i < index; ++i) {
//    curvatures_[i] = SmoothCurvature2(i);
//  }
  // 对首尾的曲率进行处理
  if (index > 1) {
    // 设置最后一个点的曲率
    curvatures_[index - 1] = curvatures_[index - 2];
    // 设置第一个点的曲率
    curvatures_[0] = curvatures_[1];
  }

  // 对组成曲线的所有线段创建k-d树
  CreateKDTree();

  return (true);
}

/**
 * @date       2020.05.14
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/14  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool Path::Construct(
    const StaticVector<PathPoint, kMaxPathPointNum>& path_points) {
  const Int32_t path_points_size = path_points.Size();
  if (path_points_size < 2) {
    return (false);
  }

  Clear();

  Int32_t index = 0;
  geo_var_t dist = 0.0F;
  geo_var_t heading = 0.0F;
  geo_var_t delta_heading = 0.0F;
  for (Int32_t i = 0; i < path_points_size; ++i) {
    const PathPoint& point = path_points[i];
    if (0 == index) {
      points_.PushBack(point.point);
      headings_.PushBack(point.heading);
      curvatures_.PushBack(point.curvature);
      accumulated_s_.PushBack(0.0F);
      ++index;
    } else  {
      dist = point.point.DistanceTo(points_[index-1]);
      if (dist < min_valid_seg_dist_) {
        LOG_WARN << "distance(" << dist <<
                    ") < min_valid_seg_dist(" << min_valid_seg_dist_ << ").";
        continue;
      }
      // heading = point.heading; （不信任地图的航向角）
      heading = com_atan2(point.point.y() - points_[index - 1].y(),
          point.point.x() - points_[index - 1].x());
      if (index > 1) {
        delta_heading = phoenix::common::NormalizeAngle(
            heading - headings_[index - 2]);
      } else {
        delta_heading = 0.0F;
      }
      if (com_abs(delta_heading) > max_valid_delta_heading_) {
        LOG_WARN << "abs delta_heading(" <<
                    phoenix::common::com_rad2deg(delta_heading) <<
                    ") > max_valid_delta_heading(" <<
                    phoenix::common::com_rad2deg(max_valid_delta_heading_) <<
                    ").";
        continue;
      }
      points_.PushBack(point.point);
      headings_[index - 1] = heading;
      headings_.PushBack(point.heading);
      accumulated_s_.PushBack(dist+accumulated_s_[index-1]);
      curvatures_.PushBack(point.curvature);
      ++index;
    }
  }
  if (index < 1) {
    return (false);
  }
  if (index > 1) {
    // 设置最后一个点的航向角
    headings_[index - 1] = headings_[index - 2];
  }

  total_length_ = accumulated_s_.Back();

  if (points_.Size() < 2) {
    LOG_ERR << "Points size " << points_.Size() << " too less.";
    return (false);
  }

  // 计算曲率
  geo_var_t smooth_range = 3.0F;
  for (Int32_t i = 0; i < index; ++i) {
    smooth_range = 3.0F;
    if (com_abs(curvatures_[i]) < 0.01F) {
      // 直线上使用更大的范围计算曲率
      smooth_range = 5.0F;
    }
    curvatures_[i] = SmoothCurvature(i, smooth_range, smooth_range);
  }
//  for (Int32_t i = 0; i < index; ++i) {
//    smooth_range = 3.0F;
//    if (com_abs(curvatures_[i]) < 0.01F) {
//      // 直线上使用更大的范围计算曲率
//      smooth_range = 5.0F;
//    }
//    curvatures_[i] = SmoothCurvature2(i, smooth_range, smooth_range);
//  }
  // 对首尾的曲率进行处理
  if (index > 1) {
    // 设置最后一个点的曲率
    curvatures_[index -1] = curvatures_[index - 2];
    // 设置第一个点的曲率
    curvatures_[0] = curvatures_[1];
  }

  // 对组成曲线的所有线段创建k-d树
  CreateKDTree();

  return (true);
}

/**
 * @date       2020.04.18
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/04/18  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void Path::CreateKDTree() {
  AABBoxKDTreeParams params;
  params.max_leaf_dimension = 5.0f;  // meters.
  params.max_leaf_size = 16;

  StaticVector<AABBoxKDTreeNodeAssociation, 32> tree_nodes_stack;
  lane_segment_kdtree_.SetKDTreeParams(params);
  lane_segment_kdtree_.Clear();
  for (Int32_t i = 0; (i + 1) < points_.Size(); ++i) {
    AABBox2d box(points_[i], points_[i + 1]);
    lane_segment_kdtree_.AddObject(i, box);
  }
  lane_segment_kdtree_.Partition(tree_nodes_stack);
}

/*
 * @date       2020.05.14
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/14  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
geo_var_t Path::SmoothCurvature(
    Int32_t index, geo_var_t left_range, geo_var_t right_range) {
  geo_var_t curvature = curvatures_[index];
  const Int32_t points_size_tmp = points_.Size();
  Int32_t index_left = index;
  Int32_t index_right = index;
  for (; index_left > 0; --index_left) {
    if ((accumulated_s_[index] - accumulated_s_[index_left]) > left_range) {
      break;
    }
  }
  if ((index_left + 1) < index) {
    index_left++;
  }
  for (; (index_right + 1) < points_size_tmp; ++index_right) {
    if ((accumulated_s_[index_right] - accumulated_s_[index]) > right_range) {
      break;
    }
  }
  if (index_right > (index + 1)) {
    index_right--;
  }
  if ((index_right > index) && (index > index_left)) {
#if 0
    // 使用外接圆半径的方式计算曲率
    const Vec2d vertex_a = points_[index_left];
    const Vec2d vertex_b = points_[index];
    const Vec2d vertex_c = points_[index_right];
    geo_var_t radius_turning = 0.0F;  // 转弯半径，单位：米
    const Int32_t calc_result = ObtainCircumcircleRadius(vertex_a, vertex_b,
        vertex_c, &radius_turning);
    if (calc_result == 1) {
      const geo_var_t delta_heading = phoenix::common::NormalizeAngle(
            headings_[index_right] - headings_[index_left]);
      curvature = 1.0F / radius_turning;
      if (delta_heading < 0.0F) {
        curvature = -curvature;
      }
    } else {
      curvature = 0.0F;
    }
#else
    curvature = NormalizeAngle(headings_[index_right] - headings_[index_left])
        / (accumulated_s_[index_right] - accumulated_s_[index_left]);
#endif
  } else if (index_right > index_left) {
    curvature = NormalizeAngle(headings_[index_right] - headings_[index_left])
        / (accumulated_s_[index_right] - accumulated_s_[index_left]);
  } else {
    // nothing to do
  }

  return (curvature);
}

geo_var_t Path::SmoothCurvature2(
    Int32_t index, geo_var_t left_range, geo_var_t right_range) {
  geo_var_t curvature = curvatures_[index];
  const Int32_t points_size_tmp = points_.Size();
  Int32_t index_left = index;
  Int32_t index_right = index;
  for (; index_left > 0; --index_left) {
    if ((accumulated_s_[index] - accumulated_s_[index_left]) > left_range) {
      break;
    }
  }
  if ((index_left + 1) < index) {
    index_left++;
  }
  for (; (index_right + 1) < points_size_tmp; ++index_right) {
    if ((accumulated_s_[index_right] - accumulated_s_[index]) > right_range) {
      break;
    }
  }
  if (index_right > (index + 1)) {
    index_right--;
  }
  if (index_right > index_left) {
    geo_var_t sum = 0;
    for (Int32_t i = index_left; i <= index_right; ++i) {
      sum += curvatures_[i];
    }
    curvature = sum / (index_right - index_left + 1);
  }

  return (curvature);
}

/**
 * @date       2020.05.14
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/14  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool Path::FindNearest(const phoenix::common::Vec2d& p,
                       PathPoint * const path_point_nearest) const {
  const Int32_t points_size_tmp = points_.Size();
  if (points_size_tmp < 2) {
    LOG_ERR << "Can't find nearest point from Path, because points size " <<
               points_size_tmp << " not enough.";
    return (false);
  }

  StaticVector<AABBoxKDTreeNodeAssociation, 16> tree_nodes_stack;
  Int32_t index = 0;
  geo_var_t min_distance_sqr = 0.0F;
  lane_segment_kdtree_.FindNearest(tree_nodes_stack,
                                   p,
                                   CalcSquareDistToPt(p, points_),
                                   &index,
                                   &min_distance_sqr);
  if ((index < 0) || (index > (points_size_tmp - 2))) {
    LOG_ERR << "Illegal nearest segment index(" << index << ") "
             "when find nearest point on path.";
    return false;
  }

  return (GetSmoothNearestPointOnSegment(p, index, path_point_nearest));
}

/**
 * @date       2020.05.14
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/14  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool Path::GetSmoothNearestPointOnSegment(
    const Vec2d& p, const Int32_t index_low,
    PathPoint * const path_point_nearest) const {
  const Int32_t points_size_tmp = points_.Size();
  if ((index_low < 0) || (index_low > points_size_tmp - 2)) {
    LOG_ERR << "Invalid parameter index_low=" << index_low;
    return (false);
  }

  geo_var_t t = 0.0F;
  phoenix::common::ClosestPtPointToSeg_2D(points_[index_low],
      points_[index_low + 1], p, &(path_point_nearest->point), &t);
  const geo_var_t squared_dist = p.DistanceSquareTo(path_point_nearest->point);
  Int32_t orientation_on_path = 0;

  path_point_nearest->s = (1.0F - t) * accumulated_s_[index_low] +
      t * accumulated_s_[index_low + 1];
  orientation_on_path = phoenix::common::Orient_2D(
        points_[index_low], points_[index_low + 1], p);

  if (((path_point_nearest->s - accumulated_s_[index_low]) > 2.0F) &&
      ((accumulated_s_[index_low + 1] - path_point_nearest->s) > 2.0F)) {
    path_point_nearest->heading = headings_[index_low];
    path_point_nearest->curvature = curvatures_[index_low];
  } else {
    path_point_nearest->heading = AngleLerp(headings_[index_low],
                                    headings_[index_low + 1], t);
    path_point_nearest->curvature = (1.0F - t) * curvatures_[index_low] +
        t * curvatures_[index_low + 1];
  }

  if (orientation_on_path < 0) {
    // right side
    path_point_nearest->l = -com_sqrt(squared_dist);
  } else {
    // left side
    path_point_nearest->l = com_sqrt(squared_dist);
  }

  return true;
}

/**
 * @date       2020.05.15
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/15  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool Path::FindProjection(const phoenix::common::Vec2d& p,
                          PathPoint * const path_point_projection) const {
  const Int32_t points_size_tmp = points_.Size();
  if (points_size_tmp < 2) {
    LOG_ERR << "Can't get projection point from "
                "Path, because points size " << points_size_tmp <<
                " not enough";
    return (false);
  }

  StaticVector<AABBoxKDTreeNodeAssociation, 16> tree_nodes_stack;
  Int32_t index = 0;
  geo_var_t min_distance_sqr = 0.0F;
  lane_segment_kdtree_.FindNearest(tree_nodes_stack,
                                   p,
                                   CalcSquareDistToPt(p, points_),
                                   &index,
                                   &min_distance_sqr);
  if ((index < 0) || (index > (points_size_tmp - 2))) {
    LOG_ERR << "Illegal nearest segment index(" << index <<
               ") when find projection.";
    return false;
  }

  return (GetSmoothPojectivePointOnSegment(p, index, path_point_projection));
}


bool Path::FindProjection(const phoenix::common::Vec2d& p, Int32_t* index,
                 PathPoint * const path_point_projection) const {
    const Int32_t points_size_tmp = points_.Size();
    if (points_size_tmp < 2) {
      LOG_ERR << "Can't get projection point from "
                  "Path, because points size " << points_size_tmp <<
                  " not enough";
      return (false);
    }

    StaticVector<AABBoxKDTreeNodeAssociation, 16> tree_nodes_stack;
    //index = 0;
    geo_var_t min_distance_sqr = 0.0F;
    lane_segment_kdtree_.FindNearest(tree_nodes_stack,
                                     p,
                                     CalcSquareDistToPt(p, points_),
                                     index,
                                     &min_distance_sqr);
    if ((*index < 0) || (*index > (points_size_tmp - 2))) {
      LOG_ERR << "Illegal nearest segment index(" << *index <<
                 ") when find projection.";
      return false;
    }

    return (GetSmoothPojectivePointOnSegment(p, *index, path_point_projection));
}

/**
 * @date       2020.05.15
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/15  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool Path::GetSmoothPojectivePointOnSegment(
    const Vec2d& p, const Int32_t index_low,
    PathPoint * const path_point_out) const {
  const Int32_t points_size_tmp = points_.Size();
  if ((index_low < 0) || (index_low > points_size_tmp - 2)) {
    LOG_ERR << "Invalid parameter index_low=" << index_low;
    return (false);
  }

  geo_var_t t = 0.0F;
  phoenix::common::ClosestPtPointToLine_2D(points_[index_low],
      points_[index_low + 1], p, &(path_point_out->point), &t);
  geo_var_t squared_dist = p.DistanceSquareTo(path_point_out->point);
  Int32_t orientation_on_path = phoenix::common::Orient_2D(
      points_[index_low], points_[index_low + 1], p);

  static const geo_var_t kMinSegDistWhenProjOutOfEnd = 2.0F;
  if ((index_low < 1) && (t < 0.0F)) {
    // before start point of path
    if (accumulated_s_[1] > kMinSegDistWhenProjOutOfEnd) {
      path_point_out->heading = headings_[0];
      path_point_out->curvature = curvatures_[0];
      path_point_out->s = t * accumulated_s_[1];
    } else {
      Int32_t next_index = 1;
      for (; next_index < (points_size_tmp-1); ++next_index) {
        if (accumulated_s_[next_index] > kMinSegDistWhenProjOutOfEnd) {
          break;
        }
      }
      phoenix::common::ClosestPtPointToLine_2D(points_[0],
          points_[next_index], p, &(path_point_out->point), &t);
      squared_dist = p.DistanceSquareTo(path_point_out->point);
      orientation_on_path = phoenix::common::Orient_2D(
          points_[0], points_[next_index], p);

      path_point_out->heading =
          com_atan2(points_[next_index].y() - points_[0].y(),
          points_[next_index].x() - points_[0].x());
      path_point_out->curvature = curvatures_[0];
      path_point_out->s = t * accumulated_s_[next_index];
    }
  } else if ((index_low >= (points_size_tmp-2)) && (t > 1.0F)) {
    // after end point of path
    if ((accumulated_s_.Back() - accumulated_s_[points_size_tmp-2]) >
        kMinSegDistWhenProjOutOfEnd) {
      path_point_out->heading = headings_.Back();
      path_point_out->curvature = curvatures_.Back();
      path_point_out->s = accumulated_s_[index_low] +
          t * (accumulated_s_[index_low + 1] - accumulated_s_[index_low]);
    } else {
      Int32_t next_index = points_size_tmp-2;
      for (; next_index > 0; --next_index) {
        if ((accumulated_s_.Back() - accumulated_s_[next_index]) >
            kMinSegDistWhenProjOutOfEnd) {
          break;
        }
      }
      phoenix::common::ClosestPtPointToLine_2D(points_[next_index],
          points_.Back(), p, &(path_point_out->point), &t);
      squared_dist = p.DistanceSquareTo(path_point_out->point);
      orientation_on_path = phoenix::common::Orient_2D(
          points_[next_index], points_.Back(), p);

      path_point_out->heading =
          com_atan2(points_.Back().y() - points_[next_index].y(),
          points_.Back().x() - points_[next_index].x());
      path_point_out->curvature = curvatures_.Back();
      path_point_out->s = accumulated_s_[next_index] +
          t * (accumulated_s_.Back() - accumulated_s_[next_index]);
    }
  } else {
    path_point_out->s = accumulated_s_[index_low] +
        t * (accumulated_s_[index_low + 1] - accumulated_s_[index_low]);

    if (((path_point_out->s - accumulated_s_[index_low]) > 2.0F) &&
       ((accumulated_s_[index_low+1] - path_point_out->s) > 2.0F)) {
      path_point_out->heading = headings_[index_low];
      path_point_out->curvature = curvatures_[index_low];
    } else {
      path_point_out->heading = AngleLerp(headings_[index_low],
                                      headings_[index_low + 1], t);
      path_point_out->curvature = (1.0F - t) * curvatures_[index_low] +
          t * curvatures_[index_low + 1];
    }
  }

  if (orientation_on_path < 0) {
    // right side
    path_point_out->l = -com_sqrt(squared_dist);
  } else {
    // left side
    path_point_out->l = com_sqrt(squared_dist);
  }

  return true;
}

/**
 * @date       2020.05.19
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/19  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool Path::FindSmoothPoint(geo_var_t s,
                           PathPoint * const path_point_out) const {
  Int32_t low_index = -1;
  return (FindSmoothPoint(s, &low_index, path_point_out));
}

/**
 * @date       2020.09.28
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/09/28  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Vec2d Path::SLToXY(geo_var_t s, geo_var_t l) const {
  PathPoint path_point;

  FindSmoothPoint(s, &path_point);

  return (Vec2d(path_point.point.x() - l * com_sin(path_point.heading),
                path_point.point.y() + l * com_cos(path_point.heading)));
}

/**
 * @date       2020.05.19
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/19  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void Path::GetSamplePoints(
    StaticVector<PathPoint, kMaxPathPointNum> * const sample_points) const {
  PathPoint* path_point_vertex = Nullptr_t;
  for (Int32_t i = 0; i < points_.Size(); ++i) {
    path_point_vertex = sample_points->Allocate();
    if (Nullptr_t != path_point_vertex) {
      path_point_vertex->point = points_[i];
      path_point_vertex->heading = headings_[i];
      path_point_vertex->curvature = curvatures_[i];
      path_point_vertex->s = accumulated_s_[i];
      path_point_vertex->l = 0.0F;
    } else {
      LOG_ERR << "There are not enough space to store points.";
      break;
    }
  }
}

/**
 * @date       2020.05.19
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/19  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool Path::GetSamplePoints(
    geo_var_t start_s, geo_var_t end_s,
    StaticVector<PathPoint, kMaxPathPointNum> * const sample_points) const {
  if ((end_s - start_s) < kMathEpsilonF) {
    LOG_ERR << "Invalid parameters start_s (" << start_s
            << ") and end_s(" << end_s << ").";
    return false;
  }

  Int32_t start_low_index = -1;
  Int32_t end_low_index = -1;
  PathPoint start_path_point;
  PathPoint end_path_point;

  if (!FindSmoothPoint(start_s, &start_low_index, &start_path_point)) {
    LOG_ERR << "Failed to find start point.";
    return false;
  }
  if (!FindSmoothPoint(end_s, &end_low_index, &end_path_point)) {
    LOG_ERR << "Failed to find end point.";
    return false;
  }

  PathPoint *path_point_sample = sample_points->Allocate();
  if (Nullptr_t == path_point_sample) {
    LOG_ERR << "There are not enough space to store points.";
    return false;
  }
  *path_point_sample = start_path_point;

  Int32_t next_index = start_low_index + 1;
  if (next_index < end_low_index) {
    if ((accumulated_s_[next_index] - start_path_point.s) >
        min_valid_seg_dist_) {
      path_point_sample = sample_points->Allocate();
      if (Nullptr_t != path_point_sample) {
        path_point_sample->point = points_[next_index];
        path_point_sample->heading = headings_[next_index];
        path_point_sample->curvature = curvatures_[next_index];
        path_point_sample->s = accumulated_s_[next_index];
        path_point_sample->l = 0.0F;
      }
    }
    for (++next_index; next_index < end_low_index; ++next_index) {
      path_point_sample = sample_points->Allocate();
      if (Nullptr_t != path_point_sample) {
        path_point_sample->point = points_[next_index];
        path_point_sample->heading = headings_[next_index];
        path_point_sample->curvature = curvatures_[next_index];
        path_point_sample->s = accumulated_s_[next_index];
        path_point_sample->l = 0.0F;
      } else {
        LOG_ERR << "There are not enough space to store points.";
        break;
      }
    }
  }

  if (((accumulated_s_[end_low_index] - start_path_point.s) >
       min_valid_seg_dist_) &&
      ((end_path_point.s - accumulated_s_[end_low_index]) >
       min_valid_seg_dist_)) {
    path_point_sample = sample_points->Allocate();
    if (Nullptr_t != path_point_sample) {
      path_point_sample->point = points_[end_low_index];
      path_point_sample->heading = headings_[end_low_index];
      path_point_sample->curvature = curvatures_[end_low_index];
      path_point_sample->s = accumulated_s_[end_low_index];
      path_point_sample->l = 0.0F;
    } else {
      LOG_ERR << "There are not enough space to store points.";
    }
  }

  path_point_sample = sample_points->Allocate();
  if (Nullptr_t != path_point_sample) {
    *path_point_sample = end_path_point;
  } else {
    LOG_ERR << "There are not enough space to store points.";
  }

  return true;
}

/**
 * @date       2021.06.23
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/23  <td>1.0      <td>pengc     <td>First edition
 * </table>
 */
void Path::CalcSamplePointsByLatOffsetting(
    geo_var_t lat_offset,
    StaticVector<Vec2d, kMaxPathPointNum>* const sample_points) const {
  sample_points->Clear();

  Int32_t points_num = points_.Size();

  Vec2d prev_seg_start;
  Vec2d prev_seg_end;

  for (Int32_t i = 0; i < points_num; ++i) {
    Vec2d* new_p = sample_points->Allocate();
    if (Nullptr_t == new_p) {
      LOG_ERR << "There are not enough space to store points.";
      break;
    }

    const Vec2d& cur_p = points_[i];
    geo_var_t heading = headings_[i];
    new_p->set_x(cur_p.x() - lat_offset * com_sin(heading));
    new_p->set_y(cur_p.y() + lat_offset * com_cos(heading));

    if (i > 0) {
      if (OverlapTestSegToSeg_2D(prev_seg_start, prev_seg_end, cur_p, *new_p)) {
        // 跟之前的线段存在交叉，忽略这个点
        sample_points->PopBack();
      } else {
        prev_seg_start = cur_p;
        prev_seg_end = *new_p;
      }
    } else {
      prev_seg_start = cur_p;
      prev_seg_end = *new_p;
    }
  }
}

/**
 * @date       2020.06.13
 * @version    v1.1
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author <th>Description
 * <tr><td>2020/05/21  <td>1.0      <td>boc    <td>First edition
 * <tr><td>2020/06/13  <td>1.1      <td>boc    <td>当采样点位于路径的延长线上\n
 *     时，对采样点的航向角和曲率进行特殊处理。
 * </table>
 */
bool Path::UniformlySamplePathForward(
    geo_var_t start_s, Int32_t sample_num, geo_var_t seg_len,
    StaticVector<PathPoint, kMaxPathPointNum>* const sample_points) const {
  const Int32_t nodes_size = points_.Size();

  if (nodes_size < 2) {
    return (false);
  }

  Int32_t span = 0;

  geo_var_t t = 0.0F;
  PathPoint point;
  if (start_s < 0.0F) {
    // 起始采样点在路径起点的后面
    span = 0;
    t = (start_s - accumulated_s_[span])
        / (accumulated_s_[span + 1] - accumulated_s_[span]);
    point.point = (1.0F - t) * points_[span] + t * points_[span + 1];
    point.heading = headings_[0];
    point.curvature = 0.0F;
    point.s = start_s;
    point.l = 0.0F;
    sample_points->PushBack(point);
  } else if (start_s > total_length_) {
    // 起始采样点在路径终点的前面
    span = nodes_size -1;
    t = (start_s - accumulated_s_[span - 1])
        / (accumulated_s_[span] - accumulated_s_[span - 1]);
    point.point = (1.0F - t) * points_[span - 1] + t * points_[span];
    point.heading = headings_[span];
    point.curvature = 0.0F;
    point.s = start_s;
    point.l = 0.0F;
    sample_points->PushBack(point);
  } else {
    // 起始采样点在路径上
    const Int32_t index = accumulated_s_.UpperBound(start_s, FuncCmpArcLen());
    if (index > (nodes_size - 1)) {
      span = nodes_size - 1;
    } else {
      if (index > 0) {
        span = index - 1;
      } else {
        span = 0;
      }
    }

    if ((span + 1) < nodes_size) {
      // 采样点在路径起点和路径终点的中间
      t = (start_s - accumulated_s_[span])
          / (accumulated_s_[span + 1] - accumulated_s_[span]);
      point.point = (1.0F - t) * points_[span] + t * points_[span + 1];
      if (((start_s - accumulated_s_[span]) > 2.0F) &&
         ((accumulated_s_[span+1] - start_s) > 2.0F)) {
        // 采样点在一条很长的路径的线段中间，直接以这条线段的起点的航向角和曲率
        // 作为采样点的航向角和曲率。
        // 这样做的原因是线段长度变大时，插值误差会变大，尤其是该线段的起点为直道，
        // 终点为弯道的情况。
        point.heading = headings_[span];
        point.curvature = curvatures_[span];
      } else {
        // 采样点在一条较短的路径的线段中间，对这条线段的起点和终点的航向角和曲率进行插值，
        // 得到采样点的航向角和曲率
        point.heading = AngleLerp(headings_[span], headings_[span + 1], t);
        point.curvature = (1.0F - t) * curvatures_[span] +
            t * curvatures_[span + 1];
      }
      point.s = start_s;
      point.l = 0.0F;
      sample_points->PushBack(point);
    } else {
      // 采样点在路径终点的前面
      t = (start_s - accumulated_s_[span-1])
          / (accumulated_s_[span] - accumulated_s_[span - 1]);
      point.point = (1.0F - t) * points_[span - 1] + t * points_[span];
      point.heading = headings_[span];
      point.curvature = 0.0F;
      point.s = start_s;
      point.l = 0.0F;
      sample_points->PushBack(point);
    }
  }

  const phoenix::common::PathSearch<StaticVector<geo_var_t, kMaxPathPointNum> >
      path_search_tool(accumulated_s_, nodes_size);
  // 对其它的点进行采样
  // 总采样点的个数为sample_num+1，采样线段的个数为sample_num。
  for (Int32_t i = 0; i < sample_num; ++i) {
    // found = path_search.StepForward(start_s, seg_len, true, &span, &t);
    path_search_tool.StepForward(start_s, seg_len, true, &span, &t);
    start_s += seg_len;

    point.point = (1.0F - t) * points_[span] + t * points_[span + 1];
    if (((start_s - accumulated_s_[span]) > 2.0F) &&
        ((accumulated_s_[span + 1] - start_s) > 2.0F)) {
      point.heading = headings_[span];
      point.curvature = curvatures_[span];
    } else {
      // point.heading = AngleLerp(headings_[span], headings_[span+1], t);
      if ((t >= -kGeometryEpsilon) && (t <= (1.0F + kGeometryEpsilon))) {
        // 只有采样点在路径上时才进行航向角插值
        point.heading = AngleLerp(headings_[span], headings_[span + 1], t);
        point.curvature = (1.0F - t) * curvatures_[span] +
            t * curvatures_[span + 1];
      } else if (t < 0.0F) {
        point.heading = headings_[span];
        point.curvature = 0.0F;
      } else {
        point.heading = headings_[span + 1];
        point.curvature = 0.0F;
      }
    }
    point.s = start_s;
    point.l = 0.0F;
    sample_points->PushBack(point);
  }

  return (true);
}

/**
 * @date       2020.05.22
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/22  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool Path::UniformlySamplePathForward(
    geo_var_t start_s, Int32_t sample_num,
    geo_var_t seg_len, geo_var_t sample_l,
    StaticVector<PathPoint, kMaxPathPointNum>* const sample_points) const {
  const Int32_t nodes_size = points_.Size();

  if (nodes_size < 2) {
    return (false);
  }

  Int32_t span = 0;

  geo_var_t t = 0.0F;
  PathPoint point;
  if (start_s < 0.0F) {
    span = 0;
    t = (start_s - accumulated_s_[span])
        / (accumulated_s_[span + 1] - accumulated_s_[span]);
    point.point = (1.0F - t) * points_[span] + t * points_[span + 1];
    point.heading = headings_[0];
    point.curvature = 0.0F;
    point.s = start_s;
    point.l = sample_l;
    point.point.set_x(point.point.x()
        - sample_l * com_sin(point.heading));
    point.point.set_y(point.point.y()
        + sample_l * com_cos(point.heading));
    sample_points->PushBack(point);
  } else if (start_s > total_length_) {
    span = nodes_size - 1;
    t = (start_s - accumulated_s_[span - 1])
        / (accumulated_s_[span] - accumulated_s_[span - 1]);
    point.point = (1.0F - t) * points_[span - 1] + t * points_[span];
    point.heading = headings_[span];
    point.curvature = 0.0F;
    point.s = start_s;
    point.l = sample_l;
    point.point.set_x(point.point.x()
        - sample_l * com_sin(point.heading));
    point.point.set_y(point.point.y()
        + sample_l * com_cos(point.heading));
    sample_points->PushBack(point);
  } else {
    const Int32_t index = accumulated_s_.UpperBound(start_s, FuncCmpArcLen());
    if (index > (nodes_size - 1)) {
      span = nodes_size - 1;
    } else {
      if (index > 0) {
        span = index - 1;
      } else {
        span = 0;
      }
    }

    if ((span + 1) < nodes_size) {
      t = (start_s - accumulated_s_[span])
          / (accumulated_s_[span+1] - accumulated_s_[span]);
      point.point = (1.0F - t) * points_[span] + t * points_[span + 1];
      if (((start_s - accumulated_s_[span]) > 2.0F) &&
          ((accumulated_s_[span + 1] - start_s) > 2.0F)) {
        point.heading = headings_[span];
        point.curvature = curvatures_[span];
      } else {
        point.heading = AngleLerp(headings_[span], headings_[span + 1], t);
        point.curvature = (1.0F - t) * curvatures_[span] +
            t * curvatures_[span + 1];
      }
      point.s = start_s;
      point.l = sample_l;
      point.point.set_x(point.point.x()
          - sample_l * com_sin(point.heading));
      point.point.set_y(point.point.y()
          + sample_l * com_cos(point.heading));
      sample_points->PushBack(point);
    } else {
      t = (start_s - accumulated_s_[span-1])
          / (accumulated_s_[span] - accumulated_s_[span-1]);
      point.point = (1.0F - t) * points_[span-1] + t * points_[span];
      point.heading = headings_[span];
      point.curvature = 0.0F;
      point.s = start_s;
      point.l = sample_l;
      point.point.set_x(point.point.x()
          - sample_l * com_sin(point.heading));
      point.point.set_y(point.point.y()
          + sample_l * com_cos(point.heading));
      sample_points->PushBack(point);
    }
  }

 // bool found = false;
  const phoenix::common::PathSearch<StaticVector<geo_var_t, kMaxPathPointNum> >
      path_search_tool(accumulated_s_, nodes_size);
  for (Int32_t i = 0; i < sample_num; ++i) {
   // found = path_search.StepForward(start_s, seg_len, true, &span, &t);
    path_search_tool.StepForward(start_s, seg_len, true, &span, &t);
    start_s += seg_len;

    point.point = (1.0F - t) * points_[span] + t * points_[span + 1];
    if (((start_s - accumulated_s_[span]) > 2.0F) &&
        ((accumulated_s_[span + 1] - start_s) > 2.0F)) {
      point.heading = headings_[span];
      point.curvature = curvatures_[span];
    } else {
      // point.heading = AngleLerp(headings_[span], headings_[span + 1], t);
      if ((t >= -kGeometryEpsilon) && (t <= (1.0F + kGeometryEpsilon))) {
        // 只有采样点在路径上时才进行航向角插值
        point.heading = AngleLerp(headings_[span], headings_[span + 1], t);
        point.curvature = (1.0F - t) * curvatures_[span] +
            t * curvatures_[span + 1];
      } else if (t < 0.0F) {
        point.heading = headings_[span];
        point.curvature = 0.0F;
      } else {
        point.heading = headings_[span + 1];
        point.curvature = 0.0F;
      }
    }
    point.s = start_s;
    point.l = sample_l;
    point.point.set_x(point.point.x()
        - sample_l * com_sin(point.heading));
    point.point.set_y(point.point.y()
        + sample_l * com_cos(point.heading));
    sample_points->PushBack(point);
  }

  return (true);
}

/**
 * @date       2020.05.25
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/25  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool Path::UniformlySamplePathBackward(
    geo_var_t start_s, Int32_t sample_num, geo_var_t seg_len,
    StaticVector<PathPoint, kMaxPathPointNum> * const sample_points) const {
  const Int32_t nodes_size = points_.Size();

  if (nodes_size < 2) {
    return (false);
  }

  Int32_t span = 0;

  geo_var_t t = 0.0F;
  PathPoint point;
  if (start_s < 0.0F) {
    span = 0;
    t = (start_s - accumulated_s_[span])
        / (accumulated_s_[span + 1] - accumulated_s_[span]);
    point.point = (1.0F - t) * points_[span] + t * points_[span + 1];
    point.heading = headings_[0];
    point.curvature = 0.0F;
    point.s = start_s;
    point.l = 0.0F;
    sample_points->PushBack(point);
  } else if (start_s > total_length_) {
    span = nodes_size - 1;
    t = (start_s - accumulated_s_[span - 1])
        / (accumulated_s_[span] - accumulated_s_[span - 1]);
    point.point = (1.0F - t) * points_[span - 1] + t * points_[span];
    point.heading = headings_[span];
    point.curvature = 0.0F;
    point.s = start_s;
    point.l = 0.0F;
    sample_points->PushBack(point);
  } else {
    const Int32_t index = accumulated_s_.UpperBound(start_s, FuncCmpArcLen());
    if (index > (nodes_size - 1)) {
      span = nodes_size - 1;
    } else {
      if (index > 0) {
        span = index - 1;
      } else {
        span = 0;
      }
    }

    if ((span + 1) < nodes_size) {
      t = (start_s - accumulated_s_[span])
          / (accumulated_s_[span + 1] - accumulated_s_[span]);
      point.point = (1.0F - t) * points_[span] + t * points_[span + 1];
      if (((start_s - accumulated_s_[span]) > 2.0F) &&
          ((accumulated_s_[span + 1] - start_s) > 2.0F)) {
        point.heading = headings_[span];
        point.curvature = curvatures_[span];
      } else {
        point.heading = AngleLerp(headings_[span], headings_[span + 1], t);
        point.curvature = (1.0F - t) * curvatures_[span] +
            t * curvatures_[span + 1];
      }
      point.s = start_s;
      point.l = 0.0F;
      sample_points->PushBack(point);
    } else {
      t = (start_s - accumulated_s_[span - 1])
          / (accumulated_s_[span] - accumulated_s_[span - 1]);
      point.point = (1.0F - t) * points_[span - 1] + t * points_[span];
      point.heading = headings_[span];
      point.curvature = 0.0F;
      point.s = start_s;
      point.l = 0.0F;
      sample_points->PushBack(point);
    }
  }

  const phoenix::common::PathSearch<StaticVector<geo_var_t, kMaxPathPointNum> >
      path_search_tool(accumulated_s_, nodes_size);
  for (Int32_t i = 0; i < sample_num; ++i) {
    path_search_tool.StepBackward(start_s, seg_len, &span, &t, true);
    start_s -= seg_len;

    point.point = (1.0F - t) * points_[span] + t * points_[span + 1];
    if (((start_s - accumulated_s_[span]) > 2.0F) &&
        ((accumulated_s_[span+1] - start_s) > 2.0F)) {
      point.heading = headings_[span];
      point.curvature = curvatures_[span];
    } else {
      if ((t >= -kGeometryEpsilon) && (t <= (1.0F + kGeometryEpsilon))) {
        // 只有采样点在路径上时才进行航向角插值
        point.heading = AngleLerp(headings_[span], headings_[span + 1], t);
        point.curvature = (1.0F - t) * curvatures_[span] +
            t * curvatures_[span + 1];
      } else if (t < 0.0F) {
        point.heading = headings_[span];
        point.curvature = 0.0F;
      } else {
        point.heading = headings_[span + 1];
        point.curvature = 0.0F;
      }
    }
    point.s = start_s;
    point.l = 0.0F;
    sample_points->PushBack(point);
  }

  return (true);
}

/**
 * @date       2020.05.27
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/27  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool Path::UniformlySamplePathBackward(
    geo_var_t start_s, Int32_t sample_num,
    geo_var_t seg_len, geo_var_t sample_l,
    StaticVector<PathPoint, kMaxPathPointNum> * const sample_points) const {
  const Int32_t nodes_size = points_.Size();

  if (nodes_size < 2) {
    return (false);
  }

  Int32_t span = 0;

  geo_var_t t = 0.0F;
  PathPoint point;
  if (start_s < 0.0F) {
    span = 0;
    t = (start_s - accumulated_s_[span])
        / (accumulated_s_[span + 1] - accumulated_s_[span]);
    point.point = (1.0F - t) * points_[span] + t * points_[span + 1];
    point.heading = headings_[0];
    point.curvature = 0.0F;
    point.s = start_s;
    point.l = sample_l;
    point.point.set_x(point.point.x()
        - sample_l * com_sin(point.heading));
    point.point.set_y(point.point.y()
        + sample_l * com_cos(point.heading));
    sample_points->PushBack(point);
  } else if (start_s > total_length_) {
    span = nodes_size - 1;
    t = (start_s - accumulated_s_[span - 1])
        / (accumulated_s_[span] - accumulated_s_[span - 1]);
    point.point = (1.0F - t) * points_[span - 1] + t * points_[span];
    point.heading = headings_[span];
    point.curvature = 0.0F;
    point.s = start_s;
    point.l = sample_l;
    point.point.set_x(point.point.x()
        - sample_l * com_sin(point.heading));
    point.point.set_y(point.point.y()
        + sample_l * com_cos(point.heading));
    sample_points->PushBack(point);
  } else {
    const Int32_t index = accumulated_s_.UpperBound(start_s, FuncCmpArcLen());
    if (index > (nodes_size - 1)) {
      span = nodes_size - 1;
    } else {
      if (index > 0) {
        span = index - 1;
      } else {
        span = 0;
      }
    }

    if ((span + 1) < nodes_size) {
      t = (start_s - accumulated_s_[span])
          / (accumulated_s_[span + 1] - accumulated_s_[span]);
      point.point = (1.0F - t) * points_[span] + t * points_[span + 1];
      if (((start_s - accumulated_s_[span]) > 2.0F) &&
         ((accumulated_s_[span + 1] - start_s) > 2.0F)) {
        point.heading = headings_[span];
        point.curvature = curvatures_[span];
      } else {
        point.heading = AngleLerp(headings_[span], headings_[span + 1], t);
        point.curvature = (1.0F - t) * curvatures_[span] +
          t * curvatures_[span + 1];
      }
      point.s = start_s;
      point.l = sample_l;
      point.point.set_x(point.point.x()
          - sample_l * com_sin(point.heading));
      point.point.set_y(point.point.y()
          + sample_l * com_cos(point.heading));
      sample_points->PushBack(point);
    } else {
      t = (start_s - accumulated_s_[span - 1])
          / (accumulated_s_[span] - accumulated_s_[span - 1]);
      point.point = (1.0F - t) * points_[span - 1] + t * points_[span];
      point.heading = headings_[span];
      point.curvature = 0.0F;
      point.s = start_s;
      point.l = sample_l;
      point.point.set_x(point.point.x()
          - sample_l * com_sin(point.heading));
      point.point.set_y(point.point.y()
          + sample_l * com_cos(point.heading));
      sample_points->PushBack(point);
    }
  }

  const phoenix::common::PathSearch<StaticVector<geo_var_t, kMaxPathPointNum> >
      path_search_tool(accumulated_s_, nodes_size);
  for (Int32_t i = 0; i < sample_num; ++i) {
    path_search_tool.StepBackward(start_s, seg_len, &span, &t, true);
    start_s -= seg_len;

    point.point = (1.0F - t) * points_[span] + t * points_[span + 1];
    if (((start_s - accumulated_s_[span]) > 2.0F) &&
       ((accumulated_s_[span + 1] - start_s) > 2.0F)) {
      point.heading = headings_[span];
      point.curvature = curvatures_[span];
    } else {
      if ((t >= -kGeometryEpsilon) && (t <= (1.0F + kGeometryEpsilon))) {
        // 只有采样点在路径上时才进行航向角插值
        point.heading = AngleLerp(headings_[span], headings_[span + 1], t);
        point.curvature = (1.0F - t) * curvatures_[span] +
            t * curvatures_[span + 1];
      } else if (t < 0.0F) {
        point.heading = headings_[span];
        point.curvature = 0.0F;
      } else {
        point.heading = headings_[span + 1];
        point.curvature = 0.0F;
      }
    }
    point.s = start_s;
    point.l = sample_l;
    point.point.set_x(point.point.x()
        - sample_l * com_sin(point.heading));
    point.point.set_y(point.point.y()
        + sample_l * com_cos(point.heading));
    sample_points->PushBack(point);
  }

  return (true);
}

/*
 * @date       2020.06.19
 * @version    v1.1
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author <th>Description
 * <tr><td>2020/06/01  <td>1.0      <td>boc    <td>First edition
 * <tr><td>2020/06/19  <td>1.1      <td>boc    <td>解决将环形路径的\n
 *     平均曲率计算为0的问题。
 * </table>
 */
void Path::AddCurveSegment(
    Int32_t start_index, Int32_t index_max_curvature,
    Int32_t sample_index, Int32_t type,
    StaticVector<CurveSegment,
    kMaxCurveSegmentNum> * const curve_segments) const {
  Path::CurveSegment seg;
  seg.type = type;
  seg.start_s = accumulated_s_[start_index];
  seg.length = accumulated_s_[sample_index] - accumulated_s_[start_index];
  if ((sample_index < start_index + 4) || (seg.length < 30.0F)) {
    seg.ave_curvature = phoenix::common::AngleDiff(
          headings_[start_index], headings_[sample_index]) / seg.length;
  } else {
    // 将整条曲线段分为两段后，再计算平均曲率，为了解决环形路径的平均曲率
    // 为0的问题。
    Path::CurveSegment seg1;
    Path::CurveSegment seg2;
    const Int32_t sample_index_seg1 = (start_index + sample_index) / 2;
    seg1.start_s = accumulated_s_[start_index];
    seg1.length = accumulated_s_[sample_index_seg1] - seg1.start_s;
    seg1.ave_curvature = phoenix::common::AngleDiff(
          headings_[start_index], headings_[sample_index_seg1]) / seg1.length;
    seg2.length = seg.length - seg1.length;
    seg2.ave_curvature = phoenix::common::AngleDiff(
          headings_[sample_index_seg1], headings_[sample_index]) / seg2.length;
    seg.ave_curvature = (seg1.length * seg1.ave_curvature +
        seg2.length * seg2.ave_curvature) / seg.length;
  }
  seg.max_curvature = curvatures_[index_max_curvature];

  if (curve_segments->Size() < 1) {
    if ((seg.length < 0.1F) && (com_abs(seg.max_curvature) < 0.02F)) {
      // 忽略长度较小，且曲率较小的弯道。
    } else {
      curve_segments->PushBack(seg);
    }
  } else {
    CurveSegment& prev_curve = curve_segments->Back();
    // 同一种类型的路径点中间，出现别的类型的间断点的最大长度
    geo_var_t length_tmp = 0.0F;
    geo_var_t leap_threhold = 3.0F;
    if (type == static_cast<Int32_t>(TYPE_STRAIGHT)) {
      leap_threhold = 10.0F;
    }
    if ((seg.type == prev_curve.type) &&
        ((prev_curve.start_s + prev_curve.length + leap_threhold) > seg.start_s)
        ) {
      length_tmp = seg.start_s - prev_curve.start_s + seg.length;
      prev_curve.ave_curvature =
          (prev_curve.ave_curvature * prev_curve.length +
           seg.ave_curvature * seg.length) / length_tmp;
      // prev_curve.length = seg.start_s - prev_curve.start_s + seg.length;
      prev_curve.length = length_tmp;
      if (com_abs(prev_curve.max_curvature) <
          com_abs(seg.max_curvature)) {
        prev_curve.max_curvature = seg.max_curvature;
      }
    } else {
      if ((seg.length < 0.1F) && (com_abs(seg.max_curvature) < 0.02F)) {
        // 忽略长度较小，且曲率较小的弯道。
      } else {
        // 1. 对‘弯道 - 直道 - 弯道’的情况进行处理，
        //    当中间那一段直道较短时，合并为弯道；
        // 2. 对‘直道 - 弯道 - 直道’的情况进行处理，
        //    当中间那一段弯道较短时，合并为直道。
        // 3. ‘直道 - 左转弯 - 右转弯 - 直道’ 或
        //    ‘直道 - 右转弯 - 左转弯 - 直道’的情况。
        if (curve_segments->Size() > 1) {
          // 倒数第二段曲率的信息
          CurveSegment& penultimate_curve =
              (*curve_segments)[curve_segments->Size() - 2];
          if ((seg.type == penultimate_curve.type) &&
              (seg.type != TYPE_STRAIGHT)) {  // ‘弯道 - 直道 - 弯道’的情况
            // 中间段的弧长小于10米时
            if (prev_curve.length < 10.0F) {
              length_tmp =
                  seg.start_s - penultimate_curve.start_s + seg.length;
              penultimate_curve.ave_curvature =
                  (penultimate_curve.ave_curvature * penultimate_curve.length +
                   prev_curve.ave_curvature * prev_curve.length +
                   seg.ave_curvature * seg.length) / length_tmp;
              penultimate_curve.length = length_tmp;
              if (com_abs(penultimate_curve.max_curvature) <
                  com_abs(prev_curve.max_curvature)) {
                penultimate_curve.max_curvature = prev_curve.max_curvature;
              }
              if (com_abs(penultimate_curve.max_curvature) <
                  com_abs(seg.max_curvature)) {
                penultimate_curve.max_curvature = seg.max_curvature;
              }
              curve_segments->PopBack();
            } else {
              curve_segments->PushBack(seg);
            }
          } else if ((seg.type == penultimate_curve.type) &&
                     (seg.type == TYPE_STRAIGHT)) {  // ‘直道 - 弯道 - 直道’的情况
            // 中间段的弧长角度小于5度时 (tan(5度) = 0.0875)
            if (com_abs(prev_curve.length * prev_curve.ave_curvature) <
                0.0875F) {
              length_tmp =
                  seg.start_s - penultimate_curve.start_s + seg.length;
              penultimate_curve.ave_curvature =
                  (penultimate_curve.ave_curvature * penultimate_curve.length +
                   prev_curve.ave_curvature * prev_curve.length +
                   seg.ave_curvature * seg.length) / length_tmp;
              penultimate_curve.length = length_tmp;
              if (com_abs(penultimate_curve.max_curvature) <
                  com_abs(prev_curve.max_curvature)) {
                penultimate_curve.max_curvature = prev_curve.max_curvature;
              }
              if (com_abs(penultimate_curve.max_curvature) <
                  com_abs(seg.max_curvature)) {
                penultimate_curve.max_curvature = seg.max_curvature;
              }
              curve_segments->PopBack();
            } else {
              curve_segments->PushBack(seg);
            }
          } else if ((curve_segments->Size() > 2) &&
                     (seg.type ==
                       (*curve_segments)[curve_segments->Size() - 3].type)
                     /*&& seg.type == TYPE_STRAIGHT*/) {
            // ‘直道 - 左转 - 右转 - 直道’或
            // ‘直道 - 右转 - 左转 - 直道’或
            // ‘左转 - 任意 - 任意 - 左转’或
            // ‘右转 - 任意 - 任意 - 右转’的情况
            // 条件1：中间两段的弧长角度小于5度时 (tan(5度) = 0.0875)
            // 条件2：中间两段的弧长之和小于10米
            CurveSegment& antepenultimate_curve =
                (*curve_segments)[curve_segments->Size() - 3];
            if ((com_abs(prev_curve.length * prev_curve.ave_curvature +
                  penultimate_curve.length * penultimate_curve.ave_curvature) <
                  0.0875F) &&
                (seg.start_s - antepenultimate_curve.start_s -
                 antepenultimate_curve.length < 10.0F)) {
              length_tmp = seg.start_s - antepenultimate_curve.start_s +
                  seg.length;
              antepenultimate_curve.ave_curvature =
                  (antepenultimate_curve.ave_curvature *
                   antepenultimate_curve.length +
                   penultimate_curve.ave_curvature * penultimate_curve.length +
                   prev_curve.ave_curvature * prev_curve.length +
                   seg.ave_curvature * seg.length) / length_tmp;
              antepenultimate_curve.length = length_tmp;
              if (com_abs(antepenultimate_curve.max_curvature) <
                  com_abs(penultimate_curve.max_curvature)) {
                antepenultimate_curve.max_curvature =
                    penultimate_curve.max_curvature;
              }
              if (com_abs(antepenultimate_curve.max_curvature) <
                  com_abs(prev_curve.max_curvature)) {
                antepenultimate_curve.max_curvature = prev_curve.max_curvature;
              }
              if (com_abs(antepenultimate_curve.max_curvature) <
                  com_abs(seg.max_curvature)) {
                antepenultimate_curve.max_curvature = seg.max_curvature;
              }
              // 删除中间的两条曲线段
              curve_segments->PopBack();
              curve_segments->PopBack();
            } else {
              curve_segments->PushBack(seg);
            }
          } else {
            curve_segments->PushBack(seg);
          }
        } else {
          curve_segments->PushBack(seg);
        }
      }
    }
  }
}

/**
 * @date       2020.06.02
 * @version    v1.1
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/06/01  <td>1.0      <td>boc       <td>First edition
 * <tr><td>2020/06/02  <td>1.1      <td>boc       <td>解决左转直接变右转时会\n
 *                                                    多加一段直道的问题
 * </table>
 */
bool Path::AnalyzeCurvature(
    StaticVector<CurveSegment, kMaxCurveSegmentNum> * const
    curve_segments) const {
  const Int32_t curvatures_size = curvatures_.Size();
  if (curvatures_size < 2) {
    return (false);
  }

  curve_segments->Clear();

  geo_var_t max_curvature = 0.0F;
  Int32_t index_max_curvature = 0;
  bool find = false;
  Int32_t type = static_cast<Int32_t>(TYPE_STRAIGHT);
  Int32_t start_index = 0;
  geo_var_t curvature = 0.0F;

  for (Int32_t i = 0; i < curvatures_size; ++i) {
    const geo_var_t abs_curv = com_abs(curvatures_[i]);
    if (abs_curv > kCriticalCurvature) {
      if (!find) {
        // 开始进入弯道
        if ((i > start_index) &&
            (type == static_cast<Int32_t>(TYPE_STRAIGHT))) {
          // 将之前的直线段加入到曲率信息表中
          AddCurveSegment(start_index, index_max_curvature, i, type,
                          curve_segments);
        }
        find = true;
        if (curvatures_[i] < 0.0F) {
          type = static_cast<Int32_t>(TYPE_CURVING_RIGHT);
        } else {
          type = static_cast<Int32_t>(TYPE_CURVING_LEFT);
        }
        start_index = i;
        curvature = curvatures_[i];
        max_curvature = abs_curv;
        index_max_curvature = i;
      } else {
        if (((static_cast<Int32_t>(TYPE_CURVING_RIGHT) == type) &&
             (curvatures_[i] < 0.0F)) ||
            ((static_cast<Int32_t>(TYPE_CURVING_LEFT) == type) &&
             (curvatures_[i] > 0.0F))) {
          // 当前点仍然处于右转弯或者仍然处于左转弯，将当前状态进行累计，
          // 存储当前弯道的最大曲率等信息。
          curvature += curvatures_[i];
          if (abs_curv > max_curvature) {
            max_curvature = abs_curv;
            index_max_curvature = i;
          }

          // 如果是道路的最后的一个点，且处于路径上，将当前弯道添加到列表中。
          if ((i + 1) == curvatures_size) {
            AddCurveSegment(start_index, index_max_curvature, i, type,
                            curve_segments);
          }
        } else {
          // 弯道的类型发生了变化（左转弯变为了右转弯，或者右转弯变成了左转弯）
          AddCurveSegment(start_index, index_max_curvature, i, type,
                          curve_segments);
          if (curvatures_[i] < 0.0F) {
            type = static_cast<Int32_t>(TYPE_CURVING_RIGHT);
          } else {
            type = static_cast<Int32_t>(TYPE_CURVING_LEFT);
          }
          start_index = i;
          curvature = curvatures_[i];
          max_curvature = abs_curv;
          index_max_curvature = i;
        }
      }
    } else {
      if (find) {
        // 弯道变为直道
        find = false;
        AddCurveSegment(start_index, index_max_curvature, i, type,
                        curve_segments);
        type = static_cast<Int32_t>(TYPE_STRAIGHT);
        start_index = i;
        curvature = curvatures_[i];
        max_curvature = abs_curv;
        index_max_curvature = i;
      } else {
        // 对直线段的信息进行更新
        curvature += curvatures_[i];
        if (abs_curv > max_curvature) {
          max_curvature = abs_curv;
          index_max_curvature = i;
        }
        // 如果是道路的最后的一个点，且处于路径上，将当前直道添加到列表中。
        if ((i + 1) == curvatures_size) {
          AddCurveSegment(start_index, index_max_curvature, i,
            static_cast<Int32_t>(TYPE_STRAIGHT), curve_segments);
        }
      }
    }
  }

  return (true);
}

/**
 * @date       2020.05.14
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/14  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Int32_t Path::ObtainCircumcircleRadius(geo_var_t a, geo_var_t b,
                                       geo_var_t c, geo_var_t* const r) {
  const geo_var_t p = (a + b + c) / 2.0F;

  // 共线
  if ((com_abs(b + c - a) < kGeometryEpsilon) ||
      (com_abs(c + a - b) < kGeometryEpsilon) ||
      (com_abs(a + b - c) < kGeometryEpsilon)) {
    *r = 1.0E6F;
    return 2;
  } else if (((b + c - a) < 0.0F) || ((c + a - b) < 0.0F) ||
             ((a + b - c) < 0.0F)) {
    // 边长不满足三角形的性质
    // 两条边的和不小于第三条边
    return -1;
  }
  *r = a * b * c / (4.0F * com_sqrt(p * (p - a) * (p - b) * (p - c)));
  return 1;
}

/**
 * @date       2020.05.14
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/14  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
Int32_t Path::ObtainCircumcircleRadius(const Vec2d& a, const Vec2d& b,
                                       const Vec2d& c, geo_var_t* const r) {
  const geo_var_t side_a = b.DistanceTo(c);
  const geo_var_t side_b = c.DistanceTo(a);
  const geo_var_t side_c = a.DistanceTo(b);
  return ObtainCircumcircleRadius(side_a, side_b, side_c, r);
}

/*
 * @date       2020.05.19
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/19  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool Path::FindSmoothPoint(
    geo_var_t s, Int32_t * const low_index,
    PathPoint * const path_point_out) const {
  // 路径上点过少时
  if (points_.Size() < 2) {
    path_point_out->point = points_[0];
    path_point_out->heading = 0.0F;
    path_point_out->curvature = 0.0F;
    path_point_out->s = 0.0F;
    path_point_out->l = 0.0F;

    *low_index = 0;
    return (false);
  }

  // 待查找点的弧长小于0时
  if (s < 0.01F) {
    path_point_out->point = points_[0];
    path_point_out->heading = headings_[0];
    path_point_out->curvature = curvatures_[0];
    path_point_out->s = 0.0F;
    path_point_out->l = 0.0F;

    *low_index = 0;
    return (true);
  }

  // 待查找点的弧长大于路径总长度时
  if (s > total_length_ - 0.01F) {
    path_point_out->point = points_.Back();
    path_point_out->heading = headings_.Back();
    path_point_out->curvature = curvatures_.Back();
    path_point_out->s = total_length_;
    path_point_out->l = 0.0F;

    *low_index = points_.Size() - 2;
    if (*low_index < 0) {
      *low_index = 0;
    }
    return (true);
  }

  // 查找待查找点所在路径上的线段的终点的索引
  const Int32_t index = accumulated_s_.LowerBound(s, FuncCmpArcLen());
  if (index > (accumulated_s_.Size() - 1)) {
    path_point_out->point = points_.Back();
    path_point_out->heading = headings_.Back();
    path_point_out->curvature = curvatures_.Back();
    path_point_out->s = total_length_;
    path_point_out->l = 0.0F;

    *low_index = points_.Size() - 2;
    if (*low_index < 0) {
      *low_index = 0;
    }
    return (true);
  }
  const geo_var_t delta_s = accumulated_s_[index] - s;
  // 待查找点与线段的终点重合时
  if ((0 == index) || (delta_s < phoenix::common::kMathEpsilonF)) {
    path_point_out->point = points_[index];
    path_point_out->heading = headings_[index];
    path_point_out->curvature = curvatures_[index];
    path_point_out->s = accumulated_s_[index];
    path_point_out->l = 0.0F;

    *low_index = index - 1;
    if (*low_index < 0) {
      *low_index = 0;
    }
    return (true);
  }

  // 待查找点在线段的中间时
  const geo_var_t t = (s - accumulated_s_[index - 1])
      / (accumulated_s_[index] - accumulated_s_[index - 1]);
  path_point_out->point = (1.0F - t) * points_[index - 1] + t * points_[index];
  path_point_out->s = s;
  path_point_out->l = 0.0F;

  *low_index = index - 1;

  if (((s - accumulated_s_[index - 1]) > 2.0F) &&
      ((accumulated_s_[index] - s) > 2.0F)) {
    // 待查找点在一条很长的路径的线段中间，直接以这条线段的起点的航向角和曲率
    // 作为待查找点的航向角和曲率。
    // 这样做的原因是线段长度变大时，插值误差会变大，尤其是该线段的起点为直道，
    // 终点为弯道的情况。
    path_point_out->heading = headings_[index - 1];
    path_point_out->curvature = curvatures_[index - 1];
  } else {
    path_point_out->heading =
        AngleLerp(headings_[index - 1], headings_[index], t);
    path_point_out->curvature = (1.0F - t) * curvatures_[index-1] +
        t * curvatures_[index];
  }

  return (true);
}

/*
 * @date       2021.12.10
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author     <th>Description
 * <tr><td>2021/12/10  <td>1.0      <td>longjiaoy  <td>First edition
 * </table>
 */
bool Path::IsIntersect(const common::Vec2d& start_point,
                       const common::Vec2d& end_point,
                       PathPoint *cross_point) const {
  PathPoint start_point_pro_on_path;
  PathPoint end_point_pro_on_path;

  if (points_.Size() < 2) {
    return (false);
  }

  Int32_t start_on_path_proj_index = -1;
  Int32_t end_on_path_proj_index = -1;

  if (!FindProjection(
        start_point, &start_on_path_proj_index, &start_point_pro_on_path)) {
    LOG_ERR << "Failed to find projection of start_point("
            << start_point.x() << "," << start_point.y()
            << ") on path(size=" << points_.Size() << ").";
    return (false);
  }
  /// TODO: 将start_point修改为end_point, 需要重新验证此函数功能是否正确
  if (!FindProjection(
        end_point, &end_on_path_proj_index, &end_point_pro_on_path)) {
    LOG_ERR << "Failed to find projection of end_point("
            << end_point.x() << "," << end_point.y()
            << ") on path(size=" << points_.Size() << ").";
    return (false);
  }

  Int32_t start_index =-1;
  Int32_t end_index = -1;
  if (start_on_path_proj_index <= end_on_path_proj_index) {
    start_index = start_on_path_proj_index - 1;
    end_index = end_on_path_proj_index + 1;
  } else {
    start_index = end_on_path_proj_index - 1;
    end_index = start_on_path_proj_index + 1;
  }

  if (end_index >= points_.Size()) {
    end_index = points_.Size() - 1;
  }
  if (start_index < 0) {
    start_index = 0;
  }

  for (Int32_t i = start_index; i < end_index; ++i) {
    Vec2d c1;
    Vec2d c2;
    geo_var_t s = 0.0F;
    geo_var_t t = 0.0F;
    geo_var_t sq_dist = ClosestPtSegToSeg_2D(
          start_point, end_point, points_[i], points_[i+1],
        &c1, &c2, &s, &t);

    if (sq_dist < 0.05F) {
      cross_point->point = c2;
      cross_point->s = (1.0F - t)*accumulated_s_[i] + t*accumulated_s_[i+1];
      cross_point->l = 0.0F;

      if (((cross_point->s - accumulated_s_[i]) > 2.0F) &&
          ((accumulated_s_[i+1] - cross_point->s) > 2.0F)) {
        cross_point->heading = headings_[i];
        cross_point->curvature = curvatures_[i];
      } else {
        cross_point->heading = AngleLerp(headings_[i], headings_[i+1], t);
        cross_point->curvature =
            (1.0F - t)*curvatures_[i] + t*curvatures_[i+1];
      }
      
      return (true);
    }
  }

  return (false);
}

/* k004 pengc 2022-12-26 (begin) */
/*
 * @brief 使用新的轨迹替换当前轨迹的重叠部分
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author     <th>Description
 * <tr><td>2022/12/29  <td>1.0      <td>pengc      <td>First edition
 * </table>
 */
bool Path::Merge(
    const MergeParam& param, const common::Path& src_path,
    MergeInfo* info) {
  if (Nullptr_t == param.points_2d) {
    LOG_ERR << "Invalid parameters.";
    return (false);
  }
  if (points().Size() < 2) {
    LOG_ERR << "Invalid destination path.";
    return (false);
  }
  if (src_path.points().Size() < 2) {
    LOG_ERR << "Invalid source path.";
    return (false);
  }

  // 源轨迹起点在目标轨迹上的投影点
  Int32_t start_point_on_dst_idx = -1;
  common::PathPoint start_point_on_dst;
  // 源轨迹终点在目标轨迹上的投影点
  Int32_t end_point_on_dst_idx = -1;
  common::PathPoint end_point_on_dst;
  // 目标轨迹起点在源轨迹上的投影点
  Int32_t start_point_on_src_idx = -1;
  common::PathPoint start_point_on_src;
  // 目标轨迹终点在源轨迹上的投影点
  Int32_t end_point_on_src_idx = -1;
  common::PathPoint end_point_on_src;

  if (!this->FindProjection(
        src_path.points().Front(),
        &start_point_on_dst_idx, &start_point_on_dst)) {
    LOG_ERR << "Failed to find projection point of start point "
               "of source path on destination path.";
    return false;
  }
  if (!this->FindProjection(
        src_path.points().Back(),
        &end_point_on_dst_idx, &end_point_on_dst)) {
    LOG_ERR << "Failed to find projection point of end point "
               "of source path on destination path.";
    return false;
  }
  if (!src_path.FindProjection(
        this->points().Front(),
        &start_point_on_src_idx, &start_point_on_src)) {
    LOG_ERR << "Failed to find projection point of start point "
               "of destination path on source path.";
    return false;
  }
  if (!src_path.FindProjection(
        this->points().Back(),
        &end_point_on_src_idx, &end_point_on_src)) {
    LOG_ERR << "Failed to find projection point of end point "
               "of destination path on source path.";
    return false;
  }

  if ((start_point_on_dst.s < 0.0F) && (end_point_on_dst.s < 0.0F)) {
    // src_path:  -----------------
    // dst_path:                     ------------------
    LOG_INFO(5) << "The two path has not overlapped part.";
    return false;
  }
  if ((start_point_on_src.s < 0.0F) && (end_point_on_src.s < 0.0F)) {
    // src_path:                      -----------------
    // dst_path:  -----------------
    LOG_INFO(5) << "The two path has not overlapped part.";
    return false;
  }
  if (end_point_on_dst.s < start_point_on_dst.s) {
    LOG_INFO(5) << "The two path has different direction.";
    return false;
  }
  if (end_point_on_src.s < start_point_on_src.s) {
    LOG_INFO(5) << "The two path has different direction.";
    return false;
  }

  // case 1:
  // src_path:  ----***********
  // dst_path:      --------------------
  //
  // case 2:
  // src_path:  ----***********---------
  // dst_path:      -----------
  //
  // case 3:
  // src_path:      ***********---------
  // dst_path:  ---------------
  //
  // case 4:
  // src_path:      ***********
  // dst_path:  ------------------------

  if (start_point_on_src.s >= 0.0F) {
    // case 1, case 2
    start_point_on_dst_idx = 0;
    start_point_on_dst.point = this->points().Front();
    start_point_on_dst.heading = this->headings().Front();
    start_point_on_dst.curvature = this->curvatures().Front();
    start_point_on_dst.s = 0;
    start_point_on_dst.l = -start_point_on_src.l;
  } else {
    // case 3, case 4
    start_point_on_src_idx = 0;
    start_point_on_src.point = src_path.points().Front();
    start_point_on_src.heading = src_path.headings().Front();
    start_point_on_src.curvature = src_path.curvatures().Front();
    start_point_on_src.s = 0;
    start_point_on_src.l = -start_point_on_dst.l;
  }

  if (end_point_on_src.s <= src_path.total_length()) {
    // case 2, case 3
    end_point_on_dst_idx = this->points().Size()-2;
    end_point_on_dst.point = this->points().Back();
    end_point_on_dst.heading = this->headings().Back();
    end_point_on_dst.curvature = this->curvatures().Back();
    end_point_on_dst.s = this->total_length();
    end_point_on_dst.l = -end_point_on_src.l;
  } else {
    // case 1, case 4
    end_point_on_src_idx = src_path.points().Size()-2;
    end_point_on_src.point = src_path.points().Back();
    end_point_on_src.heading = src_path.headings().Back();
    end_point_on_src.curvature = src_path.curvatures().Back();
    end_point_on_src.s = src_path.total_length();
    end_point_on_src.l = -end_point_on_dst.l;
  }

  geo_var_t h_diff = AngleDiff(
        start_point_on_dst.heading, start_point_on_src.heading);
#if 0
  printf("  Search start (begin), l_diff(%0.2f) h_diff(%0.2f)\n",
         start_point_on_src.l, com_rad2deg(h_diff));
#endif
  if ((common::com_abs(start_point_on_src.l) > param.max_lat_offset) ||
      (common::com_abs(h_diff) > param.max_heading_offset)) {
    for (Int32_t i = start_point_on_src_idx+1; i <= end_point_on_src_idx; ++i) {
      if ((src_path.accumulated_s()[i] - start_point_on_src.s) < param.search_step) {
        continue;
      }
      if (!this->FindProjection(
            src_path.points()[i],
            &start_point_on_dst_idx, &start_point_on_dst)) {
        LOG_ERR << "Failed to find projection point of start point "
                   "of source path on destination path.";
        return false;
      }

      start_point_on_src_idx = i;
      start_point_on_src.point = src_path.points()[i];
      start_point_on_src.heading = src_path.headings()[i];
      start_point_on_src.curvature = src_path.curvatures()[i];
      start_point_on_src.s = src_path.accumulated_s()[i];
      start_point_on_src.l = -start_point_on_dst.l;

      h_diff = AngleDiff(start_point_on_dst.heading, start_point_on_src.heading);
#if 0
      printf("  Search start [%d], l_diff(%0.2f) h_diff(%0.2f)\n",
             i, start_point_on_dst.l, com_rad2deg(h_diff));
#endif
      if ((common::com_abs(start_point_on_dst.l) < param.max_lat_offset) &&
          (common::com_abs(h_diff) < param.max_heading_offset)) {
        break;
      }
    }
  }

  h_diff = AngleDiff(end_point_on_dst.heading, end_point_on_src.heading);
#if 0
  printf("  Search end (start), l_diff(%0.2f) h_diff(%0.2f)\n",
         end_point_on_src.l, com_rad2deg(h_diff));
#endif
  if ((common::com_abs(end_point_on_src.l) > param.max_lat_offset) ||
      (common::com_abs(h_diff) > param.max_heading_offset)) {
    for (Int32_t i = end_point_on_src_idx; i > (start_point_on_src_idx+1); --i) {
      if ((end_point_on_src.s - src_path.accumulated_s()[i]) < param.search_step) {
        continue;
      }
      if (!this->FindProjection(
            src_path.points()[i],
            &end_point_on_dst_idx, &end_point_on_dst)) {
        LOG_ERR << "Failed to find projection point of end point "
                   "of source path on destination path.";
        return false;
      }

      end_point_on_src_idx = i;
      end_point_on_src.point = src_path.points()[i];
      end_point_on_src.heading = src_path.headings()[i];
      end_point_on_src.curvature = src_path.curvatures()[i];
      end_point_on_src.s = src_path.accumulated_s()[i];
      end_point_on_src.l = -end_point_on_dst.l;

      h_diff = AngleDiff(end_point_on_dst.heading, end_point_on_src.heading);
#if 0
      printf("  Search end [%d], l_diff(%0.2f) h_diff(%0.2f)\n",
             i, end_point_on_dst.l, com_rad2deg(h_diff));
#endif
      if ((common::com_abs(end_point_on_dst.l) < param.max_lat_offset) &&
          (common::com_abs(h_diff) < param.max_heading_offset)) {
        break;
      }
    }
  }

  if (common::com_abs(start_point_on_dst.l) > param.max_lat_offset) {
    LOG_INFO(5) << "The offset between two path is larger than threshold, dist="
                << start_point_on_dst.l;
    return false;
  }
  if (common::com_abs(end_point_on_dst.l) > param.max_lat_offset) {
    LOG_INFO(5) << "The offset between two path is larger than threshold, dist="
                << end_point_on_dst.l;
    return false;
  }
  if ((end_point_on_src.s - start_point_on_src.s) < param.min_matched_len) {
    LOG_INFO(5) << "The matched length is less than threshold, length="
                << end_point_on_src.s - start_point_on_src.s;
    return false;
  }

  if (Nullptr_t != info) {
    info->start_point_on_dst = start_point_on_dst;
    info->end_point_on_dst = end_point_on_dst;
    info->start_point_on_src = start_point_on_src;
    info->end_point_on_src = end_point_on_src;
  }

#if 1
  geo_var_t len = this->total_length() - start_point_on_dst.s;
  common::PathPoint other_point;
  if (len > param.max_len_for_heading) {
    this->FindSmoothPoint(
          start_point_on_dst.s+param.max_len_for_heading, &other_point);
    start_point_on_dst.heading = common::GetHeadingFromSegment(
          start_point_on_dst.point, other_point.point);
  } else if (len > param.min_len_for_heading) {
    start_point_on_dst.heading = common::GetHeadingFromSegment(
          start_point_on_dst.point, this->points().Back());
  } else {
    // nothing to do
  }

  len = src_path.total_length() - start_point_on_src.s;
  if (len > param.max_len_for_heading) {
    src_path.FindSmoothPoint(
          start_point_on_src.s+param.max_len_for_heading, &other_point);
    start_point_on_src.heading = common::GetHeadingFromSegment(
          start_point_on_src.point, other_point.point);
  } else if (len > param.min_len_for_heading) {
    start_point_on_src.heading = common::GetHeadingFromSegment(
          start_point_on_src.point, src_path.points().Back());
  } else {
    // nothing to do
  }
#endif

  geo_var_t heading_diff = AngleDiff(
        start_point_on_dst.heading, start_point_on_src.heading);
  Vec2d pos_diff = start_point_on_src.point - start_point_on_dst.point;
  common::Matrix<geo_var_t, 2, 1> point_conv;
  common::Matrix<geo_var_t, 3, 3> mat_conv;
  mat_conv.SetIdentity();
  common::Matrix<geo_var_t, 2, 1> rotate_center(
        start_point_on_dst.point.x(), start_point_on_dst.point.y());
  common::Rotate_2D<geo_var_t>(rotate_center, heading_diff, &mat_conv);
  common::Translate_2D(pos_diff.x(), pos_diff.y(), &mat_conv);

#if 0
  LOG_INFO(5) << "At start, the offset from dst to src is (" << pos_diff.x()
              << ", " << pos_diff.y()
              << ", " << common::com_rad2deg(heading_diff) << ").";
#endif

  if (Nullptr_t != info) {
    info->pos_diff_start.p_diff = pos_diff;
    info->pos_diff_start.h_diff = heading_diff;
    info->mat_conv_start = mat_conv;
  }

  param.points_2d->Clear();
  for (Int32_t i = 0; i <= start_point_on_dst_idx; ++i) {
    point_conv(0) = this->points()[i].x();
    point_conv(1) = this->points()[i].y();
    common::TransformVert_2D(mat_conv, &point_conv);
    param.points_2d->PushBack(common::Vec2d(point_conv(0), point_conv(1)));
  }

  param.points_2d->PushBack(start_point_on_src.point);
  for (Int32_t i = start_point_on_src_idx+1; i <= end_point_on_src_idx; ++i) {
    param.points_2d->PushBack(src_path.points()[i]);
  }
  param.points_2d->PushBack(end_point_on_src.point);


#if 1
  len = end_point_on_dst.s;
  if (len > param.max_len_for_heading) {
    this->FindSmoothPoint(
          end_point_on_dst.s-param.max_len_for_heading, &other_point);
    end_point_on_dst.heading = common::GetHeadingFromSegment(
          other_point.point, end_point_on_dst.point);
  } else if (len > param.min_len_for_heading) {
    end_point_on_dst.heading = common::GetHeadingFromSegment(
          this->points().Front(), end_point_on_dst.point);
  } else {
    // nothing to do
  }

  len = end_point_on_src.s;
  if (len > param.max_len_for_heading) {
    src_path.FindSmoothPoint(
          end_point_on_src.s-param.max_len_for_heading, &other_point);
    end_point_on_src.heading = common::GetHeadingFromSegment(
          other_point.point, end_point_on_src.point);
  } else if (len > param.min_len_for_heading) {
    end_point_on_src.heading = common::GetHeadingFromSegment(
          src_path.points().Front(), end_point_on_src.point);
  } else {
    // nothing to do
  }
#endif

  heading_diff = AngleDiff(
        end_point_on_dst.heading, end_point_on_src.heading);
  pos_diff = end_point_on_src.point - end_point_on_dst.point;
  mat_conv.SetIdentity();
  rotate_center(0) = end_point_on_dst.point.x();
  rotate_center(1) = end_point_on_dst.point.y();
  common::Rotate_2D<geo_var_t>(rotate_center, heading_diff, &mat_conv);
  common::Translate_2D(pos_diff.x(), pos_diff.y(), &mat_conv);

#if 0
  LOG_INFO(5) << "At end, the offset from dst to src is (" << pos_diff.x()
              << ", " << pos_diff.y()
              << ", " << common::com_rad2deg(heading_diff) << ").";
#endif

  if (Nullptr_t != info) {
    info->pos_diff_end.p_diff = pos_diff;
    info->pos_diff_end.h_diff = heading_diff;
    info->mat_conv_end = mat_conv;
  }

  for (Int32_t i = end_point_on_dst_idx+1; i < this->points().Size(); ++i) {
    point_conv(0) = this->points()[i].x();
    point_conv(1) = this->points()[i].y();
    common::TransformVert_2D(mat_conv, &point_conv);
    param.points_2d->PushBack(common::Vec2d(point_conv(0), point_conv(1)));
  }

#if 0
  printf("source path=\n");
  for (Int32_t i = 0; i < src_path.points().Size(); ++i) {
    printf("p[%d]: (%0.1f, %0.1f)\n",
           i, src_path.points()[i].x(), src_path.points()[i].y());
  }
  printf("After merge, points=\n");
  for (Int32_t i = 0; i < param.points_2d->Size(); ++i) {
    printf("p[%d]: (%0.1f, %0.1f)\n",
           i, (*param.points_2d)[i].x(), (*param.points_2d)[i].y());
  }
#endif

  if (!this->Construct(*param.points_2d)) {
    LOG_ERR << "Failed to construct destination path.";
    return false;
  }

#if 0
  LOG_INFO(5) << "### Replace dst path (" << start_point_on_dst.s
              << ", " << end_point_on_dst.s
              << ")(len=" << end_point_on_dst.s - start_point_on_dst.s
              << ") with src path (" << start_point_on_src.s
              << ", " << end_point_on_src.s
              << ")(len=" << end_point_on_src.s - start_point_on_src.s
              << ")";
#endif

  return true;
}

/*
 * @brief 延长轨迹
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author     <th>Description
 * <tr><td>2022/12/29  <td>1.0      <td>pengc      <td>First edition
 * </table>
 */
bool Path::Extend(const ExtendParam& param) {
  Int32_t path_points_num = points().Size();

  if (Nullptr_t == param.points_2d) {
    LOG_ERR << "Invalid parameters.";
    return (false);
  }
  if (path_points_num < 2) {
    LOG_ERR << "Invalid path.";
    return (false);
  }

  param.points_2d->Clear();

  if (param.ex_len_backward > 0.1F) {
    Int32_t next_index = 1;
    for (; next_index < (path_points_num-1); ++next_index) {
      if (accumulated_s()[next_index] > param.interpolation_search_len) {
        break;
      }
    }

    geo_var_t t = (-param.ex_len_backward) /
        (accumulated_s()[next_index] - accumulated_s()[0]);
    common::Vec2d p = (1.0F - t) * points()[0] + t * points()[next_index];

    param.points_2d->PushBack(p);
  }

  for (Int32_t i = 0; i < path_points_num; ++i) {
    param.points_2d->PushBack(points()[i]);
  }

  if (param.ex_len_forward > 0.1F) {
    Int32_t next_index = path_points_num-2;
    for (; next_index > 0; --next_index) {
      if ((accumulated_s()[path_points_num-1] -
           accumulated_s()[next_index]) > param.interpolation_search_len) {
        break;
      }
    }

    geo_var_t t =
        (param.ex_len_forward + accumulated_s()[path_points_num-1] -
         accumulated_s()[next_index]) /
        (accumulated_s()[path_points_num-1] - accumulated_s()[next_index]);
    common::Vec2d p = (1.0F - t) * points()[next_index] +
        t * points()[path_points_num-1];

    param.points_2d->PushBack(p);
  }

  if (!Construct(*param.points_2d)) {
    LOG_ERR << "Failed to construct path.";
    return false;
  }

  return true;
}

/*
 * @brief 对轨迹进行空间转换
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author     <th>Description
 * <tr><td>2022/12/29  <td>1.0      <td>pengc      <td>First edition
 * </table>
 */
bool Path::Convert(const ConvertParam& param) {
  Int32_t path_points_num = points().Size();

  if (Nullptr_t == param.mat_conv) {
    LOG_ERR << "Invalid parameters.";
    return (false);
  }
  if (Nullptr_t == param.points_2d) {
    LOG_ERR << "Invalid parameters.";
    return (false);
  }
  if (path_points_num < 2) {
    LOG_ERR << "Invalid path.";
    return (false);
  }

  param.points_2d->Clear();

  Matrix<geo_var_t, 2, 1> point_conv;
  for (Int32_t i = 0; i < path_points_num; ++i) {
    point_conv(0) = points()[i].x();
    point_conv(1) = points()[i].y();
    TransformVert_2D(*param.mat_conv, &point_conv);
    param.points_2d->PushBack(common::Vec2d(point_conv(0), point_conv(1)));
  }

  if (!Construct(*param.points_2d)) {
    LOG_ERR << "Failed to construct path.";
    return false;
  }

  return true;
}
/* k004 pengc 2022-12-26 (end) */


}  // namespace common
}  // namespace phoenix


