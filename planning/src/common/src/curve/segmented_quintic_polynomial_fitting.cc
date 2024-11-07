/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       segmented_quintic_polynomial_fitting.cc
 * @brief      分段五次多项式拟合
 * @details    实现了分段五次多项式拟合方法。\n
 *             实现的基本思路为先将输入的路径分为首尾相连的多个\n
 *             曲线段，然后将每一条曲线段按照长度进行切割，这样就\n
 *             得到许多个比较短的曲线片段，然后对这些曲线片段，\n
 *             使用以弧长作为参数的五次多项式进行拟合，得到一条\n
 *             五次多项式拟合曲线。最后从这些拟合出来的五次多项式\n
 *             曲线片段中进行采样，得到平滑后的采样点。
 *
 * @author     boc
 * @date       2020.05.15
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/15  <td>1.0      <td>boc       <td>First edition
 * </table>
 *
 ******************************************************************************/

#include "curve/segmented_quintic_polynomial_fitting.h"
#include "curve/quintic_polynomial_fitting.h"

// Enable to output debug information
#define ENABLE_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_TRACE (0)

#define ENABLE_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_TRACE_TIMER (0)

namespace phoenix {
namespace common {

/**
 * @date       2020.05.23
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/23  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
SegmentedQuinticPolynomialFitting::SegmentedQuinticPolynomialFitting() {
}

/**
 * @date       2020.05.23
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/23  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
SegmentedQuinticPolynomialFitting::~SegmentedQuinticPolynomialFitting() {
}

/**
 * @date       2020.05.23
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/23  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
bool SegmentedQuinticPolynomialFitting::Fit(
    const Path& path, const StaticVector<Path::CurveSegment,
    Path::kMaxCurveSegmentNum>& curve_segments,
    StaticVector<PathPoint, Path::kMaxPathPointNum>* sample_points) {
#if ENABLE_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_TRACE_TIMER
  // 对耗时进行统计
  phoenix::common::Stopwatch timer;
#endif
  Int32_t curve_segments_count = 0;
  Int32_t i = 0;
  // ‘转弯半径 - 拟合点步长’对照表
  static const geo_var_t radius_step_table[2][2] = {
    // 弯道转弯半径    弯道时的两个相邻拟合点的距离
    {1.0f / 0.100f,      1.000f},
    // 直道转弯半径    直道时的两个相邻拟合点的距离
    {1.0f / 0.008f,      3.160f}
  };
  // 弯道转弯半径
  geo_var_t r1                = radius_step_table[0][0];
  // 两个拟合点之间的最小长度 (弯道时的两个相邻拟合点的距离)
  geo_var_t step_u_min        = radius_step_table[0][1];
  // 直道转弯半径
  geo_var_t r2                = radius_step_table[1][0];
  // 直道曲率
  geo_var_t curvature_straight = 1.0f / r2;
  // 两个拟合点之间的最大长度
  geo_var_t step_u_max        = radius_step_table[1][1];
  // 两个拟合点之间的长度
  geo_var_t step_u = step_u_max;

  if (sample_points == Nullptr_t) {
    return false;
  }

  TackeleCurveSegment(path, curve_segments);
  curve_segments_count = curve_segments_.Size();
  u_sample_ = 0.0f;
  u_sample_previous_ = 0.0f;
  if (curve_segments_count == 0) {
    // 曲线上点的个数小于2时，返回失败
    if (path.points().Size() < 2) {
      return false;
    }
    // 当曲线段的个数为0时，直接将原始点进行输出。
    for (i = 0; i < path.points().Size(); ++i) {
      PathPoint point;
      point.point     = path.points()[i];
      point.heading   = path.headings()[i];
      point.curvature = path.curvatures()[i];
      point.s         = path.accumulated_s()[i];
      point.l         = 0.0f;
      sample_points->PushBack(point);
    }
    return true;
  }
  // 根据曲线段信息，进行分段拟合
  for (i = 0; i < curve_segments_count; ++i) {
  #if ENABLE_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_TRACE
    std::cout << "Curve segment " << i << std::endl;
  #endif
    geo_var_t total_u = curve_segments_[i].length;
    // 计算两个拟合点之间的距离
    geo_var_t seg_curvature_abs = com_abs(curve_segments_[i].max_curvature);
    step_u = step_u_max;
    if (seg_curvature_abs > curvature_straight) {
      // 弯道的转弯半径
      geo_var_t r = 1 / seg_curvature_abs;
      // step_u = (8 * 0.01 * r)^(1 / 2)
      // 采样步长公式来源：线段上的点到圆弧的最大距离不超过0.01米
      step_u = com_sqrt(0.08f * r);
    }

    // 防止两个拟合点之间的距离越界
    step_u = Clamp<geo_var_t>(step_u, step_u_min, step_u_max);
  #if ENABLE_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_TRACE
    std::cout << "Curve segment : start_s=" << curve_segments_[i].start_s <<
                 ", length=" << curve_segments_[i].length <<
                 ", type=" << curve_segments_[i].type <<
                 ", max_curvature=" << curve_segments_[i].max_curvature <<
                 std::endl;
  #endif
    if (i > 0) {
      total_u += curve_segments_[i].start_s - u_sample_previous_;
    }
    if ((i + 1) == curve_segments_count &&
        path.total_length() <= step_u_max * (kFitPointNum - 1)) {
      // 路径总长度较短时，使用较短的采样步长
      step_u = step_u_min;
    }
    if (total_u <= step_u_min * (kFitPointNum - 1)) {
      // 曲线段长度较短时，使用较短的采样步长
      step_u = step_u_min;
    }
#if ENABLE_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_TRACE
  std::cout << "total u : " << total_u << ", step_u=" <<
               step_u << std::endl;
#endif
    // 用于进行多项式拟合的线的长度
    fit_length_ = step_u * (kFitPointNum - 1);
    // 曲线段切割的块数
    fit_segment_num_ = com_ceil(total_u / fit_length_);
    if (fit_segment_num_ < 1) {
      fit_segment_num_ = 1;
    }

  #if ENABLE_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_TRACE
    std::cout << "fit_segment_num_ = " << fit_segment_num_ << std::endl;
  #endif
    for (Int32_t i2 = 0; i2 < fit_segment_num_; ++i2) {
      // 当处于两条曲线段的交界处时，对拟合点间隔长度进行调整
      if (i2 + 1 == fit_segment_num_) {
        if (step_u * kFitPointNum >
            (curve_segments_[i].start_s + curve_segments_[i].length -
             sample_point_previous_.s)) {
          step_u = (curve_segments_[i].start_s + curve_segments_[i].length -
                    sample_point_previous_.s) / kFitPointNum;
          if (step_u < step_u_min) {
            step_u = step_u_min;
          }
        }
      }
      if (!FitSigSegment(path, i, i2, step_u, sample_points)) {
        return true;
      }
      if (sample_points->Size() >= Path::kMaxPathPointNum) {
        // 采样点个数达到了最大值时，直接结束采样
        return true;
      }
//      if (fit_segment_num_ > 30) {
//        // 预防拟合的段数过多
//        break;
//      }
    }  // for (Int32_t i2 ...
  }  // for (i ...

#if ENABLE_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_TRACE
  // 计算采样之后的横向最大误差
  geo_var_t farthest_l = 0.0f;
  Int32_t farthest_index = -1;
  for (i = 0; i < sample_points->Size(); ++i) {
    const PathPoint& point = (*sample_points)[i];
    PathPoint farthest_point;
    path.FindNearest(point.point, &farthest_point);
    if (com_abs(farthest_point.l) > farthest_l) {
      farthest_l = com_abs(farthest_point.l);
      farthest_index = i;
    }
  }
  std::cout << "Farthest point 1 is : i=" << farthest_index << ", l=" <<
               farthest_l << std::endl;

  // 计算原始路径点到拟合后的曲线的距离
  Path path_fit;
  path_fit.Construct(*sample_points);
  farthest_l = 0.0f;
  farthest_index = -1;
  for (i = 0; i < path.points().Size(); ++i) {
    const Vec2d& point = path.points()[i];
    PathPoint farthest_point;
    path_fit.FindNearest(point, &farthest_point);
    if (com_abs(farthest_point.l) > farthest_l) {
      farthest_l = com_abs(farthest_point.l);
      farthest_index = i;
    }
  }
  std::cout << "Farthest point 2 is : i=" << farthest_index << ", l=" <<
               farthest_l << std::endl;
#endif

#if ENABLE_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_TRACE_TIMER
  std::cout << "Fit curve spend " << timer.Elapsed() << "ms." << std::endl;
#endif

  return true;
}

/**
 * @date       2020.05.23
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/23  <td>1.0      <td>boc       <td>First edition
 * </table>
 * @note 当sample_points中已经有一些点时，会将新采的点插入到已有最后一个点的后面。
 */
bool SegmentedQuinticPolynomialFitting::FitSigSegment(
    const Path& path,
    Int32_t i_curve_segment, Int32_t i_fit_segment, geo_var_t step_u,
    StaticVector<PathPoint, Path::kMaxPathPointNum>* sample_points) {
#if ENABLE_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_TRACE
  std::cout << "\nCurve segment " << i_curve_segment << ", " <<
               "fit segment " << i_fit_segment << std::endl;
#endif
  Int32_t curve_segments_count = curve_segments_.Size();
  Int32_t j = 0;  // 用于给拟合点个数进行计数的变量
  // 两个采样点之间的长度
  geo_var_t step_u_sample = 2.0;
  geo_var_t u = 0.0f;  // 与上一个拟合点之间的弧长
  geo_var_t u0 = 0.0f;  // 拟合片段的第一个拟合点在原路径上的弧长
  // 在路径上采样单个拟合点时的临时变量
  StaticVector<PathPoint, Path::kMaxPathPointNum> fit_point_sampled;
  // 用于进行曲线拟合的单个拟合点
  PathPoint fit_point;
  PathPoint point;
  Int32_t index = 0;  // 当前拟合点的索引
  // 拟合五次多项式曲线时使用的的点数
  Int32_t fit_size = kFitPointNum;

  step_u_sample = step_u;

  // 获取拟合点
  if (i_curve_segment != 0 || i_fit_segment != 0) {
    fit_point = sample_point_previous_;
    Int32_t lb_fit_index = fit_points_.LowerBound(fit_point,
                                                  FuncCmpPathPointArcLen());
    geo_var_t dist = fit_points_[lb_fit_index].point.DistanceTo(
          fit_point.point);
    // 重新计算起点拟合点的弧长
    fit_point.s = fit_points_[lb_fit_index].s - dist;
  } else {
    index = 0;
    fit_point.point     = path.points()[index];
    fit_point.heading   = path.headings()[index];
    fit_point.curvature = path.curvatures()[index];
    fit_point.s         = path.accumulated_s()[index];
    fit_point.l         = 0.0f;
  }
  fit_points_.Clear();
  fit_points_.PushBack(fit_point);
  // u0 = path_tool_.accumulated_s()[index_start_];
  u0 = fit_point.s;
  for (j = 1; j < fit_size; ++j) {
    u = u0 + j * step_u;
    if (u < path.total_length()) {
      fit_point_sampled.Clear();
      path.UniformlySamplePathForward(u, 0, step_u, &fit_point_sampled);
      if (fit_point_sampled.Size() > 0) {
        fit_point = fit_point_sampled[0];
        fit_points_.PushBack(fit_point);
      }
    }
    // 拟合长度满足条件，且拟合点个数也满足条件时，退出拟合点查询步骤
    if (fit_points_.Back().s - u0 >= fit_length_ &&
        fit_points_.Size() >= kFitPointNum) {
      break;
    }
  }

#if ENABLE_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_TRACE
  std::cout << "The size of points to fit : " << fit_points_.Size() <<
               std::endl;
#endif

  Int32_t fit_points_size_tmp = fit_points_.Size();
  if (fit_points_size_tmp < kFitPointNum) {
    // 需要额外添加的拟合点的个数
    Int32_t extra_points_size = kFitPointNum - fit_points_size_tmp;
    // if (path.points().Size() < kFitPointNum) {
    if (path.total_length() < (kFitPointNum + 1) * step_u) {
      // 路径总长过短，无法进行拟合。
      // 直接输出路径的信息。
//      LOG_WARN << "Not enough path length to fit. Path length is " <<
//                  path.total_length();
      sample_points->Clear();
      for (j = 0; j < path.points().Size(); ++j) {
        point.point     = path.points()[j];
        point.heading   = path.headings()[j];
        point.curvature = path.curvatures()[j];
        point.s         = path.accumulated_s()[j];
        point.l         = 0.0f;
        sample_points->PushBack(point);
      }
      return false;
    } else if (u0 >= extra_points_size * step_u) {
      // 点数过少时，添加一部分前面的点
      StaticVector<PathPoint, kFitPointNum> fit_points_tmp =
          fit_points_;
      fit_points_.Clear();
      for (j = extra_points_size; j > 0; --j) {
        u = u0 - j * step_u;
        if (u >= 0) {
          fit_point_sampled.Clear();
          path.UniformlySamplePathBackward(u, 0, step_u,
                                           &fit_point_sampled);
          if (fit_point_sampled.Size() > 0) {
            fit_point = fit_point_sampled[0];
            fit_points_.PushBack(fit_point);
          }
        }
      }
    #if ENABLE_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_TRACE
      std::cout << "Add extra " << extra_points_size <<
                   " previous points as fit points\n";
    #endif
      for (j = 0; j < fit_points_tmp.Size(); ++j) {
        fit_points_.PushBack(fit_points_tmp[j]);
      }
    } else {
      // 前面和后面的点数均不够时，不进行拟合。
      LOG_WARN << "Path length not enough to fit. Curve length is " <<
                  path.total_length();
      sample_points->Clear();
      for (j = 0; j < path.points().Size(); ++j) {
        point.point     = path.points()[j];
        point.heading   = path.headings()[j];
        point.curvature = path.curvatures()[j];
        point.s         = path.accumulated_s()[j];
        point.l         = 0.0f;
        sample_points->PushBack(point);
      }
      return false;
    }
  }

#if ENABLE_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_TRACE
  std::cout << "The size of points after adding extra points to fit : " <<
               fit_points_.Size() << std::endl;
  // 打印拟合点的信息
  std::cout << "Fit points is : " << std::endl;
  for (j = 0; j < fit_points_.Size(); ++j) {
    std::cout << "  " << j << "\t{" <<
                 fit_points_[j].point.x() << "," <<
                 fit_points_[j].point.y() << "},\t" <<
                 fit_points_[j].s << ",\t" <<
                 fit_points_[j].heading << ",\t" <<
                 fit_points_[j].curvature << std::endl;
  }
  std::cout << std::endl;
#endif
  // 进行拟合
  if (i_curve_segment == 0 && i_fit_segment == 0) {
    // 第一个曲线段的首条拟合段使用不带起点约束的方式进行拟合
    fitter_.ConstructWithoutConstraint(fit_points_);
  } else {
    // 非第一个拟合段使用带起点约束的方式进行拟合
    // 第一个拟合点必须与上一个拟合段的最后一个采样点相同
    fitter_.ConstructWithStartConstraint(fit_points_);
  }
  // 进行采样，根据曲率等间隔采样
  for (; u_sample_ <= fitter_.end_s(); ) {
    // 当不是最后一个曲线段或者不是最后一个拟合段时，
    // 只对中间的拟合曲线进行采样
    if (i_curve_segment != (curve_segments_count - 1) ||
        i_fit_segment != (fit_segment_num_ - 1)) {
      if (u_sample_ + 2.0f * step_u_sample > fitter_.end_s()) {
        break;
      }
    }
    if (i_curve_segment != (curve_segments_count - 1) ||
        i_fit_segment != (fit_segment_num_ - 1)) {
      // 当不是最后一个曲线段且不是最后一个拟合段时，
      // 只对中间的曲线段上的点进行采样
      if (u_sample_ > curve_segments_[i_curve_segment].start_s +
          curve_segments_[i_curve_segment].length) {
        break;
      }
    }
    fitter_.CalcSamplePoint(u_sample_, &point);
    point.s = u_sample_;
    u_sample_previous_ = u_sample_;
    sample_point_previous_ = point;
    sample_points->PushBack(point);
  #if ENABLE_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_TRACE
    std::cout << i_fit_segment << "\t{" << point.point.x() << "," <<
                 point.point.y() << "},\t" << point.s << ",\t" <<
                 point.heading << ",\t" <<
                 point.curvature << std::endl;
  #endif
    u_sample_ += step_u_sample;
  }

  if (i_fit_segment == (fit_segment_num_ - 1)) {
    // 当曲线段的最后一个拟合段与最后一个采样点离曲线段的最后一个点相隔距离较大时，
    // 再多添加一段拟合曲线进行采样。
    if (sample_points->Back().s + 4.1f * step_u_sample <
        curve_segments_[i_curve_segment].start_s +
        curve_segments_[i_curve_segment].length) {
      fit_segment_num_++;
      return true;
    }
  }

  if (i_curve_segment == (curve_segments_count - 1) &&
      i_fit_segment == (fit_segment_num_ - 1)) {
    // 当最后一个采样点离原始曲线的最后一个点相隔距离较大时，
    // 再多添加一段拟合曲线进行采样。
    if (sample_points->Back().s + 2.0f * step_u < path.total_length()) {
      fit_segment_num_++;
      return true;
    }
    // 将最后一个点添加上
    if (sample_points->Back().s + 0.2f < path.total_length()) {
      // 采样点的最后一个点与原始曲线的最后一个点的距离较大时，
      // 将原始曲线的最后一个点添加到采样点中。
      point.point         = path.points().Back();
      point.heading       = path.headings().Back();
      point.curvature     = path.curvatures().Back();
      point.s             = path.accumulated_s().Back();
      point.l             = 0.0f;
      sample_points->PushBack(point);
    } else {
      // 采样点的最后一个点与原始曲线的最后一个点的距离较小时，
      // 将原始曲线的最后一个点的信息替换采样点的最后一个点的信息。
      point.point         = path.points().Back();
      point.heading       = path.headings().Back();
      point.curvature     = path.curvatures().Back();
      point.s             = path.accumulated_s().Back();
      point.l             = 0.0f;
      (*sample_points)[sample_points->Size() - 1] = point;
    }
  }  // if (i_curve_segment == (curve_segments_count - 1) && ...

  return true;
}

/**
 * @date       2020.05.23
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2020/05/23  <td>1.0      <td>boc       <td>First edition
 * </table>
 */
void SegmentedQuinticPolynomialFitting::TackeleCurveSegment(
    const Path& path, const StaticVector<Path::CurveSegment,
    Path::kMaxCurveSegmentNum>& curve_segments) {
  geo_var_t segments_length_sum = 0.0f;
  Path::CurveSegment curve_segment;
  Int32_t i = 0;
  curve_segments_.Clear();
  if (path.total_length() <= 0.1f) {
    // 路径长度过短，不予处理
    return;
  }
  for (i = 0; i < curve_segments.Size(); ++i) {
    segments_length_sum += curve_segments[i].length;
  }
  if (com_abs(segments_length_sum - path.total_length()) > 0.2f) {
    // 中间某些点没有加入到曲线段中，将这些散落的点作为曲线段添加到曲线段的信息中。
    // 这些散落的曲线段的长度可能会小于3米。
    if (curve_segments.Size() < 1) {
      curve_segment.start_s = 0.0f;
      curve_segment.length = path.total_length();
      curve_segment.max_curvature = 0.01f;
      curve_segment.ave_curvature = 0.01f;
      curve_segment.type = Path::TYPE_CURVING_LEFT;
      curve_segments_.PushBack(curve_segment);
      return;
    }
    if (curve_segments[0].start_s > 0.01f) {
      // 第一条曲线段的起点不在路径段的起点时，将其前方的路径作为曲线段添加到曲线段列表中
      curve_segment.start_s = 0.0f;
      curve_segment.length = curve_segments[0].start_s;
      curve_segment.max_curvature = 0.01f;
      curve_segment.ave_curvature = 0.01f;
      curve_segment.type = Path::TYPE_CURVING_LEFT;
      curve_segments_.PushBack(curve_segment);
    }
    for (i = 0; i < curve_segments.Size(); ++i) {
      curve_segments_.PushBack(curve_segments[i]);
      if (i + 1 < curve_segments.Size()) {
        if (curve_segments[i].start_s + curve_segments[i].length + 0.01f <
            curve_segments[i + 1].start_s) {
          // 将两个曲线段的空隙作为曲线段添加到曲线段列表中
          curve_segment.start_s = curve_segments[i].start_s +
              curve_segments[i].length;
          curve_segment.length = curve_segments[i + 1].start_s -
              curve_segments[i].start_s - curve_segments[i].length;
          curve_segment.max_curvature = 0.01f;
          curve_segment.ave_curvature = 0.01f;
          curve_segment.type = Path::TYPE_CURVING_LEFT;
          curve_segments_.PushBack(curve_segment);
        }
      }
    }
    // 最后一个曲线段的终点离路径终点有一段距离时，将这段间隙作为曲线段进行添加
    if (curve_segments[i - 1].start_s + curve_segments[i - 1].length + 0.01f <
        path.total_length()) {
      // 将两个曲线段的空隙作为曲线段添加到曲线段列表中
      curve_segment.start_s = curve_segments[i - 1].start_s +
          curve_segments[i - 1].length;
      curve_segment.length = path.total_length() - curve_segment.start_s;
      curve_segment.max_curvature = 0.01f;
      curve_segment.ave_curvature = 0.01f;
      curve_segment.type = Path::TYPE_CURVING_LEFT;
      curve_segments_.PushBack(curve_segment);
    }
  } else {
    curve_segments_ = curve_segments;
  }
}

}  // namespace common
}  // namespace phoenix


