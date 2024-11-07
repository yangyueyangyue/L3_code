/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       segmented_quintic_polynomial_fitting.h
 * @brief      分段五次多项式拟合
 * @details    实现了分段五次多项式拟合方法
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
#ifndef PHOENIX_COMMON_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_H_
#define PHOENIX_COMMON_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_H_

#include "utils/macros.h"
#include "utils/log.h"
#include "curve/path.h"
// #include "curve/quintic_polynomial_fitting.h"
#include "curve/quintic_polynomial_fitting_fix_qr.h"


namespace phoenix {
namespace common {

/**
 * @class SegmentedQuinticPolynomialFitting
 * @brief 分段五次多项式拟合，提供了路径点信息输入，获取平滑后的采样点等功能。
 */
class SegmentedQuinticPolynomialFitting {
public:
  /**
   * @brief 构造函数。
   */
  SegmentedQuinticPolynomialFitting();
  /**
   * @brief 析构函数。
   */
  ~SegmentedQuinticPolynomialFitting();
  /**
   * @brief       使用分段五次多项式曲线对路径进行拟合，并输出平滑后的采样点。
   * @param[in]   path             需要平滑的路径
   * @param[in]   curve_segments   路径对应的曲线段信息
   * @param[out]  sample_points    平滑后的采样点
   * @return      true   采样成功；\n
   *              false  采样失败（输入的采样点指针为空时）。
   */
  bool Fit(const Path& path, const StaticVector<Path::CurveSegment,
           Path::kMaxCurveSegmentNum>& curve_segments,
           StaticVector<PathPoint, Path::kMaxPathPointNum>* sample_points);

private:
  /// 比较路径点的弧长函数类
  class FuncCmpPathPointArcLen {
  public:
    bool operator ()(PathPoint a, PathPoint b) const {
      if (a.s > b.s) {
        return true;
      }
      return false;
    }
  };

private:
  /**
  * @brief       对曲线段的信息进行处理，使得曲线段长度的和与路径总长度相等。
  */
  void TackeleCurveSegment(
      const Path& path, const StaticVector<Path::CurveSegment,
      Path::kMaxCurveSegmentNum>& curve_segments);
  /**
   * @brief           对曲线段中的某一个片段进行拟合。
   * @param[in]       i_curve_segment   曲线段的索引
   * @param[in]       i_fit_segment     曲线段中拟合片段的索引
   * @param[in]       step_u            两个拟合点之间的长度
   * @param[in&out]   sample_points     输出的采样点
   * @return          true    成功，需要进行下一个拟合片段的采样；\n
   *                  false   失败，拟合点数过少，直接以输入点作为输出。
   * @note 当sample_points中已经有一些点时，会将新采的点插入到已有最后一个点的后面。
   */
  bool FitSigSegment(
      const Path& path,
      Int32_t i_curve_segment, Int32_t i_fit_segment, geo_var_t step_u,
      StaticVector<PathPoint, Path::kMaxPathPointNum>* sample_points);

private:
  /// 拟合五次多项式时的最小点个数
  static const Int32_t kFitPointNum = 20;
  /// 曲线段列表
  StaticVector<Path::CurveSegment, Path::kMaxCurveSegmentNum> curve_segments_;
  /// 五次多项式曲线拟合器
  QuinticPolynomialFittingFixQR<geo_var_t, kFitPointNum> fitter_;
  /// 用于进行曲线拟合的所有拟合点
  StaticVector<PathPoint, kFitPointNum> fit_points_;
  /// 从拟合曲线上进行采样的采样点的弧长
  geo_var_t u_sample_;
  /// 从拟合曲线上进行采样的前一个采样点的弧长
  geo_var_t u_sample_previous_;
  /// 从拟合曲线上进行采样的前一个采样点
  PathPoint sample_point_previous_;
  /// 用于进行多项式拟合的线的长度
  geo_var_t fit_length_;
  /// 进行拟合时，曲线段切割的块数
  Int32_t fit_segment_num_;
};

}  // namespace common
}  // namespace phoenix


#endif  // PHOENIX_COMMON_SEGMENTED_QUINTIC_POLYNOMIAL_FITTING_H_
