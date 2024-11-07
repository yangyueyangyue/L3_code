/******************************************************************************
 ** Cubic Polynomial Curve
 ******************************************************************************
 *
 *  Cubic Polynomial Curve (1D)
 *
 *  @file       cubic_polynomial_curve1d.h
 *
 *  @author     kotei
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/

#ifndef PHOENIX_COMMON_CUBIC_POLYNOMIAL_CURVE1D_H_
#define PHOENIX_COMMON_CUBIC_POLYNOMIAL_CURVE1D_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "math/math_utils.h"

namespace phoenix {
namespace common {


// 1D cubic polynomial curve:
// (x0, dx0) -- [0, param] --> (x1, dx1)
template<typename Scalar>
class CubicPolynomialCurve1d {
public:
  CubicPolynomialCurve1d() {
    param_ = 0;
    com_memset(coef_, 0, sizeof(coef_));
  }

  CubicPolynomialCurve1d(const Scalar start[2],
                         const Scalar end[2],
                         const Scalar param) {
    CubicPolynomialCurve1d(start[0], start[1], end[0], end[1], param);
  }

  CubicPolynomialCurve1d(const Scalar x0, const Scalar dx0,
                         const Scalar x1, const Scalar dx1,
                         const Scalar param) {
    ComputeCoefficients(x0, dx0, x1, dx1, param);
    param_ = param;
  }

  CubicPolynomialCurve1d(const CubicPolynomialCurve1d& other) {
    param_ = other.param_;
    com_memcpy(coef_, other.coef_, sizeof(coef_));
  }

  virtual ~CubicPolynomialCurve1d() {
    // nothing to do
  }

  void Clear() {
    param_ = 0;
    com_memset(coef_, 0, sizeof(coef_));
  }

  void Construct(const Scalar x0, const Scalar dx0,
                 const Scalar x1, const Scalar dx1,
                 const Scalar param) {
    ComputeCoefficients(x0, dx0, x1, dx1, param);
    param_ = param;
  }

  void SetCoefficient(const Scalar c0, const Scalar c1,
                      const Scalar c2, const Scalar c3) {
    coef_[0] = c0;
    coef_[1] = c1;
    coef_[2] = c2;
    coef_[3] = c3;
  }

  Scalar Evaluate(const Uint32_t order, const Scalar p) const;

private:
  void ComputeCoefficients(const Scalar x0, const Scalar dx0,
                           const Scalar x1, const Scalar dx1,
                           const Scalar param);

private:
  Scalar coef_[4];
  Scalar param_;
};

template<typename Scalar>
Scalar CubicPolynomialCurve1d<Scalar>::Evaluate(
    const Uint32_t order, const Scalar p) const {
  Scalar ret = Scalar(0);

  switch (order) {
  case 0:
    ret = ((coef_[3] * p + coef_[2]) * p + coef_[1]) * p + coef_[0];
    break;
  case 1:
    ret = (Scalar(3) * coef_[3] * p + Scalar(2) * coef_[2]) * p + coef_[1];
    break;
  case 2:
    ret = Scalar(6) * coef_[3] * p + Scalar(2) * coef_[2];
    break;
  case 3:
    ret = Scalar(6) * coef_[3];
    break;
  default:
    ret = Scalar(0);
    break;
  }

  return (ret);
}

template<typename Scalar>
void CubicPolynomialCurve1d<Scalar>::ComputeCoefficients(
    const Scalar x0, const Scalar dx0,
    const Scalar x1, const Scalar dx1,
    const Scalar p) {
  COM_CHECK(p > 0);

  coef_[0] = x0;
  coef_[1] = dx0;

  const Scalar p2 = p * p;

  const Scalar c0 = (x1 - x0 - dx0 * p) / p2;
  const Scalar c1 = (dx1 - dx0) / p;

  coef_[2] = 3.0F * c0 - c1;
  coef_[3] = (c0 - coef_[2]) / p;
}


}  // namespace common
}  // namespace phoenix


#endif  // PHOENIX_COMMON_CUBIC_POLYNOMIAL_CURVE1D_H_

