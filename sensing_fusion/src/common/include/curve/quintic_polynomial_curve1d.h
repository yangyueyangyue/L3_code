
#ifndef PHOENIX_COMMON_QUINTIC_POLYNOMIAL_CURVE1D_H_
#define PHOENIX_COMMON_QUINTIC_POLYNOMIAL_CURVE1D_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "math/math_utils.h"

namespace phoenix {
namespace common {

// 1D quintic polynomial curve:
// (x0, dx0, ddx0) -- [0, param] --> (x1, dx1, ddx1)
template<typename Scalar>
class QuinticPolynomialCurve1d {
public:
  QuinticPolynomialCurve1d() {
    param_ = 0;
    com_memset(coef_, 0, sizeof(coef_));
  }

  QuinticPolynomialCurve1d(const Scalar start[3],
                           const Scalar end[3],
                           const Scalar param) :
    QuinticPolynomialCurve1d(start[0], start[1], start[2],
                             end[0], end[1], end[2], param) {
  }

  QuinticPolynomialCurve1d(const Scalar x0, const Scalar dx0, const Scalar ddx0,
                           const Scalar x1, const Scalar dx1, const Scalar ddx1,
                           const Scalar param) {
    ComputeCoefficients(x0, dx0, ddx0, x1, dx1, ddx1, param);
    param_ = param;
  }

  QuinticPolynomialCurve1d(const QuinticPolynomialCurve1d& other) {
    param_ = other.param_;
    com_memcpy(coef_, other.coef_, sizeof(coef_));
  }

  virtual ~QuinticPolynomialCurve1d() {
  }

  void Clear() {
    param_ = 0;
    com_memset(coef_, 0, sizeof(coef_));
  }

  void Construct(const Scalar x0, const Scalar dx0, const Scalar ddx0,
                 const Scalar x1, const Scalar dx1, const Scalar ddx1,
                 const Scalar param) {
    ComputeCoefficients(x0, dx0, ddx0, x1, dx1, ddx1, param);
    param_ = param;
  }

  void SetCoefficient(const Scalar c0, const Scalar c1, const Scalar c2,
                      const Scalar c3, const Scalar c4, const Scalar c5) {
    coef_[0] = c0;
    coef_[1] = c1;
    coef_[2] = c2;
    coef_[3] = c3;
    coef_[4] = c4;
    coef_[5] = c5;
  }

  Scalar Evaluate(const Uint32_t order, const Scalar p) const;

protected:
  void ComputeCoefficients(const Scalar x0, const Scalar dx0, const Scalar ddx0,
                           const Scalar x1, const Scalar dx1, const Scalar ddx1,
                           const Scalar p);

private:
  Scalar param_;
  Scalar coef_[6];
};


template<typename Scalar>
Scalar QuinticPolynomialCurve1d<Scalar>::Evaluate(
    const Uint32_t order, const Scalar p) const {
  Scalar ret = 0;

  switch (order) {
  case 0:
    ret = ((((coef_[5] * p + coef_[4]) * p + coef_[3]) * p + coef_[2]) * p +
        coef_[1]) * p + coef_[0];
    break;
  case 1:
    ret = (((5 * coef_[5] * p + 4 * coef_[4]) * p + 3 * coef_[3]) * p +
        2 * coef_[2]) * p + coef_[1];
    break;
  case 2:
    ret = (((20 * coef_[5] * p + 12 * coef_[4]) * p) + 6 * coef_[3]) *
        p + 2 * coef_[2];
    break;
  case 3:
    ret = (60 * coef_[5] * p + 24 * coef_[4]) * p + 6 * coef_[3];
    break;
  case 4:
    ret = 120 * coef_[5] * p + 24 * coef_[4];
    break;
  case 5:
    ret = 120 * coef_[5];
    break;
  default:
    ret = 0;
    break;
  }

  return (ret);
}

template<typename Scalar>
void QuinticPolynomialCurve1d<Scalar>::ComputeCoefficients(
    const Scalar x0, const Scalar dx0, const Scalar ddx0,
    const Scalar x1, const Scalar dx1, const Scalar ddx1,
    const Scalar p) {
  COM_CHECK(p > 0);

  coef_[0] = x0;
  coef_[1] = dx0;
  coef_[2] = 0.5f * ddx0;

  const Scalar p2 = p * p;
  const Scalar p3 = p * p2;

  // the direct analytical method is at least 6 times faster than using matrix
  // inversion.
  const Scalar c0 = (x1 - 0.5f * p2 * ddx0 - dx0 * p - x0) / p3;
  const Scalar c1 = (dx1 - ddx0 * p - dx0) / p2;
  const Scalar c2 = (ddx1 - ddx0) / p;

  coef_[3] = 0.5f * (20 * c0 - 8 * c1 + c2);
  coef_[4] = (-15 * c0 + 7 * c1 - c2) / p;
  coef_[5] = (6 * c0 - 3 * c1 + 0.5f * c2) / p2;
}


}  // namespace common
}  // namespace phoenix

#endif // PHOENIX_COMMON_QUINTIC_POLYNOMIAL_CURVE1D_H_
