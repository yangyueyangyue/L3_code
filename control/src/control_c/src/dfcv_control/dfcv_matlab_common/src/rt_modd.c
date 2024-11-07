/*
 * File: rt_modd.c
 *
 * Code generated for Simulink model 'LgtCtrl'.
 *
 * Model version                  : 1.329
 * Simulink Coder version         : 8.5 (R2013b) 08-Aug-2013
 * C/C++ source code generated on : Tue Aug 10 14:20:16 2021
 */

#include "rtwtypes.h"
#include "rtw_shared_utils.h"
#include <math.h>

real_T rt_modd(real_T u0, real_T u1)
{
  real_T y;
  real_T tmp;
  if (u1 == 0.0) {
    y = u0;
  } else {
    tmp = u0 / u1;
    if (u1 <= floor(u1)) {
      y = u0 - floor(tmp) * u1;
    } else {
      if (fabs(tmp - rt_roundd(tmp)) <= DBL_EPSILON * fabs(tmp)) {
        y = 0.0;
      } else {
        y = (tmp - floor(tmp)) * u1;
      }
    }
  }

  return y;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
