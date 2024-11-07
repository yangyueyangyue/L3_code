/*
 * File: intrp2d_l_pw.c
 *
 * Code generated for Simulink model 'LgtCtrl'.
 *
 * Model version                  : 1.329
 * Simulink Coder version         : 8.5 (R2013b) 08-Aug-2013
 * C/C++ source code generated on : Tue Aug 10 14:20:16 2021
 */

#include "rtwtypes.h"
#include "rtw_shared_utils.h"

real_T intrp2d_l_pw(const uint32_T bpIndex[], const real_T frac[], const real_T
                    table[], uint32_T stride)
{
  real_T yL_1d;
  uint32_T offset_1d;

  /* Interpolation 2-D
     Interpolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  offset_1d = bpIndex[1U] * stride + bpIndex[0U];
  yL_1d = (table[offset_1d + 1U] - table[offset_1d]) * frac[0U] +
    table[offset_1d];
  offset_1d += stride;
  return (((table[offset_1d + 1U] - table[offset_1d]) * frac[0U] +
           table[offset_1d]) - yL_1d) * frac[1U] + yL_1d;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
