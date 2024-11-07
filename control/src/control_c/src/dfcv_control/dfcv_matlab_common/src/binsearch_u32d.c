/*
 * File: binsearch_u32d.c
 *
 * Code generated for Simulink model 'LgtCtrl'.
 *
 * Model version                  : 1.329
 * Simulink Coder version         : 8.5 (R2013b) 08-Aug-2013
 * C/C++ source code generated on : Tue Aug 10 14:20:16 2021
 */

#include "rtwtypes.h"
#include "rtw_shared_utils.h"

uint32_T binsearch_u32d(real_T u, const real_T bp[], uint32_T startIndex,
  uint32_T maxIndex)
{
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Binary Search */
  bpIdx = startIndex;
  iLeft = 0U;
  iRght = maxIndex;
  while (iRght - iLeft > 1U) {
    if (u < bp[bpIdx]) {
      iRght = bpIdx;
    } else {
      iLeft = bpIdx;
    }

    bpIdx = (iRght + iLeft) >> 1U;
  }

  return iLeft;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
