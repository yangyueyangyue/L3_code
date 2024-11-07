/** @copyright Copyright (c) 2018-2020 Kotei Technology Co., Ltd.
 ******************************************************************************
 * @file       fft_c.h
 * @brief      THE FAST FOURIER TRANSFORM
 * @details    THE FAST FOURIER TRANSFORM
 *
 * @author     pengc
 * @date       2023.08.16
 * @version    v1.0
 *
 * @par Change Log:
 * <table>
 * <tr><th>Date        <th>Version  <th>Author    <th>Description
 * <tr><td>2023/08/16  <td>1.0      <td>pengc     <td>First edition
 * </table>
 *
 ******************************************************************************/

#ifndef PHOENIX_COMMON_MATH_FFT_C_H_
#define PHOENIX_COMMON_MATH_FFT_C_H_

#include "math/math_utils_c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief 正向FFT变换
 * @param[in] number contains the number of points in the DFT
 * @param[in&out] rex contain the real parts of the input, \n
 *                    upon return, contain the real parts of the output
 * @param[in&out] imx contain the imaginary parts of the input, \n
 *                    upon return, contain the imaginary parts of the output
 */
void Phoenix_Common_ForwardFFT_f(
    Uint32_t number, Float32_t* rex, Float32_t* imx);

/**
 * @brief 逆向FFT变换
 * @param[in] number contains the number of points in the IDFT
 * @param[in&out] rex contain the real parts of the complex frequency domain, \n
 *                    upon return, contain the real parts of the complex time domain
 * @param[in&out] imx contain the imaginary parts of the complex frequency domain, \n
 *                    upon return, contain the imaginary parts of the complex time domain
 */
void Phoenix_Common_InverseFFT_f(
    Uint32_t number, Float32_t* rex, Float32_t* imx);


#ifdef __cplusplus
}
#endif


#endif  // PHOENIX_COMMON_MATH_FFT_C_H_

