//
#include <stdio.h>
#include "math/fft_c.h"
#include "utils/log_c.h"


void Phoenix_Common_ForwardFFT_f(
    Uint32_t number, Float32_t* rex, Float32_t* imx) {
  Uint32_t ND2 = number / 2;
  Uint32_t M = phoenix_com_round_f(log2f(number));
  Uint32_t I = 0;
  Uint32_t J = ND2;
  Uint32_t K = 0;
  Float32_t TR = 0;
  Float32_t TI = 0;
  Uint32_t L = 0;
  Uint32_t LE = 0;
  Uint32_t LE2 = 0;
  Float32_t UR = 0;
  Float32_t UI = 0;
  Float32_t SR = 0;
  Float32_t SI = 0;
  Uint32_t JM1 = 0;
  Uint32_t IP = 0;

  // printf("M=%d\n", M);

  // Bit reversal sorting
#if 1
  for (I = 1; I <= (number-2); ++I) {
    if (I < J) {
      TR = rex[J];
      TI = imx[J];
      rex[J] = rex[I];
      imx[J] = imx[I];
      rex[I] = TR;
      imx[I] = TI;
      // printf("switch %d and %d\n", I, J);
    }
    K = ND2;
    while (K <= J) {
      J = J - K;
      K >>= 1;
    }
    J = J + K;
  }
#else
  for (I = 1; I <= (number-2); ++I) {
    J = (((I & 0xaaaaaaaa) >> 1) | ((I & 0x55555555) << 1));
    J = (((J & 0xcccccccc) >> 2) | ((J & 0x33333333) << 2));
    J = (((J & 0xf0f0f0f0) >> 4) | ((J & 0x0f0f0f0f) << 4));
    J = (((J & 0xff00ff00) >> 8) | ((J & 0x00ff00ff) << 8));
    J = ((J >> 16) | (J << 16));
    J >>= (32 - M);

    if (I < J) {
      TR = rex[J];
      TI = imx[J];
      rex[J] = rex[I];
      imx[J] = imx[I];
      rex[I] = TR;
      imx[I] = TI;
      printf("switch %d and %d\n", I, J);
    }
  }
#endif

  // Loop for each stage
  for (L = 1; L <= M; ++L) {
    LE = 2 << (L - 1);//phoenix_com_round_f(exp2f(L));
    LE2 = LE / 2;
    UR = 1;
    UI = 0;

    // printf("L=%d, LE=%d\n", L, LE);

    // Calculate sine & cosine values
    SR = phoenix_com_cos_f(COM_PI / LE2);
    SI = -phoenix_com_sin_f(COM_PI / LE2);

    // Loop for each sub DFT
    for (J = 1; J <= LE2; ++J) {
      JM1 = J - 1;
      // Loop for each butterfly
      for (I = JM1; I < number; I += LE) {
        IP = I + LE2;
        // Butterfly calculation
        TR = rex[IP]*UR - imx[IP]*UI;
        TI = rex[IP]*UI + imx[IP]*UR;
        rex[IP] = rex[I] - TR;
        imx[IP] = imx[I] - TI;
        rex[I] = rex[I] + TR;
        imx[I] = imx[I] + TI;
      }
      TR = UR;
      UR = TR*SR - UI*SI;
      UI = TR*SI + UI*SR;
    }
  }
}

void Phoenix_Common_InverseFFT_f(
    Uint32_t number, Float32_t* rex, Float32_t* imx) {
  Uint32_t I = 0;

  // Change the sign of IMX[ ]
  for (I = 0; I < number; ++I) {
    imx[I] = -imx[I];
  }

  // Calculate forward FFT
  Phoenix_Common_ForwardFFT_f(number, rex, imx);

  // Divide the time domain by N and change the sign of IMX[ ]
  for (I = 0; I < number; ++I) {
    rex[I] = rex[I] / number;
    imx[I] = -imx[I] / number;
  }
}
