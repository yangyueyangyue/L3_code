/* Copyright 2018,2019 Kotei Co., Ltd.
 ******************************************************************************
 ** Some common operator
 ******************************************************************************
 *
 *  Some common operator
 *
 *  @file       util.cc
 *
 *  @author     kotei
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "utils/com_utils.h"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>
#include <exception>
#include <fstream>
#include <memory>

#include <boost/thread.hpp>
#include <boost/atomic.hpp>


namespace phoenix {
namespace framework {


void uniform_slice(double start, double end, Uint32_t num,
                   std::vector<double>* sliced) {
  if (!sliced || num == 0) {
    return;
  }
  const double delta = (end - start) / num;
  sliced->resize(num + 1);
  double s = start;
  for (Uint32_t i = 0; i < num; ++i, s += delta) {
    sliced->at(i) = s;
  }
  sliced->at(num) = end;
}


}  // namespace framework
}  // namespace phoenix



