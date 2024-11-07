#ifndef RADAR_BASE_FILTER_H_
#define RADAR_BASE_FILTER_H_
#include "geometry/aabbox2d.h"
#include "geometry/geometry_utils.h"
#include "utils/log.h"
#include "math/matrix.h"
#include "math/math_utils.h"
#include "utils/macros.h"
#include "msg_common.h"
#include "msg_obstacle.h"
#include "radar_common.h"


#define TURN_OFF_RADAR_PRE_PROCESS_LOG   (1)//关闭侧雷达调试信息


#if TURN_OFF_RADAR_PRE_PROCESS_LOG

#define RADAR_PRE_LOG_LEVEL  7 

#else

#define RADAR_PRE_LOG_LEVEL  5 

#endif

namespace phoenix{
namespace perception{
namespace radar{

typedef common::Matrix<Float64_t, 4, 4>  Matrix4d;
typedef common::Matrix<Float64_t, 4, 1>  Vector4d;


class BaseFilter {
 public:
  BaseFilter() {}
  virtual ~BaseFilter() {}
  virtual void Init(const ad_msg::ObstacleRadar& object) = 0; // “=0” 表示纯虚函数
  virtual bool  Predict(double time_diff) = 0;
  virtual bool  UpdateWithObject(const ad_msg::ObstacleRadar& new_object,
                                           double time_diff) = 0;
  virtual void GetState(Float32_t &x,Float32_t &y,Float32_t &v_x,Float32_t &v_y) = 0;
  // virtual Matrix4d GetCovarianceMatrix() = 0;
//  private:
//   DISALLOW_COPY_AND_ASSIGN(BaseFilter);
};








}// end of namespace radar
}// end of namespace perception
}// end of namespace phoenix


#endif
