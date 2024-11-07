/******************************************************************************
 ** 车身参数信息
 ******************************************************************************
 *
 *  车身参数信息(保存车身参数)
 *
 *  @file       vehicle_model.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_VEH_MODEL_VEHICLE_MODEL_DF_D17_B2_H_
#define PHOENIX_VEH_MODEL_VEHICLE_MODEL_DF_D17_B2_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "utils/log.h"
#include "math/math_utils.h"
#include "vehicle_model.h"
#include "vehicle_model_base.h"

namespace phoenix {
namespace veh_model {


class VehicleModelDfD17B2 : public VehicleModelBase {
public:
  typedef VehicleModelBase::Scalar Scalar;
  typedef VehicleModelBase Base;

public:
  virtual ~VehicleModelDfD17B2();

  Scalar ClampMaxSteeringAngleByVelocity(
      Scalar angle, Scalar v) const;

private:
  DECLARE_SINGLETON(VehicleModelDfD17B2);
};


}  // namespace veh_model
}  // namespace phoenix


#endif  // PHOENIX_VEH_MODEL_VEHICLE_MODEL_DF_D17_B2_H_

