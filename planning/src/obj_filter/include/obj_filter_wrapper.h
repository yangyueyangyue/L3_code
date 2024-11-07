//
#ifndef PHOENIX_OBJ_FILTER_OBJ_FILTER_WRAPPER_H_
#define PHOENIX_OBJ_FILTER_OBJ_FILTER_WRAPPER_H_

#include "obj_filter.h"


#define OBJ_FILTER_IMPL_TYPE_UNDEFINED (0)
#define OBJ_FILTER_IMPL_TYPE_I (1)
#define OBJ_FILTER_IMPL_TYPE_II (2)
#define OBJ_FILTER_IMPL_TYPE OBJ_FILTER_IMPL_TYPE_I


namespace phoenix {
namespace obj_filter {


#if (OBJ_FILTER_IMPL_TYPE == OBJ_FILTER_IMPL_TYPE_I)
namespace impl1 {
class ObjFilterImpl;
}
#elif (OBJ_FILTER_IMPL_TYPE == OBJ_FILTER_IMPL_TYPE_II)
class Merger;
#else
class ObjFilterImpl;
#endif

class ObjFilterWrapper {
public:
  typedef Float32_t Scalar;

public:
  ObjFilterWrapper();
  ~ObjFilterWrapper();

  void EnableStaticEsrObj(bool enable);

  bool Update(const ObjFilterDataSource& data_source);

  void GetObstacleTrackedInfoList(
      ad_msg::ObstacleTrackedInfoList* obj_list) const;

  void GetObstacleList(ad_msg::ObstacleList* obj_list) const;

private:
#if (OBJ_FILTER_IMPL_TYPE == OBJ_FILTER_IMPL_TYPE_I)
  impl1::ObjFilterImpl* obj_filter_impl_;
#elif (OBJ_FILTER_IMPL_TYPE == OBJ_FILTER_IMPL_TYPE_II)
  Merger* obj_filter_impl_;
#else
  ObjFilterImpl* obj_filter_impl_;
#endif
};


}  // namespace obj_filter
}  // namespace phoenix


#endif // PHOENIX_OBJ_FILTER_OBJ_FILTER_WRAPPER_H_
