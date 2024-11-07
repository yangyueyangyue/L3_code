#include "glog/logging.h"
#if (ENABLE_GTEST)
#include "gtest/gtest.h"
#endif
#include "map_space/hd_map.h"


#if (ENABLE_GTEST)

namespace phoenix {
namespace driv_map {


TEST(HDMap, Construct) {
  // HDMap
//  HDMap hd_map;
//  ad_msg::Position position;
//  position.global_x = 0;
//  position.global_y = 0;
//  position.global_z = 0;
//  position.global_heading = 0;
//  hd_map.set_position(position);
//  hd_map.set_map_range(10);

//  // set map information
//  map::Map map;
//  map::Lane* lane = map.add_lane();
//  map::CurveSegment* curve_segment =
//      lane->mutable_central_curve()->add_segment();
//  map::LineSegment* line_segment =
//      curve_segment->mutable_line_segment();

//  map::PointENU* point = line_segment->add_point();
//  point->set_x(-14);
//  point->set_y(-14);
//  point = line_segment->add_point();
//  point->set_x(-12);
//  point->set_y(-12);
//  point = line_segment->add_point();
//  point->set_x(2);
//  point->set_y(2);
//  point = line_segment->add_point();
//  point->set_x(4);
//  point->set_y(4);
//  point = line_segment->add_point();
//  point->set_x(6);
//  point->set_y(6);
//  point = line_segment->add_point();
//  point->set_x(12);
//  point->set_y(12);
//  point = line_segment->add_point();
//  point->set_x(14);
//  point->set_y(14);
//  point = line_segment->add_point();
//  point->set_x(14);
//  point->set_y(14);
//  point = line_segment->add_point();
//  point->set_x(14);
//  point->set_y(8);
//  point = line_segment->add_point();
//  point->set_x(8);
//  point->set_y(8);
//  point = line_segment->add_point();
//  point->set_x(8);
//  point->set_y(6);
//  point = line_segment->add_point();
//  point->set_x(12);
//  point->set_y(6);
//  point = line_segment->add_point();
//  point->set_x(14);
//  point->set_y(6);

//  hd_map.ConstructMapInfo(map);
}


} // namespace driv_map
} // phoenix

#endif

