#ifndef EXP_WRAPPER_H
#define EXP_WRAPPER_H

#include "ExpWrapperComm.h"

namespace exp_map_t
{
    extern bool ExpStart();
    extern bool ExpStop();
    extern bool ExpGetLocation(exp_map_t::stMapExpLocation *const);
    extern bool ExpGetProfile(exp_map_t::stMapExpProfileInfo *const);
    extern bool ExpGetPosition(exp_map_t::stMapExpLocation *const ptr_location);
    extern bool ExpGetGeofence(exp_map_t::stMapGeofenceInfo * const ptr_geofence);
    extern bool ExpGetMapModelInfo(exp_map_t::stMapModelInfo &);
}

#endif
