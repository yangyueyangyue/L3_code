#ifndef OBJ_MNGR_H
#define OBJ_MNGR_H

#include <iostream>
#include <string>
#include <map>
#include <cmath>
#include"Common.h"
#include "UTYPEDEF.h"

using namespace std;

namespace fusion_process
{
	struct Obj_dimension
	{
		float32 height;
		float32 height_to_ground;
		float32 obj_width;
		float32 obj_length;
	};

	struct Pathplan_para
	{
		uint8 ObjID;
		uint8 ObjTyp;
		sint16 RltLtrDst;
		sint16 RltLngDst;
		sint16 RltLngVehSpd;
		sint16 RltLtrVehSpd;
		sint16 RltLngAcc;
		sint16 RltLtrAcc;
		uint8 ObjDynPrp;
		uint8 ObjMsrSt;
		uint16 ObjLnt;
		uint16 ObjWdt;
		uint16 ObjHgt;
		sint16 ObjOrnAgl;
		uint8 ObjPrbExt;
		uint16 ObjTrcTm;
	};

  struct FusionResultObjects {
  task_t::S_BASE base;
  sint32 count;
  Pathplan_para objects[100];
};

}


#endif