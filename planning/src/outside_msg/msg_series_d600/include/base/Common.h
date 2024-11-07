#ifndef COMMON_H
#define COMMON_H

#include "Commondef.h"

//#define nullptr  (void *)0

namespace task_t
{
    
typedef struct 
{
    double dStampTime;
    int    iStructSize;
    std::string sName;
}S_BASE;

typedef struct 
{
    S_BASE base;
    std::string  sData;
}S_BASE_TEST;

}

#endif
