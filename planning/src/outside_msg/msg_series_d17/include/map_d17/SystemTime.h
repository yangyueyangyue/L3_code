#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H

#include "SystemBase.h"

extern float64 getSystemTimeStamp();
extern uint64 getSystemMSecond();
extern uint64 getSystemUSecond();
extern void mSleep(int useconds);

#define TIME_SECOND getSystemTimeStamp();
#define TIME_M_SECOND getSystemMSecond();
#define TIME_U_SECOND getSystemUSecond();

#define TIME_SLEEP(A) mSleep(A);

#endif
