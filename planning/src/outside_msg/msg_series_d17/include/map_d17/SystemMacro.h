#ifndef SYSTEM_MACRO_H
#define SYSTEM_MACRO_H

#include <stddef.h>
#include "SystemTime.h"

#if 0
#define DECLARE_SINGLE_INSTANCE(T) \
public:                            \
    static T *Instance();        \
                                   \
private:                           \
    static T *mThis;               \
                                   \
public:

#define ACHIEVE_SINGLE_INSTANCE(T) \
    T *T::mThis = NULL;            \
    T *T::Instance()             \
    {                              \
        if (mThis == NULL)         \
        {                          \
            mThis = new T();       \
        }                          \
        return mThis;              \
    }
#endif

#define OBTAIN_INSTANCE(T) \
    T::Instance()

#define SYSTEM_MSECOND getSystemMSecond();

#define DECLARE_SINGLE_INSTANCE_OBJ(T) \
public:                                \
    static T *Instance();            \
                                       \
public:

#define ACHIEVE_SINGLE_INSTANCE_OBJ(T) \
    T *T::Instance()                 \
    {                                  \
        static T instance;             \
        return &instance;              \
    }

#endif