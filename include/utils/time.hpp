#pragma once


#include <omp.h>
#include <time.h>

namespace fast_livo::utils
{

inline double getWTime()
{
    #ifdef MP_EN
    return omp_get_wtime();
    #else
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
    #endif
}

} // namespace fast_livo::utils