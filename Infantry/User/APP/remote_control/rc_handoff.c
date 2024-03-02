#include "rc_handoff.h"

#define CH4_HIGH_VALUE 600
#define CH4_LOW_VALUE 500

#define int_abs(x) ((x) > 0 ? (x) : (-x))


bool_t switch_is_fric_on(int16_t ch)
{
    static bool_t fric_flag = 0;

    if(ch < -CH4_HIGH_VALUE)
    {
        fric_flag = 1;
    }
    else if (ch > -CH4_LOW_VALUE)
    {
        fric_flag = 0;
    }

    return fric_flag;
}


bool_t switch_is_shoot(int16_t ch)
{
    static bool_t shoot_flag = 0;

    if(ch > CH4_HIGH_VALUE)
    {
        shoot_flag = 1;
    }
    else if (ch < CH4_LOW_VALUE)
    {
        shoot_flag = 0;
    }

    return shoot_flag;
}

