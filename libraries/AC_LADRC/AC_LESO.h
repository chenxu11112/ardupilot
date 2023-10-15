#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <cmath>
#include <stdlib.h>

class AC_LESO {
public:
    struct Defaults {
        float w0;
        float b0;
        float h0;
    };

    AC_LESO(float init_w0, float init_b0, float init_h0);

    AC_LESO(const AC_LESO::Defaults& defaults)
        : AC_LESO(
            defaults.w0,
            defaults.b0,
            defaults.h0)
    {
    }

    CLASS_NO_COPY(AC_LESO);

    float update(float feedback, float output_u0);

    float get_z1() const { return _z1; }
    float get_z2() const { return _z2; }
    float get_u1() const { return _u_last; }
    float get_u0() const { return _u0; }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    AP_Float _w0;
    AP_Float _b0;
    AP_Float _h0;

    float _beta1;
    float _beta2;

    float _u_last;
    float _u_current;
    float _u0;

    /* LESO */
    float _z1;
    float _z2;

private:
    const float default_b0;
    const float default_h0;
    const float default_w0;
};
