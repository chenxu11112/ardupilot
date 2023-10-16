#include "AC_LESO.h"

const AP_Param::GroupInfo AC_LESO::var_info[] = {

    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("b0", 1, AC_LESO, _b0, default_b0),

    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("w0", 2, AC_LESO, _w0, default_w0),

    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("h0", 3, AC_LESO, _h0, default_h0),

    AP_GROUPEND
};

AC_LESO::AC_LESO(float init_w0, float init_b0, float init_h0)
    : default_w0(init_w0)
    , default_b0(init_b0)
    , default_h0(init_h0)

{
    AP_Param::setup_object_defaults(this, var_info);

    _beta1 = 2.0f * _w0;
    _beta2 = _w0 * _w0;

    _z1 = 0;
    _z2 = 0;

    _u1_last    = 0;
    _u1_current = 0;
    _u0         = 0;
}

float AC_LESO::update(float feedback, float output_u0)
{
    float error = _z1 - feedback;

    _u1_last = _u1_current;

    _u0 = output_u0;

    _z1 += _h0 * (_z2 + _b0 * _u1_last - _beta1 * error);

    _z2 -= _h0 * _beta2 * error;

    _u1_current = _u0 - _z2 / _b0;

    return _u1_current;
}
