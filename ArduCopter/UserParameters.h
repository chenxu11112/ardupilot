#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    AP_Float get_TREE_ANG_Param() const { return _tree_angle; }

private:
    // Put your parameter variable definitions here
    AP_Float _tree_angle;
};
