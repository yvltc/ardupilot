#pragma once

#include "AP_CustomControl_Backend.h"

#ifndef CUSTOMCONTROL_EMPTY_ENABLED
    #define CUSTOMCONTROL_EMPTY_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_EMPTY_ENABLED

class AP_CustomControl_Empty : public AP_CustomControl_Backend {
public:
    AP_CustomControl_Empty(AP_CustomControl& frontend, AP_AHRS_View*& ahrs, float dt);


    float get_roll_out(float roll_target) override;
    float get_pitch_out(float pitch_target) override;
    float get_yaw_out(void) override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // declare parameters here
    AP_Float param1;
    AP_Float param2;
    AP_Float param3;
};

#endif
