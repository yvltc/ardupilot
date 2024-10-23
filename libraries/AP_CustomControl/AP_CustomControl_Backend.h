#pragma once

#include "AP_CustomControl.h"

#if AP_CUSTOMCONTROL_ENABLED

class AP_CustomControl_Backend
{
public:
    AP_CustomControl_Backend(AP_CustomControl& frontend, AP_AHRS &ahrs, AP_TECS &tecs, float dt) :
        _frontend(frontend),
        _ahrs(ahrs),
        _tecs(tecs)
    {}

    // empty destructor to suppress compiler warning
    virtual ~AP_CustomControl_Backend() {}

    // update controller, return roll output
    virtual float get_roll_out(float roll_target) = 0;

    // update controller, return pitch output
    virtual float get_pitch_out(float pitch_target) = 0;

    // update controller, return yaw output
    virtual float get_yaw_out(void) = 0;

    // update controller
    virtual float update(float roll_target, float pitch_target) = 0;

    // reset controller to avoid build up or abrupt response upon switch, ex: integrator, filter
    virtual void reset() = 0;

    // set the PID notch sample rates
    virtual void set_notch_sample_rate(float sample_rate) {};

protected:
    // References to external libraries
    AP_AHRS &_ahrs;
    AP_TECS &_tecs;
    AP_CustomControl& _frontend;
};

#endif
