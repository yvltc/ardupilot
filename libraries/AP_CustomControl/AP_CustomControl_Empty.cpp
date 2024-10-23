/*
#include "AP_CustomControl_Empty.h"
#include "AP_CustomControl_INDI.h"

#if CUSTOMCONTROL_EMPTY_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_CustomControl_Empty::var_info[] = {
    // @Param: PARAM1
    // @DisplayName: Empty param1
    // @Description: Dummy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM1", 1, AP_CustomControl_Empty, param1, 0.0f),

    // @Param: PARAM2
    // @DisplayName: Empty param2
    // @Description: Dummy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM2", 2, AP_CustomControl_Empty, param2, 0.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dummy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM3", 3, AP_CustomControl_Empty, param3, 0.0f),

    AP_GROUPEND
};

// initialize in the constructor
AP_CustomControl_Empty::AP_CustomControl_Empty(AP_CustomControl& frontend, AP_PitchController *pitchController, AP_RollController *rollController, AP_YawController *yawController, AP_AHRS &ahrs, float dt) :
    AP_CustomControl_Backend(frontend, ahrs, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// update controller
// return roll controller output
float AP_CustomControl_Empty::get_roll_out(float roll_target)
{
    //float demanded_roll = _rollController->get_pid_info().target; 
    // ArduPlane main attitude controller already ran
    // we don't need to do anything else

    gcs().send_text(MAV_SEVERITY_INFO, "roll empty custom controller working");

    // return what ArduPlane main controller outputted
    return SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
}

//return pitch controller output
float AP_CustomControl_Empty::get_pitch_out(float pitch_target)
{
    //float demanded_pitch = _pitchController->get_pid_info().target; 
    // ArduPlane main attitude controller already ran
    // we don't need to do anything else

    gcs().send_text(MAV_SEVERITY_INFO, "pitch empty custom controller working");

    // return what ArduPlane main controller outputted
    return SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
}

//return yaw controller output
float AP_CustomControl_Empty::get_yaw_out(void)
{
    // ArduPlane main attitude controller already ran
    // we don't need to do anything else

    gcs().send_text(MAV_SEVERITY_INFO, "yaw empty custom controller working");

    // return what ArduPlane main controller outputted
    return SRV_Channels::get_output_scaled(SRV_Channel::k_rudder);
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AP_CustomControl_Empty::reset(void)
{
}

#endif
*/