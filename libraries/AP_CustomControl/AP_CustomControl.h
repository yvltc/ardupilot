#pragma once

/// @file    AP_CustomControl.h
/// @brief   ArduPlane custom control library

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS.h>
#include <APM_Control/AP_PitchController.h>
#include <APM_Control/AP_RollController.h>
#include <APM_Control/AP_YawController.h>
#include <AP_Logger/AP_Logger.h>

#if AP_CUSTOMCONTROL_ENABLED

#ifndef CUSTOMCONTROL_MAX_TYPES
#define CUSTOMCONTROL_MAX_TYPES 2
#endif

class AP_CustomControl_Backend;

class AP_CustomControl {
public:
    AP_CustomControl(AP_AHRS &ahrs, AP_PitchController *pitchController, AP_RollController *rollController, AP_YawController *yawController, float dt);

    CLASS_NO_COPY(AP_CustomControl);  /* Do not allow copies */

    void init(void);
    void update(float roll_target, float pitch_target);
    void servo_set(float roll_out, float pitch_out, float yaw_out);
    void set_custom_controller(bool enabled);
    void reset_main_att_controller(void);
    bool is_safe_to_run(void);
    void log_switch(void);

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate);

    // zero index controller type param, only use it to access _backend or _backend_var_info array
    uint8_t get_type() { return _controller_type > 0 ? (_controller_type - 1) : 0; };

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo *_backend_var_info[CUSTOMCONTROL_MAX_TYPES];

protected:
    // add custom controller here
    enum class CustomControlType : uint8_t {
        CONT_NONE            = 0,
        CONT_EMPTY           = 1,
        CONT_PID             = 2,
    };            // controller that should be used     

    enum class  CustomControlOption {
        ROLL = 1 << 0,
        PITCH = 1 << 1,
        YAW = 1 << 2,
    };

    // Intersampling period in seconds
    float _dt;
    bool _custom_controller_active;

    // References to external libraries
    AP_AHRS &_ahrs;
    AP_PitchController *_pitchController;
    AP_RollController *_rollController;
    AP_YawController *_yawController;

    AP_Enum<CustomControlType> _controller_type;
    AP_Int8 _custom_controller_mask;

private:
    AP_CustomControl_Backend *_backend;
};

#endif
