#include <AP_HAL/AP_HAL.h>

#include "AP_CustomControl.h"

#if AP_CUSTOMCONTROL_ENABLED


#include "AP_CustomControl_Backend.h"
// #include "AP_CustomControl_Empty.h"
// #include "AP_CustomControl_PID.h"
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_CustomControl::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Custom control type
    // @Description: Custom control type to be used
    // @Values: 0:None, 1:Empty, 2:PID
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_CustomControl, _controller_type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _AXIS_MASK
    // @DisplayName: Custom Controller bitmask
    // @Description: Custom Controller bitmask to chose which axis to run
    // @Bitmask: 0:Roll, 1:Pitch, 2:Yaw
    // @User: Advanced
    AP_GROUPINFO("_AXIS_MASK", 2, AP_CustomControl, _custom_controller_mask, 0),

    // parameters for empty controller. only used as a template, no need for param table 
    // AP_SUBGROUPVARPTR(_backend, "1_", 6, AC_CustomControl, _backend_var_info[0]),

    // @Param: _THR_MASK
    // @DisplayName: Custom Controller bitmask
    // @Description: Custom Controller bitmask to chose which axis to run
    // @Bitmask: 0:Roll, 1:Pitch, 2:Yaw
    // @User: Advanced
    AP_GROUPINFO("_THR_MASK", 3, AP_CustomControl, _custom_controller_thr, 0),

    // parameters for INDI controller
    AP_SUBGROUPVARPTR(_backend, "2_", 7, AP_CustomControl, _backend_var_info[1]),
    

    AP_GROUPEND
};

const struct AP_Param::GroupInfo *AP_CustomControl::_backend_var_info[CUSTOMCONTROL_MAX_TYPES];

AP_CustomControl::AP_CustomControl(AP_AHRS &ahrs, AP_TECS &tecs, AP_PitchController *pitchController, AP_RollController *rollController, AP_YawController *yawController, float dt) :
    _dt(dt),
    _ahrs(ahrs),
    _tecs(tecs),
    _pitchController(pitchController),
    _rollController(rollController),
    _yawController(yawController)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_CustomControl::init(void)
{
    switch (CustomControlType(_controller_type))
    {
        case CustomControlType::CONT_NONE:
            break;
        case CustomControlType::CONT_EMPTY: // This is template backend. Don't initialize it.
            // This is template backend. Don't initialize it.
            // _backend = new AC_CustomControl_Empty(*this, _ahrs, _att_control, _motors, _dt);
            // _backend_var_info[get_type()] = AC_CustomControl_Empty::var_info;
            break;
        case CustomControlType::CONT_INDI:
            _backend = new AP_CustomControl_INDI(*this, _ahrs, _tecs, _att_control, _motors, _dt);
            _backend_var_info[get_type()] = AP_CustomControl_INDI::var_info;
            break;
        default:
            return;
    }

    if (_backend && _backend_var_info[get_type()]) {
        AP_Param::load_object_from_eeprom(_backend, _backend_var_info[get_type()]);
    }
}

// run custom controller if it is activated by RC switch and appropriate type is selected
void AP_CustomControl::update(float roll_target, float pitch_target)
{
    if (is_safe_to_run()) {
        float roll_out, pitch_out, yaw_out, Vt_out;

        // TEMPORARY
        _backend->update;
        roll_out = _backend->get_roll_out(roll_target);
        pitch_out = _backend->get_pitch_out(pitch_target);
        yaw_out = _backend->get_yaw_out();

        if(!_tecs._flags.is_gliding)
        {
            Vt_out = _backend->get_Vt_out();
        }

        servo_set(roll_out, pitch_out, yaw_out, Vt_out);
    }
}

// choose which axis to apply custom controller output
void AP_CustomControl::servo_set(float roll_out, float pitch_out, float yaw_out, float Vt_out) {
    if (_custom_controller_mask & (uint8_t)CustomControlOption::ROLL) {
        //_motors->set_roll(rpy.x);
        //_att_control->get_rate_roll_pid().set_integrator(0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_out);
    }
    if (_custom_controller_mask & (uint8_t)CustomControlOption::PITCH) {
        //_motors->set_pitch(rpy.y);
        //_att_control->get_rate_pitch_pid().set_integrator(0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch_out);
    }
    if (_custom_controller_mask & (uint8_t)CustomControlOption::YAW) {
        //_motors->set_yaw(rpy.z);
        //_att_control->get_rate_yaw_pid().set_integrator(0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, yaw_out);
    }
    // if (_custom_controller_thr & (uint8_t)CustomControlOption::VT) {
    if (_custom_controller_thr) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, Vt_out);
    }
}
// Requires adaoptation for Plane, but should be unnecessary
// move main controller's target to current states, reset filters,
// and move integrator to motor output
// to allow smooth transition to the primary controller
void AP_CustomControl::reset_main_att_controller(void)
{

    _pitchController->reset_I();
    _rollController->reset_I();
    _yawController->reset_I();
}

void AP_CustomControl::set_custom_controller(bool enabled)
{
    // double logging switch makes the state change very clear in the log
    log_switch();

    _custom_controller_active = false;

    // don't allow accidental main controller reset without active custom controller
    if (_controller_type == CustomControlType::CONT_NONE) {
        gcs().send_text(MAV_SEVERITY_INFO, "Custom controller is not enabled");
        return;
    }

    // controller type is out of range
    if (_controller_type > CUSTOMCONTROL_MAX_TYPES) {
        gcs().send_text(MAV_SEVERITY_INFO, "Custom controller type is out of range");
        return;
    }

    // backend is not created
    if (_backend == nullptr) {
        gcs().send_text(MAV_SEVERITY_INFO, "Reboot to enable selected custom controller");
        return;
    }

    if (_custom_controller_mask == 0 && enabled) {
        gcs().send_text(MAV_SEVERITY_INFO, "Axis mask is not set");
        return;
    }

    // reset main controller
    if (!enabled) {
        gcs().send_text(MAV_SEVERITY_INFO, "Custom controller is OFF");
        // don't reset if the empty backend is selected
        if (_controller_type > CustomControlType::CONT_EMPTY) {
            reset_main_att_controller();
        }
    }

    if (enabled && _controller_type > CustomControlType::CONT_NONE) {
        // reset custom controller filter, integrator etc.
        _backend->reset();
        gcs().send_text(MAV_SEVERITY_INFO, "Custom controller is ON");
    }

    _custom_controller_active = enabled;

    // log successful switch
    log_switch();
}

// check that RC switch is on, backend is not changed mid flight and controller type is selected
bool AP_CustomControl::is_safe_to_run(void) {
    if (_custom_controller_active && (_controller_type > CustomControlType::CONT_NONE)
        && (_controller_type <= CUSTOMCONTROL_MAX_TYPES) && _backend != nullptr)
    {
        return true;
    }

    return false;
}

// log when the custom controller is switch into
void AP_CustomControl::log_switch(void) {
    AP::logger().Write("CC", "TimeUS,Type,Act","QBB",
                            AP_HAL::micros64(),
                            _controller_type,
                            _custom_controller_active);
}

void AP_CustomControl::set_notch_sample_rate(float sample_rate)
{
#if AP_FILTER_ENABLED
    if (_backend != nullptr) {
        _backend->set_notch_sample_rate(sample_rate);
    }
#endif
}

#endif
