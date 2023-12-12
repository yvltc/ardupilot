#include "mode.h"
#include "Plane.h"


bool ModeRDBWA::_enter()
{
    pos_control->set_max_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_velocity_z_max_up, quadplane.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-quadplane.get_pilot_velocity_z_max_dn(), quadplane.pilot_velocity_z_max_up, quadplane.pilot_accel_z);

    quadplane.throttle_wait = false;
    return true;
}

void ModeRDBWA::update()
{
    // set nav_roll and nav_pitch using sticks
    // Beware that QuadPlane::tailsitter_check_input (called from Plane::read_radio)
    // may alter the control_in values for roll and yaw, but not the corresponding
    // radio_in values. This means that the results for norm_input would not necessarily
    // be correct for tailsitters, so get_control_in() must be used instead.
    // normalize control_input to [-1,1]
    // in Rover mode roll is always zero - in reality we want to disable the roll control
    float forward_speed = - (float)plane.channel_pitch->get_control_in() / plane.channel_pitch->get_range() * 500;
    Vector2f target_speed_xy_cms = Vector2f(forward_speed * plane.ahrs.cos_yaw(), forward_speed * plane.ahrs.sin_yaw());
    Vector2f target_accel_cms;
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
    }
    // plane.nav_yaw_cd = degrees(target_speed_xy_cms.angle()) * 100;
    pos_control->input_vel_accel_xy(target_speed_xy_cms, target_accel_cms);
    // run horizontal velocity controller
    quadplane.run_xy_controller();
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = pos_control->get_pitch_cd();

}

// quadplane stabilize mode
void ModeRDBWA::run()
{
    // special check for ESC calibration in QSTABILIZE
    if (quadplane.esc_calibration != 0) {
        quadplane.run_esc_calibration();
        return;
    }

    // normal QSTABILIZE mode
    float pilot_throttle_scaled = 0.03;
    quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    quadplane.set_pilot_yaw_rate_time_constant();
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                  plane.nav_pitch_cd,
                                                                  quadplane.get_pilot_input_yaw_rate_cds());

    attitude_control->set_throttle_out(pilot_throttle_scaled, false, 0);
}

// set the desired roll and pitch for a tailsitter
void ModeRDBWA::set_tailsitter_roll_pitch(const float roll_input, const float pitch_input)
{
    // separate limit for roll, if set
    if (plane.quadplane.tailsitter.max_roll_angle > 0) {
        // roll param is in degrees not centidegrees
        plane.nav_roll_cd = plane.quadplane.tailsitter.max_roll_angle * 100.0f * roll_input;
    } else {
        plane.nav_roll_cd = roll_input * plane.quadplane.aparm.angle_max;
    }

    // angle max for tailsitter pitch
    plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;

    plane.quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd);
}

// set the desired roll and pitch for normal quadplanes, also limited by forward flight limtis
void ModeRDBWA::set_limited_roll_pitch(const float roll_input, const float pitch_input)
{
    plane.nav_roll_cd = roll_input * MIN(plane.roll_limit_cd, plane.quadplane.aparm.angle_max);
    // pitch is further constrained by LIM_PITCH_MIN/MAX which may impose
    // tighter (possibly asymmetrical) limits than Q_ANGLE_MAX
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * MIN(plane.aparm.pitch_limit_max_cd, plane.quadplane.aparm.angle_max);
    } else {
        plane.nav_pitch_cd = pitch_input * MIN(-plane.pitch_limit_min_cd, plane.quadplane.aparm.angle_max);
    }
}

