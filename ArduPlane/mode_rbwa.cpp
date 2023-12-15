#include "mode.h"
#include "Plane.h"


bool ModeRDBWA::_enter()
{
    // initialise loiter
    plane.quadplane.gnd_loiter_nav->clear_pilot_desired_acceleration();
    plane.quadplane.gnd_loiter_nav->init_target();

    plane.quadplane.init_throttle_wait();

    // prevent re-init of target position
    plane.quadplane.last_loiter_ms = AP_HAL::millis();
    return true;
}

void ModeRDBWA::update()
{
    plane.mode_rmanual.update();
}

// plane.quadplane stabilize mode
void ModeRDBWA::run()
{
    if (plane.quadplane.throttle_wait) {
        plane.quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        plane.quadplane.gnd_attitude_control->set_throttle_out(0, true, 0);
        plane.quadplane.relax_attitude_control();
        plane.quadplane.gnd_loiter_nav->clear_pilot_desired_acceleration();
        plane.quadplane.gnd_loiter_nav->init_target();
        return;
    }
    if (!plane.quadplane.motors->armed()) {
        plane.mode_rdbwa._enter();
    }

    const uint32_t now = AP_HAL::millis();
    if (now - plane.quadplane.last_loiter_ms > 500) {
        plane.quadplane.gnd_loiter_nav->clear_pilot_desired_acceleration();
        plane.quadplane.gnd_loiter_nav->init_target();
    }
    plane.quadplane.last_loiter_ms = now;

    // motors use full range
    plane.quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    
    // process pilot's roll and pitch input
    float target_roll_cd, target_pitch_cd;
    plane.quadplane.get_pilot_desired_lean_angles(target_roll_cd, target_pitch_cd, plane.quadplane.gnd_loiter_nav->get_angle_max_cd(), plane.quadplane.gnd_attitude_control->get_althold_lean_angle_max_cd());
    plane.quadplane.gnd_loiter_nav->set_pilot_desired_acceleration(target_roll_cd, target_pitch_cd);

    target_roll_cd = 0;
    
    // run loiter controller
    if (!plane.quadplane.gnd_pos_control->is_active_xy()) {
        plane.quadplane.gnd_pos_control->init_xy_controller();
    }
    plane.quadplane.gnd_loiter_nav->update();

    // nav roll and pitch are controller by loiter controller
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = plane.quadplane.gnd_loiter_nav->get_pitch();

    if (plane.quadplane.transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
        plane.quadplane.gnd_pos_control->set_externally_limited_xy();
    }

    // Pilot input, use yaw rate time constant
    plane.quadplane.set_pilot_yaw_rate_time_constant();

    // call attitude controller with conservative smoothing gain of 4.0f
    plane.quadplane.gnd_attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                  plane.nav_pitch_cd,
                                                                  plane.quadplane.get_desired_yaw_rate_cds());
    float pilot_throttle_scaled = plane.quadplane.roll_thr * 0.01;
    plane.quadplane.gnd_attitude_control->set_throttle_out(pilot_throttle_scaled, false, 0);
}


