#include "mode.h"
#include "Plane.h"

bool ModeRGuided::_enter()
{
    // For ground movement only idle throttle is used
    quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    plane.guided_throttle_passthru = false;
    /*
      when entering guided mode we set the target as the current
      location. This matches the behaviour of the copter code
    */
    Location loc{plane.current_loc};
        /*
            project forward by the stopping distance, for when the vehicle is moving
        */
    loc.offset_bearing(degrees(plane.ahrs.groundspeed_vector().angle()),
                        plane.quadplane.stopping_distance());
#endif

    plane.set_guided_WP(loc);
    return true;
}

void ModeRGuided::update()
{   
    const Location &loc = plane.next_WP_loc;
    Location origin;
    if (!ahrs.get_origin(origin)) {
        origin.zero();
    }
    Vector2f diff2d = origin.get_distance_NE(loc);
    diff2d += pos_control.xy_correction;
    pos_control.target_cm.x = diff2d.x * 100;
    pos_control.target_cm.y = diff2d.y * 100;
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
        }
    Vector2f zero;
    Vector2f vel_cms;
    pos_control->input_pos_vel_accel_xy(pos_control.target_cm.xy(), vel_cms, zero);


    quadplane.run_xy_controller();

    // nav roll and pitch are controlled by position controller
    plane.nav_roll_cd = pos_control->get_roll_cd();
    plane.nav_pitch_cd = pos_control->get_pitch_cd();
    plane.nav_yaw_cd = pos_control->get_yaw_cd();

    if (transition->set_VTOL_roll_pitch_limit(plane.nav_roll_cd, plane.nav_pitch_cd)) {
        pos_control->set_externally_limited_xy();
    }

    // call attitude controller
    set_pilot_yaw_rate_time_constant();
    attitude_control->input_euler_angle_roll_pitch_euler_yaw(plane.nav_roll_cd,
                                                                    plane.nav_pitch_cd,
                                                                    plane.nav_yaw_cd);
}

void ModeRGuided::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

bool ModeRGuided::handle_guided_request(Location target_loc)
{
    // add home alt if needed
    if (target_loc.relative_alt) {
        target_loc.alt += plane.home.alt;
        target_loc.relative_alt = 0;
    }

    plane.set_guided_WP(target_loc);

    return true;
}
