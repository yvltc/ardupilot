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

    plane.set_guided_WP(loc);
    return true;
}

void ModeRGuided::update()
{   
    const Location &loc = plane.next_WP_loc;
    Location origin;
    if (!plane.ahrs.get_origin(origin)) {
        origin.zero();
    }
    Vector2f diff2d = origin.get_distance_NE(loc) * 100;
    Vector2p target_pos = Vector2p(diff2d.x, diff2d.y);
    if (!pos_control->is_active_xy()) {
        pos_control->init_xy_controller();
        }
    Vector2f zero;
    Vector2f vel_cms;
    pos_control->input_pos_vel_accel_xy(target_pos, vel_cms, zero);


    quadplane.run_xy_controller();

    // nav roll and pitch are controlled by position controller
    plane.nav_roll_cd = pos_control->get_roll_cd();
    plane.nav_pitch_cd = pos_control->get_pitch_cd();
    plane.nav_yaw_cd = pos_control->get_yaw_cd();
    // call attitude controller
    quadplane.set_pilot_yaw_rate_time_constant();
    attitude_control->input_euler_angle_roll_pitch_yaw(plane.nav_roll_cd,
                                                                    plane.nav_pitch_cd,
                                                                    plane.nav_yaw_cd, 0);
    float pilot_throttle_scaled = quadplane.get_pilot_throttle()/4;
    attitude_control->set_throttle_out(pilot_throttle_scaled, false, 0);
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
