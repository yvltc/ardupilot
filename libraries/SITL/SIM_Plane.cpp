/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  very simple plane simulator class. Not aerodynamically accurate,
  just enough to be able to debug control logic for new frame types
*/

#include "SIM_Plane.h"

#include <iostream>
#include <cmath>
#include <stdio.h>
#include <AP_Filesystem/AP_Filesystem_config.h>

using namespace SITL;

Plane::Plane(const char *frame_str) :
    Aircraft(frame_str)
{
    mass = 2.0f;

    /*
       scaling from motor power to Newtons. Allows the plane to hold
       vertically against gravity when the motor is at hover_throttle
    */
    thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    frame_height = 0.1f;

    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;
    lock_step_scheduled = true;

    if (strstr(frame_str, "-heavy")) {
        mass = 8;
    }
    if (strstr(frame_str, "-jet")) {
        // a 22kg "jet", level top speed is 102m/s
        mass = 22;
        thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;
    }
    if (strstr(frame_str, "-revthrust")) {
        reverse_thrust = true;
    }
    if (strstr(frame_str, "-elevon")) {
        elevons = true;
    } else if (strstr(frame_str, "-vtail")) {
        vtail = true;
    } else if (strstr(frame_str, "-dspoilers")) {
        dspoilers = true;
    } else if (strstr(frame_str, "-flyingwing")) {
        custom_dynamics = true;
        elevons = true;
        mass = 1.2650;
        thrust_scale = (mass * GRAVITY_MSS) / hover_throttle;

        coefficient.s = 0.44;
        coefficient.b = 1.55;
        coefficient.c = 0.2839;
        coefficient.c_lift_0 = 0.2050;
        coefficient.c_lift_deltae = 0.3993;
        coefficient.c_lift_a = 3.9934;
        coefficient.c_lift_q = 3.1851;
        // float coefficient.mcoeff = 50;
        coefficient.oswald = 0.9;
        coefficient.alpha_stall = 0.2618;
        // float coefficient.c_drag_q = 0;
        coefficient.c_drag_deltae = 0.0024;
        coefficient.c_drag_p = 0.0017;        // drag p ou drag 0??
        coefficient.c_y_0 = 0;
        coefficient.c_y_b = -0.0025;
        coefficient.c_y_p = 0.2620;
        coefficient.c_y_r = -0.0673;
        // float coefficient.c_y_deltaa = 0;
        // float coefficient.c_y_deltar = -0.2;
        coefficient.c_l_0 = 0;
        coefficient.c_l_p = -0.4506;
        coefficient.c_l_b = -0.1604;
        coefficient.c_l_r = 0.3107;
        // float coefficient.c_l_deltaa = 0.25;
        // float coefficient.c_l_deltar = -0.037;
        coefficient.c_m_0 = -0.0148;       
        coefficient.c_m_a = 0.0718;
        coefficient.c_m_q = -2.4487;
        coefficient.c_m_deltae = -0.0836;
        coefficient.c_m_adot = -0.4897;
        coefficient.c_n_0 = 0;
        coefficient.c_n_b = 0.0390;
        coefficient.c_n_p = -0.1890;
        coefficient.c_n_r = 0.0028;
        coefficient.c_n_deltaa = 0.0004195;
        // float coefficient.c_n_deltar = 0.1;
        // float coefficient.deltaa_max = 0.3491;
        // float coefficient.deltae_max = 0.3491;
        // float coefficient.deltar_max = 0.3491;
        coefficient.CGOffset = {0.1260, 0, 0.0136};
    }
    if (strstr(frame_str, "-elevrev")) {
        reverse_elevator_rudder = true;
    }
    if (strstr(frame_str, "-catapult")) {
        have_launcher = true;
        launch_accel = 15;
        launch_time = 2;
    }
    if (strstr(frame_str, "-bungee")) {
        have_launcher = true;
        launch_accel = 7;
        launch_time = 4;
    }
    if (strstr(frame_str, "-throw")) {
        have_launcher = true;
        launch_accel = 25;
        launch_time = 0.4;
    }
    if (strstr(frame_str, "-tailsitter")) {
        tailsitter = true;
        ground_behavior = GROUND_BEHAVIOR_TAILSITTER;
        thrust_scale *= 1.5;
    }

#if AP_FILESYSTEM_FILE_READING_ENABLED
    if (strstr(frame_str, "-3d")) {
        aerobatic = true;
        thrust_scale *= 1.5;
        // setup parameters for plane-3d
        AP_Param::load_defaults_file("@ROMFS/models/plane.parm", false);
        AP_Param::load_defaults_file("@ROMFS/models/plane-3d.parm", false);
    }
#endif

    if (strstr(frame_str, "-ice")) {
        ice_engine = true;
    }

    if (strstr(frame_str, "-soaring")) {
        mass = 2.0;
        coefficient.c_drag_p = 0.05;
    }
}

/*
  the following functions are from last_letter
  https://github.com/Georacer/last_letter/blob/master/last_letter/src/aerodynamicsLib.cpp
  many thanks to Georacer!
 */
float Plane::liftCoeff(float alpha) const
{
    const float alpha0 = coefficient.alpha_stall;
    const float M = coefficient.mcoeff;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;

    // clamp the value of alpha to avoid exp(90) in calculation of sigmoid
    const float max_alpha_delta = 0.8f;
    if (alpha-alpha0 > max_alpha_delta) {
        alpha = alpha0 + max_alpha_delta;
    } else if (alpha0-alpha > max_alpha_delta) {
        alpha = alpha0 - max_alpha_delta;
    }
	double sigmoid = ( 1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)) ) / (1+exp(-M*(alpha-alpha0))) / (1+exp(M*(alpha+alpha0)));
	double linear = (1.0-sigmoid) * (c_lift_0 + c_lift_a0*alpha); //Lift at small AoA
	double flatPlate = sigmoid*(2*copysign(1,alpha)*pow(sin(alpha),2)*cos(alpha)); //Lift beyond stall

	float result  = linear+flatPlate;
	return result;
}

float Plane::dragCoeff(float alpha) const
{
    const float b = coefficient.b;
    const float s = coefficient.s;
    const float c_drag_p = coefficient.c_drag_p;
    const float c_lift_0 = coefficient.c_lift_0;
    const float c_lift_a0 = coefficient.c_lift_a;
    const float oswald = coefficient.oswald;
    
	double AR = pow(b,2)/s;
	double c_drag_a = c_drag_p + pow(c_lift_0+c_lift_a0*alpha,2)/(M_PI*oswald*AR);

	return c_drag_a;
}

// Torque calculation function
Vector3f Plane::getTorque(float inputAileron, float inputElevator, float inputRudder, float inputThrust, const Vector3f &force) const
{
    float alpha = angle_of_attack;

	//calculate aerodynamic torque
    float effective_airspeed = airspeed;

    if (tailsitter || aerobatic) {
        /*
          tailsitters get airspeed from prop-wash
         */
        effective_airspeed += inputThrust * 20;

        // reduce effective angle of attack as thrust increases
        alpha *= constrain_float(1 - inputThrust, 0, 1);
    }
    
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_l_0 = coefficient.c_l_0;
    const float c_l_b = coefficient.c_l_b;
    const float c_l_p = coefficient.c_l_p;
    const float c_l_r = coefficient.c_l_r;
    const float c_l_deltaa = coefficient.c_l_deltaa;
    const float c_l_deltar = coefficient.c_l_deltar;
    const float c_m_0 = coefficient.c_m_0;
    const float c_m_a = coefficient.c_m_a;
    const float c_m_q = coefficient.c_m_q;
    const float c_m_deltae = coefficient.c_m_deltae;
    const float c_n_0 = coefficient.c_n_0;
    const float c_n_b = coefficient.c_n_b;
    const float c_n_p = coefficient.c_n_p;
    const float c_n_r = coefficient.c_n_r;
    const float c_n_deltaa = coefficient.c_n_deltaa;
    const float c_n_deltar = coefficient.c_n_deltar;
    const Vector3f &CGOffset = coefficient.CGOffset;
    
    float rho = air_density;

	//read angular rates
	double p = gyro.x;
	double q = gyro.y;
	double r = gyro.z;

	double qbar = 1.0/2.0*rho*pow(effective_airspeed,2)*s; //Calculate dynamic pressure
	double la, na, ma;
	if (is_zero(effective_airspeed))
	{
		la = 0;
		ma = 0;
		na = 0;
	}
	else
	{
		la = qbar*b*(c_l_0 + c_l_b*beta + c_l_p*b*p/(2*effective_airspeed) + c_l_r*b*r/(2*effective_airspeed) + c_l_deltaa*inputAileron + c_l_deltar*inputRudder);
		ma = qbar*c*(c_m_0 + c_m_a*alpha + c_m_q*c*q/(2*effective_airspeed) + c_m_deltae*inputElevator);
		na = qbar*b*(c_n_0 + c_n_b*beta + c_n_p*b*p/(2*effective_airspeed) + c_n_r*b*r/(2*effective_airspeed) + c_n_deltaa*inputAileron + c_n_deltar*inputRudder);
	}


	// Add torque to force misalignment with CG
	// r x F, where r is the distance from CoG to CoL
	la +=  CGOffset.y * force.z - CGOffset.z * force.y;
	ma += -CGOffset.x * force.z + CGOffset.z * force.x;
	na += -CGOffset.y * force.x + CGOffset.x * force.y;

	return Vector3f(la, ma, na);
}

// Force calculation function from last_letter
Vector3f Plane::getForce(float inputAileron, float inputElevator, float inputRudder) const
{
    const float alpha = angle_of_attack;
    const float c_drag_q = coefficient.c_drag_q;
    const float c_lift_q = coefficient.c_lift_q;
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float c_drag_deltae = coefficient.c_drag_deltae;
    const float c_lift_deltae = coefficient.c_lift_deltae;
    const float c_y_0 = coefficient.c_y_0;
    const float c_y_b = coefficient.c_y_b;
    const float c_y_p = coefficient.c_y_p;
    const float c_y_r = coefficient.c_y_r;
    const float c_y_deltaa = coefficient.c_y_deltaa;
    const float c_y_deltar = coefficient.c_y_deltar;
    
    float rho = air_density;

	//request lift and drag alpha-coefficients from the corresponding functions
	double c_lift_a = liftCoeff(alpha);
	double c_drag_a = dragCoeff(alpha);

	//convert coefficients to the body frame
	double c_x_a = -c_drag_a*cos(alpha)+c_lift_a*sin(alpha);
	double c_x_q = -c_drag_q*cos(alpha)+c_lift_q*sin(alpha);
	double c_z_a = -c_drag_a*sin(alpha)-c_lift_a*cos(alpha);
	double c_z_q = -c_drag_q*sin(alpha)-c_lift_q*cos(alpha);

	//read angular rates
	double p = gyro.x;
	double q = gyro.y;
	double r = gyro.z;

	//calculate aerodynamic force
	double qbar = 1.0/2.0*rho*pow(airspeed,2)*s; //Calculate dynamic pressure
	double ax, ay, az;
	if (is_zero(airspeed))
	{
		ax = 0;
		ay = 0;
		az = 0;
	}
	else
	{
		ax = qbar*(c_x_a + c_x_q*c*q/(2*airspeed) - c_drag_deltae*cos(alpha)*fabs(inputElevator) + c_lift_deltae*sin(alpha)*inputElevator);
		// split c_x_deltae to include "abs" term
		ay = qbar*(c_y_0 + c_y_b*beta + c_y_p*b*p/(2*airspeed) + c_y_r*b*r/(2*airspeed) + c_y_deltaa*inputAileron + c_y_deltar*inputRudder);
		az = qbar*(c_z_a + c_z_q*c*q/(2*airspeed) - c_drag_deltae*sin(alpha)*fabs(inputElevator) - c_lift_deltae*cos(alpha)*inputElevator);
		// split c_z_deltae to include "abs" term
	}
    return Vector3f(ax, ay, az);
}

// Custom dynamics
float Plane::CustomDynamics_liftCoeff (float alpha, float dl, float dr)
{
    const float C_L_0 = coefficient.c_lift_0;
    const float C_L_a = coefficient.c_lift_a;
    const float C_L_q = coefficient.c_lift_q;
    const float C_L_dd = coefficient.c_lift_deltad;
    const float c = coefficient.c;
    float q = gyro.y;

    double C_L = C_L_0 + C_L_a*alpha + C_L_dd*(dl+dr) + C_L_q*q*c/(2*airspeed);

	return C_L;
}

float Plane::CustomDynamics_dragCoeff(float alpha, float dl, float dr)
{
    const float b = coefficient.b;
    const float s = coefficient.s;
    const float C_D_0 = coefficient.c_drag_p;
    const float C_L_0 = coefficient.c_lift_0;
    const float C_L_a = coefficient.c_lift_a;
    const float C_L_de = coefficient.c_lift_deltae;
    const float oswald = coefficient.oswald;
    
	double AR = pow(b,2)/s;
    double C_L = C_L_0 + C_L_a*alpha;
    double C_D_dd = C_L*C_L_de/(M_PI*oswald*AR);
	double C_D = C_D_0 + pow(C_L, 2)/(M_PI*oswald*AR) + C_D_dd*(abs(dl) + abs(dr));

	return C_D;
}

Vector3f Plane::CustomDynamics_getTorque(float da, float de)
{
    const float alpha = angle_of_attack;
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float C_l_b = coefficient.c_l_b;
    const float C_l_p = coefficient.c_l_p;
    const float C_l_r = coefficient.c_l_r;
    const float C_l_da = coefficient.c_l_deltaa;
    const float C_m_0 = coefficient.c_m_0;
    const float C_m_a = coefficient.c_m_a;
    const float C_m_q = coefficient.c_m_q;
    const float C_m_de = coefficient.c_m_deltae;
    const float C_n_b = coefficient.c_n_b;
    const float C_n_p = coefficient.c_n_p;
    const float C_n_r = coefficient.c_n_r;
    const float C_n_da = coefficient.c_n_deltaa;
    const float C_m_adot = coefficient.c_m_adot;  
    
    float rho = air_density;

    float p = gyro.x;
    float q = gyro.y;
    float r = gyro.z;

    float pdyn = 0.5*rho*pow(airspeed,2);
    float C_l, C_m, C_n;

    Vector3f M_aero;
    float alphadot = 0;     // first test adot = 0

    if (is_zero(airspeed)) {
		M_aero = {0, 0, 0};
	}
	else {
        C_l = C_l_b*beta + C_l_da*da + (C_l_p*p + C_l_r*r)*b/(2*airspeed);
        C_m = C_m_0 + C_m_a*alpha + C_m_de*de + (C_m_adot*alphadot + C_m_q*q)*c/(2*airspeed);
        C_n = C_n_b*beta + C_n_da*da + (C_n_p*p + C_n_r*r)*b/(2*airspeed);

        M_aero = Vector3f(pdyn*s*b*C_l, pdyn*s*c*C_m, pdyn*s*b*C_n);
    }
    
    return M_aero;
}

Vector3f Plane::CustomDynamics_getForce(float dl, float dr)
{
    const float alpha = angle_of_attack;
    const float s = coefficient.s;
    const float c = coefficient.c;
    const float b = coefficient.b;
    const float C_Y_0 = coefficient.c_y_0;
    const float C_Y_b = coefficient.c_y_b;
    const float C_Y_p = coefficient.c_y_p;
    const float C_Y_r = coefficient.c_y_r;
    const float C_Y_deltaa = coefficient.c_y_deltaa;
    const float C_Y_deltar = coefficient.c_y_deltar;
    
    float rho = air_density;

    float p = gyro.x;
    float q = gyro.y;
    float r = gyro.z;

	//request lift and drag coefficients from the corresponding functions
	float C_L = CustomDynamics_liftCoeff(alpha, dl, dr);
	float C_D = CustomDynamics_dragCoeff(alpha, dl, dr);
    float C_Y;
    if (is_zero(airspeed)) {
        C_Y = 0;
    } else {
        C_Y = C_Y_b*beta + (C_Y_p*p + C_Y_r*r)*b/(2*airspeed);
    }

    float pdyn = 0.5*rho*pow(airspeed,2);
    Vector3f F_aero;
    Matrix3f Ra2b;
    Ra2b.a = {cos(alpha)*cos(beta), -cos(alpha)*sin(beta) , -sin(alpha)};
    Ra2b.b = {sin(beta), cos(beta), 0};
    Ra2b.c = {sin(alpha)*cos(beta), -sin(alpha)*sin(beta), cos(alpha)};
    Vector3f force_windaxis = {-pdyn*s*C_D, pdyn*s*C_Y, -pdyn*s*C_L};

    F_aero = Ra2b*force_windaxis;

    return F_aero;
}

void Plane::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel)
{
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float rudder   = filtered_servo_angle(input, 3);
    bool launch_triggered = input.servos[6] > 1700;
    float throttle;
    if (reverse_elevator_rudder) {
        elevator = -elevator;
        rudder = -rudder;
    }
    if (elevons) {
        // fake an elevon plane
        float ch1 = aileron;        // confirmar se ch1 ch2 têm o aileron elevator ou elevonL elevonR -> ch1 dL, ch2 dR
        float ch2 = elevator;
        aileron  = (ch2-ch1)/2.0f;  
        // the minus does away with the need for RC2_REVERSED=-1
        elevator = -(ch2+ch1)/2.0f;

        if (flyingwing) {
            elevator = (ch2+ch1)/2.0f;
        }

        // assume no rudder
        rudder = 0;
    } else if (vtail) {
        // fake a vtail plane
        float ch1 = elevator;
        float ch2 = rudder;
        // this matches VTAIL_OUTPUT==2
        elevator = (ch2-ch1)/2.0f;
        rudder   = (ch2+ch1)/2.0f;
    } else if (dspoilers) {
        // fake a differential spoiler plane. Use outputs 1, 2, 4 and 5
        float dspoiler1_left = filtered_servo_angle(input, 0);
        float dspoiler1_right = filtered_servo_angle(input, 1);
        float dspoiler2_left = filtered_servo_angle(input, 3);
        float dspoiler2_right = filtered_servo_angle(input, 4);
        float elevon_left  = (dspoiler1_left + dspoiler2_left)/2;
        float elevon_right = (dspoiler1_right + dspoiler2_right)/2;
        aileron  = (elevon_right-elevon_left)/2;
        elevator = (elevon_left+elevon_right)/2;
        rudder = fabsf(dspoiler1_right - dspoiler2_right)/2 - fabsf(dspoiler1_left - dspoiler2_left)/2;
    }
    //printf("Aileron: %.1f elevator: %.1f rudder: %.1f\n", aileron, elevator, rudder);

    if (reverse_thrust) {
        throttle = filtered_servo_angle(input, 2);
    } else {
        throttle = filtered_servo_range(input, 2);
    }
    
    float thrust     = throttle;

    battery_voltage = sitl->batt_voltage - 0.7*throttle;
    battery_current = (battery_voltage/sitl->batt_voltage)*50.0f*sq(throttle);

    if (ice_engine) {
        thrust = icengine.update(input);
    }

    // calculate angle of attack
    angle_of_attack = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    beta = atan2f(velocity_air_bf.y,velocity_air_bf.x);

    if (tailsitter || aerobatic) {
        /*
          tailsitters get 4x the control surfaces
         */
        aileron *= 4;
        elevator *= 4;
        rudder *= 4;
    }

    if (custom_dynamics) {
        //printf("Custom dynamics working");
        Vector3f force = CustomDynamics_getForce(ch1, ch2);        // aerodynamic force
        rot_accel = CustomDynamics_getTorque(aileron, elevator);
    } else {
        Vector3f force = getForce(aileron, elevator, rudder);
        rot_accel = getTorque(aileron, elevator, rudder, thrust, force);
    }

    if (have_launcher) {
        /*
          simple simulation of a launcher
         */
        if (launch_triggered) {
            uint64_t now = AP_HAL::millis64();
            if (launch_start_ms == 0) {
                launch_start_ms = now;
            }
            if (now - launch_start_ms < launch_time*1000) {
                force.x += mass * launch_accel;
                force.z += mass * launch_accel/3;
            }
        } else {
            // allow reset of catapult
            launch_start_ms = 0;
        }
    }
    
    // simulate engine RPM
    motor_mask |= (1U<<2);
    rpm[2] = thrust * 7000;
    
    // scale thrust to newtons
    thrust *= thrust_scale;

    accel_body = Vector3f(thrust, 0, 0) + force;
    accel_body /= mass;

    // add some noise
    if (thrust_scale > 0) {
        add_noise(fabsf(thrust) / thrust_scale);
    }

    if (on_ground() && !tailsitter) {
        // add some ground friction
        Vector3f vel_body = dcm.transposed() * velocity_ef;
        accel_body.x -= vel_body.x * 0.3f;
    }
}
    
/*
  update the plane simulation by one time step
 */
void Plane::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    update_wind(input);
    
    calculate_forces(input, rot_accel);
    
    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
