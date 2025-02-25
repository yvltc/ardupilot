#include "AP_CustomControl_INDI.h"

#if CUSTOMCONTROL_INDI_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <iostream>

// table of user settable parameters
const AP_Param::GroupInfo AP_CustomControl_INDI::var_info[] = {
    // @Param: INDI_KFF
    // @DisplayName: INDI param1
    // @Description: Dummy parameter for INDI custom controller backend
    // @User: Advanced
    AP_GROUPINFO("INDI_KFF", 1, AP_CustomControl_INDI, Kff, 123.3015),

    // @Param: PARAM2
    // @DisplayName: INDI param2
    // @Description: Dummy parameter for INDI custom controller backend
    // @User: Advanced
    AP_GROUPINFO("INDI_KTT", 2, AP_CustomControl_INDI, Ktt, 78.2422),

    // @Param: PARAM3
    // @DisplayName: INDI param3
    // @Description: Dummy parameter for INDI custom controller backend
    // @User: Advanced
    AP_GROUPINFO("INDI_KVT", 3, AP_CustomControl_INDI, KVt, 6.2656),

    AP_GROUPINFO("INDI_KP", 4, AP_CustomControl_INDI, Kp, 75.5625),

    AP_GROUPINFO("INDI_KQ", 5, AP_CustomControl_INDI, Kq, 8.8438),

    AP_GROUPINFO("INDI_LL", 6, AP_CustomControl_INDI, lambda, 0.3),

    AP_GROUPINFO("INDI_DEBUG", 7, AP_CustomControl_INDI, PID_debugger, 0),

    // AP_GROUPINFO("PARAM1", 1, AP_CustomControl_INDI, param1, 0.0f),

    AP_GROUPEND
};

// initialize in the constructor
AP_CustomControl_INDI::AP_CustomControl_INDI(AP_CustomControl& frontend, AP_PitchController *pitchController, AP_RollController *rollController, AP_YawController *yawController, AP_AHRS &ahrs, AP_TECS &tecs, float dt) :
    AP_CustomControl_Backend(frontend, ahrs, tecs, dt)
{
    AP_Param::setup_object_defaults(this, var_info);

    _dt = dt;

    // initialise variables
    u_0.x = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)*M_PI/18000;
    u_0.y = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)*M_PI/18000;
    u_0.z = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)/100;
    // u_0.x = -0.2041;
    // u_0.y = -0.2025;
    // u_0.z = 0.6428;

    // saturation limits
    ddmax = 30*M_PI/180;
    dtmax = 1;
    dtmin = 0;

    // SOD - each vector is a row
    // SOD_A.a = {0.846153846153846,0.003076923076923,0};
    // SOD_A.b = {-30.769230769230770,-0.384615384615385,0};
    // SOD_A.c = {0,0,0};
    // SOD_B.a = {30.769230769230680,0,0};
    // SOD_B.b = {-1.384615384615385e+04,0,0};
    // SOD_B.c = {0,0,0};
    // SOD_C.a = {0.923076923076923,0.001538461538462,0};
    // SOD_C.b = {0,0,0};
    // SOD_C.c = {0,0,0};
    // SOD_D.a = {15.384615384615340,0,0};
    // SOD_D.b = {0,0,0};
    // SOD_D.c = {0,0,0};
    SOD_A.a = {0.9723,0.0062,0};
    SOD_A.b = {-5.5470,0.2327,0};
    SOD_A.c = {0,0,0};
    SOD_B.a = {5.5470,0,0};
    SOD_B.b = {-690.9009,0,0};
    SOD_B.c = {0,0,0};
    SOD_C.a = {0.9861,0.0031,0};
    SOD_C.b = {0,0,0};
    SOD_C.c = {0,0,0};
    SOD_D.a = {2.7735,0,0};
    SOD_D.b = {0,0,0};
    SOD_D.c = {0,0,0};

    SODu_A.a = {0.9394,0.0048,0};
    SODu_A.b = {-12.1212,-0.0303,0};
    SODu_A.c = {0,0,0};
    SODu_B.a = {12.1212,0,0};
    SODu_B.b = {-2575.8,0,0};
    SODu_B.c = {0,0,0};
    SODu_C.a = {0.9697,0.0024,0};
    SODu_C.b = {0,0,0};
    SODu_C.c = {0,0,0};
    SODu_D.a = {6.0606,0,0};
    SODu_D.b = {0,0,0};
    SODu_D.c = {0,0,0};

    // initial values for SOD state vectors
    xSOD_p.x = 0;
    xSOD_p.y = 0;
    xSOD_p.z = 0;
    xSOD_q.x = 0;
    xSOD_q.y = 0;
    xSOD_q.z = 0;
    xSOD_u.x = 0;
    xSOD_u.y = 0;
    xSOD_u.z = 0;

    // command filter
    _CF_A = 0.6667;
    _CF_B = 0.3333;
    _CF_C = 0.8333;
    _CF_D = 0.1667;

    _CFu_A = 0.9048;
    _CFu_B = 0.0952;
    _CFu_C = 0.9524;
    _CFu_D = 0.0476;
    // _CF_A = 0;
    // _CF_B = 1;
    // _CF_C = 0.5;
    // _CF_D = 0.5;

    // _CFu_A = 0.8182;
    // _CFu_B = 0.1818;
    // _CFu_C = 0.9091;
    // _CFu_D = 0.09091;

    // _CFu_A = 0.3333;
    // _CFu_B = 0.6667;
    // _CFu_C = 0.6667;
    // _CFu_D = 0.3333;

    // initial values for CF state vectors
    xCF.x = 0;
    xCF.y = 0;
    xCF.z = 0;

    CF_A.a = {_CF_A,0,0};
    CF_A.b = {0,_CF_A,0};
    CF_A.c = {0,0,_CFu_A};
    CF_B.a = {_CF_B,0,0};
    CF_B.b = {0,_CF_B,0};
    CF_B.c = {0,0,_CFu_B};
    CF_C.a = {_CF_C,0,0};
    CF_C.b = {0,_CF_C,0};
    CF_C.c = {0,0,_CFu_C};
    CF_D.a = {_CF_D,0,0};
    CF_D.b = {0,_CF_D,0};
    CF_D.c = {0,0,_CFu_D};

    // PID
    error_0 = {0,0,0};
}

void AP_CustomControl_INDI::sspace(Vector3f u, Vector3f x, Matrix3f A, Matrix3f B, Matrix3f C, Matrix3f D, Vector3f *y, Vector3f *x_next)
{
    *x_next = A*x + B*u;
    *y = C*x + D*u;
}

void AP_CustomControl_INDI::saturate(float min, float max, float *u)
{
    if (*u > max)
        *u = max;

    else if (*u < min)
        *u = min;
}

// update controller
// return roll controller output
float AP_CustomControl_INDI::get_roll_out(float roll_target)
{
    //float demanded_roll = _rollController->get_pid_info().target; 
    // ArduPlane main attitude controller already ran
    // we don't need to do anything else
    // uint32_t timestamp = AP_HAL::millis();
    // gcs().send_text(MAV_SEVERITY_INFO, "Timestamp: %d miliseconds", timestamp);

    //gcs().send_text(MAV_SEVERITY_INFO, "roll INDI custom controller working");
    // char buffer[80];  // Create a buffer to hold the formatted message
    // snprintf(buffer, sizeof(buffer), "roll INDI custom controller working, u_0[0] u_0[1] u_0[2]: %.2f %.2f %.2f", u_0[0]*180/M_PI,  u_0[1]*180/M_PI, u_0[2]*100);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message

    // return what ArduPlane main controller outputted
    //return SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    return -u_0[0]*18000/M_PI;
    // return u_0[0]*18000/M_PI;
}

//return pitch controller output
float AP_CustomControl_INDI::get_pitch_out(float pitch_target)
{
    //float demanded_pitch = _pitchController->get_pid_info().target; 
    // ArduPlane main attitude controller already ran
    // we don't need to do anything else

    //gcs().send_text(MAV_SEVERITY_INFO, "pitch INDI custom controller working");
    // char buffer[80];  // Create a buffer to hold the formatted message
    // snprintf(buffer, sizeof(buffer), "pitch INDI custom controller working, u_0[1]: %.2f", u_0[1]*18000/M_PI);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message

    // return what ArduPlane main controller outputted
    //return SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
    return -u_0[1]*18000/M_PI;
    // return u_0[1]*18000/M_PI;
}

//return yaw controller output
float AP_CustomControl_INDI::get_yaw_out(void)
{
    // ArduPlane main attitude controller already ran
    // we don't need to do anything else

    //gcs().send_text(MAV_SEVERITY_INFO, "yaw INDI custom controller working");

    // return what ArduPlane main controller outputted
    return SRV_Channels::get_output_scaled(SRV_Channel::k_rudder);
    //return 0;

    // não preciso disto, posso deixar estar assim
}

float AP_CustomControl_INDI::get_Vt_out(void)
{
    //gcs().send_text(MAV_SEVERITY_INFO, "Vt INDI custom controller working");

    // return what ArduPlane main controller outputted
    // return SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    return u_0[2]*100;
    // return u_0[2];
}

void AP_CustomControl_INDI::update(float roll_target, float pitch_target)
{  
    // cada vetor é uma linha
    // afinar valores depois

    float arspd_target = _tecs.get_TAS_demand();

    is_gliding = _tecs.get_is_gliding();
    if (is_gliding)
    {
        invG.a = {-864.9306,864.9306,0};
        invG.b = {-103.2060,-104.6034,0};
        invG.c = {0,0,1};
    }
    else
    {
        // X.G
        // invG.a = {-397.6476,397.6476,-6.1354};
        // invG.b = {-47.4484,-47.4484,-0.0598};
        // invG.c = {0.4008,0.4008,6.2293};
        // fwztrim.G
        invG.a = {788.1938,0,-8.0174};
        invG.b = {0,-94.0491, -0.1493};
        invG.c = {0,0.8085,8.1225};
    }

    invert_G = invG.invert();

    Vector3f angular_rates = _ahrs.get_gyro_latest();
    float phi = _ahrs.get_roll();       // radians
    float theta = _ahrs.get_pitch();
    float Vt;

    bool use_TAS = _ahrs.airspeed_estimate_true(Vt);

    if (!use_TAS)
    {
        use_TAS = _ahrs.airspeed_estimate(Vt);
    }

    // AHRS airspeed estimate true (true airspeed)
    // altitude desejada no TECS se quiser

    Vector3f error;
    Vector3f niu;
    Vector3f du;
    Vector3f u;

    // SOD
    Vector3f SOD_out;
    float p_dot;
    float q_dot;
    float Vt_dot;

    Vector3f aux;
    aux.y = 0;
    aux.z = 0;

    //debug
    // roll_target = 0;
    // pitch_target = -0.0015*18000/M_PI;
    // arspd_target = 20;
    // phi = -0.0087;
    // theta = -0.0197;
    // Vt = 19.9523;
    // angular_rates = {-0.3086, -0.3094, 19.9523};
    // u_0 = {0.0767, -0.2121, 0.0572};
    // xCF = {0.0785, -0.2183, 0.0594};
    // char buffer[80];  // Create a buffer to hold the formatted message
    // snprintf(buffer, sizeof(buffer), "xSODp: %.6f %.6f %.6f", xSOD_p.x, xSOD_p.y, xSOD_p.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    // snprintf(buffer, sizeof(buffer), "xSODq: %.6f %.6f %.6f", xSOD_q.x, xSOD_q.y, xSOD_q.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    // snprintf(buffer, sizeof(buffer), "xSODu: %.6f %.6f %.6f", xSOD_u.x, xSOD_u.y, xSOD_u.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
 
    aux.x = angular_rates[0];       // p
    sspace(aux, xSOD_p, SOD_A, SOD_B, SOD_C, SOD_D, &SOD_out, &xSOD_p);
    p_dot = SOD_out[0];

    aux.x = angular_rates[1];       // q
    sspace(aux, xSOD_q, SOD_A, SOD_B, SOD_C, SOD_D, &SOD_out, &xSOD_q);
    q_dot = SOD_out[0];

    aux.x = Vt;                     // Vt
    sspace(aux, xSOD_u, SODu_A, SODu_B, SODu_C, SODu_D, &SOD_out, &xSOD_u);
    Vt_dot = SOD_out[0];

    // snprintf(buffer, sizeof(buffer), "xSODp: %.6f %.6f %.6f", xSOD_p.x, xSOD_p.y, xSOD_p.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    // snprintf(buffer, sizeof(buffer), "xSODq: %.6f %.6f %.6f", xSOD_q.x, xSOD_q.y, xSOD_q.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    // snprintf(buffer, sizeof(buffer), "x_next_SODu: %.6f %.6f %.6f", xSOD_u.x, xSOD_u.y, xSOD_u.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    
    // MATLAB/Simulink dá as equações para converter espaço de estados diretamente
    
    //
    // p_dot = -1.5921;
    // q_dot = -1.6886;
    // Vt_dot = 80.5147;

    error.x = roll_target*M_PI/18000 - phi;     // target is in centidegrees, should be radians
    error.y = pitch_target*M_PI/18000 - theta;
    error.z = arspd_target - Vt;

    niu.x = Kff*error[0] - Kp*angular_rates[0];
    niu.y = Ktt*error[1] - Kq*angular_rates[1]; 
    // niu.x = Kff*error[0] - Kp*p;
    // niu.y = Ktt*error[1] - Kq*q; 
    niu.z = KVt*error[2];

    du.x = invG.a.x*lambda*(niu.x - p_dot) + invG.a.y*lambda*(niu.y - q_dot) + invG.a.z*lambda*(niu.z - Vt_dot);
    du.y = invG.b.x*lambda*(niu.x - p_dot) + invG.b.y*lambda*(niu.y - q_dot) + invG.b.z*lambda*(niu.z - Vt_dot);
    du.z = invG.c.x*lambda*(niu.x - p_dot) + invG.c.y*lambda*(niu.y - q_dot) + invG.c.z*lambda*(niu.z - Vt_dot);

    // test
    // u_0.x = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)*M_PI/18000;
    // u_0.y = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)*M_PI/18000;
    // u_0.z = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)/100;

    u.x = u_0.x + du.x;
    u.y = u_0.y + du.y;

    // snprintf(buffer, sizeof(buffer), "target: %.6f %.6f %.6f", roll_target, pitch_target, arspd_target);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    // snprintf(buffer, sizeof(buffer), "error: %.6f %.6f %.6f", error.x, error.y, error.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    // snprintf(buffer, sizeof(buffer), "pqVt: %.6f %.6f %.6f", angular_rates.x, angular_rates.y, Vt);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    // snprintf(buffer, sizeof(buffer), "pqVtdot: %.6f %.6f %.6f", p_dot, q_dot, Vt_dot);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    // snprintf(buffer, sizeof(buffer), "niu: %.6f %.6f %.6f", niu.x, niu.y, niu.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    // snprintf(buffer, sizeof(buffer), "du: %.6f %.6f %.6f", du.x, du.y, du.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    // snprintf(buffer, sizeof(buffer), "u: %.6f %.6f %.6f", u.x*180/M_PI, u.y*180/M_PI, u.z*100);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message

    saturate(-0.95*ddmax, 0.95*ddmax, &u.x);
    saturate(-0.95*ddmax, 0.95*ddmax, &u.y);

    if (is_gliding)
    {
        // código de planador
        u.z = 0;
    }
    else
    {
        // código de motorizado
        u.z = u_0.z + du.z;

        // snprintf(buffer, sizeof(buffer), "u_0: %.6f %.6f %.6f", u_0.x, u_0.y, u_0.z);
        // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
        // snprintf(buffer, sizeof(buffer), "du: %.6f %.6f %.6f", du.x, du.y, du.z);
        // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
        // snprintf(buffer, sizeof(buffer), "u: %.6f %.6f %.6f", u.x, u.y, u.z);
        // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message

        // saturate(-0.9*ddmax, 0.9*ddmax, &u.x);
        // saturate(-0.9*ddmax, 0.9*ddmax, &u.y);
        saturate(dtmin, dtmax, &u.z);

        // snprintf(buffer, sizeof(buffer), "usat: %.6f %.6f %.6f", u.x, u.y, u.z);
        // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    }

    // snprintf(buffer, sizeof(buffer), "xCF: %.6f %.6f %.6f", xCF.x, xCF.y, xCF.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    // command filter
    sspace(u, xCF, CF_A, CF_B, CF_C, CF_D, &u, &xCF);

    // snprintf(buffer, sizeof(buffer), "uCF: %.6f %.6f %.6f", u.x, u.y, u.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    // snprintf(buffer, sizeof(buffer), "x_next_CF: %.6f %.6f %.6f", xCF.x, xCF.y, xCF.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
    
    // snprintf(buffer, sizeof(buffer), "roll pitch: %.4f %.4f", phi, theta);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message

    Vector3f u_ardupilot;
    u_ardupilot.x = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    u_ardupilot.y = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
    u_ardupilot.z = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    // snprintf(buffer, sizeof(buffer), "ArduPilot controller output, u_0[0] u_0[1] u_0[2]: %.2f %.2f %.2f", u_ardupilot.x/100,  u_ardupilot.y/100, u_ardupilot.z);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message

    u_0.x = u.x;
    u_0.y = u.y;
    u_0.z = u.z;

    // snprintf(buffer, sizeof(buffer), "u post-sat: %.4f %.4f %.4f", u.x*180/M_PI, u.y*180/M_PI, u.z*100);
    // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message

    // PID
    if (PID_debugger == 1)
    {
        ddmax = 45*M_PI/180;
        // char buffer[80];
        // com drag certo deve ser 1 no K_p da velocidade e -150 no K_i de pitch
        // Vector3f K_p = {1,-0.11,1};
        // Vector3f K_d = {-0.04,-0.3,0};
        // Vector3f K_i = {-70,-150,0};
        Vector3f K_p = {1,-0.11,50};
        Vector3f K_d = {-0.04,-0.3,0};
        Vector3f K_i = {-70,-150,200};
        // Vector3f K_ff = {0.345, 0.345, 0};

        // {p, i, d, ...}
        // AC_PID rate_pid{0.08, 0.15, 0, 0.345, 0.666, 3, 0, 12, 150, 1};      // Roll
        // AC_PID rate_pid{0.04, 0.15, 0, 0.345, 0.666, 3, 0, 12, 150, 1};      // Pitch

        Vector3f u_Kp;
        Vector3f u_Kd;
        Vector3f u_Ki;
        Vector3f u_Kff;

        Vector3f error_d;
        Vector3f error_i;

        error_d.x = (error.x - error_0.x)/_dt;
        error_d.y = (error.y - error_0.y)/_dt;
        error_d.z = (error.z - error_0.z)/_dt;

        error_i.x += 0.5*(error.x + error_0.x)*_dt;
        error_i.y += 0.5*(error.y + error_0.y)*_dt;
        error_i.z += 0.5*(error.z + error_0.z)*_dt;

        // snprintf(buffer, sizeof(buffer), "error_0: %.2f %.2f %.2f", error_0.x,  error_0.y, error_0.z);
        // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message
        // snprintf(buffer, sizeof(buffer), "error: %.2f %.2f %.2f", error.x,  error.y, error.z);
        // gcs().send_text(MAV_SEVERITY_INFO, "%s", buffer);  // Send the formatted message

        u_Kp.x = K_p.x*error.x;
        u_Kd.x = K_d.x*error_d.x;
        u_Ki.x = K_i.x*error_i.x;
        u_Kp.y = K_p.y*error.y;
        u_Kd.y = K_d.y*error_d.y;
        u_Ki.y = K_i.y*error_i.y;
        u_Kp.z = K_p.z*error.z;
        u_Kd.z = K_d.z*error_d.z;
        u_Ki.z = K_i.z*error_i.z;
        // u_Kff.x = K_ff.x*roll_target*M_PI/18000;
        // u_Kff.y = K_ff.y*pitch_target*M_PI/18000;
        // u_Kff.z = K_ff.z*arspd_target;

        u = u_Kp + u_Kd + u_Ki; //+ u_Kff;
        saturate(-ddmax, ddmax, &u.x);
        saturate(-ddmax, ddmax, &u.y);
        saturate(dtmin, dtmax, &u.z);
        sspace(u, xCF, CF_A, CF_B, CF_C, CF_D, &u, &xCF);
        // printf("%f\n", roll_target*M_PI/18000);

        u_0 = u;
    }

    error_0 = {error.x, error.y, error.z};
    

}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AP_CustomControl_INDI::reset(void)
{
    // não tenho integrador
    // não sei se é isto que é suposto fazer
    xSOD_p.x = 0;
    xSOD_p.y = 0;
    xSOD_p.z = 0;

    xSOD_q.x = 0;
    xSOD_q.y = 0;
    xSOD_q.z = 0;

    xSOD_u.x = 0;
    xSOD_u.y = 0;
    xSOD_u.z = 0;

    xCF.x = 0;
    xCF.y = 0;
    xCF.z = 0;

    u_0.x = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)*M_PI/18000;
    u_0.y = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator)*M_PI/18000;
    u_0.z = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)/100;
    // u_0.z = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    // printf("%f\n", u_0.z*100);
}

#endif
