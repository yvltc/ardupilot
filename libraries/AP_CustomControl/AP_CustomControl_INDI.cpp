#include "AP_CustomControl_INDI.h"

#if CUSTOMCONTROL_INDI_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

void sspace(Vector3f u, Vector3f x, Matrix3f A, Matrix3f B, Matrix3f C, Matrix3f D, Vector3f *y, Vector3f *x_next)
{
    *x_next = A*x + B*u;
    *y = C*x + D*u;
}

void saturate(float min, float max, float *u)
{
    if (*u > max)
        *u = max;

    else if (*u < min)
        *u = min;
}

// table of user settable parameters
const AP_Param::GroupInfo AP_CustomControl_INDI::var_info[] = {
    // @Param: PARAM1
    // @DisplayName: INDI param1
    // @Description: Dummy parameter for INDI custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM1", 1, AP_CustomControl_INDI, param1, 0.0f),

    // @Param: PARAM2
    // @DisplayName: INDI param2
    // @Description: Dummy parameter for INDI custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM2", 2, AP_CustomControl_INDI, param2, 0.0f),

    // @Param: PARAM3
    // @DisplayName: INDI param3
    // @Description: Dummy parameter for INDI custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM3", 3, AP_CustomControl_INDI, param3, 0.0f),

    AP_GROUPEND
};

// initialize in the constructor
AP_CustomControl_INDI::AP_CustomControl_INDI(AP_CustomControl& frontend, AP_PitchController *pitchController, AP_RollController *rollController, AP_YawController *yawController, AP_AHRS &ahrs, float dt) :
    AP_CustomControl_Backend(frontend, ahrs, dt)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise variables
    u_0.x = 0;
    u_0.y = 0;
    u_0.z = 0;

    // saturation limits
    ddmax = 30*M_PI;
    dtmax = 1;
    dtmin = 0.1;

    // SOD - each vector is a row
    SOD_A.a = {0.846153846153846,0.003076923076923,0};
    SOD_A.b = {-30.769230769230770,-0.384615384615385,0};
    SOD_A.c = {0,0,0};
    SOD_B.a = {30.769230769230680,0,0};
    SOD_B.b = {-1.384615384615385e+04,0,0};
    SOD_B.c = {0,0,0};
    SOD_C.a = {0.923076923076923,0.001538461538462,0};
    SOD_C.b = {0,0,0};
    SOD_C.c = {0,0,0};
    SOD_D.a = {15.384615384615340,0,0};
    SOD_D.b = {0,0,0};
    SOD_D.c = {0,0,0};

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
    _CF_A = 0.3333;
    _CF_B = 0.6667;
    _CF_C = 0.6667;
    _CF_D = 0.3333;

    _CFu_A = 0.9048;
    _CFu_B = 0.09524;
    _CFu_C = 0.9524;
    _CFu_D = 0.04762;

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

    // valores a afinar
    lambda = 0.6;
    Kff = 140;
    Ktt = 50;
    KVt = 0.02;
    Kp = 55;
    Kq = 10;

    // cada vetor é uma linha
    // afinar valores depois
    if (is_gliding)
    {
        invG.a = {-864.9306,864.9306,0};
        invG.b = {-103.2060,-104.6034,0};
        invG.c = {0,0,1};
    }
    else
    {
        invG.a = {-397.6476,397.6476,-6.1354};
        invG.b = {-47.4484,-47.4484,-0.0598};
        invG.c = {0.4008,0.4008,6.2293};
    }

    invert_G = invG.invert();
}

// update controller
// return roll controller output
float AP_CustomControl_INDI::get_roll_out(float roll_target)
{
    //float demanded_roll = _rollController->get_pid_info().target; 
    // ArduPlane main attitude controller already ran
    // we don't need to do anything else

    gcs().send_text(MAV_SEVERITY_INFO, "roll INDI custom controller working");

    // return what ArduPlane main controller outputted
    // return SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    return u_0[0];
}

//return pitch controller output
float AP_CustomControl_INDI::get_pitch_out(float pitch_target)
{
    //float demanded_pitch = _pitchController->get_pid_info().target; 
    // ArduPlane main attitude controller already ran
    // we don't need to do anything else

    gcs().send_text(MAV_SEVERITY_INFO, "pitch INDI custom controller working");

    // return what ArduPlane main controller outputted
    // return SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
    return u_0[1];
}

//return yaw controller output
float AP_CustomControl_INDI::get_yaw_out(void)
{
    // ArduPlane main attitude controller already ran
    // we don't need to do anything else

    gcs().send_text(MAV_SEVERITY_INFO, "yaw INDI custom controller working");

    // return what ArduPlane main controller outputted
    return SRV_Channels::get_output_scaled(SRV_Channel::k_rudder);

    // não preciso disto, posso deixar estar assim
}

float AP_CustomControl_INDI::get_Vt_out(void)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Vt INDI custom controller working");

    // return what ArduPlane main controller outputted
    // return SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
    return u_0[2];
}

void AP_CustomControl_INDI::update(void)
{  
    Vector3f angular_rates = _ahrs.get_gyro_latest();
    float phi = _ahrs.roll;
    float theta = _ahrs.pitch;
    float Vt;
    bool use_TAS = _ahrs.airspeed_estimate_true(&Vt);

    if (!use_TAS)
    {
        use_TAS = _ahrs.airspeed_estimate(&Vt);
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

    aux.x = angular_rates[0];       // p
    sspace(aux, xSOD_p, SOD_A, SOD_B, SOD_C, SOD_D, &SOD_out, &xSOD_p);
    p_dot = SOD_out[0];

    aux.x = angular_rates[1];       // q
    sspace(aux, xSOD_q, SOD_A, SOD_B, SOD_C, SOD_D, &SOD_out, &xSOD_q);
    q_dot = SOD_out[0];

    aux.x = Vt;                     // Vt
    sspace(aux, xSOD_u, SOD_A, SOD_B, SOD_C, SOD_D, &SOD_out, &xSOD_u);
    p_dot = SOD_out[0];
    
    // MATLAB/Simulink dá as equações para converter espaço de estados diretamente
    
    error.x = nav_roll_cd - roll;
    error.y = nav_pitch_cd - pitch;
    error.z = _TAS_dem - Vt;

    niu.x = Kff*error[0] - Kp*angular_rates[0];
    niu.y = Ktt*error[1] - Kq*angular_rates[1]; 
    niu.z = KVt*error[2];

    du.x = invG.a.x*lambda*(niu.x - p_dot) + invG.a.y*lambda*(niu.y - q_dot) + invG.a.z*lambda*(niu.z - Vt_dot);
    du.y = invG.b.x*lambda*(niu.x - p_dot) + invG.b.y*lambda*(niu.y - q_dot) + invG.b.z*lambda*(niu.z - Vt_dot);
    du.z = invG.c.x*lambda*(niu.x - p_dot) + invG.c.y*lambda*(niu.y - q_dot) + invG.c.z*lambda*(niu.z - Vt_dot);

    u.x = u_0.x + du.x;
    u.y = u_0.y + du.y;

    saturate(-0.9*ddmax, 0.9*ddmax, &u.x);
    saturate(-0.9*ddmax, 0.9*ddmax, &u.y);

    if (is_gliding)
    {
        // código de planador
        u.z = 0;
    }
    else
    {
        // código de motorizado
        u.z = u_0.z + du.z;

        saturate(dtmin, dtmax, &u.z);
    }

    // command filter
    sspace(u, xCF, CF_A, CF_B, CF_C, CF_D, &u, &xCF);

    u_0 = u;

    // float aileron_out = get_roll_out(nav_roll_cd);
    // float elevator_out = get_pitch_out(nav_pitch_cd);

    // if (!is_gliding)
    // {
    //     float throttle_out = get_Vt_out();
    // }

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

    u_0.x = get_output_scaled(SRV_Channel::k_aileron);
    u_0.y = get_output_scaled(SRV_Channel::k_elevator);
    u_0.y = get_output_scaled(SRV_Channel::k_throttle);
}

#endif
