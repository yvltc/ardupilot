#pragma once

#include "AP_CustomControl_Backend.h"

#ifndef CUSTOMCONTROL_INDI_ENABLED
    #define CUSTOMCONTROL_INDI_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_INDI_ENABLED

class AP_CustomControl_INDI : public AP_CustomControl_Backend {
public:
    AP_CustomControl_INDI(AP_CustomControl& frontend,  AP_PitchController *pitchController, AP_RollController *rollController, AP_YawController *yawController, AP_AHRS &ahrs, float dt);


    float get_roll_out(float roll_target) override;
    float get_pitch_out(float pitch_target) override;
    float get_yaw_out(void) override;
    float get_Vt_out(void);
    void reset(void) override;

    void update(void);

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

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // declare parameters here

    Vector3f u_0;

    // saturation limits
    float ddmax;
    float dtmax;
    float dtmin;

    Matrix3f SOD_A;
    Matrix3f SOD_B;
    Matrix3f SOD_C;
    Matrix3f SOD_D;

    Vector3f xSOD_p;
    Vector3f xSOD_q;
    Vector3f xSOD_u;

    // command filter
    float _CF_A;
    float _CF_B;
    float _CF_C;
    float _CF_D;

    float _CFu_A;
    float _CFu_B;
    float _CFu_C;
    float _CFu_D;

    Vector3f xCF;

    Matrix3f CF_A;
    Matrix3f CF_B;
    Matrix3f CF_C;
    Matrix3f CF_D;

    // valores a afinar
    float lambda;
    float Kff;
    float Ktt;
    float KVt;
    float Kp;
    float Kq;

    Matrix3f invG;
    
    bool invert_G;

    AP_PitchController *_pitchController;
    AP_RollController *_rollController;
    AP_YawController *_yawController;
};

#endif
