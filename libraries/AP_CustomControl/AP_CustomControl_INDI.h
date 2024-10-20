#pragma once

#include "AP_CustomControl_Backend.h"

#ifndef CUSTOMCONTROL_INDI_ENABLED
    #define CUSTOMCONTROL_INDI_ENABLED AP_CUSTOMCONTROL_ENABLED
#endif

#if CUSTOMCONTROL_INDI_ENABLED

class AP_CustomControl_INDI : public AP_CustomControl_Backend {
public:
    AP_CustomControl_INDI(AP_CustomControl& frontend,  AP_PitchController *pitchController, AP_RollController *rollController, AP_YawController *yawController, AP_AHRS &ahrs, AP_TECS &tecs, float dt);


    float get_roll_out(float roll_target) override;
    float get_pitch_out(float pitch_target) override;
    float get_yaw_out(void) override;
    float get_Vt_out(void);
    void reset(void) override;

    void sspace(Vector3f u, Vector3f x, Matrix3f A, Matrix3f B, Matrix3f C, Matrix3f D, Vector3f *y, Vector3f *x_next);
    void saturate(float min, float max, float *u);

    void update(void);

    // user settable parameters
    // static const struct AP_Param::GroupInfo var_info[];

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
