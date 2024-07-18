
#pragma once
//#include "AC_CustomControl_config.h"
#include "AC_CustomControl_Backend.h"

class AC_CustomControl_XYZ : public AC_CustomControl_Backend {
public:
	AC_CustomControl_XYZ(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);
	Vector3f update(void) override;
	void reset(void) override;
	static const struct AP_Param::GroupInfo var_info[];

protected:
	AP_Float param1;
	AP_Float param2;
	AP_Float param3;
};
