
// #if AP_CUSTOMCONTROL_EMPTY_ENABLED

#include "AC_CustomControl_XYZ.h"
#include <GCS_MAVLink/GCS.h>
#include <algorithm> // for std::copy

#include "util.h"
#include "FIFOBuffer.h"
#include "NN_Parameters.h"

FIFOBuffer fifoBuffer(NN::N_STACK);

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_XYZ::var_info[] = {
    // @Param: PARAM1
    // @DisplayName: XYZ param1
    // @Description: Dummy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM1", 1, AC_CustomControl_XYZ, param1, 0.0f),

    // @Param: PARAM2
    // @DisplayName: XYZ param2
    // @Description: Dummy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM2", 2, AC_CustomControl_XYZ, param2, 0.0f),

    // @Param: PARAM3
    // @DisplayName: XYZ param3
    // @Description: Dummy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM3", 3, AC_CustomControl_XYZ, param3, 0.0f),

    AP_GROUPEND};

// initialize in the constructor
AC_CustomControl_XYZ::AC_CustomControl_XYZ(AC_CustomControl &frontend, AP_AHRS_View *&ahrs, AC_AttitudeControl *&att_control, AP_MotorsMulticopter *&motors, float dt) : AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    std::vector<float> dummy_action = {0,0,0,0};
    std::vector<float> dummys = vecCat(NN::OBS, dummy_action);
    for (int i = 0; i < NN::N_STACK; ++i){
        fifoBuffer.insert(dummys);
    }

    AP_Param::setup_object_defaults(this, var_info);
}

// update controller
// return roll, pitch, yaw controller output
Vector3f AC_CustomControl_XYZ::update(void)
{
    // reset controller based on spool state
    switch (_motors->get_spool_state())
    {
    case AP_Motors::SpoolState::SHUT_DOWN:
    case AP_Motors::SpoolState::GROUND_IDLE:
        // We are still at the ground. Reset custom controller to avoid
        // build up, ex: integrator
        reset();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // we are off the ground
        break;
    }

    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NN controller working");

    // (*)
    Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);
    Vector3f airspeed_earth_ned = _ahrs->airspeed_vector();
    Vector3f airspeed_body_ned = _ahrs->earth_to_body(airspeed_earth_ned);

    // Quaternion q_ned2enu(0,-std::sqrt(2)/2,-std::sqrt(2)/2,0);
    // Quaternion q_enu = attitude_body * q_ned2enu;

    // (*)
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    // attitude_target = _att_control->get_attitude_target_quat();

    // This vector represents the angular error to rotate the thrust vector using x and y and heading using z
    // (*)
    // Vector3f attitude_error;
    // float _thrust_angle, _thrust_error_angle;
    // _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, _thrust_angle, _thrust_error_angle);

    // recalculate ang vel feedforward from attitude target model
    // rotation from the target frame to the body frame
    // Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
    // target angle velocity vector in the body frame
    // Vector3f ang_vel_body_feedforward = rotation_target_to_body * _att_control->get_attitude_target_ang_vel();

    // ###### Prepare NN input ######
    // 0:4 rb_quat # xyzw in ENU
    // 4:7 rb_vel_local/30  # local xyz
    // 7:10 rb_angvel_local/25.1 # local pqr
    // 10:13 err_angle/pi # rpy
    // 13:14 err_vz/30 # local_error_v = target_forward_v - local_forward_v
    // (*)
    // Eigen::Vector4f rb_quat(q_enu[1], q_enu[2], q_enu[3], q_enu[0]); // ENU
    // Eigen::Vector3f rb_vel(rb_vel_NED[1], rb_vel_NED[0], -rb_vel_NED[2]);
    // Eigen::Vector3f rb_angvel(gyro_latest[1], gyro_latest[0], -gyro_latest[2]);
    // Eigen::Vector3f err_ang(0,0,0);
    // Eigen::Vector3f err_vel(0,0,0);
    // Eigen::Vector3f rb_ang(q_enu.get_euler_roll(), q_enu.get_euler_pitch(), q_enu.get_euler_yaw());
    // err_ang -= rb_ang;

    // NN::OBS.head(4) = rb_quat.head(4);
    // NN::OBS.segment(4,3) = rb_vel/30;
    // NN::OBS.segment(7,3) = rb_angvel/25.1;
    // NN::OBS.segment(10,3) = err_ang/M_PI;
    // NN::OBS.segment(13,1) = err_vel.segment(2,1)/30;

    // (*)
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "velocity: %s \n", matrixToString(rb_vel).c_str());
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "quaternion: %s \n", matrixToString(rb_quat).c_str());
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "roll: %s", std::to_string(q_enu.get_euler_roll()).c_str());
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "pitch: %s", std::to_string(q_enu.get_euler_pitch()).c_str());
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "yaw: %s", std::to_string(q_enu.get_euler_yaw()).c_str());

    // convert sensor value to network input. Sensor: NED coordinate; NN input: ENU coordinate.
    // rb_quat
    NN::OBS[0] = attitude_body[0];
    NN::OBS[1] = attitude_body[2];
    NN::OBS[2] = attitude_body[1];
    NN::OBS[3] = -attitude_body[3];
    // // angvel
    Vector3f rb_ned_angvel = gyro_latest/NN::AVEL_LIM;
    NN::OBS[4] = rb_ned_angvel[1];
    NN::OBS[5] = rb_ned_angvel[0];
    NN::OBS[6] = -rb_ned_angvel[2];
    // // rbvel
    Vector3f rb_ned_vel = airspeed_body_ned/NN::VEL_LIM;
    NN::OBS[7] = rb_ned_vel[1];
    NN::OBS[8] = rb_ned_vel[0];
    NN::OBS[9] = -rb_ned_vel[2];
    // NN::OBS[7] = 0;
    // NN::OBS[8] = 0;
    // NN::OBS[9] = 0;

    // ###### Inference Starts ######
    // auto t1 = high_resolution_clock::now();

    // adaptor
    std::vector<std::vector<float>> table = fifoBuffer.getReversedTransposedTable();
    std::vector<std::vector<float>> x_tmp1 = conv1d(NN::BUFFER, NN::CNN_W1, NN::CNN_B1, 1, NN::N_PADD, 1);
    std::vector<std::vector<float>> x_tmp2 = chomp1d(x_tmp1, NN::N_PADD);
    x_tmp2 = relu2D(x_tmp2);

    x_tmp2 = vec2DAdd(x_tmp2, table);
    x_tmp2 = relu2D(x_tmp2);

    std::vector<float> z_tmp = getLastColumn(x_tmp2);
    std::vector<float> z = linear_layer(NN::CNN_LB, NN::CNN_LW, z_tmp, false);

    // policy start here
    std::vector<float> obs = vecCat(NN::OBS, z);
    std::vector<float> context_input = vecCat(obs, NN::TASK);

    // context encoder
    std::vector<float> w;
    w = linear_layer(NN::WIN_B, NN::WIN_W, context_input, true);
    w = linear_layer(NN::WOUT_B, NN::WOUT_W, w, false);
    w = softmax(w);

    // // // composition layers
    std::vector<float> NN_out;
    NN_out = composition_layer(NN::LIN_W, NN::LIN_B, w, obs, true);
    NN_out = composition_layer(NN::L0_W, NN::L0_B, w, NN_out, true);
    NN_out = composition_layer(NN::MEAN_W, NN::MEAN_B, w, NN_out, false);
    clampToRange(NN_out, -1, 1);

    // std::vector<float> NN_out={0,0,0,0};
    std::vector<float> sa_pair = vecCat(NN::OBS, NN_out);
    fifoBuffer.insert(sa_pair);

    // auto t2 = high_resolution_clock::now();

    // ###### Inference Ends ######

    // printing the output of the Network
    // std::string matrixStr = matrixToString(NN_output);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NN output: %s", matrixStr.c_str());

    // printing the inference time of the Network
    // duration<float, std::milli> ms_float = t2 - t1;
    // float loop_frequency = 1000 / ms_float.count();
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NN freq: %s", std::to_string(loop_frequency).c_str());

    // return what arducopter main controller outputted
    Vector3f motor_out;
    motor_out.x = NN_out[2];
    motor_out.y = NN_out[1];
    motor_out.z = -NN_out[3];
    // motor_out.x = 0;
    // motor_out.y = 0;
    // motor_out.z = 0.1;

    return motor_out;
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_XYZ::reset(void)
{
}

// #endif  // AP_CUSTOMCONTROL_EMPTY_ENABLED
