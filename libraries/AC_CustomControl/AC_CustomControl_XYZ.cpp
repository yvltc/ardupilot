//#include "AC_CustomControl_config.h"

// #if AP_CUSTOMCONTROL_EMPTY_ENABLED
#include <stdio.h>
#include <iostream>
#include <sstream>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
//#pragma GCC diagnostic ignored "-Wframe-larger-than"
#pragma push_macro("_GLIBCXX_USE_C99_STDIO")
#undef _GLIBCXX_USE_C99_STDIO
#include <Eigen/Dense>
#include <Eigen/Geometry>
#pragma GCC diagnostic pop
#pragma pop_macro("_GLIBCXX_USE_C99_STDIO")
#include <chrono>

#include "util.h"
#include "AC_CustomControl_XYZ.h"
#include "NN_Parameters.h"

#include <GCS_MAVLink/GCS.h>

using namespace std;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

std::vector<Eigen::MatrixXf> lin_w;
std::vector<Eigen::MatrixXf> l0_w;
std::vector<Eigen::MatrixXf> mean_w;

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
    NN::obs.setZero();

    for (int i = 0; i < NN::N_CONTEXT; i++)
    {
        lin_w.push_back(Eigen::Map<const Eigen::Matrix<float, NN::N_HIDDEN, NN::N_OBS, Eigen::RowMajor>>(&NN::LIN_W[i][0][0]));
    }

    for (int i = 0; i < NN::N_CONTEXT; i++)
    {
        l0_w.push_back(Eigen::Map<const Eigen::Matrix<float, NN::N_HIDDEN, NN::N_HIDDEN, Eigen::RowMajor>>(&NN::L0_W[i][0][0]));
    }

    for (int i = 0; i < NN::N_CONTEXT; i++)
    {
        mean_w.push_back(Eigen::Map<const Eigen::Matrix<float, NN::N_ACT, NN::N_HIDDEN, Eigen::RowMajor>>(&NN::MEAN_W[i][0][0]));
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

    // run custom controller after here
    Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);

    // Quaternion q_ned2enu(0,-std::sqrt(2)/2,-std::sqrt(2)/2,0);
    // Quaternion q_enu = attitude_body * q_ned2enu;
    Quaternion q_enu(attitude_body[0], attitude_body[2], attitude_body[1], -attitude_body[3]);

    Vector3f gyro_latest = _ahrs->get_gyro_latest();

    attitude_target = _att_control->get_attitude_target_quat();
    // This vector represents the angular error to rotate the thrust vector using x and y and heading using z
    Vector3f attitude_error;
    float _thrust_angle, _thrust_error_angle;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, _thrust_angle, _thrust_error_angle);
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
    Eigen::Vector4f rb_quat(q_enu[1], q_enu[2], q_enu[3], q_enu[0]); // ENU
    Eigen::Vector3f rb_vel(0,0,0);
    Eigen::Vector3f rb_angvel(gyro_latest[1], gyro_latest[0], -gyro_latest[2]);
    Eigen::Vector3f err_ang(0,0,0);
    Eigen::Vector3f err_vel(0,0,0);
    Eigen::Vector3f rb_ang(q_enu.get_euler_roll(), q_enu.get_euler_pitch(), q_enu.get_euler_yaw());
    err_ang -= rb_ang;

    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "roll: %s", std::to_string(q_enu.get_euler_roll()).c_str());
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "pitch: %s", std::to_string(q_enu.get_euler_pitch()).c_str());
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "yaw: %s", std::to_string(q_enu.get_euler_yaw()).c_str());

    NN::obs.head(4) = rb_quat.head(4);
    NN::obs.segment(4,3) = rb_vel/30;
    NN::obs.segment(7,3) = rb_angvel/25.1;
    NN::obs.segment(10,3) = err_ang/M_PI;
    NN::obs.segment(13,1) = err_vel.segment(2,1)/30;

    Eigen::Array<float, NN::N_OBS + NN::N_TASK, 1> NN_input;
    NN_input << NN::obs, NN::task;

    // ###### Inference Starts ######
    auto t1 = high_resolution_clock::now();

    // context encoder
    auto tmp = (NN::win_b.matrix() + NN::win_w * NN_input.matrix()).array().cwiseMax(0);
    auto c = softmax(NN::wout_b.matrix() + NN::wout_w * tmp.matrix());

    // composition layer 1
    Eigen::Matrix<float, NN::N_CONTEXT, NN::N_HIDDEN, Eigen::RowMajor> x1;
    for (int i = 0; i < NN::N_CONTEXT; i++)
    {
        x1.row(i) = lin_w[i] * NN::obs.matrix();
    }
    x1 += NN::lin_b;
    Eigen::Matrix<float, NN::N_HIDDEN, 1> out1 = (x1.array().colwise() * c).matrix().colwise().sum().cwiseMax(0);

    // composition layer 2
    Eigen::Matrix<float, NN::N_CONTEXT, NN::N_HIDDEN, Eigen::RowMajor> x2;
    for (int i = 0; i < NN::N_CONTEXT; i++)
    {
        x2.row(i) = l0_w[i] * out1.matrix();
    }
    x2 += NN::l0_b;
    Eigen::Matrix<float, NN::N_HIDDEN, 1> out2 = (x2.array().colwise() * c).matrix().colwise().sum().cwiseMax(0);

    // output layer
    Eigen::Matrix<float, NN::N_CONTEXT, NN::N_ACT, Eigen::RowMajor> x3;
    for (int i = 0; i < NN::N_CONTEXT; i++)
    {
        x3.row(i) = mean_w[i] * out2.matrix();
    }
    x3 += NN::mean_b;
    Eigen::Matrix<float, NN::N_ACT, 1> NN_output = (x3.array().colwise() * c).matrix().colwise().sum();

    NN_output = NN_output.unaryExpr([](float x)
                          { return std::min(std::max(x, -1.0f), 1.0f); });

    auto t2 = high_resolution_clock::now();

    // ###### Inference Ends ######

    // printing the output of the Network
    // std::string matrixStr = matrixToString(NN_output);
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NN output: %s", matrixStr.c_str());

    // printing the inference time of the Network
    duration<double, std::milli> ms_double = t2 - t1;
    double loop_frequency = 1000 / ms_double.count();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NN freq: %s", std::to_string(loop_frequency).c_str());

    // return what arducopter main controller outputted
    Vector3f motor_out;
    motor_out.x = NN_output[1];
    motor_out.y = NN_output[2];
    motor_out.z = NN_output[3];
    return motor_out;
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_XYZ::reset(void)
{
}

// #endif  // AP_CUSTOMCONTROL_EMPTY_ENABLED
