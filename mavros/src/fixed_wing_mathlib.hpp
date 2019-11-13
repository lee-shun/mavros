#ifndef _FIXED_WING_MATHLIB_HPP_
#define _FIXED_WING_MATHLIB_HPP_

#include "pid_controller.h"

class _FIXED_WING_MATHLIB
{
private:
public:
    void quaternion_2_euler(float quat[4], float angle[3]); //四元数转欧拉角
    float pid_controller(float kp, float ki, float kd, float sampleTimeSeconds,
                         float minOutput, float maxOutput, float pidinput, float pidsetpoint);
};

void _FIXED_WING_MATHLIB::quaternion_2_euler(float quat[4], float angle[3])
{
    // 四元数转Euler
    // q0 q1 q2 q3
    // w x y z
    angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
}

float _FIXED_WING_MATHLIB::pid_controller(float kp, float ki, float kd, float sampleTimeSeconds,
                                          float minOutput, float maxOutput, float pidinput, float pidsetpoint)
{
    PIDControl controller(kp, ki, kd, sampleTimeSeconds, minOutput, maxOutput, AUTOMATIC, DIRECT);

    controller.PIDInputSet(pidinput);
    controller.PIDSetpointSet(pidsetpoint);
    controller.PIDSampleTimeSet(sampleTimeSeconds);
    controller.PIDCompute();

    return controller.PIDOutputGet();
}

#endif