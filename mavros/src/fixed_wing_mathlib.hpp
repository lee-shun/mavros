#ifndef _FIXED_WING_MATHLIB_HPP_
#define _FIXED_WING_MATHLIB_HPP_

class _FIXED_WING_MATHLIB
{
private:
public:
    void quaternion_2_euler(float quat[4], float angle[3]); //四元数转欧拉角
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

#endif