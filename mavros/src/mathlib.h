#ifndef _MATHLIB_H_
#define _MATHLIB_H_
#include <math.h>
#define PI 3.1415926535
#define CONSTANTS_RADIUS_OF_EARTH 6371000
#define EARTH_RADIUS 6378137

float constrain(float val, float min, float max)
{
    return (val < min) ? min : ((val > max) ? max : val);
}
float max(const float a, const float b)
{
    return (a > b) ? a : b;
}
float min(const float a, const float b)
{
    return (a < b) ? a : b;
}

void quaternion_2_euler(float quat[4], float angle[3])
{
    // 四元数转Euler
    // q0 q1 q2 q3
    // w x y z
    angle[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    angle[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    angle[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), -1.0 + 2.0 * (quat[0] * quat[0] + quat[1] * quat[1]));
}

void euler_2_quaternion(float angle[3], float quat[4])
{
    // Euler转四元数
    // q0 q1 q2 q3
    // w x y z
    double cosPhi_2 = cos(double(angle[0]) / 2.0);

    double sinPhi_2 = sin(double(angle[0]) / 2.0);

    double cosTheta_2 = cos(double(angle[1]) / 2.0);

    double sinTheta_2 = sin(double(angle[1]) / 2.0);

    double cosPsi_2 = cos(double(angle[2]) / 2.0);

    double sinPsi_2 = sin(double(angle[2]) / 2.0);

    quat[0] = float(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);

    quat[1] = float(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);

    quat[2] = float(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);

    quat[3] = float(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
}

void matrix_plus_vector_3(float vector_a[3], float rotmax[3][3], float vector_b[3])
{
    vector_a[0] = rotmax[0][0] * vector_b[0] + rotmax[0][1] * vector_b[1] + rotmax[0][2] * vector_b[2];

    vector_a[1] = rotmax[1][0] * vector_b[0] + rotmax[1][1] * vector_b[1] + rotmax[1][2] * vector_b[2];

    vector_a[2] = rotmax[2][0] * vector_b[0] + rotmax[2][1] * vector_b[1] + rotmax[2][2] * vector_b[2];
}

/**
	 * create rotation matrix for the quaternion
	 */
void quat_2_rotmax(float q[4], float R[3][3])
{
    float aSq = q[0] * q[0];
    float bSq = q[1] * q[1];
    float cSq = q[2] * q[2];
    float dSq = q[3] * q[3];
    R[0][0] = aSq + bSq - cSq - dSq;
    R[0][1] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
    R[0][2] = 2.0f * (q[0] * q[2] + q[1] * q[3]);
    R[1][0] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
    R[1][1] = aSq - bSq + cSq - dSq;
    R[1][2] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
    R[2][0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    R[2][1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    R[2][2] = aSq - bSq - cSq + dSq;
}

float rad_2_deg(float rad)
{
    float deg;

    deg = rad * 180 / PI;

    return deg;
}

float deg_2_rad(float deg)
{
    float rad;

    rad = deg * PI / 180;

    return rad;
}

//ref,result---lat,long,alt
void cov_m_2_lat_long_alt(const float ref[3], float x, float y, float z, float result[3])
{
    const double x_rad = (double)x / CONSTANTS_RADIUS_OF_EARTH;
    const double y_rad = (double)y / CONSTANTS_RADIUS_OF_EARTH;
    const double c = sqrt(x_rad * x_rad + y_rad * y_rad);

    if (fabs(c) > 0)
    {
        const double sin_c = sin(c);
        const double cos_c = cos(c);

        const double lat_rad = asin(cos_c * sin(ref[0]) + (x_rad * sin_c * cos(ref[0])) / c);
        const double lon_rad = (deg_2_rad(ref[1]) + atan2(y_rad * sin_c, c * cos(ref[0]) * cos_c - x_rad * sin(ref[0]) * sin_c));

        result[0] = rad_2_deg(lat_rad); //ref,result---lat,long,alt
        result[1] = rad_2_deg(lon_rad);
    }
    else
    {
        result[0] = rad_2_deg(ref[0]);
        result[1] = rad_2_deg(ref[1]);
    }

    result[2] = ref[2] + z; //高度
}

double cov_lat_long_2_m(float a_pos[2], float b_pos[2])
{ //参考点是a点，lat，long，alt
    double lat1 = a_pos[0];
    double lon1 = a_pos[1];

    double lat2 = b_pos[0];
    double lon2 = b_pos[1];

    double radLat1 = deg_2_rad(lat1);
    double radLat2 = deg_2_rad(lat2);
    double a = radLat1 - radLat2;
    double b = deg_2_rad(lon1) - deg_2_rad(lon2);
    double s = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2)));
    double m;
    m = s * EARTH_RADIUS;

    return m;
}

#endif