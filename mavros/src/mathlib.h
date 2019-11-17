#ifndef _MATHLIB_H_
#define _MATHLIB_H_

float constrain(float val, float min, float max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}
float max(const  float a, const float b)
{
	return (a > b) ? a : b;
}
#endif