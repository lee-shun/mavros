//#include <ros/ros.h>
//#include <std_msgs/String.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iomanip>
#include <iostream>
#include "mathlib.h"

#define CONSTANTS_ONE_G 9.80665

class TECS
{
private:
    // TECS tuning parameters
    float _hgtCompFiltOmega;
    float _spdCompFiltOmega;
    float _maxClimbRate;
    float _minSinkRate;
    float _maxSinkRate;
    float _timeConst;
    float _timeConstThrot;
    float _ptchDamp;
    float _thrDamp;
    float _integGain;
    float _vertAccLim;
    float _rollComp;
    float _spdWeight;
    float _heightrate_p;
    float _heightrate_ff;
    float _speedrate_p;
    // Time since last update of main TECS loop (seconds)
    float _DT;
    static const float DT_MIN = 0.001;
    static const float DT_DEFAULT = 0.02;
    static const float DT_MAX = 1.0;

    bool _states_initalized;
    bool _in_air;

    // Last time update_pitch_throttle was called
    float _update_pitch_throttle_last_usec;
    // throttle demand in the range from 0.0 to 1.0
    float _throttle_dem;

    // pitch angle demand in radians
    float _pitch_dem;

    // Integrator state 1 - height filter second derivative
    float _integ1_state;

    // Integrator state 2 - height rate
    float _integ2_state;

    // Integrator state 3 - height
    float _integ3_state;

    // Integrator state 4 - airspeed filter first derivative
    float _integ4_state;

    // Integrator state 5 - true airspeed
    float _integ5_state;

    // Integrator state 6 - throttle integrator
    float _integ6_state;

    // Integrator state 7 - pitch integrator
    float _integ7_state;

    // throttle demand rate limiter state
    float _last_throttle_dem;

    // pitch demand rate limiter state
    float _last_pitch_dem;

    // Rate of change of speed along X axis
    float _vel_dot;

    // Equivalent airspeed
    float _EAS;

    // Maximum and minimum floating point throttle limits
    float _THRmaxf;
    float _THRminf;
    // Maximum and minimum floating point pitch limits
    float _PITCHmaxf;
    float _PITCHminf;

    // pitch demand before limiting
    float _pitch_dem_unc;
    // height demands
    float _hgt_dem;
    float _hgt_dem_in_old;
    float _hgt_dem_adj;
    float _hgt_dem_adj_last;
    float _hgt_rate_dem;
    float _hgt_dem_prev;
    // Current and last true airspeed demand
    float _TAS_dem;
    float _TAS_dem_last;

    // Speed demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_dem_adj;

    // Speed rate demand after application of rate limiting
    // This is the demand tracked by the TECS control loops
    float _TAS_rate_dem;
    // Underspeed condition
    bool _underspeed;

    // Underspeed detection enabled
    bool _detect_underspeed_enabled;

    // Bad descent condition caused by unachievable airspeed demand
    bool _badDescent;

    // climbout mode
    bool _climbOutDem;

    float _indicated_airspeed_min;
    float _indicated_airspeed_max;

    float _update_speed_last_usec;

    // Equivalent airspeed demand
    float _EAS_dem;

    // True airspeed limits
    float _TASmax;
    float _TASmin;
    // Maximum and minimum specific total energy rate limits
    float _STEdot_max;
    float _STEdot_min;

public:
    void update_pitch_throttle(float time_now, const float rotMat[2][2], float pitch, float baro_altitude, float hgt_dem,
                               float EAS_dem, float indicated_airspeed, float EAS2TAS, bool climbOutDem, float ptchMinCO,
                               float throttle_min, float throttle_max, float throttle_cruise, float pitch_limit_min, float pitch_limit_max);
    // Initialise states and variables
    void _initialise_states(float pitch, float throttle_cruise, float baro_altitude, float ptchMinCO_rad, float EAS2TAS);

    // Update the airspeed internal state using a second order complementary filter
    void _update_speed(float time_now, float airspeed_demand, float indicated_airspeed,
                       float indicated_airspeed_min, float indicated_airspeed_max, float EAS2TAS);
    void _update_STE_rate_lim();
    void _detect_underspeed();
    void _update_speed_demand();
};