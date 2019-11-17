/*tecs控制器---魔改版*/
#include "tecs.hpp"

void TECS::_initialise_states(float pitch, float throttle_cruise, float baro_altitude, float ptchMinCO_rad, float EAS2TAS)
{
    // Initialise states and variables if DT > 1 second or in climbout
    if (_update_pitch_throttle_last_usec == 0 || _DT > DT_MAX || !_in_air || !_states_initalized)
    {
        _integ1_state = 0.0f;
        _integ2_state = 0.0f;
        _integ3_state = baro_altitude;
        _integ4_state = 0.0f;
        _integ5_state = _EAS * EAS2TAS;
        _integ6_state = 0.0f;
        _integ7_state = 0.0f;

        _last_throttle_dem = throttle_cruise;
        _last_pitch_dem = constrain(pitch, _PITCHminf, _PITCHmaxf);
        _pitch_dem_unc = _last_pitch_dem;

        _hgt_dem_adj_last = baro_altitude;
        _hgt_dem_adj = _hgt_dem_adj_last;
        _hgt_dem_prev = _hgt_dem_adj_last;
        _hgt_dem_in_old = _hgt_dem_adj_last;

        _TAS_dem_last = _EAS * EAS2TAS;
        _TAS_dem_adj = _TAS_dem_last;

        _underspeed = false;
        _badDescent = false;

        if (_DT > DT_MAX || _DT < DT_MIN)
        {
            _DT = DT_DEFAULT;
        }
    }
    else if (_climbOutDem)
    {
        _PITCHminf = ptchMinCO_rad;
        _THRminf = _THRmaxf - 0.01f;

        _hgt_dem_adj_last = baro_altitude;
        _hgt_dem_adj = _hgt_dem_adj_last;
        _hgt_dem_prev = _hgt_dem_adj_last;

        _TAS_dem_last = _EAS * EAS2TAS;
        _TAS_dem_adj = _EAS * EAS2TAS;

        _underspeed = false;
        _badDescent = false;
    }

    _states_initalized = true;
}

void TECS::_update_speed(float time_now, float airspeed_demand, float indicated_airspeed,
                         float indicated_airspeed_min, float indicated_airspeed_max, float EAS2TAS)
{
    // Calculate time in seconds since last update
    float now = time_now;
    float DT = max((now - _update_speed_last_usec), 0) * 1.0e-6f;

    /*********阿木实验室出品****************
	*
	* 将等效空速转换为实际的空速
	* 其中EAS为等效空速，TAS为实际空速，一般情况下，两者比例为1
	*
	**********阿木实验室出品****************/

    _EAS_dem = airspeed_demand;
    _TAS_dem = _EAS_dem * EAS2TAS;
    _TASmax = indicated_airspeed_max * EAS2TAS;
    _TASmin = indicated_airspeed_min * EAS2TAS;

    // Get airspeed or default to halfway between min and max if
    // airspeed is not being used and set speed rate to zero

    _EAS = indicated_airspeed;

    // Reset states on initial execution or if not active
    if (_update_speed_last_usec == 0 || !_in_air)
    {
        _integ4_state = 0.0f;
        _integ5_state = (_EAS * EAS2TAS);
    }

    if (DT < DT_MIN || DT > DT_MAX)
    {
        DT = DT_DEFAULT; // when first starting TECS, use small time constant
    }

    /*********阿木实验室出品****************
	*
	* _integ4_state 为空速的加速度，先对这个加速度量做一个滤波
	*
	**********阿木实验室出品****************/
    float aspdErr = (_EAS * EAS2TAS) - _integ5_state;
    float integ4_input = aspdErr * _spdCompFiltOmega * _spdCompFiltOmega;

    // Prevent state from winding up
    if (_integ5_state < 3.1f)
    {
        integ4_input = max(integ4_input, 0.0f);
    }

    _integ4_state = _integ4_state + integ4_input * DT;

    /*********阿木实验室出品****************
	*
	* 空速度的加速度平滑完了之后，再对_integ5_state即空速做滤波；
	* 最后做一个保护
	*
	**********阿木实验室出品****************/
    float integ5_input = _integ4_state + _vel_dot + aspdErr * _spdCompFiltOmega * 1.4142f;
    _integ5_state = _integ5_state + integ5_input * DT;

    // limit the airspeed to a minimum of 3 m/s
    _integ5_state = max(_integ5_state, 3.0f);
    _update_speed_last_usec = now;
}

void TECS::_update_STE_rate_lim()
{
    // Calculate Specific Total Energy Rate Limits
    // This is a trivial calculation at the moment but will get bigger once we start adding altitude effects
    _STEdot_max = _maxClimbRate * CONSTANTS_ONE_G;
    _STEdot_min = -_minSinkRate * CONSTANTS_ONE_G;
}

void TECS::_detect_underspeed()
{
    if (!_detect_underspeed_enabled)
    {
        _underspeed = false;
        return;
    }

    if (((_integ5_state < _TASmin * 0.9f) && (_throttle_dem >= _THRmaxf * 0.95f)) || ((_integ3_state < _hgt_dem_adj) && _underspeed))
    {
        _underspeed = true;
    }
    else
    {
        _underspeed = false;
    }
}

void TECS::_update_speed_demand()
{
	// Set the airspeed demand to the minimum value if an underspeed condition exists
	// or a bad descent condition exists
	// This will minimise the rate of descent resulting from an engine failure,
	// enable the maximum climb rate to be achieved and prevent continued full power descent
	// into the ground due to an unachievable airspeed value
	if ((_badDescent) || (_underspeed)) {
		_TAS_dem     = _TASmin;
	}

	// Constrain speed demand
	_TAS_dem = constrain(_TAS_dem, _TASmin, _TASmax);

	// calculate velocity rate limits based on physical performance limits
	// provision to use a different rate limit if bad descent or underspeed condition exists
	// Use 50% of maximum energy rate to allow margin for total energy controller
	float velRateMax;
	float velRateMin;

	if ((_badDescent) || (_underspeed)) {
		velRateMax = 0.5f * _STEdot_max / _integ5_state;
		velRateMin = 0.5f * _STEdot_min / _integ5_state;

	} else {
		velRateMax = 0.5f * _STEdot_max / _integ5_state;
		velRateMin = 0.5f * _STEdot_min / _integ5_state;
	}

	_TAS_dem_adj = constrain(_TAS_dem, _TASmin, _TASmax);
	_TAS_rate_dem = constrain((_TAS_dem_adj - _integ5_state) * _speedrate_p, velRateMin, velRateMax); //xxx: using a p loop for now

}

void TECS::update_pitch_throttle(float time_now, const float rotMat[2][2], float pitch, float baro_altitude, float hgt_dem,
                                 float EAS_dem, float indicated_airspeed, float EAS2TAS, bool climbOutDem, float ptchMinCO,
                                 float throttle_min, float throttle_max, float throttle_cruise, float pitch_limit_min, float pitch_limit_max)
{
    // Calculate time in seconds since last update
    float now = time_now;
    _DT = max((now - _update_pitch_throttle_last_usec), 0) * 1.0e-6f;
    _THRmaxf = throttle_max;
    _THRminf = throttle_min;
    _PITCHmaxf = pitch_limit_max;
    _PITCHminf = pitch_limit_min;
    //11.17完成
    /**************************
	* 初始化一些量，第一次进来这个函数会用到。
	*
	* _integ1_state <---- 高度的二阶导数，即高度方向的加速度；
	* _integ2_state <---- 高度的一阶导数，即高度方向的速度；
	* _integ3_state <---- 高度；
	* _integ4_state <---- 空速的一阶导数，即空速的加速度；
	* _integ5_state <---- 空速；
	* _integ6_state <---- 油门的积分量；
	* _integ7_state <---- pitch的积分量；
	***************************/
    _initialise_states(pitch, throttle_cruise, baro_altitude, ptchMinCO, EAS2TAS);
    //11.17完成
    /**************************
	*
	* 1. 计算当前的空速，对测量到的空速做一个二阶的低通滤波
	*
	***************************/
    _update_speed(time_now, EAS_dem, indicated_airspeed, _indicated_airspeed_min, _indicated_airspeed_max, EAS2TAS);
    //11.17完成
    // /**************************
    // *
    // * 2. 计算动能的极值，当爬升速度最大或最小的时候，取到动能的极值
    // *
    // ***************************/
    _update_STE_rate_lim();
    //11.17完成
    // /**************************
    // *
    // * 3. 检查有没有失速
    // *
    // ***************************/
    _detect_underspeed();
    //11.17完成
    // /**************************
    // *
    // * 4. 计算期望空速和期望空速的加速度
    // *
    // ***************************/
    _update_speed_demand();
    //11.17完成
    // /**************************
    // *
    // * 5. 计算期望高度和期望爬升率
    // *
    // ***************************/
    // _update_height_demand(hgt_dem, baro_altitude);

    // /**************************
    // *
    // * 6. 计算单位质量的势能和动能_SPE\_SKE以及其变化率
    // *
    // ***************************/
    // _update_energies();

    // /**************************
    // *
    // * 7. 根据期望的总能量和当前的总能量计算期望油门值
    // *
    // ***************************/
    // _update_throttle(throttle_cruise, rotMat);

    // /**************************
    // *
    // * 8. 根据期望的能量转化率和当前的能量转化率计算期望油门值
    // *
    // ***************************/
    // _update_pitch();

    // _tecs_state.timestamp = now;

    // if (_underspeed)
    // {
    //     _tecs_state.mode = ECL_TECS_MODE_UNDERSPEED;
    // }
    // else if (_badDescent)
    // {
    //     _tecs_state.mode = ECL_TECS_MODE_BAD_DESCENT;
    // }
    // else if (_climbOutDem)
    // {
    //     _tecs_state.mode = ECL_TECS_MODE_CLIMBOUT;
    // }
    // else
    // {
    //     // If no error flag applies, conclude normal
    //     _tecs_state.mode = ECL_TECS_MODE_NORMAL;
    // }

    // /**************************
    // *
    // * 以上的8个步骤完成后，读取得到期望的油门值和期望的pitch角
    // *
    // ***************************/
    // _tecs_state.altitude_sp = _hgt_dem_adj;
    // _tecs_state.altitude_filtered = _integ3_state;
    // _tecs_state.altitude_rate_sp = _hgt_rate_dem;
    // _tecs_state.altitude_rate = _integ2_state;

    // _tecs_state.airspeed_sp = _TAS_dem_adj;
    // _tecs_state.airspeed_rate_sp = _TAS_rate_dem;
    // _tecs_state.airspeed_filtered = _integ5_state;
    // _tecs_state.airspeed_rate = _vel_dot;

    // _tecs_state.total_energy_error = _STE_error;
    // _tecs_state.energy_distribution_error = _SEB_error;
    // _tecs_state.total_energy_rate_error = _STEdot_error;
    // _tecs_state.energy_distribution_rate_error = _SEBdot_error;

    // _tecs_state.energy_error_integ = _integ6_state;
    // _tecs_state.energy_distribution_error_integ = _integ7_state;

    // _tecs_state.throttle_integ = _integ6_state;
    // _tecs_state.pitch_integ = _integ7_state;

    // _update_pitch_throttle_last_usec = now;
}

int main(int argc, char **argv)
{
    printf("hello,tecs???\n");
    return (0);
}