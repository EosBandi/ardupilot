#include "AP_Airspeed_SITL.h"

#if AP_AIRSPEED_SITL_ENABLED

#include <AP_Baro/AP_Baro.h>
#include <SITL/SITL.h>

// return the current differential_pressure in Pascal
bool AP_Airspeed_SITL::get_differential_pressure(float &pressure)
{
    const uint8_t _instance = get_instance();

    if (_instance >= AIRSPEED_MAX_SENSORS) {
        return false;
    }

    //We have the simulated airspeed, and we calculate the pressure from that
    _update_airspeed((float)AP::sitl()->state.airspeed);

    pressure = AP::sitl()->state.airspeed_raw_pressure[_instance];
    return true;
}

// get last temperature
bool AP_Airspeed_SITL::get_temperature(float &temperature)
{
    const uint8_t _instance = get_instance();

    if (_instance >= AIRSPEED_MAX_SENSORS) {
        return false;
    }

    const auto *sitl = AP::sitl();

    // this was mostly swiped from SIM_Airspeed_DLVR:
    const float sim_alt = sitl->state.altitude;

    float sigma, delta, theta;
    AP_Baro::SimpleAtmosphere(sim_alt * 0.001f, sigma, delta, theta);

    // To Do: Add a sensor board temperature offset parameter
    temperature = (KELVIN_TO_C(SSL_AIR_TEMPERATURE * theta)) + 25.0;

    return true;
}


float AP_Airspeed_SITL::get_EAS2TAS(float altitude)
{
    float pressure = AP::baro().get_pressure();
    if (is_zero(pressure)) {
        return 1.0f;
    }

    float sigma, delta, theta;
    AP_Baro::SimpleAtmosphere(altitude * 0.001, sigma, delta, theta);

    float tempK = C_TO_KELVIN(25) - ISA_LAPSE_RATE * altitude;
    const float eas2tas_squared = SSL_AIR_DENSITY / (pressure / (ISA_GAS_CONSTANT * tempK));
    if (!is_positive(eas2tas_squared)) {
        return 1.0;
    }
    return sqrtf(eas2tas_squared);
}

void AP_Airspeed_SITL::_update_airspeed(float true_airspeed)
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        const auto &arspd = AP::sitl()->airspeed[i];
        float airspeed = true_airspeed / get_EAS2TAS(AP::sitl()->state.altitude);
        const float diff_pressure = sq(airspeed) / arspd.ratio;
        //float airspeed_raw;
    
        // apply noise to the differential pressure. This emulates the way
        // airspeed noise reduces with speed
        airspeed = sqrtf(fabsf(arspd.ratio*(diff_pressure + arspd.noise * rand_float())));

        // check sensor failure
        if (is_positive(arspd.fail)) {
            airspeed = arspd.fail;
        }

        if (!is_zero(arspd.fail_pressure)) {
            // compute a realistic pressure report given some level of trapper air pressure in the tube and our current altitude
            // algorithm taken from https://en.wikipedia.org/wiki/Calibrated_airspeed#Calculation_from_impact_pressure
            float tube_pressure = fabsf(arspd.fail_pressure - AP::baro().get_pressure() + arspd.fail_pitot_pressure);
            airspeed = 340.29409348 * sqrt(5 * (pow((tube_pressure / SSL_AIR_PRESSURE + 1), 2.0/7.0) - 1.0));
        }
        float airspeed_pressure = (airspeed * airspeed) / arspd.ratio;

        // flip sign here for simulating reversed pitot/static connections
        if (arspd.signflip) {
            airspeed_pressure *= -1;
        }

        // apply airspeed sensor offset in m/s
        //airspeed_raw = airspeed_pressure + arspd.offset;

        AP::sitl()->state.airspeed_raw_pressure[i] = airspeed_pressure;
        
    }
}


#endif // AP_AIRSPEED_SITL_ENABLED
