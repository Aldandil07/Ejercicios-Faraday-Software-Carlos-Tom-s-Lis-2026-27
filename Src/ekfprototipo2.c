#include "ekfprototipo2.h"
#include <math.h>

#define EKF_GRAVITY_MPS2 9.80665f
#define EKF_DEG_TO_RAD (3.14159265358979323846f / 180.0f)

static int filter_initialized;

static float calculate_vertical_acceleration(const struct RawSensorData *sensors)
{
    /* Assumes +Z is upward at rest and pitch/roll describe the vertical axis in sensor coordinates. */
    const float pitch = sensors->inc.pitch * EKF_DEG_TO_RAD;
    const float roll = sensors->inc.roll * EKF_DEG_TO_RAD;
    const float sin_pitch = sinf(pitch);
    const float cos_pitch = cosf(pitch);
    const float sin_roll = sinf(roll);
    const float cos_roll = cosf(roll);

    const float vertical_specific_force_g =
        sensors->acelerometro.accel_g[0] * sin_pitch * cos_roll
        + sensors->acelerometro.accel_g[1] * sin_roll
        + sensors->acelerometro.accel_g[2] * cos_pitch * cos_roll;

    return (vertical_specific_force_g - 1.0f) * EKF_GRAVITY_MPS2;
}

int ekfprototipo2_init(const struct RawSensorData *sensors)
{
    if ((sensors == 0) || !isfinite(sensors->barometro.altitud_m)) {
        return -1;
    }

    altitude_ekf_backend_init(sensors->barometro.altitud_m);
    filter_initialized = 1;
    return 0;
}

int ekfprototipo2_update(const struct RawSensorData *sensors,
                         float dt,
                         int new_barometer_sample,
                         AltitudeEstimate_t *estimate)
{
    if ((sensors == 0) || (estimate == 0) || !filter_initialized
        || !isfinite(dt) || (dt <= 0.0f)
        || !isfinite(sensors->barometro.altitud_m)
        || !isfinite(sensors->acelerometro.accel_g[0])
        || !isfinite(sensors->acelerometro.accel_g[1])
        || !isfinite(sensors->acelerometro.accel_g[2])
        || !isfinite(sensors->inc.pitch)
        || !isfinite(sensors->inc.roll)) {
        return -1;
    }

    altitude_ekf_backend_predict(calculate_vertical_acceleration(sensors), dt);
    if (new_barometer_sample) {
        altitude_ekf_backend_update(sensors->barometro.altitud_m);
    }
    estimate->altitude_m = altitude_ekf_backend_get_altitude();
    estimate->velocity_mps = altitude_ekf_backend_get_velocity();
    return 0;
}