#ifndef EKFPROTOTIPO2_H
#define EKFPROTOTIPO2_H

struct RawSensorData {
    struct {
        float longitud;
        float latitud;
        float altitud;
    } gps;

    struct {
        float altitud_m;
    } barometro;

    struct {
        float accel_g[3];
        float a_total;
    } acelerometro;

    struct {
        float pitch;
        float roll;
        float yaw;
    } inc;
};

typedef struct {
    float altitude_m;
    float velocity_mps;
} AltitudeEstimate_t;

#ifdef __cplusplus
extern "C" {
#endif

int ekfprototipo2_init(const struct RawSensorData *sensors);
/* new_barometer_sample must be nonzero only for a newly acquired barometer reading. */
int ekfprototipo2_update(const struct RawSensorData *sensors,
                         float dt,
                         int new_barometer_sample,
                         AltitudeEstimate_t *estimate);

void altitude_ekf_backend_init(float initial_altitude);
void altitude_ekf_backend_predict(float vertical_acceleration, float dt);
void altitude_ekf_backend_update(float barometer_altitude);
float altitude_ekf_backend_get_altitude(void);
float altitude_ekf_backend_get_velocity(void);

#ifdef __cplusplus
}
#endif

#endif
