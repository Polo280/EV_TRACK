#ifndef KALMAN2D_H
#define KALMAN2D_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define KF_STATE_DIM 6      // State vector dimension
#define KF_MEAS_DIM  2      

typedef struct
{
    float X[KF_STATE_DIM];                 // state vector
    float P[KF_STATE_DIM][KF_STATE_DIM];   // covariance
    float Q[KF_STATE_DIM][KF_STATE_DIM];   // process noise
} Kalman2D;

/* ---- Measurement types ---- */
typedef enum
{
    KF_MEAS_ACCEL = 0,
    KF_MEAS_GPS,
    KF_MEAS_VEL

} kf_meas_type_t;

/* ---- Message sent to Kalman task ---- */
typedef struct
{
    kf_meas_type_t type;
    float a;
    float b;
} kf_msg_t;

/* Queue handle (extern so all tasks can use it) */
extern QueueHandle_t kf_queue;

void kf_init(Kalman2D *kf);

void kf_predict(Kalman2D *kf, float dt);

/* Measurements */
void kf_update_gps(Kalman2D *kf, float x, float y);
void kf_update_accel(Kalman2D *kf, float ax, float ay);
void kf_update_velocity(Kalman2D *kf, float vx, float vy);
void kalman_task(void *arg);

#endif /* KALMAN2D_H */