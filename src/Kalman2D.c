#include "kalman2d.h"
#include <string.h>


/* -------- GLOBAL FILTER INSTANCE -------- */
static Kalman2D kf;

/* -------- QUEUE -------- */
QueueHandle_t kf_queue;


static void mat_identity(float *A, int n)
{
    memset(A, 0, n*n*sizeof(float));
    for(int i=0;i<n;i++)
        A[i*n+i] = 1.0f;
}


void kf_init(Kalman2D *kf)
{
    memset(kf, 0, sizeof(Kalman2D));

    /* Initial uncertainty */
    for(int i=0;i<KF_STATE_DIM;i++)
        kf->P[i][i] = 10.0f;

    /* Process noise (TUNE LATER) */
    kf->Q[0][0] = 0.01f;
    kf->Q[1][1] = 0.01f;
    kf->Q[2][2] = 0.1f;
    kf->Q[3][3] = 0.1f;
    kf->Q[4][4] = 0.5f;
    kf->Q[5][5] = 0.5f;
}


void kf_predict(Kalman2D *kf, float dt)
{
    float dt2 = dt * dt * 0.5f;

    float *X = kf->X;

    /* ---- State prediction ---- */
    X[0] += X[2]*dt + X[4]*dt2;
    X[1] += X[3]*dt + X[5]*dt2;

    X[2] += X[4]*dt;
    X[3] += X[5]*dt;

    /* ---- Covariance prediction ---- */

    float A[KF_STATE_DIM][KF_STATE_DIM] = {0};

    mat_identity((float*)A, KF_STATE_DIM);

    A[0][2] = dt;
    A[1][3] = dt;
    A[0][4] = dt2;
    A[1][5] = dt2;
    A[2][4] = dt;
    A[3][5] = dt;

    float AP[KF_STATE_DIM][KF_STATE_DIM] = {0};
    float APA[KF_STATE_DIM][KF_STATE_DIM] = {0};

    /* AP = A*P */
    for(int i=0;i<6;i++)
        for(int j=0;j<6;j++)
            for(int k=0;k<6;k++)
                AP[i][j] += A[i][k]*kf->P[k][j];

    /* APA = AP*Aᵀ */
    for(int i=0;i<6;i++)
        for(int j=0;j<6;j++)
            for(int k=0;k<6;k++)
                APA[i][j] += AP[i][k]*A[j][k];

    /* P = APA + Q */
    for(int i=0;i<6;i++)
        for(int j=0;j<6;j++)
            kf->P[i][j] = APA[i][j] + kf->Q[i][j];
}


/**
 * @brief Generic 2-dimensional Kalman measurement update.
 *
 * This function performs a Kalman filter correction step using a
 * 2-element measurement vector. It updates two selected components
 * of the state vector by comparing the predicted state with an
 * external sensor measurement.
 *
 * The function is designed as a lightweight alternative to the full
 * matrix formulation:
 *
 *      X = X + K(z − HX)
 *      P = (I − KH)P
 *
 * where the measurement matrix H is implicitly defined through the
 * provided state indices (idx0, idx1). This avoids constructing
 * matrices dynamically and reduces computational cost for embedded
 * systems such as the ESP32.
 *
 * Each measurement is assumed to directly observe one state variable.
 *
 * --------------------------------------------------------------------
 * State vector layout (Kalman2D):
 *
 *   Index | State variable | Units
 *   ------|----------------|-------
 *     0   | x              | meters
 *     1   | y              | meters
 *     2   | vx             | m/s
 *     3   | vy             | m/s
 *     4   | ax             | m/s²
 *     5   | ay             | m/s²
 *
 * --------------------------------------------------------------------
 *
 * @param[in,out] kf
 * Pointer to the Kalman filter instance. The state vector (X) and
 * covariance matrix (P) are updated in place.
 *
 * @param[in] z0
 * First measurement value. Represents the observed value of the
 * state variable specified by @p idx0.
 *
 * @param[in] z1
 * Second measurement value. Represents the observed value of the
 * state variable specified by @p idx1.
 *
 * @param[in] idx0
 * Index of the first state variable being measured.
 * This selects which element of the state vector is corrected.
 *
 * Example values:
 *   - 0 → position x
 *   - 2 → velocity vx
 *   - 4 → acceleration ax
 *
 * @param[in] idx1
 * Index of the second state variable being measured.
 *
 * @param[in] R
 * Measurement noise variance (sensor uncertainty).
 *
 * This parameter controls how strongly the measurement influences
 * the state estimate:
 *
 *   Smaller R  → trust sensor more (strong correction)
 *   Larger R   → trust prediction more (weak correction)
 *
 * Units must match the squared units of the measurement:
 *
 *   Measurement      Units        Example R
 *   ------------------------------------------------
 *   Position (GPS)   m²           ~4.0  (≈2 m error)
 *   Velocity         (m/s)²       ~0.2
 *   Acceleration     (m/s²)²      ~0.5
 *
 * Incorrect tuning of R may cause:
 *   - noisy estimates (R too small)
 *   - slow response / drift (R too large)
 *
 * @note
 * This simplified update assumes measurements independently observe
 * individual state variables (diagonal measurement covariance).
 * It is optimized for embedded real-time applications and avoids
 * dynamic memory allocation.
 *
 * @warning
 * Measurements must be expressed in the same reference frame and
 * units as the state vector (e.g., acceleration in world frame,
 * position in meters).
 */
static void kf_update_generic(
    Kalman2D *kf,
    float z0,
    float z1,
    int idx0,
    int idx1,
    float R)
{
    float y0 = z0 - kf->X[idx0];
    float y1 = z1 - kf->X[idx1];

    float S0 = kf->P[idx0][idx0] + R;
    float S1 = kf->P[idx1][idx1] + R;

    float K[KF_STATE_DIM][2];

    for(int i=0;i<6;i++)
    {
        K[i][0] = kf->P[i][idx0] / S0;
        K[i][1] = kf->P[i][idx1] / S1;
    }

    /* State correction */
    for(int i=0;i<6;i++)
        kf->X[i] += K[i][0]*y0 + K[i][1]*y1;

    /* Covariance update */
    for(int i=0;i<6;i++)
    {
        kf->P[i][idx0] *= (1.0f - K[idx0][0]);
        kf->P[i][idx1] *= (1.0f - K[idx1][1]);
    }
}


void kf_update_gps(Kalman2D *kf, float x, float y)
{
    kf_update_generic(kf, x, y, 0, 1, 4.0f); // GPS variance ~2m
}


void kf_update_accel(Kalman2D *kf, float ax, float ay)
{
    kf_update_generic(kf, ax, ay, 4, 5, 0.5f);
}


void kf_update_velocity(Kalman2D *kf, float vx, float vy)
{
    kf_update_generic(kf, vx, vy, 2, 3, 0.2f);
}


void kalman_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz

    while (1)
    {
        /* 1. Predict at fixed rate */
        kf_predict(&kf, 0.01f);

        /* 2. Process ALL pending measurements */
        kf_msg_t msg;

        while (xQueueReceive(kf_queue, &msg, 0) == pdTRUE)
        {
            switch(msg.type)
            {
                case KF_MEAS_ACCEL:
                    kf_update_accel(&kf, msg.a, msg.b);
                    break;

                case KF_MEAS_GPS:
                    kf_update_gps(&kf, msg.a, msg.b);
                    break;

                case KF_MEAS_VEL:
                    kf_update_velocity(&kf, msg.a, msg.b);
                    break;
            }
        }

        vTaskDelayUntil(&last, period);
    }
}