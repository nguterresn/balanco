#include "kalman.h"
#include <stdint.h>

// Q_angle  : process noise for angle   (tune: ~0.001)
// Q_bias   : process noise for bias    (tune: ~0.003)
// R_measure: measurement noise         (tune: ~0.03)
#define KALMAN_Q_ANGLE   0.001f
#define KALMAN_Q_BIAS    0.003f
#define KALMAN_R_MEASURE 0.03f

void kalman_init(struct kalman1d* k)
{
	k->angle   = 0.0f;
	k->bias    = 0.0f;
	k->P[0][0] = 0.0f;
	k->P[0][1] = 0.0f;
	k->P[1][0] = 0.0f;
	k->P[1][1] = 0.0f;
}

inline float kalman_update(struct kalman1d* k, float pitch_accel,
                           float gyr_y_deg, float dt)
{
	// ── Predict ──
	k->angle += dt * (gyr_y_deg - k->bias);

	k->P[0][0] += dt *
	              (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + KALMAN_Q_ANGLE);
	k->P[0][1] -= dt * k->P[1][1];
	k->P[1][0] -= dt * k->P[1][1];
	k->P[1][1] += KALMAN_Q_BIAS * dt;

	// ── Update ──
	float S  = k->P[0][0] + KALMAN_R_MEASURE; // innovation covariance
	float K0 = k->P[0][0] / S;                // Kalman gain for angle
	float K1 = k->P[1][0] / S;                // Kalman gain for bias

	float y = pitch_accel - k->angle; // innovation (residual)
	k->angle += K0 * y;
	k->bias += K1 * y;

	float P00_temp = k->P[0][0];
	float P01_temp = k->P[0][1];
	k->P[0][0] -= K0 * P00_temp;
	k->P[0][1] -= K0 * P01_temp;
	k->P[1][0] -= K1 * P00_temp;
	k->P[1][1] -= K1 * P01_temp;

	return k->angle;
}
