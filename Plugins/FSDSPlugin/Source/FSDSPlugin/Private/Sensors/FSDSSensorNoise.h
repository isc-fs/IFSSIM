#pragma once

#include "Math/UnrealMathUtility.h"

/**
 * Sensor-noise helpers shared by the four sensor TickComponents.
 *
 * Why this exists: every sensor was originally written with
 *   noise = FMath::FRandRange(-1, 1) * Std
 * which is *uniform*, not Gaussian. The real stddev is
 * Std·1/√3 ≈ 0.58·Std — i.e. the sensor was emitting noise at
 * 58% of the declared stddev, while the bridge's covariance matrix
 * was computed as Std² (the declared value). Net: any consumer
 * Kalman filter under-trusted the residuals it actually saw.
 *
 * Box–Muller is one log + one cos per sample. Each sensor calls it
 * at most a few times per tick (2 axes for GSS, 3 for GPS, 1 per
 * LiDAR ray-hit), so the cost is negligible. Cap u1 away from zero
 * so log() is finite even on the unlucky sample.
 *
 * Inline / static so each translation unit gets its own copy and
 * we don't need a shared library / ODR worries.
 */
namespace FSDSNoise
{
	/** One sample from N(0, 1). */
	static inline float RandStandardNormal()
	{
		const float u1 = FMath::Max(FMath::FRand(), 1e-7f);
		const float u2 = FMath::FRand();
		return FMath::Sqrt(-2.f * FMath::Loge(u1)) * FMath::Cos(2.f * PI * u2);
	}

	/**
	 * Step an Ornstein–Uhlenbeck bias process one tick. Closed-form
	 * discrete update for db/dt = -b/τ + diffusion·dW gives
	 *   b[k+1] = b[k]·exp(-Δt/τ) + SteadyStd·sqrt(1 - exp(-2Δt/τ))·N(0,1)
	 * where SteadyStd is the long-run stddev (the bound on bias).
	 *
	 * Used by IMU; GPS/GSS/LiDAR don't model bias (correct for those
	 * sensors — GPS bias is dominated by sat geometry, GSS is optical,
	 * LiDAR is shot-noise dominated).
	 */
	static inline float StepOrnsteinUhlenbeck(float Bias, float Dt, float Tau, float SteadyStd)
	{
		if (Tau <= 0.f || SteadyStd <= 0.f) return Bias;
		const float Decay = FMath::Exp(-Dt / Tau);
		const float Sigma = SteadyStd * FMath::Sqrt(1.f - Decay * Decay);
		return Bias * Decay + Sigma * RandStandardNormal();
	}
}
