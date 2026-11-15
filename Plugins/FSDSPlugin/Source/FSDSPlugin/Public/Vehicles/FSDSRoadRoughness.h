#pragma once

#include "CoreMinimal.h"
#include "Math/RandomStream.h"
#include "FSDSRandom.h"

/**
 * Random road profile along the distance driven, for the two wheel tracks.
 *
 * The level floor every generated track sits on is perfectly flat, so below
 * 15 Hz the sim car only moved from steering and the throttle: on the
 * 2026-09-20 manual bag the car's vertical acceleration at 0.5-10 Hz is 3-4x
 * the sim's. A real road is rough at every wavelength.
 *
 * The profile follows the ISO 8608 shape: displacement PSD
 *   Gd(n) = Gd0 * (n / n0)^-2,  n0 = 0.1 cycle/m,
 * rolled off below 1 / CutoffWavelength so it does not wander. That is a
 * first-order (Ornstein-Uhlenbeck) process in distance, generated on a 2 cm
 * grid: z[k+1] = z[k] e^(-a dx) + s sqrt(1 - e^(-2 a dx)) N(0,1), with
 * a = 2 pi / CutoffWavelength and s^2 = pi^2 Gd0 n0^2 / a, which gives exactly
 * Gd0 (n0/n)^2 for wavelengths well under the cutoff.
 *
 * A tyre does not follow features shorter than its footprint, so Height()
 * averages the profile over Footprint metres. Left and right tracks are
 * independent; the rear wheels read the same track Wheelbase metres back.
 *
 * Kept as a window: samples more than KeepBehind metres behind the newest
 * query are dropped, so a long run does not grow it.
 */
class FFSDSRoadRoughness
{
public:
	static constexpr double Dx = 0.02;         // grid spacing, m
	static constexpr double N0 = 0.1;          // ISO 8608 reference spatial frequency, cycle/m
	static constexpr double KeepBehind = 10.0; // m of profile kept behind the furthest query

	void Configure(double InGd0, double InCutoffWavelength, double InFootprint)
	{
		Gd0 = FMath::Max(0.0, InGd0);
		const double Cutoff = FMath::Max(1.0, InCutoffWavelength);
		const double A = 2.0 * PI / Cutoff;
		Decay = FMath::Exp(-A * Dx);
		Sigma = FMath::Sqrt(PI * PI * Gd0 * N0 * N0 / A);
		Innov = Sigma * FMath::Sqrt(1.0 - Decay * Decay);
		HalfWindow = FMath::Max(0, FMath::RoundToInt(0.5 * FMath::Max(0.0, InFootprint) / Dx));
	}

	bool IsActive() const { return Gd0 > 0.0; }
	double GetSigma() const { return Sigma; }

	/** Start a new profile at distance 0 from this stream. */
	void Reset(const FRandomStream& InStream)
	{
		Stream = InStream;
		for (int32 T = 0; T < 2; ++T)
		{
			Z[T].Reset();
			// Start in the stationary distribution, so the first metres are as rough as the rest.
			Z[T].Add(Sigma * FSDSRandom::StandardNormal(Stream));
		}
		Base = -2.0;   // grid index 0 sits two metres behind the start, for the rear wheels
		ExtendTo(0.0);
	}

	/**
	 * Height of a track (0 left, 1 right) in metres at distance S, averaged
	 * over the footprint. S may be negative down to the profile's start.
	 */
	double Height(int32 Track, double S)
	{
		ExtendTo(S + (HalfWindow + 2) * Dx);
		const TArray<double>& P = Z[Track];
		const double X = (S - Base) / Dx;
		const int32 I = FMath::Clamp(FMath::FloorToInt(X), HalfWindow, P.Num() - HalfWindow - 2);
		const double F = FMath::Clamp(X - I, 0.0, 1.0);
		return (1.0 - F) * Smoothed(P, I) + F * Smoothed(P, I + 1);
	}

	/** Drops profile older than KeepBehind metres behind S. */
	void TrimBefore(double S)
	{
		const int32 Drop = FMath::FloorToInt((S - KeepBehind - Base) / Dx);
		if (Drop > 4096 && Drop < Z[0].Num())
		{
			for (int32 T = 0; T < 2; ++T) { Z[T].RemoveAt(0, Drop, EAllowShrinking::No); }
			Base += Drop * Dx;
		}
	}

private:
	double Smoothed(const TArray<double>& P, int32 I) const
	{
		double Sum = 0.0;
		for (int32 K = I - HalfWindow; K <= I + HalfWindow; ++K) { Sum += P[K]; }
		return Sum / (2 * HalfWindow + 1);
	}

	void ExtendTo(double S)
	{
		const int32 Need = FMath::CeilToInt((S - Base) / Dx) + HalfWindow + 2;
		for (int32 T = 0; T < 2; ++T)
		{
			TArray<double>& P = Z[T];
			while (P.Num() < Need)
			{
				P.Add(P.Last() * Decay + Innov * FSDSRandom::StandardNormal(Stream));
			}
		}
	}

	double Gd0 = 0.0, Sigma = 0.0, Decay = 1.0, Innov = 0.0;
	int32 HalfWindow = 0;
	double Base = 0.0;
	TArray<double> Z[2];
	FRandomStream Stream;
};

/**
 * One corner's road input, as a private quarter car: the road moves the wheel
 * (unsprung mass Mu) through the tyre's vertical stiffness Kt, and the wheel
 * moves a sprung mass Ms through the strut (K, C).
 *
 * Chaos's wheels sit rigidly on the ground, so its own strut only reacts to
 * the body moving. Adding K*Zu + C*Vu at the corner, with Zu the wheel
 * motion from this model, makes the Chaos body move as the sprung mass of a
 * car with compliant tyres would. Without the tyre the strut's damper passes
 * the short, fast part of the road straight to the body: with the Chaos
 * damping ratio of 1.5, 10-15 Hz came out 2x the car and 3-6 Hz half of it.
 *
 * Integrated in 1 ms RK4 sub-steps (the wheel's fastest pole is ~C/Mu =
 * 350 rad/s), with the road taken as linear across the frame.
 */
struct FFSDSRoadCorner
{
	double Zs = 0.0, Vs = 0.0, Zu = 0.0, Vu = 0.0;

	void Reset(double Zr) { Zs = Zr; Zu = Zr; Vs = 0.0; Vu = 0.0; }

	void Advance(double Zr0, double Zr1, double Dt, double Ms, double Mu, double K, double C, double Kt)
	{
		if (Dt <= 0.0 || Ms <= 0.0 || Mu <= 0.0) return;
		const int32 N = FMath::Max(1, FMath::CeilToInt(Dt / 0.001));
		const double H = Dt / N;
		auto Acc = [&](double zs, double vs, double zu, double vu, double zr, double& As, double& Au)
		{
			const double Fs = K * (zu - zs) + C * (vu - vs);
			As = Fs / Ms;
			Au = (Kt * (zr - zu) - Fs) / Mu;
		};
		for (int32 J = 0; J < N; ++J)
		{
			const double R0 = Zr0 + (Zr1 - Zr0) * J / N;
			const double RM = Zr0 + (Zr1 - Zr0) * (J + 0.5) / N;
			const double R1 = Zr0 + (Zr1 - Zr0) * (J + 1.0) / N;
			double a1s, a1u, a2s, a2u, a3s, a3u, a4s, a4u;
			Acc(Zs, Vs, Zu, Vu, R0, a1s, a1u);
			Acc(Zs + 0.5 * H * Vs, Vs + 0.5 * H * a1s, Zu + 0.5 * H * Vu, Vu + 0.5 * H * a1u, RM, a2s, a2u);
			Acc(Zs + 0.5 * H * (Vs + 0.5 * H * a1s), Vs + 0.5 * H * a2s,
			    Zu + 0.5 * H * (Vu + 0.5 * H * a1u), Vu + 0.5 * H * a2u, RM, a3s, a3u);
			Acc(Zs + H * (Vs + 0.5 * H * a2s), Vs + H * a3s,
			    Zu + H * (Vu + 0.5 * H * a2u), Vu + H * a3u, R1, a4s, a4u);
			const double V2s = Vs + 0.5 * H * a1s, V3s = Vs + 0.5 * H * a2s, V4s = Vs + H * a3s;
			const double V2u = Vu + 0.5 * H * a1u, V3u = Vu + 0.5 * H * a2u, V4u = Vu + H * a3u;
			Zs += H / 6.0 * (Vs + 2.0 * V2s + 2.0 * V3s + V4s);
			Zu += H / 6.0 * (Vu + 2.0 * V2u + 2.0 * V3u + V4u);
			Vs += H / 6.0 * (a1s + 2.0 * a2s + 2.0 * a3s + a4s);
			Vu += H / 6.0 * (a1u + 2.0 * a2u + 2.0 * a3u + a4u);
		}
	}

	/** Force the strut passes to the body from the road, N. */
	double Force(double K, double C) const { return K * Zu + C * Vu; }
};
