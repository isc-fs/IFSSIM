#include "FSDSPacejkaTireModel.h"
#include "ChaosVehicleWheel.h"

// ── Magic Formula core ─────────────────────────────────────────────────────
// F(x) = D * sin( C * atan( B*x - E*(B*x - atan(B*x)) ) )
static float MagicFormula(float B, float C, float E, float D, float x)
{
	const float Bx    = B * x;
	const float AtBx  = FMath::Atan(Bx);
	const float Inner = Bx - E * (Bx - AtBx);
	return D * FMath::Sin(C * FMath::Atan(Inner));
}

namespace FSDSPacejka
{
	float EvalLateral(const FFSDSPacejkaCoeffs& C, float SlipDeg, float PeakMu)
	{
		// Chaos LateralSlipGraph X axis is in degrees; convert to radians for MF
		const float x = FMath::DegreesToRadians(FMath::Abs(SlipDeg));
		return MagicFormula(C.LatB, C.LatC, C.LatE, PeakMu, x);
	}

	float EvalLongitudinal(const FFSDSPacejkaCoeffs& C, float SlipRatio, float PeakMu)
	{
		return MagicFormula(C.LonB, C.LonC, C.LonE, PeakMu, FMath::Abs(SlipRatio));
	}

	void BakeToWheel(UChaosVehicleWheel* Wheel, const FFSDSPacejkaCoeffs& C, float PeakMu)
	{
		if (!Wheel) return;

		// ── Lateral slip graph ─────────────────────────────────────────────
		// X = slip angle in degrees (Chaos convention), Y = friction coefficient.
		// Sample densely near the peak (~5°) and sparsely in saturation (>15°).
		{
			static const float Degrees[] = {
				0.f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f,
				9.f, 10.f, 12.f, 14.f, 16.f, 18.f, 20.f, 25.f, 30.f
			};

			FRichCurve* Curve = Wheel->LateralSlipGraph.GetRichCurve();
			Curve->Reset();
			for (float Deg : Degrees)
			{
				Curve->AddKey(Deg, EvalLateral(C, Deg, PeakMu));
			}
			// Linear interpolation is accurate enough given the dense sampling
			Curve->SetDefaultValue(EvalLateral(C, 30.f, PeakMu));
		}

		// ── Longitudinal slip graph ────────────────────────────────────────
		// X = slip ratio κ in [0, 1], Y = friction coefficient.
		// Peak is around κ ≈ 0.10 for this tire; sample densely in [0, 0.2].
		{
			static const float Kappas[] = {
				0.00f, 0.02f, 0.04f, 0.06f, 0.08f, 0.10f,
				0.12f, 0.15f, 0.20f, 0.30f, 0.50f, 0.75f, 1.00f
			};

			FRichCurve* Curve = Wheel->LongitudinalSlipGraph.GetRichCurve();
			Curve->Reset();
			for (float Kappa : Kappas)
			{
				Curve->AddKey(Kappa, EvalLongitudinal(C, Kappa, PeakMu));
			}
			Curve->SetDefaultValue(EvalLongitudinal(C, 1.0f, PeakMu));
		}

		// Keep FrictionForceMultiplier = PeakMu as a safe fallback in case
		// Chaos ignores the slip graphs (e.g. if the build strips them).
		Wheel->FrictionForceMultiplier = PeakMu;

		UE_LOG(LogTemp, Log,
			TEXT("FSDS Pacejka: baked wheel %s — lat B=%.1f C=%.2f E=%.2f | lon B=%.1f C=%.2f E=%.2f | peak μ=%.2f"),
			*Wheel->GetName(),
			C.LatB, C.LatC, C.LatE,
			C.LonB, C.LonC, C.LonE,
			PeakMu);
	}
}
