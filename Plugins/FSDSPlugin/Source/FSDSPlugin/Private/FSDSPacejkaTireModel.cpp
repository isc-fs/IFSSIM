#include "FSDSPacejkaTireModel.h"
#include "ChaosVehicleWheel.h"

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
		const float x = FMath::DegreesToRadians(FMath::Abs(SlipDeg));
		return MagicFormula(C.LatB, C.LatC, C.LatE, PeakMu, x);
	}

	void BakeToWheel(UChaosVehicleWheel* Wheel, const FFSDSPacejkaCoeffs& C, float PeakMu)
	{
		if (!Wheel) return;

		// X = slip angle in degrees (Chaos convention), Y = friction coefficient.
		// Dense sampling near peak (~5°), sparse in saturation (>15°).
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
		Curve->SetDefaultValue(EvalLateral(C, 30.f, PeakMu));

		// Longitudinal grip ceiling — flat, Chaos binary has no longitudinal curve hook
		Wheel->FrictionForceMultiplier = PeakMu;

		UE_LOG(LogTemp, Log,
			TEXT("FSDS Pacejka: baked wheel %s — lat B=%.1f C=%.2f E=%.2f | peak μ=%.2f"),
			*Wheel->GetName(),
			C.LatB, C.LatC, C.LatE, PeakMu);
	}
}
