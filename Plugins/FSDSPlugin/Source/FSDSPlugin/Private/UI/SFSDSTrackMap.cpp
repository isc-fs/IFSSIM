#include "UI/SFSDSTrackMap.h"

#include "Brushes/SlateRoundedBoxBrush.h"
#include "Fonts/FontMeasure.h"
#include "Framework/Application/SlateApplication.h"
#include "Misc/FileHelper.h"
#include "Rendering/DrawElements.h"
#include "Styling/CoreStyle.h"

#define LOCTEXT_NAMESPACE "FSDSTrackMap"

// Always qualified (namespace Map = ...): other UI files in the same unity
// blob bring their own Muted etc. into scope.
namespace FSDSTrackMapStyle
{
	// Mission Control's track preview: dark plot, faint amber grid.
	const FLinearColor Backdrop = FLinearColor::FromSRGBColor(FColor(17, 17, 17));
	const FLinearColor Grid = FLinearColor::FromSRGBColor(FColor(255, 184, 28)).CopyWithNewOpacity(0.15f);
	const FLinearColor Axis = FLinearColor::FromSRGBColor(FColor(51, 51, 51));
	const FLinearColor Label = FLinearColor::FromSRGBColor(FColor(136, 136, 136));
	const FLinearColor Blue = FLinearColor::FromSRGBColor(FColor(68, 136, 255));
	const FLinearColor Yellow = FLinearColor::FromSRGBColor(FColor(255, 184, 28));
	const FLinearColor BigOrange = FLinearColor::FromSRGBColor(FColor(255, 102, 0));
	const FLinearColor SmallOrange = FLinearColor::FromSRGBColor(FColor(255, 136, 0));
	const FLinearColor Other = FLinearColor::FromSRGBColor(FColor(160, 160, 170));

	// Room for the axis labels: y on the left, x below.
	constexpr float PadLeft = 46.f;
	constexpr float PadBottom = 34.f;
	constexpr float PadTop = 30.f;   // the legend
	constexpr float PadRight = 14.f;

	/** A round step (1, 2, 5, 10, 20, 25, 50...) giving about Target lines over Span metres. */
	double GridStep(double Span, int32 Target)
	{
		const double Raw = FMath::Max(Span / FMath::Max(Target, 1), 0.5);
		const double Pow10 = FMath::Pow(10.0, FMath::FloorToDouble(FMath::LogX(10.0, Raw)));
		for (double M : { 1.0, 2.0, 2.5, 5.0, 10.0 })
		{
			if (M * Pow10 >= Raw) return M * Pow10;
		}
		return 10.0 * Pow10;
	}

	/** A filled circle D px across. A rounded box only rounds when its radius
	 *  is at most half its size, so one brush per (whole-pixel) size. */
	const FSlateBrush* DotBrush(float D)
	{
		static TMap<int32, TUniquePtr<FSlateRoundedBoxBrush>> Brushes;
		const int32 Px = FMath::Clamp(FMath::RoundToInt(D), 2, 32);
		TUniquePtr<FSlateRoundedBoxBrush>& B = Brushes.FindOrAdd(Px);
		if (!B.IsValid())
		{
			B = MakeUnique<FSlateRoundedBoxBrush>(FLinearColor::White, Px * 0.5f);
		}
		return B.Get();
	}
}

void SFSDSTrackMap::Construct(const FArguments&)
{
}

void SFSDSTrackMap::SetCones(TArray<FCone> InCones)
{
	Cones = MoveTemp(InCones);
	Bounds = FBox2D(ForceInit);
	for (const FCone& C : Cones)
	{
		Bounds += C.Pos;
	}
	Invalidate(EInvalidateWidgetReason::Paint);
}

bool SFSDSTrackMap::LoadCsv(const FString& Path, TArray<FCone>& Out)
{
	Out.Reset();
	TArray<FString> Lines;
	if (!FFileHelper::LoadFileToStringArray(Lines, *Path))
	{
		return false;
	}
	for (const FString& Line : Lines)
	{
		TArray<FString> Parts;
		Line.ParseIntoArray(Parts, TEXT(","));
		if (Parts.Num() < 3) continue;

		const FString Type = Parts[0].TrimStartAndEnd();
		FCone C;
		C.Pos = FVector2D(FCString::Atod(*Parts[1]), FCString::Atod(*Parts[2]));
		if      (Type == TEXT("blue"))                                   C.Kind = EConeKind::Blue;
		else if (Type == TEXT("yellow"))                                 C.Kind = EConeKind::Yellow;
		else if (Type == TEXT("big_orange"))                             C.Kind = EConeKind::BigOrange;
		else if (Type == TEXT("small_orange") || Type == TEXT("orange")) C.Kind = EConeKind::SmallOrange;
		else if (!Parts[1].IsNumeric())                                  continue;   // a header line
		Out.Add(C);
	}
	return Out.Num() > 0;
}

int32 SFSDSTrackMap::OnPaint(const FPaintArgs&, const FGeometry& Geo, const FSlateRect&,
	FSlateWindowElementList& Out, int32 LayerId, const FWidgetStyle&, bool) const
{
	namespace Map = FSDSTrackMapStyle;
	const FSlateBrush* White = FCoreStyle::Get().GetBrush("WhiteBrush");
	const TSharedRef<FSlateFontMeasure> Measure = FSlateApplication::Get().GetRenderer()->GetFontMeasureService();
	const FSlateFontInfo SmallFont = FCoreStyle::GetDefaultFontStyle("Regular", 9);
	const FVector2f Size = FVector2f(Geo.GetLocalSize());

	auto Text = [&](int32 Layer, const FString& S, FVector2f At, const FLinearColor& Color, const FSlateFontInfo& Font)
	{
		const FText T = FText::FromString(S);
		const FVector2f TS = FVector2f(Measure->Measure(T, Font));
		FSlateDrawElement::MakeText(Out, Layer, Geo.ToPaintGeometry(TS, FSlateLayoutTransform(At)), T, Font,
			ESlateDrawEffect::None, Color);
		return TS;
	};
	auto MeasureText = [&](const FString& S, const FSlateFontInfo& Font)
	{
		return FVector2f(Measure->Measure(FText::FromString(S), Font));
	};

	FSlateDrawElement::MakeBox(Out, LayerId, Geo.ToPaintGeometry(), White, ESlateDrawEffect::None, Map::Backdrop);

	if (Cones.Num() == 0 || !Bounds.bIsValid)
	{
		const FString S = LOCTEXT("NoTrack", "No track selected").ToString();
		const FSlateFontInfo Font = FCoreStyle::GetDefaultFontStyle("Regular", 11);
		Text(LayerId + 1, S, (Size - MeasureText(S, Font)) * 0.5f, Map::Label, Font);
		return LayerId + 1;
	}

	// The plot area, and the cones fitted into it with metres square. Screen
	// y points down, the track's y points up.
	const FVector2f PlotMin(Map::PadLeft, Map::PadTop);
	const FVector2f PlotMax(Size.X - Map::PadRight, Size.Y - Map::PadBottom);
	const FVector2f PlotSize = PlotMax - PlotMin;
	if (PlotSize.X < 40.f || PlotSize.Y < 40.f) return LayerId;

	// A little air around the cones so none sits on the frame.
	const FVector2D Extent = Bounds.GetSize();
	const double Air = FMath::Max(Extent.GetMax() * 0.04, 1.0);
	const double SpanX = Extent.X + 2.0 * Air;
	const double SpanY = Extent.Y + 2.0 * Air;
	const float Scale = (float)FMath::Max(FMath::Min(PlotSize.X / SpanX, PlotSize.Y / SpanY), 0.01);
	const FVector2D Centre = Bounds.GetCenter();
	const FVector2f PlotCentre = PlotMin + PlotSize * 0.5f;
	auto ToScreen = [&](double X, double Y)
	{
		return FVector2f(PlotCentre.X + float((X - Centre.X) * Scale), PlotCentre.Y - float((Y - Centre.Y) * Scale));
	};
	// The metres visible at the plot's edges.
	const double X0 = Centre.X - PlotSize.X * 0.5 / Scale, X1 = Centre.X + PlotSize.X * 0.5 / Scale;
	const double Y0 = Centre.Y - PlotSize.Y * 0.5 / Scale, Y1 = Centre.Y + PlotSize.Y * 0.5 / Scale;

	// Grid and axis labels, one step for both axes so squares stay square.
	const double Step = Map::GridStep(FMath::Max(X1 - X0, Y1 - Y0), 8);
	auto TickText = [Step](double V)
	{
		if (FMath::Abs(V) < Step * 1e-3) V = 0.0;   // no "-0"
		return Step < 1.0 ? FString::Printf(TEXT("%.1f"), V) : FString::Printf(TEXT("%.0f"), V);
	};
	for (double X = FMath::CeilToDouble(X0 / Step) * Step; X <= X1; X += Step)
	{
		const float Sx = ToScreen(X, 0.0).X;
		FSlateDrawElement::MakeLines(Out, LayerId + 1, Geo.ToPaintGeometry(),
			TArray<FVector2f>{ FVector2f(Sx, PlotMin.Y), FVector2f(Sx, PlotMax.Y) }, ESlateDrawEffect::None, Map::Grid, true, 1.f);
		const FString S = TickText(X);
		Text(LayerId + 1, S, FVector2f(Sx - MeasureText(S, SmallFont).X * 0.5f, PlotMax.Y + 4.f), Map::Label, SmallFont);
	}
	for (double Y = FMath::CeilToDouble(Y0 / Step) * Step; Y <= Y1; Y += Step)
	{
		const float Sy = ToScreen(0.0, Y).Y;
		FSlateDrawElement::MakeLines(Out, LayerId + 1, Geo.ToPaintGeometry(),
			TArray<FVector2f>{ FVector2f(PlotMin.X, Sy), FVector2f(PlotMax.X, Sy) }, ESlateDrawEffect::None, Map::Grid, true, 1.f);
		const FString S = TickText(Y);
		const FVector2f TS = MeasureText(S, SmallFont);
		Text(LayerId + 1, S, FVector2f(PlotMin.X - TS.X - 6.f, Sy - TS.Y * 0.5f), Map::Label, SmallFont);
	}
	// Frame, and the axis names.
	FSlateDrawElement::MakeLines(Out, LayerId + 1, Geo.ToPaintGeometry(),
		TArray<FVector2f>{ PlotMin, FVector2f(PlotMax.X, PlotMin.Y), PlotMax, FVector2f(PlotMin.X, PlotMax.Y), PlotMin },
		ESlateDrawEffect::None, Map::Axis, true, 1.f);
	{
		const FString S = LOCTEXT("XAxis", "x (m)").ToString();
		Text(LayerId + 1, S, FVector2f(PlotMax.X - MeasureText(S, SmallFont).X, PlotMax.Y + 18.f), Map::Label, SmallFont);
		Text(LayerId + 1, LOCTEXT("YAxis", "y (m)").ToString(), FVector2f(4.f, PlotMin.Y - 16.f), Map::Label, SmallFont);
	}

	// Cones. A cone is ~0.23 m across: drawn a bit bigger, so a big track
	// still shows them.
	const float Dot = FMath::Clamp(0.45f * Scale, 4.f, 9.f);
	int32 NBlue = 0, NYellow = 0, NOrange = 0;
	for (const FCone& C : Cones)
	{
		FLinearColor Color = Map::Other;
		float D = Dot;
		switch (C.Kind)
		{
		case EConeKind::Blue:        Color = Map::Blue; ++NBlue; break;
		case EConeKind::Yellow:      Color = Map::Yellow; ++NYellow; break;
		case EConeKind::SmallOrange: Color = Map::SmallOrange; ++NOrange; break;
		case EConeKind::BigOrange:   Color = Map::BigOrange; ++NOrange; D = Dot * 1.6f; break;
		default: break;
		}
		D = (float)FMath::RoundToInt(D);
		const FVector2f P = ToScreen(C.Pos.X, C.Pos.Y);
		FSlateDrawElement::MakeBox(Out, LayerId + 2,
			Geo.ToPaintGeometry(FVector2f(D, D), FSlateLayoutTransform(P - FVector2f(D, D) * 0.5f)),
			Map::DotBrush(D), ESlateDrawEffect::None, Color);
	}

	// Legend, top right: the cone counts.
	{
		struct FEntry { FString Name; int32 N; FLinearColor Color; };
		const FEntry Entries[] = {
			{ TEXT("Blue"), NBlue, Map::Blue },
			{ TEXT("Yellow"), NYellow, Map::Yellow },
			{ TEXT("Orange"), NOrange, Map::BigOrange },
		};
		float X = PlotMax.X;
		for (int32 i = UE_ARRAY_COUNT(Entries) - 1; i >= 0; --i)
		{
			const FEntry& E = Entries[i];
			if (E.N == 0) continue;
			const FString S = FString::Printf(TEXT("%s (%d)"), *E.Name, E.N);
			const FVector2f TS = MeasureText(S, SmallFont);
			X -= TS.X;
			Text(LayerId + 2, S, FVector2f(X, 8.f), Map::Label, SmallFont);
			X -= 12.f;
			FSlateDrawElement::MakeBox(Out, LayerId + 2,
				Geo.ToPaintGeometry(FVector2f(8.f, 8.f), FSlateLayoutTransform(FVector2f(X, 8.f + TS.Y * 0.5f - 4.f))),
				Map::DotBrush(8.f), ESlateDrawEffect::None, E.Color);
			X -= 16.f;
		}
	}

	return LayerId + 2;
}

#undef LOCTEXT_NAMESPACE
