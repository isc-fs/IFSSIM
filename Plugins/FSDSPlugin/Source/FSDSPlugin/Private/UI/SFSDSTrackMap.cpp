#include "UI/SFSDSTrackMap.h"

#include "Fonts/FontMeasure.h"
#include "Framework/Application/SlateApplication.h"
#include "Misc/FileHelper.h"
#include "Rendering/DrawElements.h"
#include "Styling/CoreStyle.h"

#define LOCTEXT_NAMESPACE "FSDSTrackMap"

namespace FSDSTrackMapStyle
{
	const FLinearColor Backdrop(0.020f, 0.020f, 0.026f, 1.f);
	const FLinearColor Blue = FLinearColor::FromSRGBColor(FColor(59, 130, 246));
	const FLinearColor Yellow = FLinearColor::FromSRGBColor(FColor(250, 204, 21));
	const FLinearColor Orange = FLinearColor::FromSRGBColor(FColor(249, 115, 22));
	const FLinearColor Other = FLinearColor::FromSRGBColor(FColor(160, 160, 170));
	const FLinearColor Muted = FLinearColor::FromSRGBColor(FColor(150, 150, 160));
	constexpr float MarginPx = 16.f;
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
	// Qualified: other UI files in the same unity blob bring their own Muted etc.
	namespace Map = FSDSTrackMapStyle;
	const FSlateBrush* White = FCoreStyle::Get().GetBrush("WhiteBrush");
	const FVector2f Size = FVector2f(Geo.GetLocalSize());

	FSlateDrawElement::MakeBox(Out, LayerId, Geo.ToPaintGeometry(), White, ESlateDrawEffect::None, Map::Backdrop);

	if (Cones.Num() == 0 || !Bounds.bIsValid)
	{
		const FText Text = LOCTEXT("NoTrack", "No track selected");
		const FSlateFontInfo Font = FCoreStyle::GetDefaultFontStyle("Regular", 11);
		const FVector2f TextSize = FVector2f(FSlateApplication::Get().GetRenderer()->GetFontMeasureService()->Measure(Text, Font));
		FSlateDrawElement::MakeText(Out, LayerId + 1,
			Geo.ToPaintGeometry(TextSize, FSlateLayoutTransform((Size - TextSize) * 0.5f)),
			Text, Font, ESlateDrawEffect::None, Map::Muted);
		return LayerId + 1;
	}

	// Fit the cones, keeping metres square. Screen y points down, the
	// track's y (north) points up.
	const FVector2D Extent = Bounds.GetSize();
	const float ScaleX = (Size.X - 2.f * Map::MarginPx) / FMath::Max(Extent.X, 1.0);
	const float ScaleY = (Size.Y - 2.f * Map::MarginPx) / FMath::Max(Extent.Y, 1.0);
	const float Scale = FMath::Max(FMath::Min(ScaleX, ScaleY), 0.01f);
	const FVector2D Centre = Bounds.GetCenter();
	auto ToScreen = [&](const FVector2D& P)
	{
		return FVector2f(Size.X * 0.5f + (P.X - Centre.X) * Scale, Size.Y * 0.5f - (P.Y - Centre.Y) * Scale);
	};

	// A cone is ~0.23 m across; keep it visible on big tracks and
	// small on tiny ones.
	const float Dot = FMath::Clamp(0.35f * Scale, 2.5f, 7.f);
	for (const FCone& C : Cones)
	{
		FLinearColor Color = Map::Other;
		float D = Dot;
		switch (C.Kind)
		{
		case EConeKind::Blue:        Color = Map::Blue; break;
		case EConeKind::Yellow:      Color = Map::Yellow; break;
		case EConeKind::SmallOrange: Color = Map::Orange; break;
		case EConeKind::BigOrange:   Color = Map::Orange; D = Dot * 1.6f; break;
		default: break;
		}
		const FVector2f P = ToScreen(C.Pos);
		FSlateDrawElement::MakeBox(Out, LayerId + 1,
			Geo.ToPaintGeometry(FVector2f(D, D), FSlateLayoutTransform(P - FVector2f(D, D) * 0.5f)),
			White, ESlateDrawEffect::None, Color);
	}

	// Scale bar: 10 m (or 50 m on big tracks), bottom left.
	const float BarM = Extent.GetMax() > 150.0 ? 50.f : 10.f;
	const float BarPx = BarM * Scale;
	const FVector2f BarAt(Map::MarginPx, Size.Y - Map::MarginPx);
	FSlateDrawElement::MakeBox(Out, LayerId + 1,
		Geo.ToPaintGeometry(FVector2f(BarPx, 2.f), FSlateLayoutTransform(BarAt)),
		White, ESlateDrawEffect::None, Map::Muted);
	const FText BarText = FText::Format(LOCTEXT("Bar", "{0} m"), FText::AsNumber(FMath::RoundToInt(BarM)));
	FSlateDrawElement::MakeText(Out, LayerId + 1,
		Geo.ToPaintGeometry(FVector2f(60.f, 14.f), FSlateLayoutTransform(BarAt + FVector2f(0.f, -16.f))),
		BarText, FCoreStyle::GetDefaultFontStyle("Regular", 9), ESlateDrawEffect::None, Map::Muted);

	return LayerId + 1;
}

#undef LOCTEXT_NAMESPACE
