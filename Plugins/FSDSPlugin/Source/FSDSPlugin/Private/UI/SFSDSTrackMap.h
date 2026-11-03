#pragma once

#include "CoreMinimal.h"
#include "UI/FSDSVenue.h"
#include "Widgets/SLeafWidget.h"

/**
 * A track seen from above: its cones, fitted to the widget, blue on the
 * left as the car sees it. Reads the track CSV itself, so it needs no
 * backend. The menu's previews use it; the track editor will build on it.
 */
class SFSDSTrackMap : public SLeafWidget
{
public:
	enum class EConeKind : uint8 { Blue, Yellow, SmallOrange, BigOrange, Other };

	struct FCone
	{
		FVector2D Pos = FVector2D::ZeroVector;   // metres, track CSV frame (x east, y north)
		EConeKind Kind = EConeKind::Other;
	};

	SLATE_BEGIN_ARGS(SFSDSTrackMap) {}
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	/** Show these cones; an empty list shows the "no track" text. */
	void SetCones(TArray<FCone> InCones);
	/** Draw a venue around the cones; Shift moves it into the CSV's frame. Null for none. */
	void SetVenue(const TSharedPtr<const FSDSVenue::FVenue>& InVenue, const FVector2D& InShift);

	/** A filled circle brush D px across, for cones and markers. */
	static const FSlateBrush* DotBrush(float D);

	/** Parse a track CSV (type,x,y,...) the way the cone spawner does. */
	static bool LoadCsv(const FString& Path, TArray<FCone>& Out);

	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle,
		bool bParentEnabled) const override;
	virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(320.f, 320.f); }

private:
	void UpdateBounds();

	TArray<FCone> Cones;
	TSharedPtr<const FSDSVenue::FVenue> Venue;
	FVector2D VenueShift = FVector2D::ZeroVector;
	FBox2D Bounds = FBox2D(ForceInit);
};
