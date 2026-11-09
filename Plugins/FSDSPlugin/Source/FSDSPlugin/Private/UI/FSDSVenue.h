#pragma once

#include "CoreMinimal.h"

class FSlateWindowElementList;
struct FGeometry;

/**
 * Venues (docs/venues.md): a fixed place, such as a parking lot, that several
 * tracks share. The menu reads the venue files itself, so it needs no backend:
 * the Drive page lists a venue's tracks and draws its surroundings, and the
 * track editor draws them behind the track.
 *
 * Read the way tools/envgen/venue.py reads them ("ifssim-venue/1"): rows
 * expanded into props, metres in the venue's frame. The checks (cones on the
 * lot, props off it) stay in envgen; the backend applies them while you draw.
 */
namespace FSDSVenue
{
	/** Between a venue's name and the track's: parking_a__loop. */
	const TCHAR* const Sep = TEXT("__");

	struct FProp
	{
		FString Class;
		FVector2D Pos = FVector2D::ZeroVector;
		float YawDeg = 0.f;
	};

	struct FVenue
	{
		FString Name;
		FString Title;
		TArray<FVector2D> Lot;     // the open asphalt where tracks go
		float MarginM = 2.f;       // how far inside the lot cones keep
		TArray<FProp> Props;       // rows expanded

		/** What a track drawn here is named with: parking_a__. */
		FString Prefix() const { return Name + Sep; }
		/** The lot and everything around it. */
		FBox2D Bounds() const;
	};

	using FVenueRef = TSharedRef<const FVenue>;

	bool LoadFile(const FString& Path, FVenue& Out);

	/** Every <dir>/venues/<name>.venue.json, the first dir winning a name, sorted by title. */
	TArray<FVenueRef> Scan(const TArray<FString>& TrackDirs);

	/** "parking_a" for "parking_a__loop"; empty for a track with no prefix. */
	FString VenueOfTrack(const FString& TrackName);

	const FVenue* Find(const TArray<FVenueRef>& Venues, const FString& Name);

	/**
	 * How far a track's CSV sits from its venue's frame. A track is recentred
	 * on its own cones when it is built, and its env sidecar holds the venue's
	 * props moved by that same shift (envgen.venue.sidecar), so the shift is
	 * read off the sidecar. False without a sidecar that matches the venue.
	 */
	bool TrackShift(const FVenue& V, const FString& TrackCsvPath, FVector2D& Out);

	/** Draw the lot and the props. ToScreen maps venue metres to local pixels. */
	void Paint(const FVenue& V, TFunctionRef<FVector2f(const FVector2D&)> ToScreen, float PxPerM,
		const FGeometry& Geo, FSlateWindowElementList& Out, int32 LayerId);
}
