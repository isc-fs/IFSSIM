#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"

/**
 * The track editor's data: a drawn track spec, and the preview the backend
 * builds from it. The format is tools/envgen/spec.py's ("ifssim-trackspec/1");
 * the geometry (spline, cones, FS rule checks, start pose) is envgen's, through
 * Mission Control's /api/track/preview_spec and /api/track/create. Coordinates
 * are metres in the track CSV's frame (+x east, +y north).
 */
namespace FSDSTrackSpec
{
	/** One stretch of track changed for a test, from control point From to To. */
	struct FSection
	{
		int32 From = 1;
		int32 To = 3;
		TOptional<float> Width;     // unset: the track's own
		TOptional<float> Spacing;
		float Drop = 0.f;           // fraction of cones removed, 0..1
		float Jitter = 0.f;         // metres, std dev of a random displacement
		FString Side = TEXT("both");    // both | left | right
		FString Recolor;                // "" | swap | blue | yellow | small_orange
	};

	struct FSpec
	{
		TArray<FVector2D> Points;
		bool bClosed = true;
		float Width = 3.5f;
		float SpacingStraight = 5.f;
		float SpacingCornerMin = 1.5f;
		int32 Seed = 1;
		TArray<FSection> Sections;

		/** Enough points to build: 3 for a loop, 2 for an open track. */
		bool Enough() const { return Points.Num() >= (bClosed ? 3 : 2); }
	};

	struct FCone { FString Type; FVector2D Pos; };
	struct FIssue { FString Code; bool bError = false; FString Message; TOptional<float> S; TOptional<FVector2D> XY; };

	/** What /api/track/preview_spec returns. Positions are recentred by Offset. */
	struct FPreview
	{
		bool bOk = false;
		float LengthM = 0.f;
		TArray<FCone> Cones;
		int32 Blue = 0, Yellow = 0, Orange = 0;
		FVector2D Offset = FVector2D::ZeroVector;
		TArray<FIssue> Issues;
		TArray<FVector2D> Left, Right;
		bool bHasStart = false;
		FVector2D Spawn = FVector2D::ZeroVector, Forward = FVector2D(1, 0);

		int32 NumErrors() const;
	};

	/** The editor's state, kept by the menu so it outlives the page (a test
	 *  drive and back finds the track as it was left). */
	struct FEditorState
	{
		TSharedRef<FSpec> Spec = MakeShared<FSpec>();
		FString Name;
		/** The name it was last saved as, while the drawing hasn't changed since. */
		FString SavedName;
	};

	TSharedRef<FJsonObject> ToJson(const FSpec& Spec);
	/** Parse a .track.json (or a spec object). False if it isn't one. */
	bool FromJson(const TSharedPtr<FJsonObject>& Obj, FSpec& Out);
	bool LoadFile(const FString& Path, FSpec& Out);
	bool ParsePreview(const TSharedPtr<FJsonObject>& Obj, FPreview& Out);

	/** Ramer-Douglas-Peucker: a freehand stroke down to its control points. */
	TArray<FVector2D> Simplify(const TArray<FVector2D>& Pts, double Tol);
	/** A freehand stroke as control points, the web editor's way: simplified,
	 *  points closer than 4 m merged, and a loop's tail near the start dropped. */
	TArray<FVector2D> StrokeToPoints(const TArray<FVector2D>& Stroke, bool bClosed);
	/** Sections are by point number: keep them on their stretch when a point
	 *  is inserted (+1) or deleted (-1) at At. */
	void ShiftSections(TArray<FSection>& Sections, int32 At, int32 Delta);
}
