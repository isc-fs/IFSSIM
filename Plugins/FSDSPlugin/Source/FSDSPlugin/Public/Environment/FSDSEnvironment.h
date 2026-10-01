#pragma once

#include "CoreMinimal.h"

/**
 * A track's trackside environment, read from its sidecar file.
 *
 * The cones live in <track>.csv; everything around them (ground extent and
 * props) lives next to it in <track>.env.json. Per docs/ENVIRONMENT_ROADMAP.md
 * rule 3, the sidecar is generated in Python (tools/envgen) from a seed and a
 * profile, and the plugin only instantiates it, so a world is reproduced
 * exactly by its file. Format: docs/environment_sidecar.md.
 *
 * Values are kept as authored, in the track CSV's frame: metres, x and y as
 * in the CSV's columns, yaw counter-clockwise from +x. The helpers below map
 * them into UE the same way FSDSConeSpawner maps cones.
 */

/** One prop instance, as authored. */
struct FFSDSEnvProp
{
	FString Class;
	double X = 0.0;        // m, track CSV frame
	double Y = 0.0;        // m, track CSV frame
	double YawDeg = 0.0;   // counter-clockwise from +x, track CSV frame
};

struct FFSDSEnvironment
{
	FString SidecarPath;

	/** The generator's seed and profile, recorded for provenance. The plugin
	 *  draws nothing from them. */
	int64 Seed = 0;
	FString Profile;

	/** The ground the generator laid out, in the track CSV frame (m). */
	bool bHasGroundExtent = false;
	double GroundXMin = 0.0, GroundYMin = 0.0, GroundXMax = 0.0, GroundYMax = 0.0;

	TArray<FFSDSEnvProp> Props;
};

namespace FSDSEnvironment
{
	/** The only format this build reads. */
	inline const TCHAR* FormatV1 = TEXT("ifssim-env/1");

	/** Sanity bound on any coordinate, as for cone CSVs (a few km would be a
	 *  cm-vs-m mistake, not a track). */
	constexpr double MaxCoordM = 1000.0;

	/** <dir>/<stem>.env.json for <dir>/<stem>.csv. */
	FSDSPLUGIN_API FString SidecarPathFor(const FString& TrackCsvPath);

	/** Parse sidecar text. On failure returns false and says why in OutError;
	 *  Out is left empty. Unknown keys are errors, so a misspelt field fails
	 *  loudly instead of being silently ignored. */
	FSDSPLUGIN_API bool Parse(const FString& Json, const FString& SourcePath,
		FFSDSEnvironment& Out, FString& OutError);

	/** Read and parse a sidecar file. */
	FSDSPLUGIN_API bool Load(const FString& Path, FFSDSEnvironment& Out, FString& OutError);

	/** Track CSV frame (m) -> UE (cm). Same mapping as FSDSConeSpawner::SpawnFromCSV:
	 *  UE X = x, UE Y = -y. */
	inline FVector2D CsvToUeCm(double X, double Y) { return FVector2D(X * 100.0, -Y * 100.0); }

	/** Yaw in the track CSV frame (counter-clockwise) -> UE yaw (degrees). The
	 *  Y flip above turns counter-clockwise into UE's clockwise-positive yaw. */
	inline double CsvYawToUeDeg(double YawDeg) { return -YawDeg; }
}
