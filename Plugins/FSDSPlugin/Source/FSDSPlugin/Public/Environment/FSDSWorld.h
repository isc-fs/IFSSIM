#pragma once

#include "CoreMinimal.h"

/**
 * A world: a real place the sim builds around a track, read from
 * Content/tracks/worlds/<name>/ (world.json + terrain.i16). Format and frame:
 * docs/worlds.md; the generator and the reference checker are tools/worldgen.
 *
 * World frame: metres, +x along the place's main axis, +y to its left, +z up,
 * origin on the ground at a named reference point. A track places the world
 * through its sidecar ("world": {name, x, y}), a translation in the track CSV
 * frame, so world (x, y) + (x, y) is the CSV position.
 */

struct FFSDSWorldSurface
{
	FString Material;   // asphalt | asphalt_light | concrete | grass
	double XMin = 0, XMax = 0, YMin = 0, YMax = 0;
};

struct FFSDSWorldLine
{
	double X0 = 0, Y0 = 0, X1 = 0, Y1 = 0;
	double WidthM = 0;
	FString Colour;     // white | yellow
};

struct FFSDSWorldWall
{
	FString Material;   // concrete
	double X0 = 0, Y0 = 0, X1 = 0, Y1 = 0;
	double HeightM = 0, ThicknessM = 0;
	double FenceHeightM = 0;   // 0: no fence
};

struct FFSDSWorldBuilding
{
	FString Material;   // concrete | glass
	double XMin = 0, XMax = 0, YMin = 0, YMax = 0;
	double TopZ = 0;    // roof height, world frame
};

/** A roofed grandstand along x. Seating rises from (FrontY, FrontZ) to
 *  (BackY, BackZ); the roof slopes from (RoofFrontY, RoofFrontZ) to
 *  (RoofBackY, RoofBackZ). The sign of the y values says which side it is on. */
struct FFSDSWorldGrandstand
{
	double XMin = 0, XMax = 0;
	double FrontY = 0, BackY = 0, FrontZ = 0, BackZ = 0;
	double RoofFrontY = 0, RoofBackY = 0, RoofFrontZ = 0, RoofBackZ = 0;
};

struct FSDSPLUGIN_API FFSDSWorld
{
	FString Dir;
	FString Name;
	FString Title;
	TArray<FString> Attribution;

	double TerrainX0 = 0, TerrainY0 = 0, CellM = 1, ZUnitM = 0.01;
	int32 Nx = 0, Ny = 0;
	TArray<int16> Heights;        // index = j * Nx + i (i along x)

	bool bHasDetail = false;
	double DetailXMin = 0, DetailXMax = 0;

	TArray<FFSDSWorldSurface> Surfaces;
	TArray<FFSDSWorldLine> Lines;
	TArray<FFSDSWorldWall> Walls;
	TArray<FFSDSWorldBuilding> Buildings;
	TArray<FFSDSWorldGrandstand> Grandstands;

	/** Ground height (m, world frame) at (x, y): bilinear, clamped at the edges. */
	double HeightAt(double X, double Y) const;
	double TerrainXMax() const { return TerrainX0 + (Nx - 1) * CellM; }
	double TerrainYMax() const { return TerrainY0 + (Ny - 1) * CellM; }
};

namespace FSDSWorld
{
	inline const TCHAR* FormatV1 = TEXT("ifssim-world/1");
	inline const TCHAR* TerrainFile = TEXT("terrain.i16");
	constexpr double MaxCoordM = 2000.0;
	constexpr int32 MaxTerrainCells = 2000000;

	/** A world name is letters, digits and underscores (it becomes a path). */
	FSDSPLUGIN_API bool IsValidName(const FString& Name);

	/** <folder of the track CSV>/worlds/<name> */
	FSDSPLUGIN_API FString DirFor(const FString& TrackCsvPath, const FString& Name);

	/** Parse world.json text (not the terrain). Unknown keys, wrong types and
	 *  values out of range are errors, as in tools/worldgen/world.py. */
	FSDSPLUGIN_API bool Parse(const FString& Json, FFSDSWorld& Out, FString& OutError);

	/** Read <Dir>/world.json and its terrain. */
	FSDSPLUGIN_API bool Load(const FString& Dir, FFSDSWorld& Out, FString& OutError);
}
