#pragma once

#include "CoreMinimal.h"

/**
 * A world: a real place the sim builds around a track, read from
 * Content/tracks/worlds/<name>/ (world.json + terrain.i16, and wear.u8 if the
 * world has asphalt wear). Format and frame:
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
	double EndWidthM = 0;   // width at (X1, Y1); WidthM unless the line tapers
	FString Colour;     // white | yellow | red | green
};

/** A flat panel on a roof (world frame), lying on the roof's top. */
struct FFSDSWorldPanel
{
	double XMin = 0, XMax = 0, YMin = 0, YMax = 0;
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
	double YawDeg = 0;  // the box turned about its centre, counter-clockwise from +x
	bool bHasBase = false;
	double BaseZ = 0;   // with bHasBase: a raised deck (a footbridge) from BaseZ to TopZ
	bool bGarages = false;   // pit-garage fronts on the face towards the track
	double CanopyM = 0;      // with bGarages: a canopy this deep over the garage fronts, at roof level
	TArray<FFSDSWorldPanel> RoofPanels;
};

/** A grandstand along x. Seating rises from (FrontY, FrontZ) to (BackY,
 *  BackZ); a roof, if it has one, slopes from (RoofFrontY, RoofFrontZ) to
 *  (RoofBackY, RoofBackZ). The sign of the y values says which side it is on. */
struct FFSDSWorldGrandstand
{
	double XMin = 0, XMax = 0;
	double FrontY = 0, BackY = 0, FrontZ = 0, BackZ = 0;
	bool bHasRoof = false;
	double RoofFrontY = 0, RoofBackY = 0, RoofFrontZ = 0, RoofBackZ = 0;
	TArray<FFSDSWorldPanel> RoofPanels;   // solar panels on the roof
};

/** A tree measured from the LiDAR: trunk at (X, Y), H tall, crown radius R. */
struct FFSDSWorldTree
{
	double X = 0, Y = 0, H = 0, R = 0;
	FString Kind;   // a name from FSDSWorld::TreeKinds
};

/** One placed prop from /Game/Worlds/Meshes (docs/world_assets.md). */
struct FFSDSWorldProp
{
	FString Mesh;        // a name from FSDSWorld::PropMeshes
	double X = 0, Y = 0, YawDeg = 0;
};

struct FSDSPLUGIN_API FFSDSWorld
{
	FString Dir;
	FString Name;
	FString Title;
	TArray<FString> Attribution;

	/** Compass bearing of +x (degrees clockwise from north): where the sun is. */
	double XBearingDeg = 0;

	double TerrainX0 = 0, TerrainY0 = 0, CellM = 1, ZUnitM = 0.01;
	int32 Nx = 0, Ny = 0;
	TArray<int16> Heights;        // index = j * Nx + i (i along x)

	bool bHasDetail = false;
	double DetailXMin = 0, DetailXMax = 0;

	/** Asphalt wear (wear.u8): uint8 nodes, index = j * WearNx + i, value / 128
	 *  multiplies the asphalt's colour. */
	bool bHasWear = false;
	double WearX0 = 0, WearY0 = 0, WearDx = 1, WearDy = 1;
	int32 WearNx = 0, WearNy = 0;
	TArray<uint8> Wear;

	TArray<FFSDSWorldSurface> Surfaces;
	TArray<FFSDSWorldLine> Lines;
	TArray<FFSDSWorldWall> Walls;
	TArray<FFSDSWorldBuilding> Buildings;
	TArray<FFSDSWorldGrandstand> Grandstands;
	TArray<FFSDSWorldProp> Props;
	TArray<FFSDSWorldTree> Trees;

	/** Ground height (m, world frame) at (x, y): bilinear, clamped at the edges. */
	double HeightAt(double X, double Y) const;
	double TerrainXMax() const { return TerrainX0 + (Nx - 1) * CellM; }
	double TerrainYMax() const { return TerrainY0 + (Ny - 1) * CellM; }

	/** The wear multiplier at (x, y): bilinear between nodes; 1 outside the map. */
	double WearAt(double X, double Y) const;
	/** Whether the box lies inside the wear map. */
	bool WearCovers(double XMin, double XMax, double YMin, double YMax) const;
};

namespace FSDSWorld
{
	inline const TCHAR* FormatV1 = TEXT("ifssim-world/1");
	inline const TCHAR* TerrainFile = TEXT("terrain.i16");
	inline const TCHAR* WearFile = TEXT("wear.u8");
	constexpr double MaxLineWidthM = 10.0;
	constexpr int32 MaxRoofPanels = 500;
	constexpr double MaxCoordM = 2000.0;
	constexpr int32 MaxTerrainCells = 2000000;

	/** Props a world may place by name; the rest are placed by the structures
	 *  they belong to (seats on grandstands, posts on fences...). */
	inline constexpr const TCHAR* PropMeshes[] = { TEXT("StartGantry"), TEXT("MarshalPost"), TEXT("SignBoard") };

	/** Tree kinds; each is a pair of meshes, Tree<Kind>A and Tree<Kind>B. */
	inline constexpr const TCHAR* TreeKinds[] = { TEXT("pine"), TEXT("broadleaf") };
	constexpr int32 MaxTrees = 20000;

	/** A world name is letters, digits and underscores (it becomes a path). */
	FSDSPLUGIN_API bool IsValidName(const FString& Name);

	/** <folder of the track CSV>/worlds/<name> */
	FSDSPLUGIN_API FString DirFor(const FString& TrackCsvPath, const FString& Name);

	/** Parse world.json text (not the terrain). Unknown keys, wrong types and
	 *  values out of range are errors, as in tools/worldgen/world.py. */
	FSDSPLUGIN_API bool Parse(const FString& Json, FFSDSWorld& Out, FString& OutError);

	/** Read <Dir>/world.json, its terrain and its wear map. */
	FSDSPLUGIN_API bool Load(const FString& Dir, FFSDSWorld& Out, FString& OutError);
}
