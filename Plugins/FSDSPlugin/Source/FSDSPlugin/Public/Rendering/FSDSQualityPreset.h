#pragma once

#include "CoreMinimal.h"

/**
 * Rendering presets (docs/rendering_presets.md).
 *
 *  - sim: the project default (Config/DefaultEngine.ini): 25 % screen
 *    resolution, minimum scalability, no Lumen. Fast; what tests and
 *    benchmarks run on.
 *  - showcase: full resolution, maximum scalability, Lumen global
 *    illumination and reflections, auto exposure, and physically based
 *    daylight (AFSDSDaylight). For demos, screenshots and renders; it costs
 *    frame time, so never benchmark on it.
 *
 * Neither changes what the sensors measure: the LiDAR's depth capture turns
 * lighting, GI and post-processing off whatever the preset.
 *
 * Switch with the console command `FSDS.Quality sim|showcase`, or start with
 * `-showcase` on the command line. getSimStatus reports the active one.
 */
namespace FSDSQuality
{
	enum class EPreset : uint8 { Sim, Showcase };

	/** Apply a preset now, its daylight in World. Returns false for an
	 *  unknown name in ApplyByName. */
	FSDSPLUGIN_API void Apply(EPreset Preset, UWorld* World);
	FSDSPLUGIN_API bool ApplyByName(const FString& Name, UWorld* World);

	/** Apply -showcase if it is on the command line (called once at startup). */
	FSDSPLUGIN_API void ApplyCommandLine(UWorld* World);

	FSDSPLUGIN_API EPreset Current();
	FSDSPLUGIN_API const TCHAR* Name(EPreset Preset);
}
