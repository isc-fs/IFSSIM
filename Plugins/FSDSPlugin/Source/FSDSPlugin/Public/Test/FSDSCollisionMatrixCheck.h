// Checks docs/collision_matrix.md against the engine, with test props.
#pragma once

#include "CoreMinimal.h"

class AFSDSVehiclePawn;
class UWorld;

namespace FSDSCollisionMatrixCheck
{
	/**
	 * Spawn the two test props in front of the car, replacing any earlier ones:
	 *
	 *   solid      FSDSProp,          1.0 x 1.0 x 0.6 m, AheadM + 2 m ahead, 2.5 m left
	 *   lidar_only FSDSPropLidarOnly, 0.4 x 3.0 x 0.15 m, AheadM ahead, across the path
	 *
	 * They stay until the next Spawn or Clear, so the car can be driven over
	 * the strip and a LiDAR consumer can look for them. Returns an empty string
	 * on success, or a JSON error. Game thread only.
	 */
	FSDSPLUGIN_API FString Spawn(AFSDSVehiclePawn* Pawn, double AheadM);

	/**
	 * Check every row of the collision matrix against the spawned props, with
	 * the queries the sim itself makes: the Chaos wheel trace, the WorldStatic
	 * ground query (road probe and cone snap), the CPU LiDAR's Visibility
	 * trace, and the physics responses between props, the car and the cones.
	 *
	 * Call it a few frames after Spawn: Chaos only adds new bodies to its
	 * scene queries on a later tick, so in the spawn frame every query misses
	 * the props, and an "ignores it" check would pass for the wrong reason. A
	 * control query that must hit the solid prop guards against exactly that.
	 *
	 * Returns JSON: ok, props (offsets from the car at spawn, in its own frame,
	 * metres) and checks. Game thread only.
	 */
	FSDSPLUGIN_API FString Check(AFSDSVehiclePawn* Pawn);

	/** Remove the test props; returns how many were removed. Game thread only. */
	FSDSPLUGIN_API int32 Clear(UWorld* World);
}
