#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"

/**
 * Collision channels and profiles for the trackside environment.
 *
 * They are defined in Config/DefaultEngine.ini ([/Script/Engine.CollisionProfile]),
 * and docs/collision_matrix.md explains every cell. The engine assigns custom
 * object channels to ECC_GameTraceChannelN slots in the order the ini declares
 * them, so the constants below must match it; validateCollisionMatrix checks
 * that they do.
 */
namespace FSDSCollision
{
	/** Object type of a solid prop: stops the car and the cones, seen by the
	 *  LiDAR, never ground. */
	constexpr ECollisionChannel PropChannel = ECC_GameTraceChannel1;

	/** Object type of a prop only the LiDAR sees: no physics, answers only
	 *  Visibility traces. */
	constexpr ECollisionChannel PropLidarOnlyChannel = ECC_GameTraceChannel2;

	/** Names as the ini declares them. */
	inline const TCHAR* PropChannelName = TEXT("FSDSProp");
	inline const TCHAR* PropLidarOnlyChannelName = TEXT("FSDSPropLidarOnly");

	/** Collision profiles. */
	inline const TCHAR* PropProfile = TEXT("FSDSProp");
	inline const TCHAR* PropLidarOnlyProfile = TEXT("FSDSPropLidarOnly");
	inline const TCHAR* TerrainProfile = TEXT("FSDSTerrain");
}
