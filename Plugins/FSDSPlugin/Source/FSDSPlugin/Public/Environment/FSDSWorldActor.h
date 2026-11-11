#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Environment/FSDSWorld.h"
#include "FSDSWorldActor.generated.h"

class UProceduralMeshComponent;
class UHierarchicalInstancedStaticMeshComponent;
class UPrimitiveComponent;
class UMaterialInterface;
class UStaticMesh;

/**
 * Builds a world (docs/worlds.md) at runtime from its data files: the terrain,
 * the surfaces and lines painted on it, walls with their fences, buildings and
 * grandstands. Everything is generated geometry, so a world needs no editor
 * work and moves with the track when Mission Control recentres it.
 *
 * Collision (docs/collision_matrix.md):
 *  - terrain and the asphalt, concrete and grass surfaces are FSDSTerrain:
 *    ground for the wheels, the road probe and the cone snap. Surfaces sit
 *    3 cm above the terrain, the step the matrix allows.
 *  - walls, fence posts, buildings and grandstands are FSDSProp: the car and
 *    the LiDAR hit them, they are never ground.
 *  - painted bands, lines and fence mesh have no collision.
 *  - detail props (seats, fence posts, boards, garage fronts, rails, roof
 *    columns, the props a world lists) have none either: the GPU LiDAR
 *    renders them, and the car can't reach them past the walls.
 *
 * Materials and props come from /Game/Worlds (docs/world_assets.md): CC0
 * scanned materials, and props modelled to real dimensions by
 * tools/assetgen. Generated geometry carries UVs in metres. Props are drawn
 * as hierarchical instances with LODs and cull distances.
 *
 * The rendering preset sets how far they reach (SetDetail): showcase draws
 * every prop out to its own cull distance with shadows; sim culls them at
 * SimPropCullM and drops their shadows, which keeps the sim at real time on
 * a GTX 1080. Both keep everything the LiDAR (30 m) can reach.
 *
 * While a world is built, the level's own flat floor is hidden and its
 * collision turned off (the terrain replaces it: the ground falls 7 m along
 * the Montmeló straight); Clear() puts it back.
 *
 * The cone spawner owns one and rebuilds it with every track, before the
 * props and cones are placed on the ground.
 */
UCLASS(NotPlaceable)
class FSDSPLUGIN_API AFSDSWorldActor : public AActor
{
	GENERATED_BODY()

public:
	AFSDSWorldActor();

	/** Replace what was built with World, its origin at OriginCsvM (the
	 *  sidecar's world x, y: metres in the track CSV frame). */
	void Build(const FFSDSWorld& World, const FVector2D& OriginCsvM);

	/** Remove everything and give the level its floor back. */
	void Clear();

	bool IsBuilt() const { return bBuilt; }

	/** Light or put out the start lights on the world's StartGantry props
	 *  (red while the car waits for GO, out at GO). No-op without a gantry. */
	void SetStartLights(bool bOn);
	bool AreStartLightsOn() const { return bStartLightsOn; }
	bool HasStartLights() const { return StartLights != nullptr; }
	/** Generated (procedural) triangles; props are counted apart. */
	int32 GetNumTriangles() const { return NumTriangles; }
	int32 GetNumInstances() const { return NumInstances; }
	/** Trees placed (instances of the tree models, when they are installed). */
	int32 GetNumTrees() const { return NumTrees; }
	/** Prop triangles if every instance drew its full-detail LOD. */
	int64 GetPropTriangles() const { return PropTriangles; }
	const TArray<FString>& GetHiddenFloors() const { return HiddenFloorNames; }

	/** Props culled at SimPropCullM without shadows (sim), or at their own
	 *  distance with shadows (showcase). Build applies the active preset;
	 *  FSDSQuality::Apply calls this when the preset changes. */
	void SetDetail(bool bShowcase);
	bool IsShowcaseDetail() const { return bShowcaseDetail; }
	static constexpr double SimPropCullM = 300.0;
	double GetBuildSeconds() const { return BuildSeconds; }

private:
	UProceduralMeshComponent* NewMesh(const TCHAR* Profile, bool bCastShadow);
	/** One instanced prop: Transforms in UE space, culled beyond CullM metres. */
	UHierarchicalInstancedStaticMeshComponent* AddInstances(UStaticMesh* Mesh, const TArray<FTransform>& Transforms, double CullM, bool bCastShadow);
	void HideFloors();
	void RestoreFloors();

	UPROPERTY()
	TArray<TObjectPtr<UProceduralMeshComponent>> Meshes;

	UPROPERTY()
	TArray<TObjectPtr<UHierarchicalInstancedStaticMeshComponent>> Instanced;
	/** Each instanced prop's showcase detail, index for index with Instanced. */
	struct FPropDetail { double CullM = 0.0; bool bCastShadow = false; };
	TArray<FPropDetail> InstancedDetail;
	bool bShowcaseDetail = false;
	void ApplyDetail(int32 Index);

	/** The gantry's lens material, made dynamic so it can be lit. */
	UPROPERTY()
	TObjectPtr<class UMaterialInstanceDynamic> StartLights;
	bool bStartLightsOn = false;

	struct FHiddenFloor
	{
		TWeakObjectPtr<UPrimitiveComponent> Component;
		bool bWasVisible = true;
		ECollisionEnabled::Type Collision = ECollisionEnabled::NoCollision;
	};
	TArray<FHiddenFloor> HiddenFloors;
	TArray<FString> HiddenFloorNames;

	bool bBuilt = false;
	int32 NumTriangles = 0;
	int32 NumInstances = 0;
	int32 NumTrees = 0;
	int64 PropTriangles = 0;
	double BuildSeconds = 0.0;
};
