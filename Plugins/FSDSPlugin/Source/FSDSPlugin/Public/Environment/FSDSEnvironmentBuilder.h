#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Environment/FSDSEnvironment.h"
#include "FSDSEnvironmentBuilder.generated.h"

class UInstancedStaticMeshComponent;
class UStaticMesh;

/**
 * The props of a track's environment, as the sidecar lists them
 * (docs/environment_sidecar.md).
 *
 * Every prop class is built from primitives: engine basic shapes and the
 * existing small-cone mesh, one instanced component per mesh, all with the
 * FSDSProp collision profile (docs/collision_matrix.md): the LiDAR sees them
 * and the car hits them, but they are never ground. Nothing here is random;
 * the sidecar says where everything goes (roadmap rule 3), and each prop
 * stands on the ground found by a WorldStatic trace, as the cones do.
 *
 * The cone spawner owns one, and rebuilds it with every track, before the
 * cones spawn. With Environment.Enabled off it is never created.
 *
 * The class names and footprints must match tools/envgen/catalogue.py
 * (checked by tools/envgen/tests).
 */
UCLASS(NotPlaceable)
class FSDSPLUGIN_API AFSDSEnvironmentActor : public AActor
{
	GENERATED_BODY()

public:
	AFSDSEnvironmentActor();

	/** Replace whatever was built with Env's props. Returns how many were built. */
	int32 Build(const FFSDSEnvironment& Env);

	/** Remove every prop. */
	void Clear();

	/** Props built, per class. */
	const TMap<FString, int32>& GetBuiltCounts() const { return Built; }
	int32 GetNumBuilt() const { return NumBuilt; }

	/** Props skipped: an unknown class, or no ground under them. */
	int32 GetNumSkipped() const { return NumSkipped; }
	const TArray<FString>& GetUnknownClasses() const { return UnknownClasses; }

	/** SHA-1 of the built props' transforms (class, then x, y, z in mm and yaw
	 *  in tenths of a degree, in sidecar order). The same sidecar on the same
	 *  ground gives the same hash on every boot, whatever the scenario seed. */
	const FString& GetTransformHash() const { return TransformHash; }

	/** The class names this build knows. */
	static TArray<FString> KnownClasses();

private:
	UInstancedStaticMeshComponent* PartComponent(UStaticMesh* Mesh);

	UPROPERTY()
	TMap<TObjectPtr<UStaticMesh>, TObjectPtr<UInstancedStaticMeshComponent>> Parts;

	TMap<FString, int32> Built;
	int32 NumBuilt = 0;
	int32 NumSkipped = 0;
	TArray<FString> UnknownClasses;
	FString TransformHash;
};
