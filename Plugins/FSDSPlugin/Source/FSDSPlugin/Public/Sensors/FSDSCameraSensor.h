#pragma once

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "FSDSCameraSensor.generated.h"

/**
 * Camera sensor — captures scene images via SceneCaptureComponent2D.
 * Attach to vehicle pawn. Reads pixels back to CPU as PNG bytes.
 */
UCLASS(ClassGroup=(FSDS), meta=(BlueprintSpawnableComponent))
class FSDSPLUGIN_API UFSDSCameraSensor : public USceneCaptureComponent2D
{
	GENERATED_BODY()

public:
	UFSDSCameraSensor();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Capture current frame and return as PNG bytes */
	TArray<uint8> CaptureImagePNG();

	/** Capture current frame as raw color data */
	TArray<FColor> CaptureImageRaw(int32& OutWidth, int32& OutHeight);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS Camera")
	int32 ImageWidth = 785;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS Camera")
	int32 ImageHeight = 785;

private:
	void InitializeRenderTarget();
};
