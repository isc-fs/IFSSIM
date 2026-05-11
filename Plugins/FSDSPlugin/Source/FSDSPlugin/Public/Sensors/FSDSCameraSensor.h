#pragma once

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "FSDSSettings.h"
#include "FSDSCameraSensor.generated.h"

/**
 * Camera sensor — captures scene images via SceneCaptureComponent2D.
 * Supports multiple image types: Scene, Depth, Segmentation.
 */
UCLASS(ClassGroup=(FSDS), meta=(BlueprintSpawnableComponent))
class FSDSPLUGIN_API UFSDSCameraSensor : public USceneCaptureComponent2D
{
	GENERATED_BODY()

public:
	UFSDSCameraSensor();

	virtual void BeginPlay() override;

	/** Configure from settings */
	void Configure(const FString& InCameraName, const FFSDSCaptureSettings& Settings);

	/** Capture current frame and return as PNG bytes */
	TArray<uint8> CaptureImagePNG(EFSDSImageType ImageType = EFSDSImageType::Scene);

	/** Capture current frame as raw color data */
	TArray<FColor> CaptureImageRaw(int32& OutWidth, int32& OutHeight);

	/** Get camera name */
	const FString& GetCameraName() const { return CameraName; }

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS Camera")
	int32 ImageWidth = 785;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FSDS Camera")
	int32 ImageHeight = 785;

private:
	void InitializeRenderTarget();
	void ConfigureForImageType(EFSDSImageType ImageType);

	FString CameraName = TEXT("cam1");

	UPROPERTY()
	UTextureRenderTarget2D* DepthRenderTarget = nullptr;

	UPROPERTY()
	UTextureRenderTarget2D* SegmentationRenderTarget = nullptr;
};
