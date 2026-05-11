#include "Sensors/FSDSCameraSensor.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/World.h"

UFSDSCameraSensor::UFSDSCameraSensor()
{
	PrimaryComponentTick.bCanEverTick = false;
	bCaptureEveryFrame = false; // Only capture on demand (CaptureScene called from RPC)
	bCaptureOnMovement = false;
}

void UFSDSCameraSensor::BeginPlay()
{
	Super::BeginPlay();
	InitializeRenderTarget();
}

void UFSDSCameraSensor::Configure(const FString& InCameraName, const FFSDSCaptureSettings& Settings)
{
	CameraName = InCameraName;
	ImageWidth = Settings.Width;
	ImageHeight = Settings.Height;
	FOVAngle = Settings.FOV_Degrees;

	// Projection mode
	if (Settings.bOrthographic)
	{
		ProjectionType = ECameraProjectionMode::Orthographic;
		OrthoWidth = Settings.OrthoWidth * 100.f; // meters to cm
	}

	// Motion blur
	if (Settings.MotionBlurAmount > 0.f)
	{
		ShowFlags.SetMotionBlur(true);
		PostProcessSettings.bOverride_MotionBlurAmount = true;
		PostProcessSettings.MotionBlurAmount = Settings.MotionBlurAmount;
	}
	else
	{
		ShowFlags.SetMotionBlur(false);
	}

	// Auto-exposure
	PostProcessSettings.bOverride_AutoExposureMethod = true;
	PostProcessSettings.bOverride_AutoExposureBias = true;
	PostProcessSettings.AutoExposureBias = Settings.AutoExposureBias;

	UE_LOG(LogTemp, Log, TEXT("FSDS Camera '%s': %dx%d, FOV=%.0f, Type=%d, MotionBlur=%.1f, Ortho=%d"),
		*CameraName, ImageWidth, ImageHeight, FOVAngle, (int)Settings.ImageType,
		Settings.MotionBlurAmount, Settings.bOrthographic ? 1 : 0);
}

void UFSDSCameraSensor::InitializeRenderTarget()
{
	if (!TextureTarget)
	{
		TextureTarget = NewObject<UTextureRenderTarget2D>(this);
		TextureTarget->InitAutoFormat(ImageWidth, ImageHeight);
		TextureTarget->RenderTargetFormat = RTF_RGBA8;
		TextureTarget->bAutoGenerateMips = false;
		UE_LOG(LogTemp, Log, TEXT("FSDS Camera '%s': Render target %dx%d"), *CameraName, ImageWidth, ImageHeight);
	}
}

void UFSDSCameraSensor::ConfigureForImageType(EFSDSImageType ImageType)
{
	switch (ImageType)
	{
	case EFSDSImageType::Scene:
		CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
		break;
	case EFSDSImageType::DepthPlanner:
	case EFSDSImageType::DepthPerspective:
	case EFSDSImageType::DepthVis:
		CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
		break;
	case EFSDSImageType::Segmentation:
		CaptureSource = ESceneCaptureSource::SCS_BaseColor;
		// For proper segmentation, use stencil buffer
		// This is a simplified version — full segmentation needs custom post-process
		break;
	case EFSDSImageType::SurfaceNormals:
		CaptureSource = ESceneCaptureSource::SCS_Normal;
		break;
	default:
		CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
		break;
	}
}

TArray<FColor> UFSDSCameraSensor::CaptureImageRaw(int32& OutWidth, int32& OutHeight)
{
	TArray<FColor> Pixels;

	if (!TextureTarget)
	{
		InitializeRenderTarget();
	}
	if (!TextureTarget) return Pixels;

	CaptureScene();

	FTextureRenderTargetResource* RTResource = TextureTarget->GameThread_GetRenderTargetResource();
	if (RTResource)
	{
		FReadSurfaceDataFlags ReadFlags(RCM_UNorm);
		RTResource->ReadPixels(Pixels, ReadFlags);
		OutWidth = TextureTarget->SizeX;
		OutHeight = TextureTarget->SizeY;
	}

	return Pixels;
}

TArray<uint8> UFSDSCameraSensor::CaptureImagePNG(EFSDSImageType ImageType)
{
	TArray<uint8> PNGData;

	// Switch capture source for the requested image type
	ESceneCaptureSource OriginalSource = CaptureSource;
	ConfigureForImageType(ImageType);

	int32 Width, Height;
	TArray<FColor> Pixels = CaptureImageRaw(Width, Height);

	// Restore original capture source
	CaptureSource = OriginalSource;

	if (Pixels.Num() > 0)
	{
		IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);

		if (ImageWrapper.IsValid())
		{
			if (ImageWrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor), Width, Height, ERGBFormat::BGRA, 8))
			{
				PNGData = ImageWrapper->GetCompressed();
			}
		}
	}

	return PNGData;
}
