#include "Sensors/FSDSCameraSensor.h"
#include "IImageWrapperModule.h"
#include "IImageWrapper.h"
#include "Engine/TextureRenderTarget2D.h"

UFSDSCameraSensor::UFSDSCameraSensor()
{
	PrimaryComponentTick.bCanEverTick = true;
	bCaptureEveryFrame = false; // We capture on demand, not every frame
	bCaptureOnMovement = false;
}

void UFSDSCameraSensor::BeginPlay()
{
	Super::BeginPlay();
	InitializeRenderTarget();
}

void UFSDSCameraSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

void UFSDSCameraSensor::InitializeRenderTarget()
{
	if (!TextureTarget)
	{
		TextureTarget = NewObject<UTextureRenderTarget2D>(this);
		TextureTarget->InitAutoFormat(ImageWidth, ImageHeight);
		TextureTarget->RenderTargetFormat = RTF_RGBA8;
		TextureTarget->bAutoGenerateMips = false;
		UE_LOG(LogTemp, Log, TEXT("FSDS Camera: Render target initialized (%dx%d)"), ImageWidth, ImageHeight);
	}
}

TArray<FColor> UFSDSCameraSensor::CaptureImageRaw(int32& OutWidth, int32& OutHeight)
{
	TArray<FColor> Pixels;

	if (!TextureTarget)
	{
		InitializeRenderTarget();
	}

	// Trigger a capture
	CaptureScene();

	// Read pixels from render target
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

TArray<uint8> UFSDSCameraSensor::CaptureImagePNG()
{
	TArray<uint8> PNGData;

	int32 Width, Height;
	TArray<FColor> Pixels = CaptureImageRaw(Width, Height);

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
