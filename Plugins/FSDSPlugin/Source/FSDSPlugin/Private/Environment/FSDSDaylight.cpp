#include "Environment/FSDSDaylight.h"

#include "Components/DirectionalLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/PostProcessComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/SkyLightComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/SkyLight.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "UObject/UObjectIterator.h"
#include "FSDSConeSpawner.h"
#include "HAL/IConsoleManager.h"

namespace
{
	constexpr float SunLux = 10.f;   // UE's daylight convention

	TAutoConsoleVariable<float> CVarExposure(
		TEXT("fsds.Daylight.Exposure"), -1.f,   // sunlit asphalt at about sRGB 120, as in photos
		TEXT("Exposure compensation in stops for the showcase daylight (manual exposure)."),
		FConsoleVariableDelegate::CreateLambda([](IConsoleVariable*) {
			for (TObjectIterator<AFSDSDaylight> It; It; ++It)
			{
				if (!It->IsTemplate()) It->ApplyExposure();
			}
		}));
}

AFSDSDaylight::AFSDSDaylight()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickInterval = 1.f;   // re-aim when a track with another world loads

	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));

	Atmosphere = CreateDefaultSubobject<USkyAtmosphereComponent>(TEXT("Atmosphere"));
	Atmosphere->SetupAttachment(RootComponent);

	Sky = CreateDefaultSubobject<USkyLightComponent>(TEXT("Sky"));
	Sky->SetupAttachment(RootComponent);
	Sky->SetMobility(EComponentMobility::Movable);
	Sky->SourceType = ESkyLightSourceType::SLS_CapturedScene;
	Sky->bRealTimeCapture = true;
	Sky->bLowerHemisphereIsBlack = false;   // Lumen adds the ground bounce
	Sky->Intensity = 1.f;

	Fog = CreateDefaultSubobject<UExponentialHeightFogComponent>(TEXT("Fog"));
	Fog->SetupAttachment(RootComponent);
	Fog->FogDensity = 0.004f;              // a summer haze, not a mist
	Fog->FogHeightFalloff = 0.2f;

	// Outranks the level's post-process volume, which only sets exposure.
	Exposure = CreateDefaultSubobject<UPostProcessComponent>(TEXT("Exposure"));
	Exposure->SetupAttachment(RootComponent);
	Exposure->bUnbound = true;
	Exposure->Priority = 100.f;
	FPostProcessSettings& P = Exposure->Settings;
	P.bOverride_AutoExposureMethod = true;
	P.AutoExposureMethod = EAutoExposureMethod::AEM_Manual;
	P.bOverride_AutoExposureApplyPhysicalCameraExposure = true;
	P.AutoExposureApplyPhysicalCameraExposure = false;
	P.bOverride_AutoExposureBias = true;
}

void AFSDSDaylight::ApplyExposure()
{
	const float Ev = CVarExposure.GetValueOnGameThread();
	if (Ev == AppliedExposure) return;
	AppliedExposure = Ev;
	Exposure->Settings.AutoExposureBias = Ev;
	UE_LOG(LogTemp, Log, TEXT("FSDS Daylight: exposure %.2f"), Ev);
}

double AFSDSDaylight::XBearing() const
{
	if (UWorld* W = GetWorld())
	{
		if (TActorIterator<AFSDSConeSpawner> It(W); It && It->GetWorldStatus() == TEXT("loaded"))
		{
			return It->GetTrackWorld().XBearingDeg;
		}
	}
	return 0.0;
}

void AFSDSDaylight::AimSun()
{
	UDirectionalLightComponent* L = Sun.Light.Get();
	if (!L) return;
	const double Bx = XBearing();
	AimedBearing = Bx;
	// Towards the sun. Rel is its bearing relative to world +x; a sun to the
	// right of +x has world y < 0, which is UE +Y, so in UE axes:
	const double H = FMath::DegreesToRadians(SunElevationDeg);
	const double Rel = FMath::DegreesToRadians(SunAzimuthDeg - Bx);
	const FVector ToSun(FMath::Cos(H) * FMath::Cos(Rel), FMath::Cos(H) * FMath::Sin(Rel), FMath::Sin(H));
	L->SetWorldRotation((-ToSun).Rotation());
}

void AFSDSDaylight::BeginPlay()
{
	Super::BeginPlay();
	UWorld* W = GetWorld();
	if (!W) return;

	// The level's sun: kept (it carries the shadows), aimed and set to daylight.
	for (TActorIterator<ADirectionalLight> It(W); It; ++It)
	{
		if (UDirectionalLightComponent* L = Cast<UDirectionalLightComponent>(It->GetLightComponent()))
		{
			Sun = {L, L->GetComponentRotation(), L->Intensity, L->bAtmosphereSunLight};
			L->SetIntensity(SunLux);
			L->SetAtmosphereSunLight(true);
			break;
		}
	}
	AimSun();
	ApplyExposure();

	// The level's baked sky light and legacy sky: hidden while this is up.
	for (TActorIterator<ASkyLight> It(W); It; ++It)
	{
		if (USkyLightComponent* S = It->GetLightComponent(); S && S->IsVisible())
		{
			S->SetVisibility(false);
			HiddenComponents.Add(S);
		}
	}
	for (TActorIterator<AActor> It(W); It; ++It)
	{
		const FString Class = It->GetClass()->GetName();
		if (*It != this && !It->IsHidden() && (Class.Contains(TEXT("AtmosphericFog")) || Class.Contains(TEXT("Sky_Sphere"))))
		{
			It->SetActorHiddenInGame(true);
			HiddenActors.Add(*It);
		}
	}
	UE_LOG(LogTemp, Log, TEXT("FSDS Daylight: on (sun %.1f deg up at bearing %.1f; world +x bearing %.1f); hid %d actors, %d sky lights"),
		SunElevationDeg, SunAzimuthDeg, AimedBearing, HiddenActors.Num(), HiddenComponents.Num());
}

void AFSDSDaylight::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (XBearing() != AimedBearing) AimSun();
}

void AFSDSDaylight::EndPlay(const EEndPlayReason::Type Reason)
{
	if (UDirectionalLightComponent* L = Sun.Light.Get())
	{
		L->SetWorldRotation(Sun.Rotation);
		L->SetIntensity(Sun.Intensity);
		L->SetAtmosphereSunLight(Sun.bAtmosphereSunLight);
	}
	for (const TWeakObjectPtr<USceneComponent>& C : HiddenComponents)
	{
		if (C.IsValid()) C->SetVisibility(true);
	}
	for (const TWeakObjectPtr<AActor>& A : HiddenActors)
	{
		if (A.IsValid()) A->SetActorHiddenInGame(false);
	}
	HiddenComponents.Reset();
	HiddenActors.Reset();
	Super::EndPlay(Reason);
}
