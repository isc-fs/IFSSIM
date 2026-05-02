#include "Sensors/FSDSLidarGPUSpike.h"
#include "Sensors/FSDSLidarSensor.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Kismet/KismetRenderingLibrary.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/Paths.h"

static TAutoConsoleVariable<int32> CVarLidarGPUSpikeEnable(
	TEXT("fsds.LidarGPUSpike.Enable"),
	0,
	TEXT("Phase-0 GPU LiDAR spike (#223). 0=off (default), 1=on.\n")
	TEXT("Creates a depth-only SceneCapture at the LiDAR pose for cost measurement."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarLidarGPUSpikeRTW(
	TEXT("fsds.LidarGPUSpike.RTW"),
	2048,
	TEXT("Phase-0 spike depth RT width (default 2048). #223 Phase 0."),
	ECVF_Default);

static TAutoConsoleVariable<int32> CVarLidarGPUSpikeRTH(
	TEXT("fsds.LidarGPUSpike.RTH"),
	256,
	TEXT("Phase-0 spike depth RT height (default 256). #223 Phase 0."),
	ECVF_Default);

UFSDSLidarGPUSpike::UFSDSLidarGPUSpike()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UFSDSLidarGPUSpike::BeginPlay()
{
	Super::BeginPlay();
}

void UFSDSLidarGPUSpike::Initialize(const UFSDSLidarSensor* SourceLidar)
{
	if (CVarLidarGPUSpikeEnable.GetValueOnGameThread() == 0)
	{
		UE_LOG(LogTemp, Log,
			TEXT("FSDS LiDAR GPU Spike: disabled (fsds.LidarGPUSpike.Enable=0)"));
		return;
	}
	if (!SourceLidar)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("FSDS LiDAR GPU Spike: no source LiDAR; aborting init"));
		return;
	}
	AActor* Owner = GetOwner();
	if (!Owner) return;

	const int32 RTW  = FMath::Max(64, CVarLidarGPUSpikeRTW.GetValueOnGameThread());
	const int32 RTH  = FMath::Max(64, CVarLidarGPUSpikeRTH.GetValueOnGameThread());
	const float HFov = SourceLidar->HorizontalFOVEnd - SourceLidar->HorizontalFOVStart;

	// Depth-only render target. R32f gives ~mm precision at 30 m range,
	// well below the LiDAR's 3 cm σ noise floor — won't be the limiting
	// factor on point quality. Bandwidth at 2048×256×4B = 2 MB / scan
	// = 20 MB/s at 10 Hz, trivial.
	DepthRT = NewObject<UTextureRenderTarget2D>(this);
	DepthRT->RenderTargetFormat = ETextureRenderTargetFormat::RTF_R32f;
	DepthRT->ClearColor          = FLinearColor::Black;
	DepthRT->bAutoGenerateMips   = false;
	DepthRT->InitAutoFormat(RTW, RTH);
	DepthRT->UpdateResourceImmediate(true);

	// Capture component pinned to the LiDAR's mount pose. Owner-relative
	// because SensorOffset is expressed in the vehicle's local frame.
	CaptureComponent = NewObject<USceneCaptureComponent2D>(Owner);
	CaptureComponent->SetupAttachment(Owner->GetRootComponent());
	CaptureComponent->RegisterComponent();
	CaptureComponent->SetRelativeLocation(SourceLidar->SensorOffset);
	CaptureComponent->SetRelativeRotation(FRotator::ZeroRotator);
	CaptureComponent->TextureTarget         = DepthRT;
	CaptureComponent->CaptureSource         = ESceneCaptureSource::SCS_SceneDepth;
	CaptureComponent->bCaptureEveryFrame    = false;
	CaptureComponent->bCaptureOnMovement    = false;
	CaptureComponent->bAlwaysPersistRenderingState = true;

	// USceneCapture's FOVAngle is *horizontal* FOV; vertical FOV is
	// implicit via aspect ratio (FOVAngle / aspect). At 2048×256 H-FOV
	// 120° gives V-FOV ≈ 15°, close to the Hesai's 18.3° spec — good
	// enough for the Phase-0 cost measurement. Asymmetric V-FOV via
	// CustomProjectionMatrix lands in Phase 1.
	CaptureComponent->FOVAngle = HFov;

	// Strip every irrelevant feature off the capture pass — we only
	// need scene depth, not lit colour, post-processing, or AA. AA
	// edge-blending in particular would create fake intermediate-depth
	// hits at cone silhouettes (risk #3 in the #223 design); turning
	// it off here is the right move both for cost and correctness.
	CaptureComponent->ShowFlags.SetAntiAliasing(false);
	CaptureComponent->ShowFlags.SetTemporalAA(false);
	CaptureComponent->ShowFlags.SetMotionBlur(false);
	CaptureComponent->ShowFlags.SetBloom(false);
	CaptureComponent->ShowFlags.SetTonemapper(false);
	CaptureComponent->ShowFlags.SetEyeAdaptation(false);
	CaptureComponent->ShowFlags.SetVignette(false);
	CaptureComponent->ShowFlags.SetGrain(false);
	CaptureComponent->ShowFlags.SetLensFlares(false);
	CaptureComponent->ShowFlags.SetScreenSpaceReflections(false);
	CaptureComponent->ShowFlags.SetReflectionEnvironment(false);
	CaptureComponent->ShowFlags.SetAmbientOcclusion(false);

	ScanIntervalSeconds = 1.f / FMath::Max(1.f, SourceLidar->RotationsPerSecond);

	UE_LOG(LogTemp, Log,
		TEXT("FSDS LiDAR GPU Spike: ENABLED  RT=%dx%d (R32f, depth-only)  H-FOV=%.1f°  rate=%.1f Hz"),
		RTW, RTH, HFov, 1.f / ScanIntervalSeconds);
}

void UFSDSLidarGPUSpike::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (!CaptureComponent) return;

	ScanAccumulator += DeltaTime;
	if (ScanAccumulator < ScanIntervalSeconds) return;
	ScanAccumulator -= ScanIntervalSeconds;

	const double T0 = FPlatformTime::Seconds();
	CaptureComponent->CaptureScene();
	const double T1 = FPlatformTime::Seconds();

	CapAccumulatorMs += (T1 - T0) * 1000.0;
	CapSampleCount++;
	CaptureCount++;

	// Rolling avg every ~5 s. The captured number is *game-thread*
	// cost (CaptureScene blocks the GT until view-family setup is done
	// and the render-thread queue accepts the work). The actual GPU
	// pass cost shows up under `stat gpu` as SceneCaptures, captured
	// separately via tools/measure_gpu_lidar.sh.
	if (T1 - LastReportTime > 5.0)
	{
		const double AvgMs = CapAccumulatorMs / FMath::Max(1, CapSampleCount);
		UE_LOG(LogTemp, Log,
			TEXT("FSDS LiDAR GPU Spike: avg CaptureScene() game-thread = %.3f ms over %d samples"),
			AvgMs, CapSampleCount);
		CapAccumulatorMs = 0.0;
		CapSampleCount   = 0;
		LastReportTime   = T1;
	}

	// One-shot RT dump for visual sanity check. EXR for float depth —
	// import in any HDR-aware viewer; nearby cones should appear as
	// distinct bright bars over a darker road background.
	if (!bDumpedRT && CaptureCount >= DumpAfterNCaptures && DepthRT)
	{
		const FString OutDir  = FPaths::ProjectSavedDir() / TEXT("LidarGPUSpike");
		const FString OutName = TEXT("DepthRT");
		UKismetRenderingLibrary::ExportRenderTarget(GetWorld(), DepthRT, OutDir, OutName + TEXT(".exr"));
		UE_LOG(LogTemp, Log,
			TEXT("FSDS LiDAR GPU Spike: dumped depth RT after capture #%d → %s/%s.exr"),
			CaptureCount, *OutDir, *OutName);
		bDumpedRT = true;
	}
}

void UFSDSLidarGPUSpike::EndPlay(const EEndPlayReason::Type Reason)
{
	if (CaptureComponent)
	{
		CaptureComponent->DestroyComponent();
		CaptureComponent = nullptr;
	}
	DepthRT = nullptr;
	Super::EndPlay(Reason);
}
