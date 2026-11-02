#pragma once

#include "CoreMinimal.h"
#include "Containers/Ticker.h"
#include "FSDSReferee.h"

class ACameraActor;
class AFSDSVehiclePawn;
class APlayerController;
class FFSDSLauncher;
class FFSDSRpcServer;
class SFSDSMenu;
class SWidget;
class UWorld;

/** A track the menu can offer: a CSV in one of loadTrack's search dirs. */
struct FFSDSMenuTrack
{
	FString Name;        // file name without .csv
	FString Path;        // absolute
	int32 Blue = 0;
	int32 Yellow = 0;
	int32 Orange = 0;
	bool bDrawn = false; // has a .track.json (made with the track creator)
};

/** What START asks for. */
struct FFSDSDriveRequest
{
	FFSDSMenuTrack Track;
	EFSDSEventType Event = EFSDSEventType::Autocross;
	int32 Laps = 1;
	bool bAutonomous = false;
};

/** One manual run, for the Results page. */
struct FFSDSRunResult
{
	FString Track;
	EFSDSEventType Event = EFSDSEventType::Autocross;
	TArray<float> Laps;
	int32 RequiredLaps = 1;
	int32 ConesDown = 0;
	int32 OffCourse = 0;
	bool bFinished = false;
	FDateTime When;
};

/**
 * The front-end menu: what the sim opens on. Pick an event, a track and who
 * drives, then START. A manual run needs nothing but the sim (no Docker);
 * Esc pauses it, and the run's laps and cones land on the Results page.
 *
 * Off with -NoMenu, -RenderOffscreen, "Menu": {"Enabled": false}, or no
 * viewport. It also steps aside when a session arrives from outside (a
 * client loads a track or resets the scenario over RPC), so the backend,
 * the pipeline and the benchmark run as before.
 */
class FSDSPLUGIN_API FFSDSMenu
{
public:
	FFSDSMenu();
	~FFSDSMenu();

	void Init(UWorld* InWorld, AFSDSReferee* InReferee, AFSDSVehiclePawn* InPawn,
		FFSDSRpcServer* InRpc, FFSDSLauncher* InLauncher);
	void Shutdown();

	bool IsEnabled() const { return bEnabled; }
	bool IsOpen() const { return Root.IsValid(); }

	/** Esc while driving: pause a manual run, or bring the menu back. */
	void OnEscape();

	static FString EventName(EFSDSEventType Event);

private:
	enum class EMode : uint8 { Main, Pause, Finished };

	void Show(EMode Mode);
	void Hide();
	bool Tick(float DeltaTime);

	void ScanTracks();
	void Drive(const FFSDSDriveRequest& Request);
	void Resume();
	void Restart();
	void BackToMain();
	void Quit();
	void RecordRun();
	void HoldCar();

	void StartOrbit();
	void StopOrbit();
	void UpdateOrbit(float DeltaTime);

	APlayerController* GetPC() const;
	bool IsManualRunLive() const;

	bool bEnabled = false;
	bool bShowOnFirstTick = false;
	EMode Mode = EMode::Main;

	TWeakObjectPtr<UWorld> World;
	TWeakObjectPtr<AFSDSReferee> Referee;
	TWeakObjectPtr<AFSDSVehiclePawn> Pawn;
	FFSDSRpcServer* Rpc = nullptr;
	FFSDSLauncher* Launcher = nullptr;

	TSharedPtr<SFSDSMenu> Widget;
	TSharedPtr<SWidget> Root;
	FTSTicker::FDelegateHandle TickHandle;

	TArray<FFSDSMenuTrack> Tracks;
	TSharedPtr<TArray<FFSDSRunResult>> Results;

	/** The last START, kept for Restart and to preselect the menu. */
	TOptional<FFSDSDriveRequest> Current;
	bool bRunLive = false;
	bool bRunRecorded = false;
	bool bPausedGame = false;

	uint32 SeenExternalGen = 0;

	TWeakObjectPtr<ACameraActor> OrbitCam;
	float OrbitAngle = 0.f;
};
