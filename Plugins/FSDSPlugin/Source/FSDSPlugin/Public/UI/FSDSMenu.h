#pragma once

#include "CoreMinimal.h"
#include "FSDSRealTime.h"
#include "Containers/Ticker.h"
#include "FSDSReferee.h"

namespace FSDSVenue { struct FVenue; }

class ACameraActor;
class FFSDSMissionControlClient;
namespace FSDSTrackSpec { struct FEditorState; }
class AFSDSVehiclePawn;
class APlayerController;
class FFSDSLauncher;
class FFSDSStackStarter;
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
	bool bClosed = true; // a loop; the .track.json says when it isn't
	FString Venue;       // the venue it is drawn on (docs/venues.md), from a <venue>__ name
};

/** What START asks for. */
struct FFSDSDriveRequest
{
	FFSDSMenuTrack Track;
	EFSDSEventType Event = EFSDSEventType::Autocross;
	int32 Laps = 1;
	bool bPractice = false;   // any track, no lap target, no results box
	bool bAutonomous = false;
};

/** One run, manual or autonomous, for the Results page. */
struct FFSDSRunResult
{
	FString Track;
	EFSDSEventType Event = EFSDSEventType::Autocross;
	TArray<float> Laps;
	int32 RequiredLaps = 1;
	int32 ConesDown = 0;
	int32 OffCourse = 0;
	bool bFinished = false;
	bool bPractice = false;
	bool bAutonomous = false;
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

	/** "Acceleration", ..., or "Practice". */
	static FString EventName(EFSDSEventType Event, bool bPractice = false);

	/**
	 * The frame rate while the menu is open and nothing drives (Settings,
	 * Performance): 60, 30 or 15, kept in the user's GameUserSettings.ini.
	 * Driving always runs at the sim's own rate: every frame is a fixed
	 * physics step, so a lower one would slow the simulation itself.
	 */
	static int32 GetMenuFrameRate();
	static void SetMenuFrameRate(int32 Fps);
	/** While the menu is open and the window is minimised. */
	static constexpr int32 MinimisedFrameRate = 5;

private:
	enum class EMode : uint8 { Main, Pause, Finished };

	void Show(EMode Mode);
	void Hide();
	bool Tick(float DeltaTime);

	void ScanTracks();
	void Drive(const FFSDSDriveRequest& Request);
	/**
	 * START with the pipeline driving, all from the menu: through Mission
	 * Control's backend, as the panel and the mission benchmark do. Stop
	 * any session, load the picked track (cones, car on the start gate,
	 * referee cleared), then start the event's mission; the menu shows each
	 * step under START and closes once the pipeline has the car.
	 */
	void StartAutonomous(const FFSDSDriveRequest& Request, bool bStackTried = false);
	/** StartAutonomous's steps through the backend, once the code is synced. */
	void RunSessionSteps(const FFSDSDriveRequest& Request, bool bStackTried);
	/**
	 * Copy the pipeline's source from the team's code into the running
	 * container (the folders compose watch syncs: docker compose config
	 * says which), then Then. So an edit runs on the next START without
	 * compose watch, and the container's copy never goes stale. A failure
	 * (no stack yet, Docker down) just goes on to Then.
	 */
	void SyncPipelineSource(TFunction<void()> Then);

public:
	/** Settings, Rebuild pipeline: build the pipeline and backend images
	 *  from the team's code and recreate their containers (C++, messages,
	 *  launch files, setup.py; after a git pull). */
	void RebuildPipeline();
	bool IsRebuilding() const;
	/** What the rebuild is doing, or how it ended; empty before the first. */
	FString GetRebuildStatus() const { return RebuildStatus; }
	bool RebuildFailed() const { return bRebuildFailed; }

private:
	void TickRebuild();
	/**
	 * START found no backend: bring up the pipeline and the backend
	 * (docker compose, opening Docker Desktop first if its engine is down),
	 * wait until the backend sees the sim, then START again. False when
	 * there is no docker-compose.yml to start them from.
	 */
	bool BringUpStack(const FFSDSDriveRequest& Request);
	/** docker compose for the services START needs (building them the first time). */
	void StartCompose();
	/** The answer to START's question when the team's code isn't on this PC
	 *  (SFSDSMenu::SetRepoPrompt): 0 get it here, 1 somewhere else, 2 I have
	 *  it, 3 not now. */
	void OnRepoChoice(int32 Choice);
	/** git clone into Dir, installing git first if it isn't there. */
	void StartClone(const FString& Dir);
	void ShowRepoPrompt(const FString& Error = FString());
	void TickStackUp();
	/** BringUpStack is over: START again, or show why not (Error). */
	void EndStackUp(const FString& Error);
	/** Stop the pipeline's session (the car stays where it is, under EBS). */
	void StopSession();
	/** The mission Mission Control runs for an event (/api/event/start). */
	static FString MissionName(EFSDSEventType Event);
	/** The track editor's Test drive: the saved track, in Practice. */
	void TestDrive(const FString& TrackName);
	void StartStack();
	void Resume();
	void Restart();
	void BackToMain();
	void Quit();
	void RecordRun();
	void HoldCar();

	/** Cap the frame rate while the menu is up and nothing drives; the sim's own otherwise. */
	void UpdateFrameCap();
	/** Under a cap: sleep out the rest of the frame. */
	void ThrottleFrame();

	void StartOrbit();
	void StopOrbit();
	void UpdateOrbit(float DeltaTime);

	APlayerController* GetPC() const;
	bool IsManualRunLive() const;
	bool IsAutonomousRunLive() const;

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
	TArray<TSharedRef<const FSDSVenue::FVenue>> Venues;
	TSharedPtr<TArray<FFSDSRunResult>> Results;

	/** The track editor's backend (tools/envgen builds the cones) and its state. */
	TSharedPtr<FFSDSMissionControlClient> Client;
	TSharedPtr<FSDSTrackSpec::FEditorState> Editor;
	/** The repo's Content/tracks, where the backend saves: listed first. */
	FString RepoTracksDir;
	/** After a test drive, Main menu goes back to the editor. */
	bool bReturnToEditor = false;

	/** The last START, kept for Restart and to preselect the menu. */
	TOptional<FFSDSDriveRequest> Current;
	bool bRunLive = false;
	bool bRunRecorded = false;
	bool bPausedGame = false;

	uint32 SeenExternalGen = 0;

	/** While StartAutonomous runs its steps: the backend's own track load isn't an outside session. */
	bool bStartingSession = false;

	/** BringUpStack's steps. */
	enum class EStackUp : uint8 { Idle, NeedRepo, GitInstall, Clone, Compose, Docker, Backend, SyncConfig, Sync };
	EStackUp StackUp = EStackUp::Idle;
	TSharedPtr<FFSDSStackStarter> StackStarter;
	/** The folder with docker-compose.yml; empty when none was found. */
	FString ComposeDir;
	TOptional<FFSDSDriveRequest> StackUpRequest;
	double StackUpSince = 0.0;
	double DockerOpenedAt = 0.0;
	double StackUpNextTry = 0.0;
	bool bStackProbe = false;
	/** Where StartClone puts the team's code. */
	FString CloneDir;
	/** What SyncPipelineSource does once the code is in the container. */
	TFunction<void()> AfterSync;

	TSharedPtr<FFSDSStackStarter> Rebuilder;
	FString RebuildStatus;
	bool bRebuildFailed = false;
	/** Gone with the menu (Shutdown): the backend's replies check it before touching anything. */
	TSharedPtr<bool> Alive;

	/** The cap now in force; 0 for none. */
	int32 AppliedCap = 0;
	FFSDSFramePacer MenuPacer;

	TWeakObjectPtr<ACameraActor> OrbitCam;
	float OrbitAngle = 0.f;
};
