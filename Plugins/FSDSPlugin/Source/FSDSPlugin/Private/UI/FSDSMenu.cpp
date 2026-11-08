#include "UI/FSDSMenu.h"

#include "FSDSReferee.h"
#include "FSDSSettings.h"
#include "FSDSVehiclePawn.h"
#include "RPC/FSDSRpcServer.h"
#include "UI/FSDSLauncher.h"
#include "UI/SFSDSMenu.h"
#include "UI/FSDSMissionControlClient.h"
#include "UI/FSDSStackStarter.h"
#include "UI/FSDSTrackSpec.h"
#include "UI/FSDSVenue.h"
#include "UI/FSDSGraphics.h"

#include "Camera/CameraActor.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Engine/GameViewportClient.h"
#include "Engine/Engine.h"
#include "Misc/ConfigCacheIni.h"
#include "Widgets/SWindow.h"
#include "Engine/World.h"
#include "Framework/Application/SlateApplication.h"
#include "GameFramework/PlayerController.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformProcess.h"
#include "Misc/App.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"

namespace
{
	// Over the HUD and the Mission Control panel.
	constexpr int32 MenuZOrder = 60;

	// The menu's camera: circles the car while the front end is up.
	constexpr float OrbitRadiusCm = 650.f;
	constexpr float OrbitHeightCm = 220.f;
	constexpr float OrbitLookUpCm = 60.f;
	constexpr float OrbitRadPerS = 0.12f;

	// Practice has no lap target; the referee needs one, so a big one.
	constexpr int32 PracticeLaps = 99;

	// The menu frame rate's home in GameUserSettings.ini.
	const TCHAR* PrefsSection = TEXT("FSDSMenu");
	const TCHAR* FrameRateKey = TEXT("FrameRate");
	constexpr int32 DefaultMenuFrameRate = 30;
}

namespace
{
	// Read once, then kept: the menu asks every frame.
	int32 CachedMenuFrameRate = 0;
}

int32 FFSDSMenu::GetMenuFrameRate()
{
	if (CachedMenuFrameRate == 0)
	{
		int32 Fps = DefaultMenuFrameRate;
		if (GConfig) GConfig->GetInt(PrefsSection, FrameRateKey, Fps, GGameUserSettingsIni);
		CachedMenuFrameRate = (Fps == 60 || Fps == 30 || Fps == 15) ? Fps : DefaultMenuFrameRate;
	}
	return CachedMenuFrameRate;
}

void FFSDSMenu::SetMenuFrameRate(int32 Fps)
{
	CachedMenuFrameRate = Fps;
	if (!GConfig) return;
	GConfig->SetInt(PrefsSection, FrameRateKey, Fps, GGameUserSettingsIni);
	GConfig->Flush(false, GGameUserSettingsIni);
}

FFSDSMenu::FFSDSMenu() = default;

FFSDSMenu::~FFSDSMenu()
{
	Shutdown();
}

FString FFSDSMenu::EventName(EFSDSEventType Event, bool bPractice)
{
	if (bPractice) return TEXT("Practice");
	switch (Event)
	{
	case EFSDSEventType::Acceleration: return TEXT("Acceleration");
	case EFSDSEventType::Skidpad:      return TEXT("Skidpad");
	case EFSDSEventType::Autocross:    return TEXT("Autocross");
	case EFSDSEventType::Trackdrive:   return TEXT("Trackdrive");
	default:                           return TEXT("Unknown");
	}
}

void FFSDSMenu::Init(UWorld* InWorld, AFSDSReferee* InReferee, AFSDSVehiclePawn* InPawn,
	FFSDSRpcServer* InRpc, FFSDSLauncher* InLauncher)
{
	World = InWorld;
	Referee = InReferee;
	Pawn = InPawn;
	Rpc = InRpc;
	Launcher = InLauncher;

	const TCHAR* Cmd = FCommandLine::Get();
	bEnabled = FFSDSSettings::Get().bMenuEnabled
		&& FApp::CanEverRender()
		&& !FParse::Param(Cmd, TEXT("NoMenu"))
		&& !FParse::Param(Cmd, TEXT("RenderOffscreen"))
		&& InWorld && InWorld->GetGameViewport() != nullptr
		&& FSlateApplication::IsInitialized()
		&& InRpc != nullptr;
	if (!bEnabled)
	{
		UE_LOG(LogTemp, Log, TEXT("FSDS Menu: off (-NoMenu, -RenderOffscreen, no viewport, or Menu.Enabled=false)"));
		return;
	}

	Results = MakeShared<TArray<FFSDSRunResult>>();
	SeenExternalGen = Rpc->GetExternalSessionGeneration();
	Alive = MakeShared<bool>(true);

	// The track editor talks to the backend as the panel does.
	const FFSDSSettings& S = FFSDSSettings::Get();
	FString ApiKey = S.MissionControlApiKey;
	if (ApiKey.IsEmpty()) ApiKey = FPlatformMisc::GetEnvironmentVariable(TEXT("IFSSIM_MC_API_KEY"));
	Client = MakeShared<FFSDSMissionControlClient>(S.MissionControlUrl, ApiKey);
	Editor = MakeShared<FSDSTrackSpec::FEditorState>();

	// The backend saves into the repo's Content/tracks (docker-compose mounts
	// it); a packaged sim reads its own copy. List the repo's first, so a
	// track saved in the editor shows up and loads.
	ScanTracks();

	// Shown on the first tick, once the controller has its pawn and camera.
	bShowOnFirstTick = true;
	TickHandle = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateRaw(this, &FFSDSMenu::Tick));
	UE_LOG(LogTemp, Log, TEXT("FSDS Menu: on, %d tracks"), Tracks.Num());
}

void FFSDSMenu::Shutdown()
{
	if (TickHandle.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(TickHandle);
		TickHandle.Reset();
	}
	Hide();
	StopOrbit();
	UpdateFrameCap();   // closed: back to the sim's own rate
	Alive.Reset();      // replies still in flight find nothing to update
	bEnabled = false;
}

// ---------------------------------------------------------------------------

void FFSDSMenu::ScanTracks()
{
	// The backend saves into the repo's Content/tracks (docker-compose mounts
	// it); a packaged sim reads its own copy. List the repo's first, so a
	// track saved in the editor shows up and loads. Found again each time:
	// the menu may have just cloned it.
	RepoTracksDir.Reset();
	const FString Repo = FFSDSStackStarter::FindComposeDir();
	if (!Repo.IsEmpty())
	{
		const FString Dir = FPaths::Combine(Repo, TEXT("Content"), TEXT("tracks"));
		if (FPaths::DirectoryExists(Dir)) RepoTracksDir = Dir;
	}

	Tracks.Reset();
	TSet<FString> Seen;
	TArray<FString> Dirs = FFSDSRpcServer::TrackSearchDirs();
	if (!RepoTracksDir.IsEmpty()) Dirs.Insert(RepoTracksDir, 0);
	Venues = FSDSVenue::Scan(Dirs);
	for (const FString& Dir : Dirs)
	{
		TArray<FString> Files;
		IFileManager::Get().FindFiles(Files, *FPaths::Combine(Dir, TEXT("*.csv")), true, false);
		for (const FString& File : Files)
		{
			const FString Name = FPaths::GetBaseFilename(File);
			if (Seen.Contains(Name)) continue;   // the first dir wins, as in loadTrack
			Seen.Add(Name);

			FFSDSMenuTrack T;
			T.Name = Name;
			const FString VenueName = FSDSVenue::VenueOfTrack(Name);
			if (FSDSVenue::Find(Venues, VenueName)) T.Venue = VenueName;
			T.Path = FPaths::ConvertRelativePathToFull(FPaths::Combine(Dir, File));
			// A drawn track's spec says whether it is a loop; a track without
			// one (the FS events, generated tracks) is taken as one.
			const FString SpecPath = FPaths::Combine(Dir, Name + TEXT(".track.json"));
			FString Spec;
			if (FFileHelper::LoadFileToString(Spec, *SpecPath))
			{
				T.bDrawn = true;
				TSharedPtr<FJsonObject> Obj;
				if (FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Spec), Obj) && Obj.IsValid())
				{
					Obj->TryGetBoolField(TEXT("closed"), T.bClosed);
				}
			}
			TArray<FString> Lines;
			FFileHelper::LoadFileToStringArray(Lines, *T.Path);
			for (const FString& Line : Lines)
			{
				if (Line.StartsWith(TEXT("blue"))) ++T.Blue;
				else if (Line.StartsWith(TEXT("yellow"))) ++T.Yellow;
				else if (Line.Contains(TEXT("orange"))) ++T.Orange;
			}
			if (T.Blue + T.Yellow + T.Orange > 0)
			{
				Tracks.Add(MoveTemp(T));
			}
		}
	}
	Tracks.Sort([](const FFSDSMenuTrack& A, const FFSDSMenuTrack& B) { return A.Name < B.Name; });
	if (Widget) Widget->SetTracks(Tracks, Venues);   // the open menu holds its own copy
}

APlayerController* FFSDSMenu::GetPC() const
{
	UWorld* W = World.Get();
	return W ? W->GetFirstPlayerController() : nullptr;
}

bool FFSDSMenu::IsManualRunLive() const
{
	return bRunLive && Current.IsSet() && !Current->bAutonomous;
}

bool FFSDSMenu::IsAutonomousRunLive() const
{
	return bRunLive && Current.IsSet() && Current->bAutonomous;
}

FString FFSDSMenu::MissionName(EFSDSEventType Event)
{
	// tools/sim_benchmark/missions.yaml's mc_name, and the panel's missions.
	switch (Event)
	{
	case EFSDSEventType::Acceleration: return TEXT("accel");
	case EFSDSEventType::Skidpad:      return TEXT("skidpad");
	case EFSDSEventType::Autocross:    return TEXT("autocross");
	default:                           return TEXT("trackdrive");
	}
}

// ---------------------------------------------------------------------------

void FFSDSMenu::Show(EMode InMode)
{
	UWorld* W = World.Get();
	UGameViewportClient* Viewport = W ? W->GetGameViewport() : nullptr;
	if (!Viewport) return;

	Hide();
	Mode = InMode;
	if (Launcher) Launcher->Close();
	if (Mode == EMode::Main) ScanTracks();   // as the folders are now, not as they were at start-up

	SFSDSMenu::EScreen Screen = SFSDSMenu::EScreen::Main;
	if (Mode == EMode::Pause) Screen = SFSDSMenu::EScreen::Pause;
	if (Mode == EMode::Finished) Screen = SFSDSMenu::EScreen::Finished;

	SAssignNew(Widget, SFSDSMenu)
		.Screen(Screen)
		.Tracks(Tracks)
		.Venues(Venues)
		.Results(Results)
		.Last(Current)
		.Referee(Referee)
		.Client(Client)
		.EditorState(Editor)
		.StartInEditor(Mode == EMode::Main && bReturnToEditor)
		.OnTestDrive(FOnFSDSTestDrive::CreateRaw(this, &FFSDSMenu::TestDrive))
		.OnTracksChanged(FSimpleDelegate::CreateRaw(this, &FFSDSMenu::ScanTracks))
		.OnStartStack(FSimpleDelegate::CreateRaw(this, &FFSDSMenu::StartStack))
		.OnRebuildPipeline(FSimpleDelegate::CreateRaw(this, &FFSDSMenu::RebuildPipeline))
		.RebuildStatus([this]() { return GetRebuildStatus(); })
		.IsRebuilding([this]() { return IsRebuilding(); })
		.RebuildFailed([this]() { return RebuildFailed(); })
		.LauncherKey(FFSDSLauncher::ToggleKey().GetDisplayName())
		.OnDrive(FOnFSDSDrive::CreateRaw(this, &FFSDSMenu::Drive))
		.OnResume(FSimpleDelegate::CreateRaw(this, &FFSDSMenu::Resume))
		.OnRestart(FSimpleDelegate::CreateRaw(this, &FFSDSMenu::Restart))
		.OnMainMenu(FSimpleDelegate::CreateRaw(this, &FFSDSMenu::BackToMain))
		.OnQuit(FSimpleDelegate::CreateRaw(this, &FFSDSMenu::Quit));
	Root = Widget;
	Viewport->AddViewportWidgetContent(Root.ToSharedRef(), MenuZOrder);
	if (Mode == EMode::Main) bReturnToEditor = false;

	if (APlayerController* PC = GetPC())
	{
		FInputModeUIOnly InputMode;
		InputMode.SetWidgetToFocus(Widget);
		InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
		PC->SetInputMode(InputMode);
		PC->SetShowMouseCursor(true);

		// Pause a manual run only: an autonomous one keeps its pipeline's
		// clock, and the front end needs the world ticking for its camera.
		if (Mode == EMode::Pause && IsManualRunLive())
		{
			bPausedGame = PC->SetPause(true);
		}
	}
	FSlateApplication::Get().SetKeyboardFocus(Widget, EFocusCause::SetDirectly);

	if (Mode == EMode::Main)
	{
		StartOrbit();
	}
}

void FFSDSMenu::Hide()
{
	if (!Root.IsValid()) return;

	if (UWorld* W = World.Get())
	{
		if (UGameViewportClient* Viewport = W->GetGameViewport())
		{
			Viewport->RemoveViewportWidgetContent(Root.ToSharedRef());
		}
	}
	Root.Reset();
	Widget.Reset();
	StopOrbit();
	UpdateFrameCap();   // at once, before the run's first frame

	if (APlayerController* PC = GetPC())
	{
		if (bPausedGame)
		{
			PC->SetPause(false);
			bPausedGame = false;
		}
		PC->SetInputMode(FInputModeGameOnly());
		PC->SetShowMouseCursor(false);
	}
	if (FSlateApplication::IsInitialized())
	{
		FSlateApplication::Get().SetAllUserFocusToGameViewport();
	}
}

bool FFSDSMenu::Tick(float DeltaTime)
{
	if (!bEnabled) return true;

	if (bShowOnFirstTick)
	{
		bShowOnFirstTick = false;
		if (SeenExternalGen == Rpc->GetExternalSessionGeneration())
		{
			Show(EMode::Main);
		}
	}

	// A session from outside (a track loaded or a scenario reset over RPC):
	// whoever sent it is running the sim now, so get out of the way.
	const uint32 Gen = Rpc->GetExternalSessionGeneration();
	if (Gen != SeenExternalGen && bStartingSession)
	{
		SeenExternalGen = Gen;   // the backend loading the track the menu asked for
	}
	else if (Gen != SeenExternalGen)
	{
		SeenExternalGen = Gen;
		if (bRunLive) RecordRun();
		bRunLive = false;
		if (IsOpen())
		{
			UE_LOG(LogTemp, Log, TEXT("FSDS Menu: a session started over RPC; closing the menu"));
			Hide();
		}
	}

	if (IsOpen() && Mode == EMode::Main)
	{
		UpdateOrbit(DeltaTime);
	}
	TickStackUp();
	TickRebuild();
	UpdateFrameCap();
	ThrottleFrame();

	// Settings, Graphics: the view off while the pipeline drives, to give its
	// GPU and CPU time back (the LiDAR renders on its own). Drawn again
	// whenever the menu is up, so the box over the run has the scene behind it.
	{
		// API control alone isn't driving: the pipeline stack's bridge takes
		// it as soon as it connects, and holds the car under EBS until a
		// session releases it.
		const AFSDSVehiclePawn* P = Pawn.Get();
		const bool bHide = FFSDSGraphics::Get().GetSettings().bHideViewWhileAutonomous
			&& P && P->IsApiControlEnabled() && !P->IsEbsLatched() && !IsOpen();
		FFSDSGraphics::Get().SetViewDrawn(!bHide);
	}

	// The pipeline stopped the car itself (its watchdog, RES, a failure): the
	// EBS latched with the run still on. End the run there, as the referee
	// would, so the box says so instead of "the pipeline is driving".
	if (IsAutonomousRunLive() && !bStartingSession)
	{
		const AFSDSVehiclePawn* EbsPawn = Pawn.Get();
		const AFSDSReferee* EbsRef = Referee.Get();
		const bool bFinishing = EbsRef && EbsRef->GetState().bFinished;   // the referee's own ending, below
		if (EbsPawn && EbsPawn->IsEbsLatched() && !bFinishing)
		{
			const double Now = FPlatformTime::Seconds();
			if (EbsSince == 0.0) EbsSince = Now;
			else if (Now - EbsSince > 1.5)
			{
				EbsSince = 0.0;
				UE_LOG(LogTemp, Log, TEXT("FSDS Menu: the car was stopped (EBS) during the autonomous run"));
				RecordRun();
				StopSession();
				bRunLive = false;
				Show(EMode::Finished);
				return true;
			}
		}
		else
		{
			EbsSince = 0.0;
		}
	}

	// The referee ends the run (all laps done, or the far gate of a one-way
	// track): show how it went. Practice on a loop never ends this way. With
	// the pipeline driving, its session stops too, and the car waits.
	if (bRunLive && Current.IsSet() && (!IsOpen() || IsAutonomousRunLive()))
	{
		const AFSDSReferee* R = Referee.Get();
		if (R && R->GetState().bFinished)
		{
			RecordRun();
			if (IsAutonomousRunLive()) StopSession();
			HoldCar();
			bRunLive = false;
			Show(EMode::Finished);
		}
	}
	return true;
}

// ---------------------------------------------------------------------------
// Actions

void FFSDSMenu::Drive(const FFSDSDriveRequest& Request)
{
	if (bRunLive) RecordRun();

	if (Request.bAutonomous)
	{
		StartAutonomous(Request);
		return;
	}

	AFSDSVehiclePawn* P = Pawn.Get();
	AFSDSReferee* R = Referee.Get();
	if (!P || !R || !Rpc)
	{
		if (Widget) Widget->SetMessage(TEXT("No car or referee in this level."), true);
		return;
	}

	// The loadTrack RPC's own code: cones, referee reset, car on the start gate.
	const FString Reply = Rpc->LoadTrackLocal(Request.Track.Path);
	int32 Cones = 0;
	{
		const int32 At = Reply.Find(TEXT("\"cones\":"));
		if (At != INDEX_NONE) Cones = FCString::Atoi(*Reply.Mid(At + 8));
	}
	if (Reply.Contains(TEXT("\"error\"")) || Cones == 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("FSDS Menu: loading %s failed: %s"), *Request.Track.Path, *Reply);
		if (Widget) Widget->SetMessage(FString::Printf(TEXT("Couldn't load %s."), *Request.Track.Name), true);
		return;
	}

	// After the load: the load resets the referee, and Acceleration starts
	// its timer when the event is set. Practice counts laps but never ends.
	if (Request.bPractice)
	{
		R->SetEventType(EFSDSEventType::Trackdrive, PracticeLaps);
	}
	else
	{
		R->SetEventType(Request.Event, Request.Laps);
	}

	// The driver has the car: EBS off, keyboard in.
	P->ReleaseEbs();
	Rpc->SetApiControl(false);

	Current = Request;
	bRunLive = true;
	bRunRecorded = false;
	Hide();
	UE_LOG(LogTemp, Log, TEXT("FSDS Menu: manual %s on %s (%d cones, %d laps)"),
		*EventName(Request.Event, Request.bPractice), *Request.Track.Name, Cones, Request.Laps);
}

void FFSDSMenu::StartAutonomous(const FFSDSDriveRequest& Request, bool bStackTried)
{
	if (bStartingSession) return;   // already on its way
	if (IsRebuilding())
	{
		if (Widget) Widget->SetMessage(TEXT("The pipeline is being rebuilt (Settings): START when it's done."), true);
		return;
	}
	ComposeDir = FFSDSStackStarter::FindComposeDir();
	if (ComposeDir.IsEmpty() || !Client.IsValid())
	{
		RunSessionSteps(Request, bStackTried);
		return;
	}
	bStartingSession = true;
	const TWeakPtr<bool> Token = Alive;
	SyncPipelineSource([this, Token, Request, bStackTried]()
	{
		if (!Token.IsValid()) return;
		bStartingSession = false;
		RunSessionSteps(Request, bStackTried);
	});
}

void FFSDSMenu::RunSessionSteps(const FFSDSDriveRequest& Request, bool bStackTried)
{
	if (bStartingSession) return;   // already on its way
	if (!Client.IsValid())
	{
		if (Widget) Widget->SetMessage(TEXT("No Mission Control backend configured."), true);
		return;
	}
	bRunLive = false;
	bStartingSession = true;
	const TWeakPtr<bool> Token = Alive;
	const FFSDSDriveRequest Req = Request;
	const FString Csv = Req.Track.Name + TEXT(".csv");
	auto Say = [this, Token](const FString& Msg, bool bError)
	{
		if (Token.IsValid() && Widget) Widget->SetMessage(Msg, bError);
	};
	auto Fail = [this, Token, Say](const FString& Msg)
	{
		if (!Token.IsValid()) return;
		bStartingSession = false;
		Say(Msg, true);
		UE_LOG(LogTemp, Warning, TEXT("FSDS Menu: autonomous start: %s"), *Msg);
	};
	Say(TEXT("Stopping any session that is running (after a run with a bag, saving it can take a few minutes)..."), false);

	// 1. The previous session's pipeline, if any (as the mission benchmark).
	Client->Post(TEXT("/api/pipeline/stop"), nullptr, [this, Token, Req, Csv, Say, Fail, bStackTried](const FFSDSMcReply& Stop)
	{
		if (!Token.IsValid()) return;
		if (!Stop.Reached())
		{
			// Nothing answers: start the stack, then START again.
			if (!bStackTried && BringUpStack(Req)) return;
			Fail(FString::Printf(TEXT("Mission Control's backend isn't reachable at %s: start the Docker stack. (%s)"),
				*Client->GetBaseUrl(), *Stop.Error));
			return;
		}
		Say(FString::Printf(TEXT("Loading %s..."), *Req.Track.Name), false);

		// 2. The track, through the backend: it re-centres it, loads it into
		//    the sim (cones, the car on the start gate, the referee cleared)
		//    and knows which track the session runs on.
		Client->Post(FString::Printf(TEXT("/api/track/%s/load"), *FFSDSMissionControlClient::EncodeSegment(Csv)), nullptr,
			[this, Token, Req, Say, Fail](const FFSDSMcReply& Load)
		{
			if (!Token.IsValid()) return;
			if (!Load.Ok())
			{
				Fail(FString::Printf(TEXT("Couldn't load %s: %s"), *Req.Track.Name, *Load.Error));
				return;
			}
			Say(TEXT("Starting the pipeline (up to a minute)..."), false);

			// 3. The mission. The backend holds the car under EBS while the
			//    pipeline configures and its EKF calibrates, then releases it.
			const TSharedPtr<FJsonObject> Body = MakeShared<FJsonObject>();
			Body->SetStringField(TEXT("mission"), MissionName(Req.Event));
			Body->SetNumberField(TEXT("num_laps"), Req.Event == EFSDSEventType::Trackdrive ? Req.Laps : 1);
			Body->SetBoolField(TEXT("record_bag"), Req.bRecordBag);
			Client->Post(TEXT("/api/event/start"), Body, [this, Token, Req, Fail](const FFSDSMcReply& Start)
			{
				if (!Token.IsValid()) return;
				if (!Start.Ok())
				{
					StopSession();
					Fail(FString::Printf(TEXT("The pipeline didn't start: %s"), *Start.Error));
					return;
				}
				bStartingSession = false;
				SeenExternalGen = Rpc ? Rpc->GetExternalSessionGeneration() : SeenExternalGen;
				Current = Req;
				bRunLive = true;
				bRunRecorded = false;
				Hide();
				UE_LOG(LogTemp, Log, TEXT("FSDS Menu: autonomous %s on %s, %d laps"),
					*EventName(Req.Event), *Req.Track.Name, Req.Laps);
			}, 300.f);
		}, 60.f);
	// The backend copies a recorded bag out of the pipeline's container before
	// it answers: hundreds of MB (an 870 MB one took four minutes here).
	}, 420.f);
}

namespace StackUpUi
{
	// Only what START needs: a plain `up -d` would also build Lichtblick
	// (minutes) and the web UI. Each is built when its image is missing (a
	// fresh clone), and left as it is otherwise.
	const TCHAR* Services = TEXT("dv_pipeline_stack mission_control_backend");
	constexpr double DockerTimeoutS = 180.0;   // Docker Desktop's engine, from cold
	constexpr double BackendTimeoutS = 120.0;  // the backend up and connected to the sim
	constexpr double DockerRetryS = 5.0;
	constexpr double ProbeEveryS = 2.0;
}

bool FFSDSMenu::BringUpStack(const FFSDSDriveRequest& Request)
{
	// bStartingSession stays set from here on: START is still on its way.
	StackUpRequest = Request;
	DockerOpenedAt = 0.0;
	bStackProbe = false;
	if (!StackStarter.IsValid()) StackStarter = MakeShared<FFSDSStackStarter>();
	ComposeDir = FFSDSStackStarter::FindComposeDir();
	if (ComposeDir.IsEmpty())
	{
#if PLATFORM_WINDOWS
		// A sim that came as a zip: the stack runs from the team's code.
		StackUp = EStackUp::NeedRepo;
		ShowRepoPrompt();
		return true;
#else
		StackUpRequest.Reset();
		return false;
#endif
	}
	StartCompose();
	return true;
}

void FFSDSMenu::StartCompose()
{
	if (!FFSDSStackStarter::HasTool(TEXT("docker")))
	{
		FPlatformProcess::LaunchURL(TEXT("https://www.docker.com/products/docker-desktop/"), nullptr, nullptr);
		EndStackUp(TEXT("Docker isn't installed: its download page is open in your browser. Install Docker Desktop, "
			"open it once, then START again."));
		return;
	}
	FString Error;
	if (!StackStarter->Start(ComposeDir, Error, StackUpUi::Services))
	{
		EndStackUp(FString::Printf(TEXT("Couldn't start the stack: %s"), *Error));
		return;
	}
	StackUp = EStackUp::Compose;
	StackUpSince = FPlatformTime::Seconds();
	if (Widget) Widget->SetMessage(TEXT("Starting the pipeline and the backend (the first time, they're built: 5 to 10 minutes)..."), false);
	UE_LOG(LogTemp, Log, TEXT("FSDS Menu: starting the stack in %s"), *ComposeDir);
}

void FFSDSMenu::SyncPipelineSource(TFunction<void()> Then)
{
	if (!StackStarter.IsValid()) StackStarter = MakeShared<FFSDSStackStarter>();
	FString Error;
	if (!FFSDSStackStarter::HasTool(TEXT("docker"))
		|| !StackStarter->Run(ComposeDir, TEXT("docker compose config --format json"), Error))
	{
		Then();
		return;
	}
	AfterSync = MoveTemp(Then);
	StackUp = EStackUp::SyncConfig;
	if (Widget) Widget->SetMessage(TEXT("Copying your pipeline code into the container..."), false);
}

bool FFSDSMenu::IsRebuilding() const
{
	return Rebuilder.IsValid() && Rebuilder->IsRunning();
}

void FFSDSMenu::RebuildPipeline()
{
	if (IsRebuilding()) return;
	bRebuildFailed = false;
	if (bStartingSession || IsAutonomousRunLive())
	{
		RebuildStatus = TEXT("Not during an autonomous start or run: stop it first.");
		bRebuildFailed = true;
		return;
	}
	ComposeDir = FFSDSStackStarter::FindComposeDir();
	if (ComposeDir.IsEmpty())
	{
		RebuildStatus = TEXT("The team's code isn't on this PC: START with Autonomous offers to get it.");
		bRebuildFailed = true;
		return;
	}
	if (!FFSDSStackStarter::HasTool(TEXT("docker")))
	{
		RebuildStatus = TEXT("Docker isn't installed: see docs/TEAM_SETUP.md.");
		bRebuildFailed = true;
		return;
	}
	if (!Rebuilder.IsValid()) Rebuilder = MakeShared<FFSDSStackStarter>();
	// As tools/refresh-bridge.sh, for both: the code volumes are left, since
	// START copies the team's code over them before every run.
	FString Error;
	if (!Rebuilder->Run(ComposeDir, TEXT("docker compose build dv_pipeline_stack mission_control_backend && "
		"docker compose up -d --force-recreate dv_pipeline_stack mission_control_backend"), Error))
	{
		RebuildStatus = FString::Printf(TEXT("Couldn't start the rebuild: %s"), *Error);
		bRebuildFailed = true;
		return;
	}
	RebuildStatus = TEXT("Rebuilding the pipeline and the backend...");
	UE_LOG(LogTemp, Log, TEXT("FSDS Menu: rebuilding the pipeline in %s"), *ComposeDir);
}

void FFSDSMenu::TickRebuild()
{
	if (!Rebuilder.IsValid() || !Rebuilder->IsRunning()) return;
	if (Rebuilder->Poll())
	{
		const FString& Line = Rebuilder->LastLine();
		if (!Line.IsEmpty()) RebuildStatus = FString::Printf(TEXT("Rebuilding: %s"), *Line.Left(100));
		return;
	}
	if (Rebuilder->ExitCode() == 0)
	{
		RebuildStatus = FString::Printf(TEXT("Rebuilt at %s: the next START runs it."), *FDateTime::Now().ToString(TEXT("%H:%M")));
		return;
	}
	bRebuildFailed = true;
	RebuildStatus = FFSDSStackStarter::SaysDockerIsDown(Rebuilder->Output())
		? FString(TEXT("Docker isn't running: open Docker Desktop, then Rebuild again."))
		: FString::Printf(TEXT("The rebuild failed: %s"), *Rebuilder->LastLine().Left(160));
}

void FFSDSMenu::ShowRepoPrompt(const FString& Error)
{
	if (!Widget) return;
	FString Where = FFSDSStackStarter::SuggestedRepoDir();
	FPaths::MakePlatformFilename(Where);   // C:\Users\..., as Windows shows it
	FString Text = FString::Printf(TEXT("The pipeline runs from the team's code (the IFSSIM repo, with the pipeline "
		"in it), and it isn't on this PC yet. Get it puts it in %s: your browser opens to sign in to GitHub with "
		"an account in the isc-fs organization."), *Where);
	if (!Error.IsEmpty()) Text = Error + TEXT("\n\n") + Text;
	Widget->SetMessage(FString(), false);
	Widget->SetRepoPrompt(Text, [this](int32 Choice) { OnRepoChoice(Choice); });
}

void FFSDSMenu::OnRepoChoice(int32 Choice)
{
	if (StackUp != EStackUp::NeedRepo) return;
	FString Picked;
	switch (Choice)
	{
	case 0:
		StartClone(FFSDSStackStarter::SuggestedRepoDir());
		return;
	case 1:
		if (!FFSDSStackStarter::PickFolder(TEXT("Where to put the team's code (IFSSIM)"),
			FPaths::GetPath(FFSDSStackStarter::SuggestedRepoDir()), Picked)) return;
		{
			// An empty folder is used as it is; otherwise IFSSIM-repo inside it.
			TArray<FString> Inside;
			IFileManager::Get().FindFiles(Inside, *FPaths::Combine(Picked, TEXT("*")), true, true);
			StartClone(Inside.Num() == 0 || FFSDSStackStarter::IsRepo(Picked) ? Picked : FPaths::Combine(Picked, TEXT("IFSSIM-repo")));
		}
		return;
	case 2:
		if (!FFSDSStackStarter::PickFolder(TEXT("The IFSSIM folder (with docker-compose.yml in it)"),
			FPaths::GetPath(FFSDSStackStarter::SuggestedRepoDir()), Picked)) return;
		if (!FFSDSStackStarter::IsRepo(Picked))
		{
			ShowRepoPrompt(FString::Printf(TEXT("%s has no docker-compose.yml: pick the IFSSIM folder itself "
				"(the one with Content and pipeline in it)."), *Picked));
			return;
		}
		StartClone(Picked);   // already there: used as it is
		return;
	default:
		if (Widget) Widget->SetRepoPrompt(FString(), nullptr);
		EndStackUp(TEXT("Autonomous needs the team's code: START again when you want to get it."));
		return;
	}
}

void FFSDSMenu::StartClone(const FString& Dir)
{
	CloneDir = Dir;
	if (FFSDSStackStarter::IsRepo(Dir))
	{
		if (Widget) Widget->SetRepoPrompt(FString(), nullptr);
		FFSDSStackStarter::SaveRepoDir(Dir);
		ComposeDir = Dir;
		ScanTracks();
		StartCompose();
		return;
	}
	if (FPaths::DirectoryExists(Dir))
	{
		TArray<FString> Inside;
		IFileManager::Get().FindFiles(Inside, *FPaths::Combine(Dir, TEXT("*")), true, true);
		if (Inside.Num() > 0)
		{
			ShowRepoPrompt(FString::Printf(TEXT("%s already has other things in it: choose somewhere else."), *Dir));
			return;
		}
	}
	if (Widget) Widget->SetRepoPrompt(FString(), nullptr);

	FString Error;
	if (!FFSDSStackStarter::HasTool(TEXT("git")))
	{
		if (!FFSDSStackStarter::HasTool(TEXT("winget")))
		{
			FPlatformProcess::LaunchURL(TEXT("https://git-scm.com/download/win"), nullptr, nullptr);
			EndStackUp(TEXT("git isn't installed: its download page is open in your browser. Install it, then START again."));
			return;
		}
		if (!StackStarter->Run(FPlatformProcess::UserTempDir(), TEXT("winget install --id Git.Git -e --source winget "
			"--accept-package-agreements --accept-source-agreements"), Error))
		{
			EndStackUp(FString::Printf(TEXT("Couldn't install git: %s"), *Error));
			return;
		}
		StackUp = EStackUp::GitInstall;
		if (Widget) Widget->SetMessage(TEXT("Installing git (Windows asks for permission)..."), false);
		return;
	}

	const FString Parent = FPaths::GetPath(Dir);
	IFileManager::Get().MakeDirectory(*Parent, true);
	// The Unreal assets (Git LFS) aren't needed to run the stack.
	FPlatformMisc::SetEnvironmentVar(TEXT("GIT_LFS_SKIP_SMUDGE"), TEXT("1"));
	if (!StackStarter->Run(Parent, FFSDSStackStarter::CloneCommand(Dir), Error))
	{
		EndStackUp(FString::Printf(TEXT("Couldn't run git: %s"), *Error));
		return;
	}
	StackUp = EStackUp::Clone;
	if (Widget) Widget->SetMessage(TEXT("Getting the team's code: sign in to GitHub in your browser if it asks..."), false);
	UE_LOG(LogTemp, Log, TEXT("FSDS Menu: cloning the team's code into %s"), *Dir);
}

void FFSDSMenu::EndStackUp(const FString& Error)
{
	if (Widget) Widget->SetRepoPrompt(FString(), nullptr);
	StackUp = EStackUp::Idle;
	bStartingSession = false;
	const TOptional<FFSDSDriveRequest> Req = StackUpRequest;
	StackUpRequest.Reset();
	if (!Error.IsEmpty())
	{
		if (Widget) Widget->SetMessage(Error, true);
		UE_LOG(LogTemp, Warning, TEXT("FSDS Menu: autonomous start: %s"), *Error);
		return;
	}
	if (Req.IsSet()) StartAutonomous(Req.GetValue(), /*bStackTried=*/true);
}

void FFSDSMenu::TickStackUp()
{
	if (StackUp == EStackUp::Idle || !StackStarter.IsValid()) return;
	const double Now = FPlatformTime::Seconds();
	auto Say = [this](const FString& Msg) { if (Widget) Widget->SetMessage(Msg, false); };

	switch (StackUp)
	{
	case EStackUp::SyncConfig:
	case EStackUp::Sync:
	{
		if (StackStarter->Poll()) return;
		auto Done = [this]()
		{
			StackUp = EStackUp::Idle;
			TFunction<void()> Then = MoveTemp(AfterSync);
			AfterSync = nullptr;
			if (Then) Then();
		};
		if (StackUp == EStackUp::Sync)
		{
			// Not fatal: the container's own copy runs (and with no stack
			// yet, the start brings it up and syncs again).
			if (StackStarter->ExitCode() != 0)
			{
				UE_LOG(LogTemp, Warning, TEXT("FSDS Menu: copying the pipeline code: %s"), *StackStarter->LastLine());
			}
			Done();
			return;
		}
		// The folders compose watch syncs, as compose resolves them.
		FString Cmd;
		const FString& Out = StackStarter->Output();
		int32 Brace = INDEX_NONE;
		Out.FindChar(TEXT('{'), Brace);
		TSharedPtr<FJsonObject> Config;
		const TSharedPtr<FJsonObject>* Services = nullptr;
		const TSharedPtr<FJsonObject>* Pipeline = nullptr;
		const TSharedPtr<FJsonObject>* Develop = nullptr;
		const TArray<TSharedPtr<FJsonValue>>* Watch = nullptr;
		if (StackStarter->ExitCode() == 0 && Brace != INDEX_NONE
			&& FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Out.Mid(Brace)), Config) && Config.IsValid()
			&& Config->TryGetObjectField(TEXT("services"), Services)
			&& (*Services)->TryGetObjectField(TEXT("dv_pipeline_stack"), Pipeline)
			&& (*Pipeline)->TryGetObjectField(TEXT("develop"), Develop)
			&& (*Develop)->TryGetArrayField(TEXT("watch"), Watch))
		{
			for (const TSharedPtr<FJsonValue>& V : *Watch)
			{
				const TSharedPtr<FJsonObject> W = V->AsObject();
				FString Path, Target, Action;
				if (!W.IsValid() || !W->TryGetStringField(TEXT("path"), Path) || !W->TryGetStringField(TEXT("target"), Target)) continue;
				if (W->TryGetStringField(TEXT("action"), Action) && Action != TEXT("sync")) continue;
				if (!FPaths::DirectoryExists(Path)) continue;
				Path.ReplaceInline(TEXT("\\"), TEXT("/"));
				if (!Cmd.IsEmpty()) Cmd += TEXT(" && ");
				// /. : the folder's contents, onto the container's copy.
				Cmd += FString::Printf(TEXT("docker compose cp \"%s/.\" dv_pipeline_stack:%s"), *Path, *Target);
			}
		}
		FString Error;
		if (Cmd.IsEmpty() || !StackStarter->Run(ComposeDir, Cmd, Error))
		{
			Done();
			return;
		}
		StackUp = EStackUp::Sync;
		return;
	}
	case EStackUp::GitInstall:
	{
		if (StackStarter->Poll())
		{
			Say(TEXT("Installing git (Windows asks for permission)..."));
			return;
		}
		if (FFSDSStackStarter::HasTool(TEXT("git")))
		{
			StartClone(CloneDir);
			return;
		}
		FPlatformProcess::LaunchURL(TEXT("https://git-scm.com/download/win"), nullptr, nullptr);
		EndStackUp(FString::Printf(TEXT("git didn't install (%s): its download page is open in your browser. "
			"Install it, then START again."), *StackStarter->LastLine().Left(120)));
		return;
	}
	case EStackUp::Clone:
	{
		if (StackStarter->Poll())
		{
			const FString& Line = StackStarter->LastLine();
			Say(Line.IsEmpty() ? FString(TEXT("Getting the team's code: sign in to GitHub in your browser if it asks..."))
				: FString::Printf(TEXT("Getting the team's code: %s"), *Line.Left(90)));
			return;
		}
		if (StackStarter->ExitCode() == 0 && FFSDSStackStarter::IsRepo(CloneDir))
		{
			FFSDSStackStarter::SaveRepoDir(CloneDir);
			ComposeDir = CloneDir;
			ScanTracks();
			StartCompose();
			return;
		}
		const FString& Out = StackStarter->Output();
		if (Out.Contains(TEXT("Repository not found")) || Out.Contains(TEXT("403"))
			|| Out.Contains(TEXT("Authentication failed")) || Out.Contains(TEXT("could not read Username")))
		{
			EndStackUp(TEXT("GitHub said no: the account you signed in with can't see isc-fs/IFSSIM. Ask to be added "
				"to the isc-fs organization (or sign in with that account), then START again."));
			return;
		}
		EndStackUp(FString::Printf(TEXT("git clone failed: %s"), *StackStarter->LastLine().Left(160)));
		return;
	}
	case EStackUp::Compose:
	{
		if (StackStarter->Poll())
		{
			const FString& Line = StackStarter->LastLine();
			Say(Line.IsEmpty() ? FString(TEXT("Starting the pipeline and the backend..."))
				: FString::Printf(TEXT("Starting the pipeline and the backend: %s"), *Line.Left(90)));
			return;
		}
		if (StackStarter->ExitCode() == 0)
		{
			// Up, but out of reach: the containers listen inside Docker's VM.
			if (FFSDSStackStarter::HostNetworkingOff())
			{
				EndStackUp(TEXT("Docker's host networking is off, so the sim can't reach the pipeline. In Docker Desktop: "
					"Settings, Resources, Network, tick Enable host networking, Apply & restart. Then START again."));
				return;
			}
			// Docker Desktop may still have put its window over the sim.
			if (DockerOpenedAt > 0.0)
			{
				const UWorld* W = World.Get();
				UGameViewportClient* V = W ? W->GetGameViewport() : nullptr;
				if (const TSharedPtr<SWindow> Win = V ? V->GetWindow() : nullptr)
				{
					Win->HACK_ForceToFront();
				}
			}
			StackUp = EStackUp::Backend;
			StackUpSince = Now;
			StackUpNextTry = Now;
			return;
		}
		if (FFSDSStackStarter::SaysDockerIsDown(StackStarter->Output()))
		{
			if (DockerOpenedAt == 0.0)
			{
				FString Error;
				if (!FFSDSStackStarter::StartDocker(Error)) { EndStackUp(Error); return; }
				DockerOpenedAt = Now;
			}
			if (Now - DockerOpenedAt < StackUpUi::DockerTimeoutS)
			{
				StackUp = EStackUp::Docker;   // its engine is still on its way
				StackUpNextTry = Now + StackUpUi::DockerRetryS;
				return;
			}
			EndStackUp(TEXT("Docker didn't start within 3 minutes: open Docker Desktop, then START again."));
			return;
		}
		EndStackUp(FString::Printf(TEXT("docker compose failed: %s"), *StackStarter->LastLine()));
		return;
	}
	case EStackUp::Docker:
	{
		Say(FString::Printf(TEXT("Starting Docker (%.0f s)..."), Now - DockerOpenedAt));
		if (Now < StackUpNextTry) return;
		FString Error;
		if (!StackStarter->Start(ComposeDir, Error, StackUpUi::Services)) { EndStackUp(Error); return; }
		StackUp = EStackUp::Compose;
		return;
	}
	case EStackUp::Backend:
	{
		Say(FString::Printf(TEXT("Waiting for the backend to see the sim (%.0f s)..."), Now - StackUpSince));
		if (Now - StackUpSince > StackUpUi::BackendTimeoutS)
		{
			EndStackUp(FString::Printf(TEXT("The stack started, but the backend at %s didn't see the sim within 2 minutes: "
				"check docker compose logs mission_control_backend."), Client.IsValid() ? *Client->GetBaseUrl() : TEXT("?")));
			return;
		}
		if (bStackProbe || Now < StackUpNextTry || !Client.IsValid()) return;
		bStackProbe = true;
		const TWeakPtr<bool> Token = Alive;
		Client->Get(TEXT("/api/sim/status"), [this, Token](const FFSDSMcReply& R)
		{
			if (!Token.IsValid()) return;
			bStackProbe = false;
			if (StackUp != EStackUp::Backend) return;
			const TSharedPtr<FJsonObject> O = R.Ok() ? R.Object() : nullptr;
			bool bConnected = false;
			if (O.IsValid()) O->TryGetBoolField(TEXT("connected"), bConnected);
			if (bConnected) { EndStackUp(FString()); return; }
			StackUpNextTry = FPlatformTime::Seconds() + StackUpUi::ProbeEveryS;
		}, 3.f);
		return;
	}
	default:
		return;
	}
}

void FFSDSMenu::StopSession()
{
	if (!Client.IsValid()) return;
	Client->Post(TEXT("/api/pipeline/stop"), nullptr, [](const FFSDSMcReply& R)
	{
		if (!R.Ok()) UE_LOG(LogTemp, Warning, TEXT("FSDS Menu: stopping the session: %s"), *R.Error);
	}, 60.f);
}

void FFSDSMenu::TestDrive(const FString& TrackName)
{
	ScanTracks();
	const FFSDSMenuTrack* T = Tracks.FindByPredicate([&](const FFSDSMenuTrack& X) { return X.Name == TrackName; });
	if (!T)
	{
		if (Widget) Widget->SetMessage(FString::Printf(TEXT("Saved, but %s isn't in the tracks folders the sim reads."), *TrackName), true);
		return;
	}
	FFSDSDriveRequest R;
	R.Track = *T;
	R.bPractice = true;     // a loop or an open track alike
	R.Event = EFSDSEventType::Trackdrive;
	R.bAutonomous = false;
	bReturnToEditor = true;
	Drive(R);
}

void FFSDSMenu::StartStack()
{
	if (Launcher) Launcher->StartStack();
}

void FFSDSMenu::OnEscape()
{
	if (!bEnabled || IsOpen()) return;
	if (bRunLive && Current.IsSet())
	{
		// A manual run pauses; an autonomous one keeps going under the box
		// (pausing would stop the pipeline's clock).
		Show(EMode::Pause);
	}
	else
	{
		// No run of the menu's (a session from Mission Control or the
		// benchmark, or nothing yet): the front end, without touching the car.
		Show(EMode::Main);
	}
}

void FFSDSMenu::Resume()
{
	Hide();
}

void FFSDSMenu::Restart()
{
	if (!Current.IsSet()) return;
	const FFSDSDriveRequest Again = *Current;
	if (Again.bAutonomous)
	{
		// The Drive page, so the steps of the new start show under START.
		if (bRunLive) RecordRun();
		bRunLive = false;
		Hide();
		Show(EMode::Main);
		StartAutonomous(Again);
		return;
	}
	Hide();   // unpause first: the load teleports the car
	Drive(Again);
}

void FFSDSMenu::BackToMain()
{
	if (bRunLive) RecordRun();
	if (IsAutonomousRunLive()) StopSession();
	bRunLive = false;
	Hide();
	HoldCar();
	Show(EMode::Main);
}

void FFSDSMenu::Quit()
{
	UE_LOG(LogTemp, Log, TEXT("FSDS Menu: quit"));
	FPlatformMisc::RequestExit(false);
}

void FFSDSMenu::RecordRun()
{
	if (bRunRecorded || !Current.IsSet()) return;
	const AFSDSReferee* R = Referee.Get();
	if (!R) return;
	bRunRecorded = true;

	const FFSDSRefereeState& S = R->GetState();
	// A run where nothing happened (restarted at once) isn't worth a line;
	// an autonomous one always is (it may have been stopped at the start).
	if (!Current->bAutonomous && S.Laps.Num() == 0 && S.DooCounter == 0 && S.OffTrackCounter == 0) return;

	FFSDSRunResult Run;
	Run.Track = Current->Track.Name;
	Run.Event = Current->Event;
	Run.Laps = S.Laps;
	Run.RequiredLaps = S.RequiredLaps;
	Run.ConesDown = S.DooCounter;
	Run.OffCourse = S.OffTrackCounter;
	Run.bFinished = S.bFinished;
	Run.bPractice = Current->bPractice;
	Run.bAutonomous = Current->bAutonomous;
	Run.When = FDateTime::Now();
	Run.bBag = Current->bAutonomous && Current->bRecordBag;
	const int32 Index = Results->Add(MoveTemp(Run));

	// An autonomous run's FS score, as Mission Control computes it: asked now,
	// before the next track load clears the referee.
	if (Current->bAutonomous && Client.IsValid())
	{
		const TWeakPtr<bool> Token = Alive;
		const TSharedPtr<TArray<FFSDSRunResult>> Runs = Results;
		const FString Path = Current->Reference > 0.0
			? FString::Printf(TEXT("/api/scoring/summary?t_best=%.3f"), Current->Reference)
			: FString(TEXT("/api/scoring/summary"));
		Client->Get(Path, [Token, Runs, Index](const FFSDSMcReply& Reply)
		{
			if (!Token.IsValid() || !Runs.IsValid() || !Runs->IsValidIndex(Index)) return;
			const TSharedPtr<FJsonObject> O = Reply.Object();
			FString Err;
			if (!Reply.Ok() || !O.IsValid() || O->TryGetStringField(TEXT("error"), Err)) return;
			FFSDSRunResult& R = (*Runs)[Index];
			O->TryGetNumberField(TEXT("corrected_time"), R.Corrected);
			O->TryGetNumberField(TEXT("doo_penalty_s"), R.DooPenalty);
			O->TryGetNumberField(TEXT("oc_penalty_s"), R.OcPenalty);
			O->TryGetNumberField(TEXT("score"), R.Points);
			O->TryGetNumberField(TEXT("max_points"), R.MaxPoints);
			R.bHasReference = O->TryGetNumberField(TEXT("t_best_reference"), R.Reference) && R.Reference > 0.0;
			R.bScored = true;
		}, 5.f);
	}
}

void FFSDSMenu::UpdateFrameCap()
{
	// The sim's own rate: with bUseFixedFrameRate the engine paces frames by
	// FixedFrameRate and ignores t.MaxFPS, so the cap is a wait of our own
	// (ThrottleFrame), which keeps every frame the same fixed physics step.
	const float BaseFps = GEngine && GEngine->bUseFixedFrameRate ? GEngine->FixedFrameRate : 60.f;

	// Only while nothing drives: the menu is up, and the pipeline doesn't have
	// the car (Esc during an autonomous session brings the menu up over it).
	int32 Cap = 0;
	const AFSDSVehiclePawn* P = Pawn.Get();
	// Nor while a session starts: the pipeline configures and calibrates on
	// the sim's clock, and it should not take twice the time. Nor while Find
	// the best measures the GPU: capped, its frame time counts the waits too,
	// and the GPU drops to a power-saving clock.
	if (bEnabled && IsOpen() && !bStartingSession && !FFSDSGraphics::Get().IsRecommending()
		&& !(P && P->IsApiControlEnabled()))
	{
		Cap = GetMenuFrameRate();
		const UWorld* W = World.Get();
		const UGameViewportClient* Viewport = W ? W->GetGameViewport() : nullptr;
		const TSharedPtr<SWindow> Window = Viewport ? Viewport->GetWindow() : nullptr;
		if (Window.IsValid() && Window->IsWindowMinimized()) Cap = MinimisedFrameRate;
		if (Cap >= BaseFps) Cap = 0;   // no lower than the sim's own: nothing to do
	}
	if (Cap == AppliedCap) return;
	AppliedCap = Cap;
	MenuPacer.Reset();
	UE_LOG(LogTemp, Log, TEXT("FSDS Menu: frame rate %s"),
		Cap > 0 ? *FString::Printf(TEXT("capped at %d while the menu is up"), Cap) : *FString::Printf(TEXT("back to %.0f"), BaseFps));
}

void FFSDSMenu::ThrottleFrame()
{
	// Wait out the rest of this frame's share of a second. The game thread
	// sleeps, so the render thread and the GPU idle with it.
	if (AppliedCap > 0) MenuPacer.Wait(1.0 / AppliedCap);
}

void FFSDSMenu::HoldCar()
{
	// The car waits under EBS while nobody drives it; START releases it.
	if (AFSDSVehiclePawn* P = Pawn.Get())
	{
		P->ActivateEbs();
	}
}

// ---------------------------------------------------------------------------
// Camera

void FFSDSMenu::StartOrbit()
{
	UWorld* W = World.Get();
	APlayerController* PC = GetPC();
	AFSDSVehiclePawn* P = Pawn.Get();
	if (!W || !PC || !P) return;

	if (!OrbitCam.IsValid())
	{
		FActorSpawnParameters Params;
		Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		OrbitCam = W->SpawnActor<ACameraActor>(ACameraActor::StaticClass(), P->GetActorTransform(), Params);
	}
	if (ACameraActor* Cam = OrbitCam.Get())
	{
		// Start behind the car, where the chase camera is.
		OrbitAngle = FMath::DegreesToRadians(P->GetActorRotation().Yaw) + PI;
		UpdateOrbit(0.f);
		PC->SetViewTarget(Cam);
	}
}

void FFSDSMenu::StopOrbit()
{
	ACameraActor* Cam = OrbitCam.Get();
	if (!Cam) return;
	if (APlayerController* PC = GetPC())
	{
		if (AFSDSVehiclePawn* P = Pawn.Get())
		{
			PC->SetViewTarget(P);
		}
	}
	Cam->Destroy();
	OrbitCam.Reset();
}

void FFSDSMenu::UpdateOrbit(float DeltaTime)
{
	ACameraActor* Cam = OrbitCam.Get();
	const AFSDSVehiclePawn* P = Pawn.Get();
	if (!Cam || !P) return;

	OrbitAngle += OrbitRadPerS * DeltaTime;
	const FVector Car = P->GetActorLocation();
	const FVector At = Car + FVector(FMath::Cos(OrbitAngle) * OrbitRadiusCm, FMath::Sin(OrbitAngle) * OrbitRadiusCm, OrbitHeightCm);
	const FRotator Look = (Car + FVector(0.f, 0.f, OrbitLookUpCm) - At).Rotation();
	Cam->SetActorLocationAndRotation(At, Look);
}
