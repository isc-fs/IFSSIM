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
	ComposeDir = S.MissionControlComposeDir.IsEmpty() ? FFSDSStackStarter::FindComposeDir() : S.MissionControlComposeDir;
	if (!ComposeDir.IsEmpty())
	{
		const FString Dir = FPaths::Combine(ComposeDir, TEXT("Content"), TEXT("tracks"));
		if (FPaths::DirectoryExists(Dir)) RepoTracksDir = Dir;
	}
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
	Say(TEXT("Stopping any session that is running..."), false);

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
			Body->SetBoolField(TEXT("record_bag"), false);
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
	}, 60.f);
}

namespace StackUpUi
{
	// Only what START needs, and never a build: a plain `up -d` would also
	// build Lichtblick (minutes) and the web UI.
	const TCHAR* Services = TEXT("--no-build dv_pipeline_stack mission_control_backend");
	constexpr double DockerTimeoutS = 180.0;   // Docker Desktop's engine, from cold
	constexpr double BackendTimeoutS = 120.0;  // the backend up and connected to the sim
	constexpr double DockerRetryS = 5.0;
	constexpr double ProbeEveryS = 2.0;
}

bool FFSDSMenu::BringUpStack(const FFSDSDriveRequest& Request)
{
	if (ComposeDir.IsEmpty()) return false;
	if (!StackStarter.IsValid()) StackStarter = MakeShared<FFSDSStackStarter>();
	FString Error;
	if (!StackStarter->Start(ComposeDir, Error, StackUpUi::Services))
	{
		UE_LOG(LogTemp, Warning, TEXT("FSDS Menu: can't start the stack: %s"), *Error);
		return false;
	}
	// bStartingSession stays set: START is still on its way.
	StackUpRequest = Request;
	StackUp = EStackUp::Compose;
	StackUpSince = FPlatformTime::Seconds();
	DockerOpenedAt = 0.0;
	bStackProbe = false;
	if (Widget) Widget->SetMessage(TEXT("Mission Control's backend isn't running: starting the pipeline and the backend..."), false);
	UE_LOG(LogTemp, Log, TEXT("FSDS Menu: no backend; starting the stack in %s"), *ComposeDir);
	return true;
}

void FFSDSMenu::EndStackUp(const FString& Error)
{
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
	// A run where nothing happened (restarted at once) isn't worth a line.
	if (S.Laps.Num() == 0 && S.DooCounter == 0 && S.OffTrackCounter == 0) return;

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
	Results->Add(MoveTemp(Run));
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
