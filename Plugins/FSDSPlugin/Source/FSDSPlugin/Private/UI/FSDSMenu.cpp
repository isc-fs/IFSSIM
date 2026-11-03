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

#include "Camera/CameraActor.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Engine/GameViewportClient.h"
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

	// The track editor talks to the backend as the panel does.
	const FFSDSSettings& S = FFSDSSettings::Get();
	FString ApiKey = S.MissionControlApiKey;
	if (ApiKey.IsEmpty()) ApiKey = FPlatformMisc::GetEnvironmentVariable(TEXT("IFSSIM_MC_API_KEY"));
	Client = MakeShared<FFSDSMissionControlClient>(S.MissionControlUrl, ApiKey);
	Editor = MakeShared<FSDSTrackSpec::FEditorState>();

	// The backend saves into the repo's Content/tracks (docker-compose mounts
	// it); a packaged sim reads its own copy. List the repo's first, so a
	// track saved in the editor shows up and loads.
	const FString ComposeDir = S.MissionControlComposeDir.IsEmpty() ? FFSDSStackStarter::FindComposeDir() : S.MissionControlComposeDir;
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
	if (Gen != SeenExternalGen)
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

	// The referee ends the run (all laps done, or the far gate of a one-way
	// track): show how it went. Practice on a loop never ends this way.
	if (IsManualRunLive() && !IsOpen())
	{
		const AFSDSReferee* R = Referee.Get();
		if (R && R->GetState().bFinished)
		{
			RecordRun();
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
		// The pipeline's session runs from Mission Control, through the
		// backend, as before; the menu just hands over.
		Current = Request;
		bRunLive = false;
		Hide();
		if (Launcher) Launcher->Open(GetPC());
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
	if (IsManualRunLive())
	{
		Show(EMode::Pause);
	}
	else
	{
		// Not in a manual run (an autonomous session, or nothing yet):
		// the front end, without touching the car.
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
	Hide();   // unpause first: the load teleports the car
	Drive(Again);
}

void FFSDSMenu::BackToMain()
{
	if (bRunLive) RecordRun();
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
	if (bRunRecorded || !Current.IsSet() || Current->bAutonomous) return;
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
	Run.When = FDateTime::Now();
	Results->Add(MoveTemp(Run));
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
