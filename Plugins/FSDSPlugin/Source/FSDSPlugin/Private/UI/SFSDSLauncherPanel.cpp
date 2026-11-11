#include "UI/SFSDSLauncherPanel.h"

#include "FSDSReferee.h"
#include "EmraxMotor.h"
#include "FSDSCoordinates.h"
#include "FSDSVehiclePawn.h"
#include "UI/FSDSMissionControlClient.h"
#include "UI/FSDSStackStarter.h"

#include "Dom/JsonObject.h"
#include "HAL/PlatformProcess.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Policies/PrettyJsonPrintPolicy.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Framework/Application/SlateApplication.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Layout/SScaleBox.h"
#include "Widgets/SOverlay.h"
#include "Brushes/SlateDynamicImageBrush.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/Base64.h"
#include "Modules/ModuleManager.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "FSDSLauncherPanel"

namespace FSDSPanel
{
	// The web UI's palette, so the two read as one product.
	const FLinearColor Accent = FLinearColor::FromSRGBColor(FColor(255, 184, 28));
	const FLinearColor Ink = FLinearColor::FromSRGBColor(FColor(232, 232, 236));
	const FLinearColor Muted = FLinearColor::FromSRGBColor(FColor(150, 150, 160));
	const FLinearColor Good = FLinearColor::FromSRGBColor(FColor(74, 222, 128));
	const FLinearColor Warn = FLinearColor::FromSRGBColor(FColor(251, 191, 36));
	const FLinearColor Bad = FLinearColor::FromSRGBColor(FColor(248, 113, 113));
	const FLinearColor Ground(0.010f, 0.010f, 0.013f, 0.92f);
	const FLinearColor GoBtn = FLinearColor::FromSRGBColor(FColor(21, 128, 61));
	const FLinearColor StopBtn = FLinearColor::FromSRGBColor(FColor(185, 28, 28));
	const FLinearColor PlainBtn = FLinearColor::FromSRGBColor(FColor(64, 64, 72));
	const FLinearColor ResBtn = FLinearColor::FromSRGBColor(FColor(127, 29, 29));
	const FLinearColor ResLive = FLinearColor::FromSRGBColor(FColor(220, 38, 38));

	constexpr float Width = 440.f;
	constexpr float LabelWidth = 92.f;
	constexpr double ArmSeconds = 3.0;

	FSlateFontInfo Font(int32 Size, const TCHAR* Weight = TEXT("Regular"))
	{
		return FCoreStyle::GetDefaultFontStyle(Weight, Size);
	}
}

using namespace FSDSPanel;

void SFSDSLauncherPanel::Construct(const FArguments& InArgs)
{
	Client = InArgs._Client;
	Stack = InArgs._Stack;
	Referee = InArgs._Referee;
	Pawn = InArgs._Pawn;
	ComposeDir = InArgs._ComposeDir;
	ToggleKey = InArgs._ToggleKey;
	OnClose = InArgs._OnClose;
	OnStartStack = InArgs._OnStartStack;

	const FText CloseHint = FText::Format(LOCTEXT("CloseHint", "{0} or Esc to close"),
		ToggleKey.GetDisplayName());

	ChildSlot
	[
		SNew(SBox)
		.WidthOverride(Width)
		[
			SNew(SBorder)
			.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
			.BorderBackgroundColor(Ground)
			.Padding(FMargin(18.f, 14.f))
			[
				SNew(SVerticalBox)

				// Title
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 10)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
					[
						SNew(STextBlock)
						.Text(LOCTEXT("Title", "MISSION CONTROL"))
						.Font(Font(13, TEXT("Bold")))
						.ColorAndOpacity(Accent)
					]
					+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
					[
						SNew(STextBlock).Text(CloseHint).Font(Font(9)).ColorAndOpacity(Muted)
					]
				]

				// Tabs
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8) [ BuildTabBar() ]
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SWidgetSwitcher)
					.WidgetIndex_Lambda([this]() { return (int32)ActiveTab; })
					+ SWidgetSwitcher::Slot() [ BuildSessionTab() ]
					+ SWidgetSwitcher::Slot() [ BuildLogTab() ]
					+ SWidgetSwitcher::Slot() [ BuildScoringTab() ]
					+ SWidgetSwitcher::Slot() [ BuildTracksTab() ]
					+ SWidgetSwitcher::Slot() [ BuildTelemetryTab() ]
				]

				// Outcome of the last action
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 0)
				[
					SNew(STextBlock)
					.AutoWrapText(true)
					.Font(Font(10))
					.Text_Lambda([this]()
					{
						return FText::FromString(Busy.IsEmpty() ? Message : Busy + TEXT("..."));
					})
					.ColorAndOpacity_Lambda([this]()
					{
						if (!Busy.IsEmpty()) return FSlateColor(Warn);
						return FSlateColor(bMessageError ? Bad : Muted);
					})
				]

				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
				[
					SNew(STextBlock)
					.AutoWrapText(true)
					.Font(Font(9))
					.ColorAndOpacity(Muted)
					.Text(LOCTEXT("Footer",
						"Same backend as the web UI: both show the same session."))
				]
			]
		]
	];

	RegisterActiveTimer(1.f, FWidgetActiveTimerDelegate::CreateSP(this, &SFSDSLauncherPanel::Poll));
	Poll(0.0, 0.f);
}

// ---------------------------------------------------------------------------
// Keys

FReply SFSDSLauncherPanel::OnKeyDown(const FGeometry& Geometry, const FKeyEvent& KeyEvent)
{
	if (!KeyEvent.IsRepeat() && (KeyEvent.GetKey() == ToggleKey || KeyEvent.GetKey() == EKeys::Escape))
	{
		OnClose.ExecuteIfBound();
		return FReply::Handled();
	}
	// Everything else falls through to the game, so the car still drives.
	return FReply::Unhandled();
}

// ---------------------------------------------------------------------------
// Backend

EActiveTimerReturnType SFSDSLauncherPanel::Poll(double, float)
{
	// The log only while it is on screen; it has its own in-flight guard.
	if (ActiveTab == ETab::Log)
	{
		FetchLog();
	}
	else if (ActiveTab == ETab::Scoring)
	{
		FetchScoring();
	}

	if (PollsInFlight > 0 || !Client.IsValid())
	{
		return EActiveTimerReturnType::Continue;
	}

	TWeakPtr<SFSDSLauncherPanel> Weak = StaticCastSharedRef<SFSDSLauncherPanel>(AsShared());
	auto Done = [Weak]()
	{
		if (TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin()) P->PollsInFlight = FMath::Max(0, P->PollsInFlight - 1);
	};

	PollsInFlight = 4;
	Client->Get(TEXT("/api/sim/status"), [Weak, Done](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin();
		if (!P) return;
		P->bPolled = true;
		P->bBackendUp = R.Reached();
		P->BackendError = R.Error;
		if (const TSharedPtr<FJsonObject> O = R.Object())
		{
			P->bSimConnected = O->GetBoolField(TEXT("connected"));
			P->bPaused = O->HasTypedField<EJson::Boolean>(TEXT("paused")) && O->GetBoolField(TEXT("paused"));
			P->Fps = O->HasTypedField<EJson::Number>(TEXT("fps")) ? (int32)O->GetNumberField(TEXT("fps")) : 0;
		}
		else
		{
			P->bSimConnected = false;
		}
		if (P->bBackendUp && !P->bListsLoaded && !P->bListsLoading)
		{
			P->FetchLists();
		}
		Done();
	});
	Client->Get(TEXT("/api/pipeline/status"), [Weak, Done](const FFSDSMcReply& R)
	{
		if (TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin())
		{
			const TSharedPtr<FJsonObject> O = R.Object();
			P->bPipeline = O.IsValid() && O->HasTypedField<EJson::Boolean>(TEXT("enabled")) && O->GetBoolField(TEXT("enabled"));
		}
		Done();
	});
	Client->Get(TEXT("/api/res/status"), [Weak, Done](const FFSDSMcReply& R)
	{
		if (TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin())
		{
			const TSharedPtr<FJsonObject> O = R.Object();
			P->bResActive = O.IsValid() && O->HasTypedField<EJson::Boolean>(TEXT("res_active")) && O->GetBoolField(TEXT("res_active"));
		}
		Done();
	});
	Client->Get(TEXT("/api/event/state"), [Weak, Done](const FFSDSMcReply& R)
	{
		if (TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin())
		{
			P->BagState.Reset();
			P->BagName.Reset();
			if (const TSharedPtr<FJsonObject> O = R.Object())
			{
				O->TryGetStringField(TEXT("bag_state"), P->BagState);
				O->TryGetStringField(TEXT("bag_name"), P->BagName);
			}
		}
		Done();
	});
	return EActiveTimerReturnType::Continue;
}

void SFSDSLauncherPanel::FetchLists()
{
	bListsLoading = true;
	TWeakPtr<SFSDSLauncherPanel> Weak = StaticCastSharedRef<SFSDSLauncherPanel>(AsShared());

	Client->Get(TEXT("/api/track/list"), [Weak](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin();
		if (!P) return;
		P->bListsLoading = false;
		if (!R.Ok() || !R.Body.IsValid() || R.Body->Type != EJson::Array)
		{
			P->Say(FString::Printf(TEXT("Couldn't list tracks: %s"), *R.Error), true);
			return;
		}
		P->bListsLoaded = true;
		const FString Keep = P->SelectedTrack.IsValid() ? *P->SelectedTrack : FString();
		P->Tracks.Reset();
		P->SelectedTrack.Reset();
		P->TrackInfos.Reset();
		for (const TSharedPtr<FJsonValue>& V : R.Body->AsArray())
		{
			const TSharedPtr<FJsonObject>* O = nullptr;
			FString Name;
			if (V->TryGetObject(O) && (*O)->TryGetStringField(TEXT("name"), Name))
			{
				P->Tracks.Add(MakeShared<FString>(Name));
				if (Name == Keep) P->SelectedTrack = P->Tracks.Last();

				FTrackInfo Info;
				Info.Name = Name;
				(*O)->TryGetNumberField(TEXT("cones"), Info.Cones);
				(*O)->TryGetNumberField(TEXT("blue"), Info.Blue);
				(*O)->TryGetNumberField(TEXT("yellow"), Info.Yellow);
				(*O)->TryGetNumberField(TEXT("orange"), Info.Orange);
				(*O)->TryGetBoolField(TEXT("builtin"), Info.bBuiltin);
				(*O)->TryGetStringField(TEXT("event_type"), Info.EventType);
				P->TrackInfos.Add(MoveTemp(Info));
			}
		}
		P->RebuildTrackInfoList();
		if (!P->SelectedInfo.IsEmpty() && !P->TrackInfos.ContainsByPredicate(
				[&P](const FTrackInfo& I) { return I.Name == P->SelectedInfo; }))
		{
			P->SelectedInfo.Reset();
		}
		if (!P->SelectedTrack.IsValid() && P->Tracks.Num() > 0) P->SelectedTrack = P->Tracks[0];
		if (P->TrackCombo.IsValid())
		{
			P->TrackCombo->RefreshOptions();
			P->TrackCombo->SetSelectedItem(P->SelectedTrack);
		}
	});

	Client->Get(TEXT("/api/pipeline/missions"), [Weak](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin();
		if (!P) return;
		const TSharedPtr<FJsonObject> O = R.Object();
		const TArray<TSharedPtr<FJsonValue>>* List = nullptr;
		if (!R.Ok() || !O.IsValid() || !O->TryGetArrayField(TEXT("missions"), List))
		{
			P->Say(FString::Printf(TEXT("Couldn't list missions: %s"), *R.Error), true);
			return;
		}
		const FString Keep = P->SelectedMission.IsValid() ? P->SelectedMission->Name : FString(TEXT("trackdrive"));
		P->Missions.Reset();
		P->SelectedMission.Reset();
		for (const TSharedPtr<FJsonValue>& V : *List)
		{
			const TSharedPtr<FJsonObject>* M = nullptr;
			if (!V->TryGetObject(M)) continue;
			FMissionPtr Mission = MakeShared<FMission>();
			(*M)->TryGetStringField(TEXT("name"), Mission->Name);
			(*M)->TryGetStringField(TEXT("label"), Mission->Label);
			(*M)->TryGetStringField(TEXT("kind"), Mission->Kind);
			(*M)->TryGetBoolField(TEXT("supports_laps"), Mission->bLaps);
			if (Mission->Label.IsEmpty()) Mission->Label = Mission->Name;
			P->Missions.Add(Mission);
			if (Mission->Name == Keep) P->SelectedMission = Mission;
		}
		if (!P->SelectedMission.IsValid() && P->Missions.Num() > 0) P->SelectedMission = P->Missions[0];
		if (P->MissionCombo.IsValid())
		{
			P->MissionCombo->RefreshOptions();
			P->MissionCombo->SetSelectedItem(P->SelectedMission);
		}
	});
}

void SFSDSLauncherPanel::Act(const FString& What, const FString& Path, const TSharedPtr<FJsonObject>& Body,
	float TimeoutS, TFunction<FString(const FFSDSMcReply&)> Describe)
{
	Busy = What;
	Armed = NAME_None;
	TWeakPtr<SFSDSLauncherPanel> Weak = StaticCastSharedRef<SFSDSLauncherPanel>(AsShared());
	Client->Post(Path, Body, [Weak, What, Describe = MoveTemp(Describe)](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin();
		if (!P) return;
		P->Busy.Reset();
		if (!R.Ok())
		{
			P->Say(FString::Printf(TEXT("%s failed: %s"), *What, *R.Error), true);
			return;
		}
		P->Say(Describe ? Describe(R) : What + TEXT(": done"), false);
		P->Poll(0.0, 0.f);
	}, TimeoutS);
}

bool SFSDSLauncherPanel::Confirmed(FName Action)
{
	const double Now = FPlatformTime::Seconds();
	if (Armed == Action && Now < ArmedUntil)
	{
		Armed = NAME_None;
		return true;
	}
	Armed = Action;
	ArmedUntil = Now + ArmSeconds;
	return false;
}

bool SFSDSLauncherPanel::IsArmed(FName Action) const
{
	return Armed == Action && FPlatformTime::Seconds() < ArmedUntil;
}

void SFSDSLauncherPanel::Say(const FString& Msg, bool bError)
{
	Message = Msg;
	bMessageError = bError;
	UE_LOG(LogTemp, Log, TEXT("FSDS Launcher: %s"), *Msg);
}

// ---------------------------------------------------------------------------
// Actions. Each is one backend call, the same one the web UI makes.

FReply SFSDSLauncherPanel::OnLoadTrack()
{
	if (!SelectedTrack.IsValid()) return FReply::Handled();
	const FString Name = *SelectedTrack;
	Act(FString::Printf(TEXT("Loading %s"), *Name),
		FString::Printf(TEXT("/api/track/%s/load"), *FFSDSMissionControlClient::EncodeSegment(Name)),
		nullptr, 30.f,
		[Name](const FFSDSMcReply& R)
		{
			double Cones = 0;
			const TSharedPtr<FJsonObject> O = R.Object();
			const TSharedPtr<FJsonObject>* Result = nullptr;
			if (O.IsValid() && O->TryGetObjectField(TEXT("result"), Result))
			{
				(*Result)->TryGetNumberField(TEXT("cones"), Cones);
			}
			return Cones > 0
				? FString::Printf(TEXT("Loaded %s: %d cones"), *Name, (int32)Cones)
				: FString::Printf(TEXT("Loaded %s"), *Name);
		});
	return FReply::Handled();
}

FReply SFSDSLauncherPanel::OnStartSession()
{
	if (!SelectedMission.IsValid()) return FReply::Handled();
	const TSharedPtr<FJsonObject> Body = MakeShared<FJsonObject>();
	Body->SetStringField(TEXT("mission"), SelectedMission->Name);
	Body->SetNumberField(TEXT("num_laps"), NumLaps);
	const bool bBag = bRecordBag && SelectedMission->Kind != TEXT("sim_benchmark_control");
	Body->SetBoolField(TEXT("record_bag"), bBag);

	const FString Label = SelectedMission->Label;
	// The backend holds the car under EBS while the pipeline configures and
	// the EKF calibrates, then releases it: this call can take a minute.
	Act(FString::Printf(TEXT("Starting %s (the pipeline takes up to a minute)"), *Label),
		TEXT("/api/event/start"), Body, 300.f,
		[Label](const FFSDSMcReply& R)
		{
			FString Msg = FString::Printf(TEXT("Session started: %s"), *Label);
			const TSharedPtr<FJsonObject> O = R.Object();
			const TSharedPtr<FJsonObject>* Bag = nullptr;
			if (O.IsValid() && O->TryGetObjectField(TEXT("bag"), Bag))
			{
				FString Err, Name;
				if ((*Bag)->TryGetStringField(TEXT("error"), Err) && !Err.IsEmpty())
					Msg += FString::Printf(TEXT(". Recording failed: %s"), *Err);
				else if ((*Bag)->TryGetStringField(TEXT("name"), Name) && !Name.IsEmpty())
					Msg += FString::Printf(TEXT(". Recording %s"), *Name);
			}
			return Msg;
		});
	return FReply::Handled();
}

FReply SFSDSLauncherPanel::OnStopSession()
{
	if (Confirmed(TEXT("stop")))
	{
		Act(TEXT("Stopping the session"), TEXT("/api/pipeline/stop"), nullptr, 60.f,
			[](const FFSDSMcReply&) { return FString(TEXT("Session stopped; the sim keeps running")); });
	}
	return FReply::Handled();
}

FReply SFSDSLauncherPanel::OnReset()
{
	if (Confirmed(TEXT("reset")))
	{
		Act(TEXT("Resetting the car"), TEXT("/api/sim/reset"), nullptr, 30.f,
			[](const FFSDSMcReply&) { return FString(TEXT("Car back at the start")); });
	}
	return FReply::Handled();
}

FReply SFSDSLauncherPanel::OnPause()
{
	const bool bResume = bPaused;
	Act(bResume ? TEXT("Resuming") : TEXT("Pausing"),
		bResume ? TEXT("/api/sim/resume") : TEXT("/api/sim/pause"), nullptr, 10.f,
		[bResume](const FFSDSMcReply&) { return FString(bResume ? TEXT("Resumed") : TEXT("Paused")); });
	return FReply::Handled();
}

FReply SFSDSLauncherPanel::OnRes()
{
	// The emergency stop: no confirm, and allowed while another action runs.
	const bool bRelease = bResActive;
	TWeakPtr<SFSDSLauncherPanel> Weak = StaticCastSharedRef<SFSDSLauncherPanel>(AsShared());
	Client->Post(bRelease ? TEXT("/api/res/release") : TEXT("/api/res/activate"), nullptr,
		[Weak, bRelease](const FFSDSMcReply& R)
		{
			TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin();
			if (!P) return;
			if (R.Ok())
			{
				P->bResActive = !bRelease;
				P->Say(bRelease ? TEXT("RES released") : TEXT("RES active: car held"), false);
			}
			else
			{
				P->Say(FString::Printf(TEXT("RES failed: %s"), *R.Error), true);
			}
		}, 10.f);
	return FReply::Handled();
}

FReply SFSDSLauncherPanel::OnExportLog()
{
	if (!CanAct()) return FReply::Handled();
	Busy = TEXT("Exporting the session log");

	TWeakPtr<SFSDSLauncherPanel> Weak = StaticCastSharedRef<SFSDSLauncherPanel>(AsShared());
	Client->Get(TEXT("/api/session/export"), [Weak](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin();
		if (!P) return;
		P->Busy.Reset();

		const TArray<TSharedPtr<FJsonValue>>* Items = nullptr;
		if (!R.Ok() || !R.Body.IsValid() || !R.Body->TryGetArray(Items))
		{
			P->Say(FString::Printf(TEXT("Export failed: %s"),
				R.Error.IsEmpty() ? TEXT("malformed reply") : *R.Error), true);
			return;
		}

		// The web UI downloads it; the sim has no browser, so it goes where
		// people look for files: Documents/IFSSIM/SessionLogs.
		FString Json;
		TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer =
			TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&Json);
		FJsonSerializer::Serialize(*Items, Writer);

		const FString Dir = FPaths::Combine(FPlatformProcess::UserDir(), TEXT("IFSSIM"), TEXT("SessionLogs"));
		const FString File = FPaths::ConvertRelativePathToFull(FPaths::Combine(Dir,
			FString::Printf(TEXT("session_%s.json"), *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")))));
		if (FFileHelper::SaveStringToFile(Json, *File, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
		{
			P->Say(FString::Printf(TEXT("Saved %d events to %s"), Items->Num(), *File), false);
		}
		else
		{
			P->Say(FString::Printf(TEXT("Could not write %s"), *File), true);
		}
	}, 10.f);
	return FReply::Handled();
}

FReply SFSDSLauncherPanel::OnTracksLoad()
{
	if (SelectedInfo.IsEmpty()) return FReply::Handled();
	// One load path: the Session tab's, with the track picked here.
	const FTrackPtr* Match = Tracks.FindByPredicate([this](const FTrackPtr& T) { return T.IsValid() && *T == SelectedInfo; });
	SelectedTrack = Match ? *Match : MakeShared<FString>(SelectedInfo);
	if (TrackCombo.IsValid()) TrackCombo->SetSelectedItem(SelectedTrack);
	return OnLoadTrack();
}

FReply SFSDSLauncherPanel::OnTracksDelete()
{
	if (SelectedInfo.IsEmpty() || !Confirmed(TEXT("delete"))) return FReply::Handled();
	const FString Name = SelectedInfo;
	Busy = FString::Printf(TEXT("Deleting %s"), *Name);

	TWeakPtr<SFSDSLauncherPanel> Weak = StaticCastSharedRef<SFSDSLauncherPanel>(AsShared());
	Client->Delete(FString::Printf(TEXT("/api/track/%s"), *FFSDSMissionControlClient::EncodeSegment(Name)),
		[Weak, Name](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin();
		if (!P) return;
		P->Busy.Reset();
		if (!R.Ok())
		{
			P->Say(FString::Printf(TEXT("Deleting %s failed: %s"), *Name, *R.Error), true);
			return;
		}
		P->Say(FString::Printf(TEXT("Deleted %s"), *Name), false);
		P->SelectedInfo.Reset();
		P->PreviewFor.Reset();
		P->PreviewBrush.Reset();
		P->PreviewState.Reset();
		P->FetchLists();
	});
	return FReply::Handled();
}

FReply SFSDSLauncherPanel::OnGenerate()
{
	TSharedPtr<FJsonObject> Body = MakeShared<FJsonObject>();
	Body->SetNumberField(TEXT("n_points"), GenPoints);
	Body->SetNumberField(TEXT("n_regions"), GenRegions);
	Body->SetNumberField(TEXT("max_bound"), GenMaxBound);
	Body->SetStringField(TEXT("name"), GenName);

	Busy = TEXT("Generating a track");
	TWeakPtr<SFSDSLauncherPanel> Weak = StaticCastSharedRef<SFSDSLauncherPanel>(AsShared());
	Client->Post(TEXT("/api/track/generate"), Body, [Weak](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin();
		if (!P) return;
		P->Busy.Reset();
		FString Name;
		const TSharedPtr<FJsonObject> O = R.Object();
		if (!R.Ok() || !O.IsValid() || !O->TryGetStringField(TEXT("name"), Name))
		{
			P->Say(FString::Printf(TEXT("Generating failed: %s"), R.Error.IsEmpty() ? TEXT("malformed reply") : *R.Error), true);
			return;
		}
		double Cones = 0;
		O->TryGetNumberField(TEXT("cones"), Cones);
		P->Say(FString::Printf(TEXT("Generated %s (%d cones)"), *Name, (int32)Cones), false);
		P->GenName.Reset();
		// Show it: the list gains it, and it becomes the selection.
		P->SelectedInfo = Name;
		P->FetchLists();
		P->FetchPreview(Name);
	}, 60.f);
	return FReply::Handled();
}

// ---------------------------------------------------------------------------
// Layout

TSharedRef<SWidget> SFSDSLauncherPanel::MakeStatusRow(const FText& Label, TAttribute<FText> Value,
	TAttribute<FSlateColor> Color)
{
	return SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth().Padding(0, 1)
		[
			SNew(SBox).WidthOverride(LabelWidth)
			[
				SNew(STextBlock).Text(Label).Font(Font(10)).ColorAndOpacity(Muted)
			]
		]
		+ SHorizontalBox::Slot().FillWidth(1.f).Padding(0, 1)
		[
			SNew(STextBlock).Text(Value).Font(Font(10, TEXT("Bold"))).ColorAndOpacity(Color).AutoWrapText(true)
		];
}

TSharedRef<SWidget> SFSDSLauncherPanel::MakeHeading(const FText& Title)
{
	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 4)
		[
			SNew(SSeparator).Thickness(1.f).ColorAndOpacity(FLinearColor(1, 1, 1, 0.08f))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[
			SNew(STextBlock).Text(Title).Font(Font(9, TEXT("Bold"))).ColorAndOpacity(Accent)
		];
}

TSharedRef<SWidget> SFSDSLauncherPanel::MakeButton(TAttribute<FText> Label, FLinearColor Color,
	FOnClicked OnClicked, TAttribute<bool> Enabled)
{
	return SNew(SButton)
		.IsFocusable(false)          // a focused button would eat Space (the brake)
		.ButtonColorAndOpacity(Color)
		.ContentPadding(FMargin(12.f, 5.f))
		.HAlign(HAlign_Center)
		.OnClicked(OnClicked)
		.IsEnabled(Enabled)
		[
			SNew(STextBlock).Text(Label).Font(Font(10, TEXT("Bold"))).ColorAndOpacity(FLinearColor::White)
		];
}

TSharedRef<SWidget> SFSDSLauncherPanel::BuildSessionTab()
{
	return SNew(SVerticalBox)

		// Status
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeStatusRow(LOCTEXT("Backend", "Backend"),
				MakeAttributeSP(this, &SFSDSLauncherPanel::BackendText),
				MakeAttributeSP(this, &SFSDSLauncherPanel::BackendColor))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeStatusRow(LOCTEXT("Sim", "Simulator"),
				MakeAttributeSP(this, &SFSDSLauncherPanel::SimText),
				MakeAttributeSP(this, &SFSDSLauncherPanel::SimColor))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeStatusRow(LOCTEXT("Track", "Track"),
				MakeAttributeSP(this, &SFSDSLauncherPanel::TrackText), FSlateColor(Ink))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeStatusRow(LOCTEXT("Run", "Run"),
				MakeAttributeSP(this, &SFSDSLauncherPanel::LapText), FSlateColor(Ink))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeStatusRow(LOCTEXT("Pipeline", "Pipeline"),
				MakeAttributeSP(this, &SFSDSLauncherPanel::PipelineText),
				MakeAttributeSP(this, &SFSDSLauncherPanel::PipelineColor))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeStatusRow(LOCTEXT("Res", "RES"),
				MakeAttributeSP(this, &SFSDSLauncherPanel::ResText),
				MakeAttributeSP(this, &SFSDSLauncherPanel::ResColor))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SBox)
			.Visibility_Lambda([this]() { return BagText().IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible; })
			[
				MakeStatusRow(LOCTEXT("Bag", "Recording"),
					MakeAttributeSP(this, &SFSDSLauncherPanel::BagText), FSlateColor(Ink))
			]
		]

		+ SVerticalBox::Slot().AutoHeight() [ BuildStackBlock() ]
		+ SVerticalBox::Slot().AutoHeight() [ BuildTrackBlock() ]
		+ SVerticalBox::Slot().AutoHeight() [ BuildSessionBlock() ]
		+ SVerticalBox::Slot().AutoHeight() [ BuildCarBlock() ]
		;
}

TSharedRef<SWidget> SFSDSLauncherPanel::BuildTabBar()
{
	return SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0)
		[
			MakeTab(LOCTEXT("TabSession", "Session"), ETab::Session)
		]
		+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0)
		[
			MakeTab(LOCTEXT("TabLog", "Log"), ETab::Log)
		]
		+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0)
		[
			MakeTab(LOCTEXT("TabScoring", "Scoring"), ETab::Scoring)
		]
		+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0)
		[
			MakeTab(LOCTEXT("TabTracks", "Tracks"), ETab::Tracks)
		]
		+ SHorizontalBox::Slot().AutoWidth()
		[
			MakeTab(LOCTEXT("TabTelemetry", "Telemetry"), ETab::Telemetry)
		];
}

TSharedRef<SWidget> SFSDSLauncherPanel::MakeTab(const FText& Label, ETab Tab)
{
	return SNew(SButton)
		.IsFocusable(false)          // a focused button would eat Space (the brake)
		.ContentPadding(FMargin(7.f, 3.f))
		// Slate darkens a button's colour, so the active tab is marked by its
		// text (the accent) rather than by a yellow button black text can't
		// be read on.
		.ButtonColorAndOpacity_Lambda([this, Tab]()
		{
			return FSlateColor(ActiveTab == Tab ? PlainBtn : FLinearColor(0.f, 0.f, 0.f, 0.f));
		})
		.OnClicked_Lambda([this, Tab]()
		{
			ActiveTab = Tab;
			if (Tab == ETab::Log) FetchLog();
			if (Tab == ETab::Scoring) FetchScoring();
			if (Tab == ETab::Tracks && !SelectedInfo.IsEmpty() && SelectedInfo != PreviewFor) FetchPreview(SelectedInfo);
			return FReply::Handled();
		})
		[
			SNew(STextBlock)
			.Text(Label)
			.Font(Font(10, TEXT("Bold")))
			.ColorAndOpacity_Lambda([this, Tab]()
			{
				return FSlateColor(ActiveTab == Tab ? Accent : Muted);
			})
		];
}

TSharedRef<SWidget> SFSDSLauncherPanel::BuildLogTab()
{
	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeHeading(LOCTEXT("LogHeading", "SESSION LOG"))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
			[
				SNew(STextBlock)
				.Font(Font(9))
				.AutoWrapText(true)
				.ColorAndOpacity_Lambda([this]() { return FSlateColor(LogError.IsEmpty() ? Muted : Warn); })
				.Text_Lambda([this]()
				{
					if (!LogError.IsEmpty())
					{
						return FText::Format(LOCTEXT("LogError", "Can't read the log: {0}"), FText::FromString(LogError));
					}
					return FText::Format(LOCTEXT("LogCount", "{0} events, newest first"), FText::AsNumber(LogEntries.Num()));
				})
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				MakeButton(LOCTEXT("ExportLog", "Export JSON"), PlainBtn,
					FOnClicked::CreateSP(this, &SFSDSLauncherPanel::OnExportLog),
					TAttribute<bool>::CreateLambda([this]() { return CanAct(); }))
			]
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SBox)
			.MaxDesiredHeight(380.f)
			[
				SNew(SScrollBox)
				+ SScrollBox::Slot()
				[
					SAssignNew(LogList, SVerticalBox)
				]
			]
		];
}

void SFSDSLauncherPanel::FetchLog()
{
	if (bLogInFlight || !Client.IsValid()) return;
	bLogInFlight = true;

	TWeakPtr<SFSDSLauncherPanel> Weak = StaticCastSharedRef<SFSDSLauncherPanel>(AsShared());
	Client->Get(TEXT("/api/session/log"), [Weak](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin();
		if (!P) return;
		P->bLogInFlight = false;

		// The web UI checks the same thing: an {"error": ...} envelope is
		// not a log.
		const TArray<TSharedPtr<FJsonValue>>* Items = nullptr;
		if (!R.Ok() || !R.Body.IsValid() || !R.Body->TryGetArray(Items))
		{
			P->LogError = R.Error.IsEmpty() ? TEXT("malformed reply") : R.Error;
			return;
		}
		P->LogError.Reset();

		FString Signature;
		TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Signature);
		FJsonSerializer::Serialize(*Items, Writer);
		if (Signature == P->LogSignature && P->LogList.IsValid() && P->LogList->NumSlots() > 0) return;
		P->LogSignature = MoveTemp(Signature);

		P->LogEntries.Reset();
		for (const TSharedPtr<FJsonValue>& Item : *Items)
		{
			const TSharedPtr<FJsonObject>* O = nullptr;
			if (!Item.IsValid() || !Item->TryGetObject(O)) continue;
			FLogEntry E;
			FString Stamp;
			(*O)->TryGetStringField(TEXT("timestamp"), Stamp);
			(*O)->TryGetStringField(TEXT("type"), E.Type);
			(*O)->TryGetStringField(TEXT("message"), E.Text);
			// "2026-11-02T13:22:05.123456" -> "13:22:05", as the web UI shows it.
			int32 T = INDEX_NONE;
			E.Time = Stamp.FindChar(TEXT('T'), T) ? Stamp.Mid(T + 1, 8) : Stamp;
			P->LogEntries.Add(MoveTemp(E));
		}
		P->RebuildLogList();
	});
}

void SFSDSLauncherPanel::RebuildLogList()
{
	if (!LogList.IsValid()) return;
	LogList->ClearChildren();

	if (LogEntries.Num() == 0)
	{
		LogList->AddSlot().AutoHeight()
		[
			SNew(STextBlock).Font(Font(10)).ColorAndOpacity(Muted)
			.Text(LOCTEXT("LogEmpty", "No events logged yet. Actions will appear here."))
		];
		return;
	}

	// The web UI's colours per event type.
	auto TypeColor = [](const FString& Type) -> FLinearColor
	{
		if (Type == TEXT("event_set")) return Accent;
		if (Type == TEXT("event_start")) return Good;
		if (Type == TEXT("res")) return Bad;
		if (Type.Contains(TEXT("reset"))) return FLinearColor::FromSRGBColor(FColor(96, 165, 250));
		if (Type == TEXT("track_load")) return FLinearColor::FromSRGBColor(FColor(192, 132, 252));
		if (Type == TEXT("track_generate")) return FLinearColor::FromSRGBColor(FColor(34, 211, 238));
		return Muted;
	};

	for (int32 i = LogEntries.Num() - 1; i >= 0; --i)
	{
		const FLogEntry& E = LogEntries[i];
		LogList->AddSlot().AutoHeight().Padding(0, 2)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 8, 0)
			[
				SNew(STextBlock).Font(Font(9)).ColorAndOpacity(Muted).Text(FText::FromString(E.Time))
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 8, 0)
			[
				SNew(SBox).WidthOverride(104.f)
				[
					SNew(STextBlock).Font(Font(9, TEXT("Bold"))).ColorAndOpacity(TypeColor(E.Type))
					.Text(FText::FromString(E.Type.ToUpper()))
				]
			]
			+ SHorizontalBox::Slot().FillWidth(1.f)
			[
				SNew(STextBlock).Font(Font(9)).ColorAndOpacity(Ink).AutoWrapText(true)
				.Text(FText::FromString(E.Text))
			]
		];
	}
}

TSharedRef<SWidget> SFSDSLauncherPanel::BuildScoringTab()
{
	auto Seconds = [](double S) { return FText::FromString(FString::Printf(TEXT("%.3f s"), S)); };

	return SNew(SVerticalBox)
		// The score, as big as the web UI makes it.
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeHeading(LOCTEXT("ScoreHeading", "EVENT SCORE"))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Bottom)
			[
				SNew(STextBlock)
				.Font(Font(28, TEXT("Bold")))
				.ColorAndOpacity(Accent)
				.Text_Lambda([this]()
				{
					// With no reference the backend compares the run with itself,
					// so every run "scores" the maximum: show no number at all.
					if (!Scoring || !Scoring->bHasReference) return LOCTEXT("NoScore", "--");
					return FText::FromString(FString::Printf(TEXT("%.1f"), Scoring->Score));
				})
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Bottom).Padding(8, 0, 0, 6)
			[
				SNew(STextBlock)
				.Font(Font(10))
				.ColorAndOpacity(Muted)
				.Text_Lambda([this]()
				{
					if (!Scoring) return FText::GetEmpty();
					return FText::Format(LOCTEXT("OfPoints", "/ {0} points   {1}"),
						FText::AsNumber(FMath::RoundToInt(Scoring->MaxPoints)), FText::FromString(Scoring->Event));
				})
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
		[
			SNew(STextBlock)
			.Font(Font(9))
			.AutoWrapText(true)
			.ColorAndOpacity(Warn)
			.Visibility_Lambda([this]() { return ScoringError.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible; })
			.Text_Lambda([this]() { return FText::Format(LOCTEXT("ScoreError", "No score: {0}"), FText::FromString(ScoringError)); })
		]

		// The time breakdown.
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			MakeStatusRow(LOCTEXT("Elapsed", "Elapsed"),
				TAttribute<FText>::CreateLambda([this, Seconds]() { return Scoring ? Seconds(Scoring->Total) : FText::GetEmpty(); }),
				FSlateColor(Ink))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeStatusRow(LOCTEXT("Penalties", "Penalties"),
				TAttribute<FText>::CreateLambda([this]()
				{
					if (!Scoring) return FText::GetEmpty();
					return FText::FromString(FString::Printf(TEXT("+%.1f s   (DOO %d: +%.1f s, OC %d: +%.1f s)"),
						Scoring->DooPenalty + Scoring->OcPenalty, Scoring->Doo, Scoring->DooPenalty,
						Scoring->Oc, Scoring->OcPenalty));
				}),
				TAttribute<FSlateColor>::CreateLambda([this]()
				{
					return FSlateColor(Scoring && (Scoring->DooPenalty + Scoring->OcPenalty) > 0.0 ? Warn : Ink);
				}))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeStatusRow(LOCTEXT("Corrected", "Corrected"),
				TAttribute<FText>::CreateLambda([this, Seconds]() { return Scoring ? Seconds(Scoring->Corrected) : FText::GetEmpty(); }),
				FSlateColor(Accent))
		]

		// The reference time the score is computed against.
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeHeading(LOCTEXT("RefHeading", "REFERENCE TIME (T_BEST)"))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 8, 0)
			[
				SNew(SBox).WidthOverride(120.f)
				[
					SNew(SEditableTextBox)
					.Font(Font(10))
					.HintText(LOCTEXT("RefHint", "seconds"))
					.Text_Lambda([this]() { return FText::FromString(ReferenceInput); })
					.OnTextCommitted_Lambda([this](const FText& Text, ETextCommit::Type)
					{
						const FString In = Text.ToString().TrimStartAndEnd();
						if (In.IsEmpty() || (In.IsNumeric() && FCString::Atod(*In) > 0.0))
						{
							ReferenceInput = In;
							FetchScoring();
						}
						else
						{
							Say(TEXT("The reference time is a number of seconds, or empty for the best lap."), true);
						}
						// Hand the keys back to the car.
						FSlateApplication::Get().SetAllUserFocusToGameViewport();
					})
				]
			]
			+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
			[
				SNew(STextBlock)
				.Font(Font(9))
				.AutoWrapText(true)
				.ColorAndOpacity_Lambda([this]()
				{
					return FSlateColor(Scoring && !Scoring->bHasReference && ReferenceInput.IsEmpty() ? Warn : Muted);
				})
				.Text_Lambda([this]()
				{
					if (!ReferenceInput.IsEmpty()) return FText::Format(LOCTEXT("RefUsing", "Scoring against {0} s"), FText::FromString(ReferenceInput));
					if (Scoring && Scoring->bHasReference)
					{
						return FText::FromString(FString::Printf(TEXT("Scoring against %.3f s"), Scoring->Reference));
					}
					return LOCTEXT("RefNone",
						"None set. Without one the run is compared with itself and always gets full points, so no score is shown.");
				})
			]
		]

		// The laps.
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeHeading(LOCTEXT("LapsHeading", "LAP TIMES"))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SBox)
			.MaxDesiredHeight(220.f)
			[
				SNew(SScrollBox)
				+ SScrollBox::Slot()
				[
					SAssignNew(LapList, SVerticalBox)
				]
			]
		];
}

void SFSDSLauncherPanel::FetchScoring()
{
	if (bScoringInFlight || !Client.IsValid()) return;
	bScoringInFlight = true;

	FString Path = TEXT("/api/scoring/summary");
	if (!ReferenceInput.IsEmpty())
	{
		Path += TEXT("?t_best=") + ReferenceInput;
	}

	TWeakPtr<SFSDSLauncherPanel> Weak = StaticCastSharedRef<SFSDSLauncherPanel>(AsShared());
	Client->Get(Path, [Weak](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin();
		if (!P) return;
		P->bScoringInFlight = false;

		const TSharedPtr<FJsonObject> O = R.Object();
		FString Err;
		if (!R.Ok() || !O.IsValid())
		{
			P->ScoringError = R.Error.IsEmpty() ? TEXT("malformed reply") : R.Error;
			return;
		}
		// No referee yet comes back as 200 {"error": "No referee data"}.
		if (O->TryGetStringField(TEXT("error"), Err))
		{
			P->ScoringError = Err;
			return;
		}
		P->ScoringError.Reset();

		FScoring S;
		O->TryGetStringField(TEXT("event"), S.Event);
		O->TryGetNumberField(TEXT("laps_completed"), S.Laps);
		O->TryGetNumberField(TEXT("best_lap"), S.BestLap);
		O->TryGetNumberField(TEXT("total_elapsed"), S.Total);
		O->TryGetNumberField(TEXT("doo_count"), S.Doo);
		O->TryGetNumberField(TEXT("doo_penalty_s"), S.DooPenalty);
		O->TryGetNumberField(TEXT("oc_count"), S.Oc);
		O->TryGetNumberField(TEXT("oc_penalty_s"), S.OcPenalty);
		O->TryGetNumberField(TEXT("corrected_time"), S.Corrected);
		O->TryGetNumberField(TEXT("max_points"), S.MaxPoints);
		O->TryGetNumberField(TEXT("score"), S.Score);
		S.bHasReference = O->TryGetNumberField(TEXT("t_best_reference"), S.Reference);
		const TArray<TSharedPtr<FJsonValue>>* Laps = nullptr;
		if (O->TryGetArrayField(TEXT("lap_times"), Laps))
		{
			for (const TSharedPtr<FJsonValue>& L : *Laps)
			{
				double T = 0.0;
				if (L.IsValid() && L->TryGetNumber(T)) S.LapTimes.Add(T);
			}
		}

		const bool bLapsChanged = !P->Scoring || P->Scoring->LapTimes != S.LapTimes
			|| P->Scoring->BestLap != S.BestLap || (P->LapList.IsValid() && P->LapList->NumSlots() == 0);
		P->Scoring = MoveTemp(S);
		if (bLapsChanged) P->RebuildLapList();
	});
}

void SFSDSLauncherPanel::RebuildLapList()
{
	if (!LapList.IsValid()) return;
	LapList->ClearChildren();

	if (!Scoring || Scoring->LapTimes.Num() == 0)
	{
		LapList->AddSlot().AutoHeight()
		[
			SNew(STextBlock).Font(Font(10)).ColorAndOpacity(Muted).Text(LOCTEXT("NoLaps", "No laps completed yet"))
		];
		return;
	}

	for (int32 i = 0; i < Scoring->LapTimes.Num(); ++i)
	{
		const double T = Scoring->LapTimes[i];
		const bool bBest = FMath::IsNearlyEqual(T, Scoring->BestLap, 1e-6);
		LapList->AddSlot().AutoHeight().Padding(0, 1)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SBox).WidthOverride(LabelWidth)
				[
					SNew(STextBlock).Font(Font(10)).ColorAndOpacity(Muted)
					.Text(FText::Format(LOCTEXT("LapN", "Lap {0}"), FText::AsNumber(i + 1)))
				]
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 16, 0)
			[
				SNew(STextBlock).Font(Font(10, TEXT("Bold"))).ColorAndOpacity(Ink)
				.Text(FText::FromString(FString::Printf(TEXT("%.3f s"), T)))
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(STextBlock).Font(Font(10)).ColorAndOpacity(bBest ? Good : Muted)
				.Text(bBest ? LOCTEXT("Best", "BEST")
					: FText::FromString(FString::Printf(TEXT("+%.3f"), T - Scoring->BestLap)))
			]
		];
	}
}

TSharedRef<SWidget> SFSDSLauncherPanel::BuildTracksTab()
{
	auto NumberRow = [this](const FText& Label, int32* Value, int32 Min, int32 Max) -> TSharedRef<SWidget>
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				SNew(SBox).WidthOverride(LabelWidth)
				[
					SNew(STextBlock).Text(Label).Font(Font(10)).ColorAndOpacity(Muted)
				]
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SBox).WidthOverride(90.f)
				[
					SNew(SSpinBox<int32>)
					.Font(Font(10))
					.MinValue(Min).MaxValue(Max)
					.Value_Lambda([Value]() { return *Value; })
					.OnValueChanged_Lambda([Value](int32 V) { *Value = V; })
				]
			];
	};

	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeHeading(LOCTEXT("TracksHeading", "TRACKS"))
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SBox)
			.MaxDesiredHeight(170.f)
			[
				SNew(SScrollBox)
				+ SScrollBox::Slot()
				[
					SAssignNew(TrackInfoList, SVerticalBox)
				]
			]
		]

		// Preview of the selected track, as the web UI draws it.
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
		[
			SNew(SBox)
			.HeightOverride(230.f)
			.HAlign(HAlign_Center)
			.VAlign(VAlign_Center)
			[
				SNew(SOverlay)
				+ SOverlay::Slot()
				[
					SNew(SScaleBox)
					.Stretch(EStretch::ScaleToFit)
					.Visibility_Lambda([this]() { return PreviewBrush.IsValid() ? EVisibility::Visible : EVisibility::Collapsed; })
					[
						SNew(SImage).Image_Lambda([this]() -> const FSlateBrush* { return PreviewBrush.Get(); })
					]
				]
				+ SOverlay::Slot().HAlign(HAlign_Center).VAlign(VAlign_Center)
				[
					SNew(STextBlock)
					.Font(Font(10))
					.ColorAndOpacity(Muted)
					.Visibility_Lambda([this]() { return PreviewState.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible; })
					.Text_Lambda([this]() { return FText::FromString(PreviewState); })
				]
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
			[
				MakeButton(LOCTEXT("TracksLoad", "Load"), GoBtn,
					FOnClicked::CreateSP(this, &SFSDSLauncherPanel::OnTracksLoad),
					TAttribute<bool>::CreateLambda([this]() { return CanAct() && bSimConnected && !SelectedInfo.IsEmpty(); }))
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
			[
				MakeButton(
					TAttribute<FText>::CreateLambda([this]()
					{
						return IsArmed(TEXT("delete")) ? LOCTEXT("DeleteConfirm", "Click again to delete")
							: LOCTEXT("Delete", "Delete");
					}),
					StopBtn,
					FOnClicked::CreateSP(this, &SFSDSLauncherPanel::OnTracksDelete),
					TAttribute<bool>::CreateLambda([this]()
					{
						const FTrackInfo* T = TrackInfos.FindByPredicate([this](const FTrackInfo& I) { return I.Name == SelectedInfo; });
						return CanAct() && T && !T->bBuiltin;
					}))
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				MakeButton(LOCTEXT("TracksRefresh", "Refresh"), PlainBtn,
					FOnClicked::CreateLambda([this]() { FetchLists(); return FReply::Handled(); }),
					TAttribute<bool>::CreateLambda([this]() { return bBackendUp && !bListsLoading; }))
			]
		]

		// A random track, from the same generator and defaults as the web UI.
		+ SVerticalBox::Slot().AutoHeight()
		[
			MakeHeading(LOCTEXT("GenerateHeading", "GENERATE A RANDOM TRACK"))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 1)
		[
			NumberRow(LOCTEXT("GenPoints", "Points"), &GenPoints, 10, 200)
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 1)
		[
			NumberRow(LOCTEXT("GenRegions", "Regions"), &GenRegions, 5, 100)
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 1)
		[
			NumberRow(LOCTEXT("GenMax", "Max size (m)"), &GenMaxBound, 50, 300)
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 1)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				SNew(SBox).WidthOverride(LabelWidth)
				[
					SNew(STextBlock).Text(LOCTEXT("GenName", "Name")).Font(Font(10)).ColorAndOpacity(Muted)
				]
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
			[
				SNew(SBox).WidthOverride(160.f)
				[
					SNew(SEditableTextBox)
					.Font(Font(10))
					.HintText(LOCTEXT("GenNameHint", "auto"))
					.Text_Lambda([this]() { return FText::FromString(GenName); })
					.OnTextCommitted_Lambda([this](const FText& Text, ETextCommit::Type)
					{
						GenName = Text.ToString().TrimStartAndEnd();
						FSlateApplication::Get().SetAllUserFocusToGameViewport();
					})
				]
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				MakeButton(LOCTEXT("Generate", "Generate"), GoBtn,
					FOnClicked::CreateSP(this, &SFSDSLauncherPanel::OnGenerate),
					TAttribute<bool>::CreateLambda([this]() { return CanAct(); }))
			]
		];
}

void SFSDSLauncherPanel::RebuildTrackInfoList()
{
	if (!TrackInfoList.IsValid()) return;
	TrackInfoList->ClearChildren();

	if (TrackInfos.Num() == 0)
	{
		TrackInfoList->AddSlot().AutoHeight()
		[
			SNew(STextBlock).Font(Font(10)).ColorAndOpacity(Muted)
			.Text(bBackendUp ? LOCTEXT("NoTracks", "No tracks") : LOCTEXT("TracksNoBackend", "The backend isn't running"))
		];
		return;
	}

	for (const FTrackInfo& T : TrackInfos)
	{
		const FString Name = T.Name;
		const FString Detail = FString::Printf(TEXT("%d cones (B %d  Y %d  O %d)%s"), T.Cones, T.Blue, T.Yellow, T.Orange,
			T.bBuiltin ? *FString::Printf(TEXT("   built-in%s%s"), T.EventType.IsEmpty() ? TEXT("") : TEXT(", "), *T.EventType) : TEXT(""));
		TrackInfoList->AddSlot().AutoHeight()
		[
			SNew(SButton)
			.IsFocusable(false)          // a focused button would eat Space (the brake)
			.ContentPadding(FMargin(6.f, 2.f))
			.ButtonColorAndOpacity_Lambda([this, Name]()
			{
				return FSlateColor(SelectedInfo == Name ? PlainBtn : FLinearColor(0.f, 0.f, 0.f, 0.f));
			})
			.OnClicked_Lambda([this, Name]() { SelectTrackInfo(Name); return FReply::Handled(); })
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(1.f).Padding(0, 0, 10, 0)
				[
					SNew(STextBlock)
					.Font(Font(10, TEXT("Bold")))
					.OverflowPolicy(ETextOverflowPolicy::Ellipsis)
					.ColorAndOpacity_Lambda([this, Name]() { return FSlateColor(SelectedInfo == Name ? Accent : Ink); })
					.Text(FText::FromString(Name))
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[
					SNew(STextBlock).Font(Font(9)).ColorAndOpacity(Muted).Text(FText::FromString(Detail))
				]
			]
		];
	}
}

void SFSDSLauncherPanel::SelectTrackInfo(const FString& Name)
{
	SelectedInfo = Name;
	if (Name != PreviewFor) FetchPreview(Name);
}

void SFSDSLauncherPanel::FetchPreview(const FString& Name)
{
	if (!Client.IsValid() || Name.IsEmpty()) return;
	PreviewFor = Name;
	PreviewBrush.Reset();
	PreviewState = TEXT("Drawing the track...");

	TWeakPtr<SFSDSLauncherPanel> Weak = StaticCastSharedRef<SFSDSLauncherPanel>(AsShared());
	Client->Get(FString::Printf(TEXT("/api/track/%s/preview"), *FFSDSMissionControlClient::EncodeSegment(Name)),
		[Weak, Name](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSLauncherPanel> P = Weak.Pin();
		if (!P || P->PreviewFor != Name) return;   // a newer selection won

		FString Base64;
		const TSharedPtr<FJsonObject> O = R.Object();
		if (!R.Ok() || !O.IsValid() || !O->TryGetStringField(TEXT("image"), Base64))
		{
			P->PreviewState = FString::Printf(TEXT("No preview: %s"), R.Error.IsEmpty() ? TEXT("malformed reply") : *R.Error);
			return;
		}

		// The backend draws it with matplotlib and sends a base64 PNG.
		TArray<uint8> Png;
		TArray<uint8> Bgra;
		IImageWrapperModule& Images = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		const TSharedPtr<IImageWrapper> Wrapper = Images.CreateImageWrapper(EImageFormat::PNG);
		if (!FBase64::Decode(Base64, Png) || !Wrapper.IsValid()
			|| !Wrapper->SetCompressed(Png.GetData(), Png.Num())
			|| !Wrapper->GetRaw(ERGBFormat::BGRA, 8, Bgra))
		{
			P->PreviewState = TEXT("No preview: the image didn't decode");
			return;
		}
		P->PreviewBrush = FSlateDynamicImageBrush::CreateWithImageData(
			FName(*FString::Printf(TEXT("FSDSTrackPreview_%d"), ++P->PreviewSerial)),
			FVector2D((double)Wrapper->GetWidth(), (double)Wrapper->GetHeight()), Bgra);
		P->PreviewState = P->PreviewBrush.IsValid() ? FString() : TEXT("No preview");
	}, 15.f);
}

TSharedRef<SWidget> SFSDSLauncherPanel::BuildTelemetryTab()
{
	// Read from the car itself at paint time: the panel runs in the sim, so
	// it needs neither the backend nor its websocket for this.
	auto Row = [this](const FText& Label, TFunction<FString(const AFSDSVehiclePawn&)> Value) -> TSharedRef<SWidget>
	{
		return MakeStatusRow(Label,
			TAttribute<FText>::CreateLambda([this, Value]()
			{
				const AFSDSVehiclePawn* P = Pawn.Get();
				return P ? FText::FromString(Value(*P)) : LOCTEXT("NoCar", "no car");
			}),
			FSlateColor(Ink));
	};
	// The motor's regen state. Torque is at the motor shaft; negative is
	// regen. The cap is whichever of the two limits binds at this speed.
	struct FRegen { float Torque = 0, PowerW = 0, Avail = 0; bool bPowerCapped = false, bValid = false; };
	auto Regen = [](const AFSDSVehiclePawn& P) -> FRegen
	{
		FRegen R;
		const UEmraxMotor* M = P.Motor;
		if (!M) return R;
		R.bValid = true;
		const float Omega = FMath::Abs(M->GetMechRpm()) * 2.f * PI / 60.f;
		R.Torque = FMath::Max(0.f, -M->GetTorqueNm());
		R.PowerW = R.Torque * Omega;
		const float PowerCapTorque = Omega > 1e-3f ? M->P.MaxRegenPowerW / Omega : TNumericLimits<float>::Max();
		R.bPowerCapped = PowerCapTorque < M->P.MaxRegenTorqueNm;
		R.Avail = FMath::Min(M->P.MaxRegenTorqueNm, PowerCapTorque);
		return R;
	};

	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("TelCar", "CAR")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			Row(LOCTEXT("TelSpeed", "Speed"), [](const AFSDSVehiclePawn& P)
			{
				const float V = P.GetCarState().Speed;
				return FString::Printf(TEXT("%.2f m/s   %.1f km/h"), V, V * 3.6f);
			})
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			Row(LOCTEXT("TelRpm", "Motor"), [](const AFSDSVehiclePawn& P)
			{
				// Rounded to an int so a tiny negative jitter at rest shows 0, not -0.
				return FString::Printf(TEXT("%d rpm"), FMath::RoundToInt(P.GetCarState().RPM));
			})
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			Row(LOCTEXT("TelPedals", "Pedals"), [](const AFSDSVehiclePawn& P)
			{
				const AFSDSVehiclePawn::FCarControls C = P.GetCarControls();
				return FString::Printf(TEXT("throttle %.0f%%   brake %.0f%%%s"),
					FMath::Clamp(C.Throttle, 0.f, 1.f) * 100.f, FMath::Clamp(C.Regen, 0.f, 1.f) * 100.f,
					C.bHandbrake ? TEXT("   mechanical brake on") : TEXT(""));
			})
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			Row(LOCTEXT("TelSteer", "Steering"), [](const AFSDSVehiclePawn& P)
			{
				const float S = P.GetCarControls().Steering;
				return FString::Printf(TEXT("%+.2f   %s"), S,
					FMath::Abs(S) < 0.01f ? TEXT("straight") : S > 0.f ? TEXT("right") : TEXT("left"));
			})
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			Row(LOCTEXT("TelDriver", "Driver"), [](const AFSDSVehiclePawn& P)
			{
				return FString::Printf(TEXT("%s%s"), P.IsApiControlEnabled() ? TEXT("autonomous") : TEXT("manual"),
					P.IsEbsLatched() ? TEXT(", EBS latched") : TEXT(""));
			})
		]

		+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("TelRegen", "REGEN (MOTOR SIDE)")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			Row(LOCTEXT("TelRegenNow", "Absorbing"), [Regen](const AFSDSVehiclePawn& P)
			{
				const FRegen R = Regen(P);
				if (!R.bValid) return FString(TEXT("no motor model"));
				return FString::Printf(TEXT("%.1f Nm   %.2f kW"), R.Torque, R.PowerW / 1000.f);
			})
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			Row(LOCTEXT("TelRegenAvail", "Available"), [Regen](const AFSDSVehiclePawn& P)
			{
				const FRegen R = Regen(P);
				if (!R.bValid) return FString();
				return FString::Printf(TEXT("%.1f Nm   (%s)"), R.Avail,
					R.bPowerCapped ? TEXT("power cap binds") : TEXT("torque cap binds"));
			})
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			Row(LOCTEXT("TelRegenLimits", "Limits"), [](const AFSDSVehiclePawn& P)
			{
				const UEmraxMotor* M = P.Motor;
				return M ? FString::Printf(TEXT("%.0f Nm, %.1f kW"), M->P.MaxRegenTorqueNm, M->P.MaxRegenPowerW / 1000.f)
					: FString();
			})
		]

		+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("TelPose", "POSITION (ENU, METRES)")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			Row(LOCTEXT("TelXyz", "x  y  z"), [](const AFSDSVehiclePawn& P)
			{
				const FVector E = FSDSCoord::UEToENU(P.GetCarState().Position);
				return FString::Printf(TEXT("%.3f   %.3f   %.3f"), E.X, E.Y, E.Z);
			})
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			Row(LOCTEXT("TelYaw", "Heading"), [](const AFSDSVehiclePawn& P)
			{
				const FQuat Q = FSDSCoord::UEQuatToENU(P.GetCarState().Orientation);
				const float Yaw = FMath::RadiansToDegrees(FMath::Atan2(2.f * (Q.W * Q.Z + Q.X * Q.Y),
					1.f - 2.f * (Q.Y * Q.Y + Q.Z * Q.Z)));
				return FString::Printf(TEXT("%.1f deg (0 = east, counter-clockwise)"), Yaw);
			})
		];
}

TSharedRef<SWidget> SFSDSLauncherPanel::BuildStackBlock()
{
	return SNew(SBox)
		.Visibility_Lambda([this]()
		{
			const bool bShow = (bPolled && !bBackendUp) || (Stack.IsValid() && (Stack->IsRunning() || Stack->HasFinished()) && !bBackendUp);
			return bShow ? EVisibility::Visible : EVisibility::Collapsed;
		})
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("StackHeading", "STACK")) ]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
			[
				SNew(STextBlock)
				.AutoWrapText(true)
				.Font(Font(10))
				.ColorAndOpacity(Ink)
				.Text_Lambda([this]()
				{
					if (ComposeDir.IsEmpty())
					{
						return LOCTEXT("NoCompose",
							"Mission Control isn't running, and no docker-compose.yml was found. "
							"Start it from the repo with `docker compose up -d`, or set "
							"MissionControl.ComposeDir in settings.json.");
					}
					return FText::Format(LOCTEXT("StackHint",
						"Mission Control isn't running. Start the Docker stack in {0}."),
						FText::FromString(ComposeDir));
				})
			]
			+ SVerticalBox::Slot().AutoHeight()
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth()
				[
					MakeButton(
						TAttribute<FText>::CreateLambda([this]()
						{
							return Stack.IsValid() && Stack->IsRunning()
								? LOCTEXT("StackRunning", "Starting...")
								: LOCTEXT("StackStart", "Start stack");
						}),
						GoBtn,
						FOnClicked::CreateLambda([this]()
						{
							OnStartStack.ExecuteIfBound();
							return FReply::Handled();
						}),
						TAttribute<bool>::CreateLambda([this]()
						{
							return !ComposeDir.IsEmpty() && !(Stack.IsValid() && Stack->IsRunning());
						}))
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
			[
				SNew(STextBlock)
				.AutoWrapText(true)
				.Font(Font(9))
				.ColorAndOpacity(Muted)
				.Text(this, &SFSDSLauncherPanel::StackText)
			]
		];
}

TSharedRef<SWidget> SFSDSLauncherPanel::BuildTrackBlock()
{
	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("TrackHeading", "TRACK")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.f).Padding(0, 0, 6, 0)
			[
				SAssignNew(TrackCombo, SComboBox<FTrackPtr>)
				.IsFocusable(false)
				.OptionsSource(&Tracks)
				.OnGenerateWidget_Lambda([](FTrackPtr Item)
				{
					return SNew(STextBlock).Text(FText::FromString(*Item)).Font(Font(10));
				})
				.OnSelectionChanged_Lambda([this](FTrackPtr Item, ESelectInfo::Type)
				{
					if (Item.IsValid()) SelectedTrack = Item;
				})
				.IsEnabled_Lambda([this]() { return bBackendUp && Tracks.Num() > 0; })
				[
					SNew(STextBlock)
					.Font(Font(10))
					.Text_Lambda([this]()
					{
						if (SelectedTrack.IsValid()) return FText::FromString(*SelectedTrack);
						return bListsLoaded ? LOCTEXT("NoTracks", "no tracks") : LOCTEXT("TracksLoading", "...");
					})
				]
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
			[
				MakeButton(LOCTEXT("Load", "Load"), PlainBtn,
					FOnClicked::CreateSP(this, &SFSDSLauncherPanel::OnLoadTrack),
					TAttribute<bool>::CreateLambda([this]() { return CanAct() && bSimConnected && SelectedTrack.IsValid(); }))
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				MakeButton(LOCTEXT("Refresh", "Refresh"), PlainBtn,
					FOnClicked::CreateLambda([this]() { FetchLists(); return FReply::Handled(); }),
					TAttribute<bool>::CreateLambda([this]() { return bBackendUp && !bListsLoading; }))
			]
		];
}

TSharedRef<SWidget> SFSDSLauncherPanel::BuildSessionBlock()
{
	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("SessionHeading", "SESSION")) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[
			SAssignNew(MissionCombo, SComboBox<FMissionPtr>)
			.IsFocusable(false)
			.OptionsSource(&Missions)
			.OnGenerateWidget_Lambda([](FMissionPtr Item)
			{
				return SNew(STextBlock).Text(FText::FromString(Item->Label)).Font(Font(10));
			})
			.OnSelectionChanged_Lambda([this](FMissionPtr Item, ESelectInfo::Type)
			{
				if (Item.IsValid()) SelectedMission = Item;
			})
			.IsEnabled_Lambda([this]() { return bBackendUp && Missions.Num() > 0 && Busy.IsEmpty(); })
			[
				SNew(STextBlock)
				.Font(Font(10))
				.Text_Lambda([this]()
				{
					return SelectedMission.IsValid() ? FText::FromString(SelectedMission->Label)
						: LOCTEXT("MissionsLoading", "...");
				})
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
		[
			SNew(SHorizontalBox)
			// Laps, for the missions that have them. Buttons rather than a
			// spin box: a spin box takes the keyboard while you edit it.
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 14, 0)
			[
				SNew(SHorizontalBox)
				.Visibility_Lambda([this]()
				{
					return SelectedMission.IsValid() && SelectedMission->bLaps ? EVisibility::Visible : EVisibility::Collapsed;
				})
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
				[
					SNew(STextBlock).Text(LOCTEXT("Laps", "Laps")).Font(Font(10)).ColorAndOpacity(Muted)
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[
					MakeButton(LOCTEXT("Minus", "-"), PlainBtn,
						FOnClicked::CreateLambda([this]() { NumLaps = FMath::Max(1, NumLaps - 1); return FReply::Handled(); }),
						TAttribute<bool>::CreateLambda([this]() { return Busy.IsEmpty(); }))
				]
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(8, 0)
				[
					SNew(SBox).WidthOverride(22.f).HAlign(HAlign_Center)
					[
						SNew(STextBlock).Font(Font(11, TEXT("Bold"))).ColorAndOpacity(Ink)
						.Text_Lambda([this]() { return FText::AsNumber(NumLaps); })
					]
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[
					MakeButton(LOCTEXT("Plus", "+"), PlainBtn,
						FOnClicked::CreateLambda([this]() { NumLaps = FMath::Min(99, NumLaps + 1); return FReply::Handled(); }),
						TAttribute<bool>::CreateLambda([this]() { return Busy.IsEmpty(); }))
				]
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				SNew(SCheckBox)
				.IsFocusable(false)
				.IsChecked_Lambda([this]() { return bRecordBag ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
				.OnCheckStateChanged_Lambda([this](ECheckBoxState S) { bRecordBag = S == ECheckBoxState::Checked; })
				.IsEnabled_Lambda([this]()
				{
					return Busy.IsEmpty() && !(SelectedMission.IsValid() && SelectedMission->Kind == TEXT("sim_benchmark_control"));
				})
				[
					SNew(STextBlock).Text(LOCTEXT("RecordBag", "Record bag (mcap)")).Font(Font(10)).ColorAndOpacity(Ink)
				]
			]
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
			[
				MakeButton(LOCTEXT("Start", "Start session"), GoBtn,
					FOnClicked::CreateSP(this, &SFSDSLauncherPanel::OnStartSession),
					TAttribute<bool>::CreateLambda([this]() { return CanAct() && bSimConnected && SelectedMission.IsValid(); }))
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				MakeButton(
					TAttribute<FText>::CreateLambda([this]()
					{
						return IsArmed(TEXT("stop")) ? LOCTEXT("StopConfirm", "Click again to stop")
							: LOCTEXT("Stop", "Stop session");
					}),
					StopBtn,
					FOnClicked::CreateSP(this, &SFSDSLauncherPanel::OnStopSession),
					TAttribute<bool>::CreateSP(this, &SFSDSLauncherPanel::CanAct))
			]
		];
}

TSharedRef<SWidget> SFSDSLauncherPanel::BuildCarBlock()
{
	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("CarHeading", "CAR")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
			[
				MakeButton(
					TAttribute<FText>::CreateLambda([this]()
					{
						return IsArmed(TEXT("reset")) ? LOCTEXT("ResetConfirm", "Click again to reset")
							: LOCTEXT("Reset", "Reset car");
					}),
					PlainBtn,
					FOnClicked::CreateSP(this, &SFSDSLauncherPanel::OnReset),
					TAttribute<bool>::CreateLambda([this]() { return CanAct() && bSimConnected; }))
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
			[
				MakeButton(
					TAttribute<FText>::CreateLambda([this]()
					{
						return bPaused ? LOCTEXT("Resume", "Resume") : LOCTEXT("Pause", "Pause");
					}),
					PlainBtn,
					FOnClicked::CreateSP(this, &SFSDSLauncherPanel::OnPause),
					TAttribute<bool>::CreateLambda([this]() { return CanAct() && bSimConnected; }))
			]
			+ SHorizontalBox::Slot().FillWidth(1.f)
			[
				SNew(SButton)
				.IsFocusable(false)
				.HAlign(HAlign_Center)
				.ContentPadding(FMargin(12.f, 5.f))
				.ButtonColorAndOpacity_Lambda([this]() { return FSlateColor(bResActive ? ResLive : ResBtn); })
				.OnClicked(this, &SFSDSLauncherPanel::OnRes)
				.IsEnabled_Lambda([this]() { return bBackendUp && bSimConnected; })
				[
					SNew(STextBlock)
					.Font(Font(10, TEXT("Bold")))
					.ColorAndOpacity(FLinearColor::White)
					.Text_Lambda([this]()
					{
						return bResActive ? LOCTEXT("ResRelease", "RES ACTIVE: release")
							: LOCTEXT("ResActivate", "RES");
					})
				]
			]
		];
}

// ---------------------------------------------------------------------------
// Status text

bool SFSDSLauncherPanel::CanAct() const
{
	return bBackendUp && Busy.IsEmpty();
}

FText SFSDSLauncherPanel::BackendText() const
{
	if (!bPolled) return LOCTEXT("Checking", "checking...");
	if (bBackendUp) return FText::FromString(Client->GetBaseUrl());
	return FText::Format(LOCTEXT("BackendDown", "not running at {0}"), FText::FromString(Client->GetBaseUrl()));
}

FSlateColor SFSDSLauncherPanel::BackendColor() const
{
	if (!bPolled) return Muted;
	return bBackendUp ? Good : Bad;
}

FText SFSDSLauncherPanel::SimText() const
{
	if (!bBackendUp) return LOCTEXT("SimUnknown", "-");
	if (!bSimConnected) return LOCTEXT("SimDown", "the backend can't reach the sim");
	return bPaused ? LOCTEXT("SimPaused", "paused")
		: FText::Format(LOCTEXT("SimUp", "linked, {0} fps"), FText::AsNumber(Fps));
}

FSlateColor SFSDSLauncherPanel::SimColor() const
{
	if (!bBackendUp) return Muted;
	if (!bSimConnected) return Bad;
	return bPaused ? Warn : Good;
}

FText SFSDSLauncherPanel::TrackText() const
{
	const AFSDSReferee* Ref = Referee.Get();
	const int32 Cones = Ref ? Ref->GetState().Cones.Num() : 0;
	return Cones > 0 ? FText::Format(LOCTEXT("Cones", "{0} cones on the ground"), FText::AsNumber(Cones))
		: LOCTEXT("NoTrack", "no track loaded");
}

FText SFSDSLauncherPanel::LapText() const
{
	const AFSDSReferee* Ref = Referee.Get();
	if (!Ref) return LOCTEXT("NoRef", "-");
	const FFSDSRefereeState S = Ref->GetState();
	FString Out = FString::Printf(TEXT("lap %d/%d   DOO %d   OC %d"),
		S.Laps.Num(), S.RequiredLaps, S.DooCounter, S.OffTrackCounter);
	if (S.bFinished) Out += TEXT("   finished");
	return FText::FromString(Out);
}

FText SFSDSLauncherPanel::PipelineText() const
{
	if (!bBackendUp) return LOCTEXT("PipeUnknown", "-");
	return bPipeline ? LOCTEXT("PipeOn", "running") : LOCTEXT("PipeOff", "idle");
}

FSlateColor SFSDSLauncherPanel::PipelineColor() const
{
	return bBackendUp && bPipeline ? Good : Muted;
}

FText SFSDSLauncherPanel::ResText() const
{
	if (!bBackendUp) return LOCTEXT("ResUnknown", "-");
	return bResActive ? LOCTEXT("ResOn", "ACTIVE: car held") : LOCTEXT("ResOff", "released");
}

FSlateColor SFSDSLauncherPanel::ResColor() const
{
	return bBackendUp && bResActive ? Bad : Muted;
}

FText SFSDSLauncherPanel::BagText() const
{
	if (BagState.IsEmpty() || BagState == TEXT("none")) return FText::GetEmpty();
	return FText::FromString(BagName.IsEmpty() ? BagState : FString::Printf(TEXT("%s, %s"), *BagState, *BagName));
}

FText SFSDSLauncherPanel::StackText() const
{
	if (!Stack.IsValid()) return FText::GetEmpty();
	if (Stack->IsRunning())
	{
		return FText::FromString(Stack->LastLine().IsEmpty() ? FString(TEXT("docker compose up -d")) : Stack->LastLine());
	}
	if (Stack->HasFinished())
	{
		if (Stack->ExitCode() != 0)
		{
			if (Stack->Output().Contains(TEXT("Cannot connect to the Docker daemon")))
			{
				return LOCTEXT("DockerDown", "Docker isn't running. Start Docker, then press Start stack again.");
			}
			return FText::Format(LOCTEXT("StackFailed", "docker compose failed ({0}): {1}"),
				FText::AsNumber(Stack->ExitCode()), FText::FromString(Stack->LastLine()));
		}
		return LOCTEXT("StackWaiting", "Containers are up; waiting for the backend to answer.");
	}
	return FText::GetEmpty();
}

#undef LOCTEXT_NAMESPACE
