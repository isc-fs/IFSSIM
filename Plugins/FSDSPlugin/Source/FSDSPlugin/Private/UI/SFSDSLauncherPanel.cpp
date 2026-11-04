#include "UI/SFSDSLauncherPanel.h"

#include "FSDSReferee.h"
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
		for (const TSharedPtr<FJsonValue>& V : R.Body->AsArray())
		{
			const TSharedPtr<FJsonObject>* O = nullptr;
			FString Name;
			if (V->TryGetObject(O) && (*O)->TryGetStringField(TEXT("name"), Name))
			{
				P->Tracks.Add(MakeShared<FString>(Name));
				if (Name == Keep) P->SelectedTrack = P->Tracks.Last();
			}
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
		+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 4, 0)
		[
			MakeTab(LOCTEXT("TabSession", "Session"), ETab::Session)
		]
		+ SHorizontalBox::Slot().AutoWidth()
		[
			MakeTab(LOCTEXT("TabLog", "Log"), ETab::Log)
		];
}

TSharedRef<SWidget> SFSDSLauncherPanel::MakeTab(const FText& Label, ETab Tab)
{
	return SNew(SButton)
		.IsFocusable(false)          // a focused button would eat Space (the brake)
		.ContentPadding(FMargin(12.f, 3.f))
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
