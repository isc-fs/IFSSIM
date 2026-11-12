#include "UI/SFSDSBenchPage.h"

#include "UI/FSDSMissionControlClient.h"
#include "UI/FSDSStackStarter.h"

#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "Widgets/Layout/SWrapBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "FSDSBenchPage"

// Always qualified: the UI files share unity blobs.
namespace BenchUi
{
	const FLinearColor Accent = FLinearColor::FromSRGBColor(FColor(255, 184, 28));
	const FLinearColor Ink = FLinearColor::FromSRGBColor(FColor(232, 232, 236));
	const FLinearColor Muted = FLinearColor::FromSRGBColor(FColor(182, 182, 192));
	const FLinearColor Good = FLinearColor::FromSRGBColor(FColor(74, 222, 128));
	const FLinearColor Bad = FLinearColor::FromSRGBColor(FColor(248, 113, 113));
	const FLinearColor Yellow = FLinearColor::FromSRGBColor(FColor(250, 204, 21));
	const FLinearColor Grey = FLinearColor::FromSRGBColor(FColor(110, 110, 120));
	const FLinearColor PlainBtn = FLinearColor::FromSRGBColor(FColor(64, 64, 72));
	const FLinearColor GoBtn = FLinearColor::FromSRGBColor(FColor(34, 197, 94));
	const FLinearColor Card = FLinearColor::FromSRGBColor(FColor(34, 34, 40));
	// Solid colours: the page is opaque, so the world behind doesn't show through.
	const FLinearColor Page = FLinearColor::FromSRGBColor(FColor(20, 20, 24));
	const FLinearColor Field = FLinearColor::FromSRGBColor(FColor(30, 30, 36));
	const FLinearColor Edge = FLinearColor::FromSRGBColor(FColor(62, 62, 72));
	const FLinearColor ChoiceOff = FLinearColor::FromSRGBColor(FColor(54, 54, 62));
	const FLinearColor ChoiceOffHover = FLinearColor::FromSRGBColor(FColor(72, 72, 82));
	const FLinearColor ChoiceOnText = FLinearColor::FromSRGBColor(FColor(24, 20, 10));

	constexpr double QueueEveryS = 3.0;
	constexpr double RigEveryS = 15.0;     // the worker checks the rig once a minute
	constexpr double CheckAfterS = 0.5;    // after the last edit
	constexpr double WorkerStartS = 20.0;  // for its first heartbeat
	constexpr float LabelW = 170.f;
	constexpr float SideW = 400.f;

	const TCHAR* const Missions = TEXT("missions");
	const TCHAR* const PipelineOnSetting = TEXT("pipeline_on");
	const TCHAR* const OnThisPc = TEXT("bench_pc");
	const TCHAR* const OnPanda = TEXT("latte_panda");

	/** The settings a person picks, in words; the rest keep bench.yaml's help. */
	struct FNice { const TCHAR* Name; const TCHAR* Label; const TCHAR* Help; };
	const FNice Nice[] = {
		{ TEXT("laps"), TEXT("Laps"), TEXT("For every mission. Empty: each mission's own.") },
		{ TEXT("timeout_s"), TEXT("Time limit (s)"), TEXT("Per run, for every mission. Empty: each mission's own.") },
		{ TEXT("stack"), TEXT("Pipeline stack"), TEXT("Its own: started from the code being tested. The running one: the stack that's already up.") },
		{ TEXT("replace_stack"), TEXT("Stop the running stack"), TEXT("Stop a running pipeline stack for the benchmark, and start it again after.") },
		{ TEXT("panda_start"), TEXT("LattePanda: start"), TEXT("The command that starts the pipeline on the LattePanda.") },
		{ TEXT("panda_stop"), TEXT("LattePanda: stop"), TEXT("The command that stops it.") },
		{ TEXT("panda_sha"), TEXT("LattePanda: commit"), TEXT("The command that prints the pipeline commit there.") },
		{ TEXT("gt_range_m"), TEXT("Ground truth range (m)"), TEXT("Cones within this distance count.") },
		{ TEXT("gt_hfov_deg"), TEXT("Ground truth view (deg)"), TEXT("Half the horizontal field of view.") },
		{ TEXT("gt_min_range_m"), TEXT("Ground truth from (m)"), TEXT("Cones closer than this are left out.") },
		{ TEXT("gt_scan_period_ms"), TEXT("LiDAR period (ms)"), TEXT("Used to time the ground-truth cones.") },
		{ TEXT("max_frames"), TEXT("First frames only"), TEXT("Perception on the first N LiDAR frames only.") },
		{ TEXT("profile"), TEXT("Time each stage"), TEXT("Perception's per-stage timings.") },
		{ TEXT("strategy"), TEXT("SLAM strategy"), TEXT("Empty: trackdrive.") },
		{ TEXT("resynth_odom"), TEXT("Rebuild odometry"), TEXT("SLAM rebuilds /odom from the bag's sensors.") },
	};

	FSlateFontInfo Font(int32 Size, const TCHAR* Weight = TEXT("Regular"))
	{
		return FCoreStyle::GetDefaultFontStyle(Weight, Size);
	}

	const FSlateBrush* White()
	{
		return FCoreStyle::Get().GetBrush("WhiteBrush");
	}

	TSharedRef<SWidget> Button(TAttribute<FText> Label, FLinearColor Color, TFunction<void()> OnClick, TAttribute<bool> Enabled = true)
	{
		return SNew(SButton)
			.IsFocusable(false)
			.ButtonColorAndOpacity(Color)
			.ContentPadding(FMargin(10.f, 4.f))
			.HAlign(HAlign_Center)
			.IsEnabled(Enabled)
			.OnClicked_Lambda([OnClick]() { OnClick(); return FReply::Handled(); })
			[
				SNew(STextBlock).Text(Label).Font(Font(10, TEXT("Bold"))).ColorAndOpacity(FLinearColor::White)
			];
	}

	/** A choice button in flat colours: on is filled amber with dark text, off is grey
	 *  with white text, a little lighter under the mouse. */
	TSharedRef<SWidget> Choice(TAttribute<FText> Label, TFunction<bool()> IsOn, TFunction<void()> OnPick,
		const FText& Tip = FText::GetEmpty(), TAttribute<bool> Enabled = true, int32 Size = 10)
	{
		// The fill needs the button's hover, and the button is made after its content.
		const TSharedRef<TWeakPtr<SButton>> Self = MakeShared<TWeakPtr<SButton>>();
		const TSharedRef<SButton> Btn = SNew(SButton)
			.IsFocusable(false)
			.ButtonStyle(&FCoreStyle::Get().GetWidgetStyle<FButtonStyle>("NoBorder"))
			.ContentPadding(FMargin(0.f))
			.ToolTipText(Tip)
			.IsEnabled(Enabled)
			.OnClicked_Lambda([OnPick]() { OnPick(); return FReply::Handled(); })
			[
				SNew(SBorder)
				.BorderImage(White())
				.Padding(FMargin(10.f, 5.f))
				.BorderBackgroundColor_Lambda([IsOn, Self, Enabled]()
				{
					const TSharedPtr<SButton> B = Self->Pin();
					const bool bOn = Enabled.Get(true);
					FLinearColor C = IsOn() ? Accent : (B.IsValid() && B->IsHovered() && bOn) ? ChoiceOffHover : ChoiceOff;
					if (!bOn) C.A = 0.45f;
					return FSlateColor(C);
				})
				[
					SNew(STextBlock).Text(Label).Font(Font(Size, TEXT("Bold")))
					.ColorAndOpacity_Lambda([IsOn, Enabled]()
					{
						return FSlateColor(IsOn() ? ChoiceOnText : Enabled.Get(true) ? FLinearColor::White : Muted);
					})
				]
			];
		*Self = Btn;
		return Btn;
	}

	TSharedRef<SWidget> Heading(const FText& Title)
	{
		return SNew(STextBlock).Text(Title).Font(Font(10, TEXT("Bold"))).ColorAndOpacity(Accent);
	}

	TSharedRef<SWidget> Small(const TAttribute<FText>& Text, FLinearColor Color = Muted)
	{
		return SNew(STextBlock).Text(Text).Font(Font(10)).ColorAndOpacity(Color).AutoWrapText(true);
	}

	TSharedRef<SWrapBox> Wrap()
	{
		return SNew(SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(4.f, 4.f));
	}

	/** A form row: the label, then the control with its help under it. */
	TSharedRef<SWidget> Row(const FString& Label, const FString& Help, TSharedRef<SWidget> Control)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Top).Padding(0, 5, 8, 0)
			[
				SNew(SBox).WidthOverride(LabelW)
				[
					SNew(STextBlock).Text(FText::FromString(Label)).Font(Font(10, TEXT("Bold"))).ColorAndOpacity(Ink).AutoWrapText(true)
				]
			]
			+ SHorizontalBox::Slot().FillWidth(1.f)
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight() [ Control ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
				[
					SNew(STextBlock).Text(FText::FromString(Help)).Font(Font(10)).ColorAndOpacity(Muted).AutoWrapText(true)
					.Visibility(Help.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible)
				]
			];
	}

	FString Human(const FString& Name)
	{
		FString S = Name.Replace(TEXT("_"), TEXT(" "));
		if (S.Len() > 0) S[0] = FChar::ToUpper(S[0]);
		return S;
	}

	/** A setting's label and help: in words when Nice knows it, else bench.yaml's. */
	void Words(const FString& Name, const FString& ManifestHelp, FString& Label, FString& Help)
	{
		for (const FNice& N : Nice)
		{
			if (Name == N.Name) { Label = N.Label; Help = N.Help; return; }
		}
		Label = Human(Name);
		Help = ManifestHelp;
	}

	/** The benchmark as a choice of what to test. */
	void BenchWords(const FString& Name, FString& Title, FString& About)
	{
		if (Name == TEXT("missions"))
		{
			Title = TEXT("Drive the missions");
			About = TEXT("The pipeline drives the sim through the missions, scored by the referee. The sim and Mission Control must be running.");
		}
		else if (Name == TEXT("sim_bag"))
		{
			Title = TEXT("Replay recorded bags");
			About = TEXT("Perception and SLAM run offline on recorded simulator bags, scored against the sim's ground truth.");
		}
	}

	/** "benchmarks.sim_bag.settings.max_frames: expected integer" as "max frames: expected integer". */
	FString Pretty(const FString& Error)
	{
		FString E = Error;
		if (E.StartsWith(TEXT("benchmarks.")))
		{
			int32 Dot = INDEX_NONE;
			E.RightChop(11).FindChar(TEXT('.'), Dot);
			if (Dot != INDEX_NONE) E = E.RightChop(11 + Dot + 1);
			E.RemoveFromStart(TEXT("settings."));
		}
		int32 Colon = INDEX_NONE;
		if (E.FindChar(TEXT(':'), Colon) && Colon > 0)
		{
			FString Label, Help;
			Words(E.Left(Colon), FString(), Label, Help);
			E = Label + E.RightChop(Colon);
		}
		return E;
	}

	FLinearColor StateColor(const FString& State)
	{
		if (State == TEXT("running")) return Good;
		if (State == TEXT("queued")) return Accent;
		if (State == TEXT("failed")) return Bad;
		if (State == TEXT("done")) return Ink;
		return Muted;   // cancelled
	}

	/** "2026-11-12T09:41:07+00:00" as local "09:41", or the date too when not today. */
	FString When(const FString& Iso)
	{
		FDateTime Utc;
		if (!FDateTime::ParseIso8601(*Iso, Utc)) return FString();
		const FDateTime Local = Utc + (FDateTime::Now() - FDateTime::UtcNow());
		return Local.GetDate() == FDateTime::Now().GetDate() ? Local.ToString(TEXT("%H:%M"))
			: Local.ToString(TEXT("%Y-%m-%d %H:%M"));
	}

	FString ToJson(const TSharedRef<FJsonObject>& O)
	{
		FString S;
		const TSharedRef<TJsonWriter<>> W = TJsonWriterFactory<>::Create(&S);
		FJsonSerializer::Serialize(O, W);
		return S;
	}

	TSharedPtr<FJsonObject> Copy(const TSharedPtr<FJsonObject>& O)
	{
		if (!O.IsValid()) return MakeShared<FJsonObject>();
		TSharedPtr<FJsonObject> Out;
		FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(ToJson(O.ToSharedRef())), Out);
		return Out.IsValid() ? Out : MakeShared<FJsonObject>();
	}

	/** A setting's value as a form field shows it; empty when unset. */
	FString FieldValue(const TSharedPtr<FJsonObject>& O, const FString& Key)
	{
		if (!O.IsValid() || !O->HasField(Key)) return FString();
		const TSharedPtr<FJsonValue> V = O->TryGetField(Key);
		if (!V.IsValid() || V->IsNull()) return FString();
		double N = 0.0;
		if (V->Type == EJson::Number && V->TryGetNumber(N))
		{
			return FMath::IsNearlyEqual(N, FMath::RoundToDouble(N)) && FMath::Abs(N) < 1e15
				? FString::Printf(TEXT("%lld"), (long long)FMath::RoundToDouble(N)) : FString::SanitizeFloat(N);
		}
		FString S;
		return V->TryGetString(S) ? S : FString();
	}

	/** What was typed into a field: a number when it is one and the setting is, else the
	 *  text, so the backend's check says what's wrong with it. Empty: unset (the default). */
	void SetTyped(const TSharedPtr<FJsonObject>& O, const FString& Key, const FString& Typed, const FString& Type)
	{
		const FString T = Typed.TrimStartAndEnd();
		if (T.IsEmpty()) { O->RemoveField(Key); return; }
		if ((Type == TEXT("integer") || Type == TEXT("number")) && T.IsNumeric())
		{
			O->SetNumberField(Key, FCString::Atod(*T));
			return;
		}
		O->SetStringField(Key, T);
	}

	TArray<FString> Strings(const TSharedPtr<FJsonObject>& O, const FString& Key)
	{
		TArray<FString> Out;
		const TArray<TSharedPtr<FJsonValue>>* L = nullptr;
		FString One;
		if (O.IsValid() && O->TryGetArrayField(Key, L)) for (const auto& V : *L) Out.Add(V->AsString());
		else if (O.IsValid() && O->TryGetStringField(Key, One)) Out.Add(One);
		return Out;
	}

	void SetStrings(const TSharedPtr<FJsonObject>& O, const FString& Key, const TArray<FString>& Values)
	{
		TArray<TSharedPtr<FJsonValue>> J;
		for (const FString& X : Values) J.Add(MakeShared<FJsonValueString>(X));
		O->SetArrayField(Key, J);
	}
}

void SFSDSBenchPage::Construct(const FArguments& InArgs)
{
	IsActive = InArgs._IsActive;
	Client = InArgs._Client;
	Draft = MakeShared<FJsonObject>();

	ChildSlot
	[
		SNew(SBorder)
		.BorderImage(BenchUi::White())
		.BorderBackgroundColor(BenchUi::Page)
		.Padding(FMargin(16.f, 12.f))
		[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				SNew(STextBlock).Text(LOCTEXT("Title", "Benchmarks"))
				.Font(BenchUi::Font(20, TEXT("Bold"))).ColorAndOpacity(BenchUi::Ink)
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(24, 0, 4, 0)
			[
				BenchUi::Choice(LOCTEXT("TabRun", "Run"), [this]() { return Tab == 0; }, [this]() { Tab = 0; },
					LOCTEXT("TabRunTip", "What to test, and the queue."), true, 12)
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				BenchUi::Choice(TAttribute<FText>::CreateLambda([this]()
					{
						// a part this run needs that isn't ready: say so on the tab
						for (const FString& P : Needs()) if (BoxState(P) == EBox::NotNow || BoxState(P) == EBox::Never) return LOCTEXT("TabSetupWarn", "Setup  (!)");
						return LOCTEXT("TabSetup", "Setup");
					}),
					[this]() { return Tab == 1; }, [this]() { Tab = 1; },
					LOCTEXT("TabSetupTip", "The components a run uses: their versions, where the pipeline runs, and what this machine can run."), true, 12)
			]
			+ SHorizontalBox::Slot().FillWidth(1.f)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				BenchUi::Button(LOCTEXT("Refresh", "Refresh"), BenchUi::PlainBtn, [this]()
				{
					FetchPresets();
					FetchManifest();
					FetchRig();
					FetchQueue();
					CheckMachine();
				})
			]
		]
		+ SVerticalBox::Slot().FillHeight(1.f).Padding(0, 10, 0, 0)
		[
			SNew(SWidgetSwitcher)
			.WidgetIndex_Lambda([this]() { return Tab; })
			+ SWidgetSwitcher::Slot()
			[
			SNew(SHorizontalBox)
			// ---- the run: what, how, then what will run next to Queue ----------
			+ SHorizontalBox::Slot().FillWidth(1.f)
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().FillHeight(1.f)
				[
					SNew(SScrollBox)
					+ SScrollBox::Slot()
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight()
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
							[
								SNew(STextBlock).Text(LOCTEXT("QuickStart", "Start from")).Font(BenchUi::Font(10, TEXT("Bold"))).ColorAndOpacity(BenchUi::Muted)
							]
							+ SHorizontalBox::Slot().FillWidth(1.f) [ SAssignNew(PresetBox, SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(4.f, 4.f)) ]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
						[
							SNew(STextBlock)
							.Text_Lambda([this]()
							{
								if (!PresetsNote.IsEmpty()) return FText::FromString(PresetsNote);
								const FPreset* P = Chosen();
								return FText::FromString(!P ? FString() : !P->Error.IsEmpty() ? P->Error : P->About);
							})
							.ColorAndOpacity_Lambda([this]()
							{
								const FPreset* P = Chosen();
								return FSlateColor(!PresetsNote.IsEmpty() || (P && !P->Error.IsEmpty()) ? BenchUi::Bad : BenchUi::Muted);
							})
							.Font(BenchUi::Font(9)).AutoWrapText(true)
						]
						+ SVerticalBox::Slot().AutoHeight() [ SAssignNew(FormBox, SVerticalBox) ]
					]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
				[
					SNew(SBorder)
					.BorderImage(BenchUi::White())
					.BorderBackgroundColor(BenchUi::Card)
					.Padding(FMargin(14.f, 10.f))
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight()
						[
							SNew(SHorizontalBox)
							+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
							[
								SNew(STextBlock)
								.Text_Lambda([this]()
								{
									if (!HasBenchmarks()) return LOCTEXT("PickWhat", "Pick what to test.");
									if (!bManifestKnown || !ManifestError.IsEmpty()) return LOCTEXT("AsIs", "The preset, as it is.");
									if (CheckDueAt > 0.0 || !bChecked) return LOCTEXT("Checking", "Checking...");
									if (!bCheckOk) return LOCTEXT("NotReady", "Not ready to queue:");
									const FText On = bRunner ? LOCTEXT("OnRunner", "the bench PC") : LOCTEXT("OnLocal", "this machine");
									if (!CanPickPipeline())
									{
										return FText::Format(LOCTEXT("Runs", "{0} {0}|plural(one=run,other=runs) on {1}"), FText::AsNumber(CheckRuns), On);
									}
									return FText::Format(LOCTEXT("RunsWhere", "{0} {0}|plural(one=run,other=runs) on {1}, pipeline on {2}"),
										FText::AsNumber(CheckRuns), On,
										PipelineOn() == BenchUi::OnPanda ? LOCTEXT("WherePanda", "the LattePanda") : LOCTEXT("WherePc", "this PC"));
								})
								.ColorAndOpacity_Lambda([this]()
								{
									const bool bBad = HasBenchmarks() && bManifestKnown && ManifestError.IsEmpty() && CheckDueAt <= 0.0 && bChecked && !bCheckOk;
									return FSlateColor(bBad ? BenchUi::Bad : BenchUi::Ink);
								})
								.Font(BenchUi::Font(14, TEXT("Bold")))
							]
							+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(8, 0, 4, 0)
							[
								BenchUi::Choice(LOCTEXT("Local", "This machine"), [this]() { return !bRunner; },
									[this]() { if (bRunner) { bRunner = false; FetchRig(); } })
							]
							+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 10, 0)
							[
								BenchUi::Choice(LOCTEXT("Runner", "Bench PC"), [this]() { return bRunner; }, [this]() { bRunner = true; },
									LOCTEXT("RunnerTip", "Runs on the bench PC through GitHub: not built yet."))
							]
							+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
							[
								SNew(SButton)
								.IsFocusable(false)
								.ButtonColorAndOpacity(BenchUi::GoBtn)
								.ContentPadding(FMargin(18.f, 9.f))
								.IsEnabled_Lambda([this]() { return CanQueue(); })
								.OnClicked_Lambda([this]() { Launch(); return FReply::Handled(); })
								[
									SNew(STextBlock)
									.Text_Lambda([this]() { return bLaunching ? LOCTEXT("Queueing", "QUEUEING...") : LOCTEXT("QueueIt", "QUEUE"); })
									.Font(BenchUi::Font(13, TEXT("Bold"))).ColorAndOpacity(FLinearColor::White)
								]
							]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
						[
							SNew(STextBlock)
							.Text_Lambda([this]()
							{
								TArray<FString> Out;
								const FString Never = NeverHere();
								if (!Never.IsEmpty()) Out.Add(FString::Printf(TEXT("This machine can't run it: %s."), *Never));
								if (bChecked && CheckDueAt <= 0.0 && HasBenchmarks())
								{
									if (bCheckOk) Out.Add(FString::Join(CheckLines, TEXT("   ·   ")));
									else for (const FString& E : CheckErrors) Out.Add(BenchUi::Pretty(E));
								}
								return FText::FromString(FString::Join(Out, TEXT("\n")));
							})
							.ColorAndOpacity_Lambda([this]() { return FSlateColor(bCheckOk && NeverHere().IsEmpty() ? BenchUi::Muted : BenchUi::Bad); })
							.Font(BenchUi::Font(10)).AutoWrapText(true)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
						[
							SNew(STextBlock)
							.Text_Lambda([this]() { return FText::FromString(Note); })
							.Visibility_Lambda([this]() { return Note.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible; })
							.ColorAndOpacity_Lambda([this]() { return FSlateColor(bNoteError ? BenchUi::Bad : BenchUi::Accent); })
							.Font(BenchUi::Font(10, TEXT("Bold"))).AutoWrapText(true)
						]
					]
				]
			]
			// ---- this machine and the queue -------------------------------------
			+ SHorizontalBox::Slot().AutoWidth().Padding(18, 0, 0, 0)
			[
				SNew(SBox).WidthOverride(BenchUi::SideW)
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6) [ BenchUi::Heading(LOCTEXT("Queue", "QUEUE")) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
					[
						SNew(SHorizontalBox)
						+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
						[
							SNew(STextBlock)
							.Text_Lambda([this]()
							{
								if (!bQueueKnown) return LOCTEXT("QueueUnknown", "Asking the backend...");
								if (bWorkerAlive && bWorkerStopping) return LOCTEXT("WorkerStopping", "Stopping once no spec is running.");
								if (bWorkerAlive) return LOCTEXT("Worker", "Worker running: queued specs run one at a time.");
								if (FPlatformTime::Seconds() < WorkerStartingUntil) return LOCTEXT("WorkerStarting", "Starting the worker...");
#if PLATFORM_WINDOWS
								return LOCTEXT("NoWorkerWin", "No worker: queued specs wait. Start one from WSL: python3 tools/sim_benchmark/run_spec.py --work");
#else
								return LOCTEXT("NoWorker", "No worker: queued specs wait until you start it.");
#endif
							})
							.ToolTipText_Lambda([this]()
							{
								return bWorkerAlive ? FText::Format(LOCTEXT("WorkerTip", "On {0}, pid {1}"), FText::FromString(WorkerHost),
									FText::AsNumber(WorkerPid, &FNumberFormattingOptions::DefaultNoGrouping())) : FText::GetEmpty();
							})
							.ColorAndOpacity_Lambda([this]() { return FSlateColor(!bQueueKnown ? BenchUi::Muted : bWorkerAlive ? BenchUi::Good : BenchUi::Accent); })
							.Font(BenchUi::Font(10, TEXT("Bold"))).AutoWrapText(true)
						]
						+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(8, 0, 0, 0)
						[
							BenchUi::Button(TAttribute<FText>::CreateLambda([this]()
								{
									return bWorkerAlive ? (bWorkerStopping ? LOCTEXT("Stopping", "Stopping...") : LOCTEXT("StopWorker", "Stop worker"))
										: LOCTEXT("StartWorker", "Start worker");
								}),
								BenchUi::PlainBtn,
								[this]() { if (bWorkerAlive) StopWorker(); else StartWorker(); },
								TAttribute<bool>::CreateLambda([this]()
								{
									if (!bQueueKnown || !Client.IsValid()) return false;
									if (bWorkerAlive) return !bWorkerStopping;
#if PLATFORM_WINDOWS
									return false;
#else
									return FPlatformTime::Seconds() >= WorkerStartingUntil && !RepoDir().IsEmpty();
#endif
								}))
						]
					]
					+ SVerticalBox::Slot().FillHeight(1.f)
					[
						SNew(SScrollBox)
						+ SScrollBox::Slot() [ SAssignNew(QueueList, SVerticalBox) ]
					]
				]
			]
			]
			// ---- the setup: components and this machine ------------------------
			+ SWidgetSwitcher::Slot()
			[
				SNew(SScrollBox)
				+ SScrollBox::Slot() [ SAssignNew(SetupBox, SVerticalBox) ]
			]
		]
		]
	];

	RebuildForm();
	RegisterActiveTimer(0.25f, FWidgetActiveTimerDelegate::CreateSP(this, &SFSDSBenchPage::Poll));
}

// ---------------------------------------------------------------------------
// The draft spec

TSharedPtr<FJsonObject> SFSDSBenchPage::BenchObj(const FString& Bench, bool bCreate) const
{
	const TSharedPtr<FJsonObject>* All = nullptr;
	TSharedPtr<FJsonObject> Benchmarks;
	if (Draft->TryGetObjectField(TEXT("benchmarks"), All)) Benchmarks = *All;
	else if (bCreate)
	{
		Benchmarks = MakeShared<FJsonObject>();
		Draft->SetObjectField(TEXT("benchmarks"), Benchmarks);
	}
	if (!Benchmarks.IsValid()) return nullptr;
	const TSharedPtr<FJsonObject>* One = nullptr;
	if (Benchmarks->TryGetObjectField(Bench, One)) return *One;
	// "benchmarks: {missions: }" (null in YAML) is the benchmark with its defaults
	if (Benchmarks->HasField(Bench) || bCreate)
	{
		const TSharedPtr<FJsonObject> New = MakeShared<FJsonObject>();
		Benchmarks->SetObjectField(Bench, New);
		return New;
	}
	return nullptr;
}

bool SFSDSBenchPage::HasBenchmarks() const
{
	const TSharedPtr<FJsonObject>* All = nullptr;
	return Draft->TryGetObjectField(TEXT("benchmarks"), All) && (*All)->Values.Num() > 0;
}

TSharedPtr<FJsonObject> SFSDSBenchPage::SettingsObj(const FString& Bench, bool bCreate) const
{
	const TSharedPtr<FJsonObject> B = BenchObj(Bench, bCreate);
	if (!B.IsValid()) return nullptr;
	const TSharedPtr<FJsonObject>* S = nullptr;
	if (B->TryGetObjectField(TEXT("settings"), S)) return *S;
	if (!bCreate) return nullptr;
	const TSharedPtr<FJsonObject> New = MakeShared<FJsonObject>();
	B->SetObjectField(TEXT("settings"), New);
	return New;
}

void SFSDSBenchPage::SetBench(const FString& Bench, bool bOn)
{
	if (bOn) { BenchObj(Bench, true); }
	else
	{
		const TSharedPtr<FJsonObject>* All = nullptr;
		if (Draft->TryGetObjectField(TEXT("benchmarks"), All)) (*All)->RemoveField(Bench);
	}
	Edited();
	RebuildForm();
}

void SFSDSBenchPage::PickBench(const FString& Bench)
{
	const TSharedPtr<FJsonObject>* All = nullptr;
	if (Draft->TryGetObjectField(TEXT("benchmarks"), All))
	{
		for (const auto& KV : (*All)->Values)
		{
			const FString Name(*KV.Key);
			const TSharedPtr<FJsonObject>* O = nullptr;
			if (Name != Bench && KV.Value.IsValid() && KV.Value->TryGetObject(O)) Stash.Add(Name, *O);
		}
		TArray<FString> Others;
		for (const auto& KV : (*All)->Values) if (FString(*KV.Key) != Bench) Others.Add(FString(*KV.Key));
		for (const FString& O : Others) (*All)->RemoveField(O);
	}
	if (!HasBench(Bench))
	{
		const TSharedPtr<FJsonObject>* Kept = Stash.Find(Bench);
		BenchObj(Bench, true);
		if (Kept && Kept->IsValid())
		{
			const TSharedPtr<FJsonObject>* BenchesObj = nullptr;
			Draft->TryGetObjectField(TEXT("benchmarks"), BenchesObj);
			(*BenchesObj)->SetObjectField(Bench, *Kept);
		}
	}
	Edited();
	RebuildForm();
}

TArray<FString> SFSDSBenchPage::CsvSetting(const FString& Bench, const FString& Setting) const
{
	TArray<FString> Out;
	BenchUi::FieldValue(SettingsObj(Bench), Setting).ParseIntoArray(Out, TEXT(","));
	for (FString& V : Out) V.TrimStartAndEndInline();
	return Out;
}

void SFSDSBenchPage::SetCsvSetting(const FString& Bench, const FString& Setting, const TArray<FString>& Values)
{
	const TSharedPtr<FJsonObject> S = SettingsObj(Bench, true);
	if (Values.Num() == 0) S->RemoveField(Setting);
	else S->SetStringField(Setting, FString::Join(Values, TEXT(",")));
	Edited();
}

void SFSDSBenchPage::Edited()
{
	bEdited = true;
	CheckDueAt = FPlatformTime::Seconds() + BenchUi::CheckAfterS;
}

FString SFSDSBenchPage::SpecName() const
{
	if (bNameTouched) return DraftName;
	const FPreset* P = Chosen();
	const FString Base = P ? P->Title : TEXT("custom run");
	return bEdited && P ? Base + TEXT(" (changed)") : Base;
}

TSharedRef<FJsonObject> SFSDSBenchPage::BuildSpec() const
{
	const TSharedPtr<FJsonObject> S = BenchUi::Copy(Draft);
	S->SetStringField(TEXT("name"), SpecName());
	if (ChosenOverrides.Num() > 0)
	{
		const TSharedPtr<FJsonObject>* Have = nullptr;
		const TSharedPtr<FJsonObject> Pipeline = S->TryGetObjectField(TEXT("pipeline"), Have) ? *Have : MakeShared<FJsonObject>();
		// in the order they were picked: a later file wins, as run_spec.py merges files
		for (const FString& Name : ChosenOverrides)
		{
			const FOverride* O = Overrides.FindByPredicate([&Name](const FOverride& X) { return X.Name == Name; });
			if (!O || !O->Pipeline.IsValid()) continue;
			for (const auto& Comp : O->Pipeline->Values)
			{
				const TSharedPtr<FJsonObject>* Params = nullptr;
				if (!Comp.Value.IsValid() || !Comp.Value->TryGetObject(Params)) continue;
				const FString Key(*Comp.Key);
				const TSharedPtr<FJsonObject>* Existing = nullptr;
				const TSharedPtr<FJsonObject> Into = Pipeline->TryGetObjectField(Key, Existing) ? *Existing : MakeShared<FJsonObject>();
				for (const auto& P : (*Params)->Values) Into->SetField(FString(*P.Key), P.Value);
				Pipeline->SetObjectField(Key, Into);
			}
		}
		S->SetObjectField(TEXT("pipeline"), Pipeline);
	}
	return S.ToSharedRef();
}

bool SFSDSBenchPage::CanPickPipeline() const
{
	for (const FBench& B : Benches)
	{
		if (HasBench(B.Name) && B.Settings.ContainsByPredicate([](const FSetting& S) { return S.Name == BenchUi::PipelineOnSetting; }))
		{
			return true;
		}
	}
	return false;
}

FString SFSDSBenchPage::PipelineOn() const
{
	for (const FBench& B : Benches)
	{
		if (!HasBench(B.Name)) continue;
		const FString V = BenchUi::FieldValue(SettingsObj(B.Name), BenchUi::PipelineOnSetting);
		if (!V.IsEmpty()) return V;
	}
	return BenchUi::OnThisPc;
}

void SFSDSBenchPage::SetPipelineOn(const FString& Where)
{
	for (const FBench& B : Benches)
	{
		if (!HasBench(B.Name) || !B.Settings.ContainsByPredicate([](const FSetting& S) { return S.Name == BenchUi::PipelineOnSetting; })) continue;
		const TSharedPtr<FJsonObject> S = SettingsObj(B.Name, true);
		if (Where == BenchUi::OnThisPc) S->RemoveField(BenchUi::PipelineOnSetting);   // the default
		else S->SetStringField(BenchUi::PipelineOnSetting, Where);
	}
	Edited();
}

void SFSDSBenchPage::PickPreset(const FString& Name)
{
	PresetName = Name;
	const FPreset* P = Chosen();
	Draft = BenchUi::Copy(P ? P->Spec : nullptr);
	Draft->RemoveField(TEXT("name"));   // SpecName() names it
	Stash.Reset();
	bEdited = false;
	bNameTouched = false;
	DraftName.Reset();
	bTracksOpen = false;
	CheckDueAt = FPlatformTime::Seconds();   // check the preset as it is
	bChecked = false;
	RebuildForm();
}

// ---------------------------------------------------------------------------
// The form

void SFSDSBenchPage::RebuildSetup()
{
	if (!SetupBox.IsValid()) return;
	SetupBox->ClearChildren();
	SetupBox->AddSlot().AutoHeight().Padding(0, 0, 0, 10)
	[
		BenchUi::Small(LOCTEXT("SetupHint", "What a run uses, apart from what it tests: the version of each component, where the "
			"pipeline runs, and whether this machine can run it now. It applies to the run on the Run tab."))
	];
	SetupBox->AddSlot().AutoHeight() [ MakeComponents() ];
}

void SFSDSBenchPage::RebuildForm()
{
	RebuildSetup();
	if (!FormBox.IsValid()) return;
	FormBox->ClearChildren();
	FieldText.Reset();

	if (!bManifestKnown || !ManifestError.IsEmpty())
	{
		FormBox->AddSlot().AutoHeight().Padding(0, 14, 0, 0)
		[
			BenchUi::Small(ManifestError.IsEmpty() ? LOCTEXT("ManifestWait", "Asking the backend what a run can set...")
				: FText::Format(LOCTEXT("ManifestErr", "The preset runs as it is: changing it needs the backend's "
					"benchmark list, and {0}"), FText::FromString(ManifestError)),
				ManifestError.IsEmpty() ? BenchUi::Muted : BenchUi::Bad)
		];
		return;
	}

	// What to test: one benchmark, in words.
	TArray<const FBench*> Order;
	for (const FBench& B : Benches) if (B.Name == BenchUi::Missions) Order.Add(&B);
	for (const FBench& B : Benches) if (B.Name != BenchUi::Missions) Order.Add(&B);
	TSharedRef<SWrapBox> What = BenchUi::Wrap();
	for (const FBench* B : Order)
	{
		FString Title = B->Title, About = B->Description;
		BenchUi::BenchWords(B->Name, Title, About);
		What->AddSlot()
		[
			BenchUi::Choice(FText::FromString(Title), [this, N = B->Name]() { return HasBench(N); },
				[this, N = B->Name]() { PickBench(N); }, FText::FromString(About), true, 12)
		];
	}
	FormBox->AddSlot().AutoHeight().Padding(0, 16, 0, 6) [ BenchUi::Heading(LOCTEXT("What", "WHAT TO TEST")) ];
	FormBox->AddSlot().AutoHeight() [ What ];

	const FBench* Shown = nullptr;
	for (const FBench* B : Order) if (HasBench(B->Name)) { Shown = B; break; }
	if (!Shown)
	{
		FormBox->AddSlot().AutoHeight().Padding(0, 8, 0, 0) [ BenchUi::Small(LOCTEXT("PickOne", "Pick one, or start from a preset above.")) ];
		return;
	}
	{
		FString Title = Shown->Title, About = Shown->Description;
		BenchUi::BenchWords(Shown->Name, Title, About);
		FormBox->AddSlot().AutoHeight().Padding(0, 4, 0, 4) [ BenchUi::Small(FText::FromString(About)) ];
	}
	AddBenchRows(*Shown);
}

void SFSDSBenchPage::AddBenchRows(const FBench& B)
{
	auto Add = [this](TSharedRef<SWidget> W) { FormBox->AddSlot().AutoHeight().Padding(0, 8, 0, 0) [ W ]; };
	TSet<FString> Done;   // settings shown above More settings

	if (B.Name == BenchUi::Missions)
	{
		if (B.Settings.ContainsByPredicate([](const FSetting& S) { return S.Name == BenchUi::Missions; }))
		{
			Add(MakeMissionsRow(B));
			Done.Add(BenchUi::Missions);
		}
		if (B.Settings.ContainsByPredicate([](const FSetting& S) { return S.Name == TEXT("tracks"); }))
		{
			Add(MakeTracksRow(B));
			Done.Add(TEXT("tracks"));
		}
		if (B.Settings.ContainsByPredicate([](const FSetting& S) { return S.Name == TEXT("repeats"); }))
		{
			Add(MakeCountRow(B, TEXT("repeats"), TEXT("Runs per track"), TEXT("Each run is scored on its own; run i uses seed i.")));
			Done.Add(TEXT("repeats"));
		}
		if (B.Settings.ContainsByPredicate([](const FSetting& S) { return S.Name == TEXT("start_noise"); }))
		{
			const FString N = B.Name;
			Add(BenchUi::Row(TEXT("Vary the start"), TEXT("Start each run from a slightly different, seeded position."),
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 4, 0)
				[
					BenchUi::Choice(LOCTEXT("No", "No"), [this, N]() { bool V = false; const auto O = SettingsObj(N); return !(O.IsValid() && O->TryGetBoolField(TEXT("start_noise"), V) && V); },
						[this, N]() { SettingsObj(N, true)->RemoveField(TEXT("start_noise")); Edited(); })
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[
					BenchUi::Choice(LOCTEXT("Yes", "Yes"), [this, N]() { bool V = false; const auto O = SettingsObj(N); return O.IsValid() && O->TryGetBoolField(TEXT("start_noise"), V) && V; },
						[this, N]() { SettingsObj(N, true)->SetBoolField(TEXT("start_noise"), true); Edited(); })
				]));
			Done.Add(TEXT("start_noise"));
		}
	}
	else
	{
		if (!B.Bags.IsEmpty()) Add(MakeBagsRow(B));
		if (B.Parts.Num() > 0) Add(MakePartsRow(B));
		Add(MakeCountRow(B, FString(), TEXT("Runs per bag"), TEXT("Each run is a job of its own.")));
	}
	if (B.Settings.ContainsByPredicate([](const FSetting& S) { return S.Name == BenchUi::PipelineOnSetting; }))
	{
		Done.Add(BenchUi::PipelineOnSetting);   // in the Pipeline box
	}
	Add(MakeOverridesRow());

	Add(BenchUi::Row(TEXT("Name"), TEXT("How it shows in the queue."),
		SNew(SBox).HAlign(HAlign_Left)
		[
			SNew(SEditableTextBox)
			.MinDesiredWidth(320.f)
			.Text_Lambda([this]() { return FText::FromString(SpecName()); })
			.OnTextChanged_Lambda([this](const FText& T) { DraftName = T.ToString(); bNameTouched = true; })
			.Font(BenchUi::Font(10))
		]));

	// More settings: the rest of bench.yaml's, closed until asked for.
	TArray<const FSetting*> Rest;
	for (const FSetting& S : B.Settings) if (!Done.Contains(S.Name)) Rest.Add(&S);
	const bool bOpen = MoreOpen.Contains(B.Name);
	FormBox->AddSlot().AutoHeight().Padding(0, 14, 0, 0)
	[
		BenchUi::Choice(FText::Format(bOpen ? LOCTEXT("Fewer", "Hide the other settings") : LOCTEXT("More", "More settings ({0})"),
				FText::AsNumber(Rest.Num() + (B.Name == BenchUi::Missions ? 1 : 0))),
			[]() { return false; },
			[this, N = B.Name]()
			{
				if (MoreOpen.Remove(N) == 0) MoreOpen.Add(N);
				RebuildForm();
			})
	];
	if (!bOpen) return;
	for (const FSetting* S : Rest)
	{
		// The LattePanda's commands matter only when the pipeline runs there.
		if (S->Name.StartsWith(TEXT("panda_")))
		{
			Add(SNew(SBox)
				.Visibility_Lambda([this]() { return PipelineOn() == BenchUi::OnPanda ? EVisibility::Visible : EVisibility::Collapsed; })
				[ MakeSettingRow(B, *S) ]);
			continue;
		}
		Add(MakeSettingRow(B, *S));
	}
	if (B.Name == BenchUi::Missions)
	{
		Add(MakeCountRow(B, FString(), TEXT("Repeat everything"), TEXT("The whole benchmark again, each time a job of its own.")));
	}
}

TSharedRef<SWidget> SFSDSBenchPage::MakeMissionsRow(const FBench& B)
{
	// All lit: every mission (the setting unset).
	const FString N = B.Name;
	auto Lit = [this, N]()
	{
		TArray<FString> L = CsvSetting(N, BenchUi::Missions);
		return L.Num() == 0 || L.Contains(TEXT("all")) ? Missions : L;
	};
	TSharedRef<SWrapBox> W = BenchUi::Wrap();
	for (const FString& M : Missions)
	{
		W->AddSlot()
		[
			BenchUi::Choice(FText::FromString(M), [Lit, M]() { return Lit().Contains(M); },
				[this, Lit, N, M]()
				{
					TArray<FString> L = Lit();
					if (L.Remove(M) == 0) L.Add(M);
					if (L.Num() == 0) return;   // at least one
					SetCsvSetting(N, BenchUi::Missions, L.Num() == Missions.Num() ? TArray<FString>() : L);
				})
		];
	}
	return BenchUi::Row(TEXT("Missions"), TEXT("All lit: every mission."), W);
}

TSharedRef<SWidget> SFSDSBenchPage::MakeTracksRow(const FBench& B)
{
	const FString N = B.Name;
	// Which tracks "each mission's own" are, for the missions that are lit.
	auto OwnText = [this, N]()
	{
		TArray<FString> Lit = CsvSetting(N, BenchUi::Missions);
		if (Lit.Num() == 0 || Lit.Contains(TEXT("all"))) Lit = Missions;
		TArray<FString> Lines;
		for (const FString& M : Lit)
		{
			const TArray<FString>* Own = MissionTracks.Find(M);
			TArray<FString> Names;
			for (const FString& T : Own ? *Own : TArray<FString>()) Names.Add(T.EndsWith(TEXT(".csv")) ? T.LeftChop(4) : T);
			Lines.Add(FString::Printf(TEXT("%s: %s"), *M, Names.Num() ? *FString::Join(Names, TEXT(", ")) : TEXT("?")));
		}
		return FText::FromString(FString::Join(Lines, TEXT("\n")));
	};
	const bool bChosen = CsvSetting(N, TEXT("tracks")).Num() > 0;
	if (!bChosen && !bTracksOpen)
	{
		return BenchUi::Row(TEXT("Tracks"), FString(),
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight()
			[
				SNew(STextBlock).Text(LOCTEXT("OwnTracksText", "Each mission on its own tracks:")).Font(BenchUi::Font(10)).ColorAndOpacity(BenchUi::Ink)
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(10, 2, 0, 6)
			[
				SNew(STextBlock).Text_Lambda(OwnText).Font(BenchUi::Font(9)).ColorAndOpacity(BenchUi::Muted).AutoWrapText(true)
			]
			+ SVerticalBox::Slot().AutoHeight().HAlign(HAlign_Left)
			[
				BenchUi::Button(LOCTEXT("ChooseTracks", "Use other tracks"), BenchUi::PlainBtn, [this]() { bTracksOpen = true; RebuildForm(); })
			]);
	}
	TSharedRef<SWrapBox> W = BenchUi::Wrap();
	W->AddSlot()
	[
		BenchUi::Choice(LOCTEXT("OwnTracks", "Back to each mission's own"), [this, N]() { return CsvSetting(N, TEXT("tracks")).Num() == 0; },
			[this, N]() { SetCsvSetting(N, TEXT("tracks"), {}); bTracksOpen = false; RebuildForm(); })
	];
	for (const FString& T : TrackNames)
	{
		W->AddSlot()
		[
			BenchUi::Choice(FText::FromString(T.EndsWith(TEXT(".csv")) ? T.LeftChop(4) : T),
				[this, N, T]() { return CsvSetting(N, TEXT("tracks")).Contains(T); },
				[this, N, T]()
				{
					TArray<FString> L = CsvSetting(N, TEXT("tracks"));
					if (L.Remove(T) == 0) L.Add(T);
					SetCsvSetting(N, TEXT("tracks"), L);
				})
		];
	}
	return BenchUi::Row(TEXT("Tracks"), TEXT("The tracks picked here replace every mission's own: each picked mission runs on all of them."),
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight() [ W ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			BenchUi::Small(TAttribute<FText>::CreateLambda([OwnText]()
			{
				return FText::Format(LOCTEXT("OwnWere", "Their own would be:\n{0}"), OwnText());
			}))
		]);
}

TSharedRef<SWidget> SFSDSBenchPage::MakeBagsRow(const FBench& B)
{
	const FString N = B.Name;
	const TArray<FString>* Have = BagsByKind.Find(B.Bags);
	if (!Have || Have->Num() == 0)
	{
		return BenchUi::Row(TEXT("Bags"), FString(), BenchUi::Small(
			LOCTEXT("NoBags", "No recorded bags yet: record a run with Record a bag (Drive, Autonomous), or tools/sim_benchmark/capture_benchmark_bag.py."),
			BenchUi::Bad));
	}
	auto IsAll = [this, N]()
	{
		const TSharedPtr<FJsonObject> O = BenchObj(N);
		FString S;
		return O.IsValid() && O->TryGetStringField(TEXT("bags"), S) && S == TEXT("all");
	};
	TSharedRef<SWrapBox> W = BenchUi::Wrap();
	W->AddSlot()
	[
		BenchUi::Choice(LOCTEXT("AllBags", "All"), IsAll, [this, N]() { BenchObj(N, true)->SetStringField(TEXT("bags"), TEXT("all")); Edited(); })
	];
	for (const FString& Bag : *Have)
	{
		W->AddSlot()
		[
			BenchUi::Choice(FText::FromString(Bag), [this, IsAll, N, Bag]() { return !IsAll() && BenchUi::Strings(BenchObj(N), TEXT("bags")).Contains(Bag); },
				[this, IsAll, N, Bag]()
				{
					TArray<FString> L = IsAll() ? TArray<FString>() : BenchUi::Strings(BenchObj(N), TEXT("bags"));
					if (L.Remove(Bag) == 0) L.Add(Bag);
					BenchUi::SetStrings(BenchObj(N, true), TEXT("bags"), L);
					Edited();
				})
		];
	}
	return BenchUi::Row(TEXT("Bags"), TEXT("Pick one or more, or all."), W);
}

TSharedRef<SWidget> SFSDSBenchPage::MakePartsRow(const FBench& B)
{
	// All lit: every part (no "only").
	const FString N = B.Name;
	const TArray<FString> All = B.Parts;
	auto Lit = [this, N, All]() { TArray<FString> L = BenchUi::Strings(BenchObj(N), TEXT("only")); return L.Num() == 0 ? All : L; };
	TSharedRef<SWrapBox> W = BenchUi::Wrap();
	for (const FString& Part : All)
	{
		W->AddSlot()
		[
			BenchUi::Choice(FText::FromString(BenchUi::Human(Part)), [Lit, Part]() { return Lit().Contains(Part); },
				[this, Lit, N, Part, All]()
				{
					TArray<FString> L = Lit();
					if (L.Remove(Part) == 0) L.Add(Part);
					if (L.Num() == 0) return;   // at least one
					const TSharedPtr<FJsonObject> O = BenchObj(N, true);
					if (L.Num() == All.Num()) O->RemoveField(TEXT("only")); else BenchUi::SetStrings(O, TEXT("only"), L);
					Edited();
				})
		];
	}
	return BenchUi::Row(TEXT("Measure"), TEXT("All lit: all of them."), W);
}

TSharedRef<SWidget> SFSDSBenchPage::MakeCountRow(const FBench& B, const FString& Setting, const FString& Label, const FString& Help)
{
	const FString N = B.Name;
	auto Holder = [this, N, Setting](bool bCreate) { return Setting.IsEmpty() ? BenchObj(N, bCreate) : SettingsObj(N, bCreate); };
	const FString Key = Setting.IsEmpty() ? FString(TEXT("repeats")) : Setting;
	return BenchUi::Row(Label, Help,
		SNew(SBox).HAlign(HAlign_Left)
		[
			SNew(SSpinBox<int32>)
			.MinValue(1).MaxValue(100).MinSliderValue(1).MaxSliderValue(20)
			.MinDesiredWidth(90.f)
			.Value_Lambda([Holder, Key]()
			{
				const TSharedPtr<FJsonObject> O = Holder(false);
				double R = 1.0;
				if (O.IsValid()) O->TryGetNumberField(Key, R);
				return FMath::Max(1, (int32)R);
			})
			.OnValueChanged_Lambda([this, Holder, Key](int32 V)
			{
				const TSharedPtr<FJsonObject> O = Holder(true);
				double Was = 1.0;
				O->TryGetNumberField(Key, Was);
				if ((int32)Was == V) return;
				if (V <= 1) O->RemoveField(Key); else O->SetNumberField(Key, V);
				Edited();
			})
			.Font(BenchUi::Font(10))
		]);
}

TSharedRef<SWidget> SFSDSBenchPage::MakePipelineChoice()
{
	auto Where = [this](const FString& Choice, const FString& Part, const FText& Label)
	{
		return BenchUi::Choice(
			TAttribute<FText>::CreateLambda([this, Part, Label]()
			{
				switch (BoxState(Part))
				{
				case EBox::NotNow: return FText::Format(LOCTEXT("NotNowSuffix", "{0} (not now)"), Label);
				case EBox::Never: return FText::Format(LOCTEXT("NeverSuffix", "{0} (not here)"), Label);
				default: return Label;
				}
			}),
			[this, Choice]() { return PipelineOn() == Choice; },
			[this, Choice]() { SetPipelineOn(Choice); },
			LOCTEXT("WhereTip", "Where the pipeline runs. Not now: the run waits in the queue until it's free."),
			TAttribute<bool>::CreateLambda([this, Part, Choice]() { return PipelineOn() == Choice || BoxState(Part) != EBox::Never; }));
	};
	return SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 4, 0) [ Where(BenchUi::OnThisPc, TEXT("pipeline_bench_pc"), LOCTEXT("OnPc", "This PC")) ]
		+ SHorizontalBox::Slot().AutoWidth() [ Where(BenchUi::OnPanda, TEXT("latte_panda"), LOCTEXT("OnPanda", "LattePanda")) ];
}

TSharedRef<SWidget> SFSDSBenchPage::MakeComponents()
{
	auto VersionOf = [this](const TCHAR* Repo, const TCHAR* Prefix)
	{
		return [this, Repo = FString(Repo), Prefix = FString(Prefix)]()
		{
			const FCode* C = Code.Find(Repo);
			if (!C) return FPlatformTime::Seconds() < MachineCheckUntil ? FString(TEXT("Checking...")) : FString(TEXT("Version not known yet"));
			FString S = C->Branch.IsEmpty()
				? FString::Printf(TEXT("%s @ %s, the commit IFSSIM pins"), *Prefix, *C->Commit)
				: FString::Printf(TEXT("%s %s @ %s"), *Prefix, *C->Branch, *C->Commit);
			if (C->bDirty) S += TEXT(", with local changes");
			return S;
		};
	};
	TSharedRef<SWrapBox> Boxes = SNew(SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(8.f, 8.f));
	Boxes->AddSlot()
	[
		MakeComponent(LOCTEXT("CompSim", "Sim"), []() { return FString(TEXT("sim")); }, VersionOf(TEXT("ifssim"), TEXT("IFSSIM")), nullptr, true)
	];
	TSharedPtr<SWidget> Where;
	if (CanPickPipeline()) Where = MakePipelineChoice();
	Boxes->AddSlot()
	[
		MakeComponent(LOCTEXT("CompPipeline", "Pipeline"),
			[this]() { return PipelineOn() == BenchUi::OnPanda ? FString(TEXT("latte_panda")) : FString(TEXT("pipeline_bench_pc")); },
			VersionOf(TEXT("pipeline"), TEXT("IFS09")), Where, true)
	];
	Boxes->AddSlot()
	[
		MakeComponent(LOCTEXT("CompBoards", "ECU, AMS, uDV"), []() { return FString(TEXT("hil")); },
			[]() { return FString(TEXT("Real or emulated boards: with car-in-the-loop")); }, nullptr, false)
	];

	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center) [ BenchUi::Heading(LOCTEXT("Components", "COMPONENTS")) ]
			+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center).Padding(10, 0, 0, 0)
			[
				BenchUi::Small(TAttribute<FText>::CreateLambda([this]()
				{
					if (bRunner) return LOCTEXT("RunnerUnknown", "The bench PC doesn't publish its checks yet.");
					if (FPlatformTime::Seconds() < MachineCheckUntil && (!bRigKnown || bRigStale)) return LOCTEXT("CheckingMachine", "Checking this machine...");
					if (!bRigKnown || bRigStale) return FText::Format(LOCTEXT("RigUnknown", "Not known: {0}."), FText::FromString(RigReason));
					return FText::Format(LOCTEXT("RigKnown", "This machine ({0}), checked {1} s ago. Outlined: what this run needs."),
						FText::FromString(RigId), FText::AsNumber(FMath::RoundToInt(RigAge)));
				}))
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				BenchUi::Button(LOCTEXT("CheckAgain", "Check again"), BenchUi::PlainBtn, [this]() { CheckMachine(); },
					TAttribute<bool>::CreateLambda([this]() { return FPlatformTime::Seconds() >= MachineCheckUntil && !RepoDir().IsEmpty(); }))
			]
		]
		+ SVerticalBox::Slot().AutoHeight() [ Boxes ];
}

TSharedRef<SWidget> SFSDSBenchPage::MakeComponent(const FText& Title, TFunction<FString()> Part, TFunction<FString()> Version,
	TSharedPtr<SWidget> Extra, bool bVersioned)
{
	auto Color = [this, Part]()
	{
		switch (BoxState(Part()))
		{
		case EBox::Ready: return BenchUi::Good;
		case EBox::NotNow: return BenchUi::Yellow;
		default: return BenchUi::Grey;
		}
	};
	auto Needed = [this, Part]() { return Needs().Contains(Part()); };
	TSharedRef<SVerticalBox> Body = SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(STextBlock).Text(Title).Font(BenchUi::Font(12, TEXT("Bold"))).ColorAndOpacity(FLinearColor::White)
		]
		// the state: a coloured dot, the words in white
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Top).Padding(0, 3, 6, 0)
			[
				SNew(SBox).WidthOverride(10.f).HeightOverride(10.f)
				[
					SNew(SBorder).BorderImage(BenchUi::White()).BorderBackgroundColor_Lambda([Color]() { return FSlateColor(Color()); })
				]
			]
			+ SHorizontalBox::Slot().FillWidth(1.f)
			[
				SNew(STextBlock)
				.Text_Lambda([this, Part]()
				{
					FString Why;
					const EBox State = BoxState(Part(), &Why);
					const TCHAR* Word = State == EBox::Ready ? TEXT("Ready") : State == EBox::NotNow ? TEXT("Not now")
						: State == EBox::Never ? TEXT("Not on this machine") : TEXT("Not known");
					return FText::FromString(Why.IsEmpty() ? FString(Word) : FString::Printf(TEXT("%s: %s"), Word, *Why));
				})
				.ColorAndOpacity(BenchUi::Ink)
				.Font(BenchUi::Font(10, TEXT("Bold"))).AutoWrapText(true)
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
		[
			SNew(STextBlock).Text_Lambda([Version]() { return FText::FromString(Version()); })
			.Font(BenchUi::Font(10)).ColorAndOpacity(BenchUi::Ink).AutoWrapText(true)
		];
	if (Extra.IsValid())
	{
		Body->AddSlot().AutoHeight().Padding(0, 8, 0, 0) [ Extra.ToSharedRef() ];
	}
	if (bVersioned)
	{
		Body->AddSlot().AutoHeight().Padding(0, 8, 0, 0)
		[
			SNew(STextBlock)
			.Text(LOCTEXT("VersionLater", "Other versions: with the bench PC runner. A local run uses this checkout."))
			.Font(BenchUi::Font(9)).ColorAndOpacity(BenchUi::Muted).AutoWrapText(true)
		];
	}
	// A coloured edge marks what the run needs.
	return SNew(SBorder)
		.BorderImage(BenchUi::White())
		.BorderBackgroundColor_Lambda([Color, Needed]() { return FSlateColor(Needed() ? Color() : BenchUi::Edge); })
		.Padding(2.f)
		[
			SNew(SBox).WidthOverride(270.f).MinDesiredHeight(150.f)
			[
				SNew(SBorder)
				.BorderImage(BenchUi::White())
				.BorderBackgroundColor(BenchUi::Field)
				.Padding(FMargin(12.f, 10.f))
				[
					Body
				]
			]
		];
}

TSharedRef<SWidget> SFSDSBenchPage::MakeOverridesRow()
{
	auto Open = BenchUi::Button(LOCTEXT("OpenOverrides", "Open the folder"), BenchUi::PlainBtn, []()
	{
		const FString Repo = RepoDir();
		if (Repo.IsEmpty()) return;
		const FString Dir = FPaths::Combine(Repo, TEXT("tools/sim_benchmark/specs/overrides"));
		IFileManager::Get().MakeDirectory(*Dir, true);
		FPlatformProcess::ExploreFolder(*Dir);
	}, TAttribute<bool>::CreateLambda([]() { return !RepoDir().IsEmpty(); }));

	TSharedRef<SWrapBox> W = BenchUi::Wrap();
	W->AddSlot()
	[
		BenchUi::Choice(LOCTEXT("NoOverride", "None"), [this]() { return ChosenOverrides.Num() == 0; },
			[this]() { ChosenOverrides.Reset(); Edited(); })
	];
	for (const FOverride& O : Overrides)
	{
		if (!O.Error.IsEmpty())
		{
			W->AddSlot() [ BenchUi::Small(FText::FromString(O.Error), BenchUi::Bad) ];
			continue;
		}
		W->AddSlot()
		[
			BenchUi::Choice(FText::FromString(O.Name), [this, N = O.Name]() { return ChosenOverrides.Contains(N); },
				[this, N = O.Name]() { if (ChosenOverrides.Remove(N) == 0) ChosenOverrides.Add(N); Edited(); },
				FText::FromString(O.About))
		];
	}
	W->AddSlot() [ Open ];
	return BenchUi::Row(TEXT("Parameter changes"),
		Overrides.Num() == 0 ? TEXT("Files of pipeline parameter changes (YAML, a pipeline: section) go in the overrides folder; Refresh shows them.")
			: TEXT("Files of pipeline parameter changes, over the preset's own; a file picked later wins."),
		W);
}

TSharedRef<SWidget> SFSDSBenchPage::MakeSettingRow(const FBench& B, const FSetting& S)
{
	FString Label, Help;
	BenchUi::Words(S.Name, S.Help, Label, Help);
	const FString N = B.Name, Key = S.Name;

	// Yes, no, or the benchmark's default.
	if (S.Type == TEXT("boolean"))
	{
		auto Is = [this, N, Key](int32 Want)   // 0 default, 1 yes, 2 no
		{
			const TSharedPtr<FJsonObject> O = SettingsObj(N);
			bool V = false;
			if (!O.IsValid() || !O->TryGetBoolField(Key, V)) return Want == 0;
			return Want == (V ? 1 : 2);
		};
		auto Set = [this, N, Key](int32 To)
		{
			const TSharedPtr<FJsonObject> O = SettingsObj(N, true);
			if (To == 0) O->RemoveField(Key); else O->SetBoolField(Key, To == 1);
			Edited();
		};
		TSharedRef<SHorizontalBox> H = SNew(SHorizontalBox);
		const FText Names[] = { LOCTEXT("Default", "Default"), LOCTEXT("On", "Yes"), LOCTEXT("Off", "No") };
		for (int32 I = 0; I < 3; ++I)
		{
			H->AddSlot().AutoWidth().Padding(0, 0, 4, 0) [ BenchUi::Choice(Names[I], [Is, I]() { return Is(I); }, [Set, I]() { Set(I); }) ];
		}
		return BenchUi::Row(Label, Help, H);
	}

	// One of a few values: the manifest's choices, or the stack's.
	TArray<TPair<FString, FText>> Choices;
	for (const FString& C : S.Choices) Choices.Emplace(C, FText::FromString(C));
	if (S.Name == TEXT("stack") && Choices.Num() == 0)
	{
		Choices.Emplace(TEXT("own"), LOCTEXT("StackOwn", "Its own"));
		Choices.Emplace(TEXT("running"), LOCTEXT("StackRunning", "The running one"));
	}
	if (Choices.Num() > 0)
	{
		TSharedRef<SWrapBox> W = BenchUi::Wrap();
		W->AddSlot()
		[
			BenchUi::Choice(LOCTEXT("Default", "Default"), [this, N, Key]() { return BenchUi::FieldValue(SettingsObj(N), Key).IsEmpty(); },
				[this, N, Key]() { SettingsObj(N, true)->RemoveField(Key); Edited(); })
		];
		for (const TPair<FString, FText>& C : Choices)
		{
			W->AddSlot()
			[
				BenchUi::Choice(C.Value, [this, N, Key, V = C.Key]() { return BenchUi::FieldValue(SettingsObj(N), Key) == V; },
					[this, N, Key, V = C.Key, Type = S.Type]() { BenchUi::SetTyped(SettingsObj(N, true), Key, V, Type); Edited(); })
			];
		}
		return BenchUi::Row(Label, Help, W);
	}

	// Typed: a number or text; empty is the default.
	const FString FieldKey = N + TEXT(".") + Key;
	FieldText.Add(FieldKey, BenchUi::FieldValue(SettingsObj(N), Key));
	return BenchUi::Row(Label, Help,
		SNew(SBox).HAlign(HAlign_Left)
		[
			SNew(SEditableTextBox)
			.HintText(LOCTEXT("DefaultHint", "default"))
			.MinDesiredWidth(S.Type == TEXT("string") ? 320.f : 110.f)
			.Text_Lambda([this, FieldKey]() { const FString* T = FieldText.Find(FieldKey); return FText::FromString(T ? *T : FString()); })
			.OnTextChanged_Lambda([this, N, Key, FieldKey, Type = S.Type](const FText& T)
			{
				FieldText.Add(FieldKey, T.ToString());
				BenchUi::SetTyped(SettingsObj(N, true), Key, T.ToString(), Type);
				Edited();
			})
			.Font(BenchUi::Font(10))
		]);
}

// ---------------------------------------------------------------------------
// This machine

SFSDSBenchPage::EBox SFSDSBenchPage::BoxState(const FString& Part, FString* Why) const
{
	auto Say = [Why](EBox State, const FString& Text) { if (Why) *Why = Text; return State; };
	if (bRunner || !bRigKnown || bRigStale)
	{
		// Old or missing checks can't be trusted (bench_queue.rig); the why is above the list.
		return Say(EBox::Unknown, FString());
	}
	const FPart* P = Parts.Find(Part);
	if (!P) return Say(EBox::Unknown, TEXT("Not checked."));
	if (P->Status == TEXT("ready")) return Say(EBox::Ready, P->Reason);
	if (P->Status == TEXT("unavailable")) return Say(EBox::NotNow, P->Reason);
	return Say(EBox::Never, P->Reason);
}

const TArray<FString>& SFSDSBenchPage::Needs() const
{
	static const TArray<FString> None;
	if (bChecked) return CheckNeeds;
	const FPreset* P = Chosen();
	return P ? P->Needs : None;
}

FString SFSDSBenchPage::NeverHere() const
{
	if (bRunner) return FString();
	TArray<FString> Out;
	for (const FString& Part : Needs())
	{
		FString Why;
		if (BoxState(Part, &Why) == EBox::Never) Out.Add(Why.IsEmpty() ? Part : Why);
	}
	return FString::Join(Out, TEXT("; "));
}

bool SFSDSBenchPage::CanQueue() const
{
	if (!Client.IsValid() || bLaunching || !Chosen() || !HasBenchmarks() || !NeverHere().IsEmpty()) return false;
	// Without the manifest the preset goes as it is, and the worker checks it.
	if (!bManifestKnown || !ManifestError.IsEmpty()) return true;
	return CheckDueAt <= 0.0 && bChecked && bCheckOk;
}

const SFSDSBenchPage::FPreset* SFSDSBenchPage::Chosen() const
{
	return Presets.FindByPredicate([this](const FPreset& P) { return P.Name == PresetName; });
}

// ---------------------------------------------------------------------------
// The backend

EActiveTimerReturnType SFSDSBenchPage::Poll(double Now, float Dt)
{
	const bool bActive = IsActive.Get(true);
	const double T = FPlatformTime::Seconds();
	if (bActive && !bWasActive)
	{
		FetchPresets();
		FetchManifest();
		NextQueue = NextRig = NextMachineCheck = 0.0;
	}
	bWasActive = bActive;
	if (!bActive) return EActiveTimerReturnType::Continue;
	// The worker checks the machine every minute; without one, the page does.
	if (bQueueKnown && !bWorkerAlive && !bRunner && T >= NextMachineCheck) CheckMachine();
	if (T >= NextQueue) FetchQueue();
	if (T >= NextRig && !bRunner) FetchRig();
	if (CheckDueAt > 0.0 && T >= CheckDueAt) Check();
	return EActiveTimerReturnType::Continue;
}

FString SFSDSBenchPage::Unreachable(const FString& What, const FFSDSMcReply& R) const
{
	return R.Reached() ? FString::Printf(TEXT("%s: %s"), *What, *R.Error)
		: TEXT("Mission Control's backend isn't running (START brings it up).");
}

void SFSDSBenchPage::FetchPresets()
{
	if (!Client.IsValid()) { PresetsNote = TEXT("No Mission Control backend configured."); return; }
	TWeakPtr<SFSDSBenchPage> Weak = SharedThis(this);
	Client->Get(TEXT("/api/bench/presets"), [Weak](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSBenchPage> Self = Weak.Pin();
		if (!Self.IsValid()) return;
		SFSDSBenchPage* Page = Self.Get();
		const TSharedPtr<FJsonObject> O = R.Object();
		const TArray<TSharedPtr<FJsonValue>>* List = nullptr;
		if (!R.Ok() || !O.IsValid() || !O->TryGetArrayField(TEXT("presets"), List))
		{
			Page->PresetsNote = Page->Unreachable(TEXT("Couldn't read the presets"), R);
			return;
		}
		Page->PresetsNote.Reset();
		Page->Presets.Reset();
		for (const TSharedPtr<FJsonValue>& V : *List)
		{
			const TSharedPtr<FJsonObject>* PO = nullptr;
			if (!V.IsValid() || !V->TryGetObject(PO)) continue;
			FPreset P;
			(*PO)->TryGetStringField(TEXT("name"), P.Name);
			(*PO)->TryGetStringField(TEXT("title"), P.Title);
			(*PO)->TryGetStringField(TEXT("about"), P.About);
			(*PO)->TryGetStringField(TEXT("error"), P.Error);
			const TSharedPtr<FJsonObject>* Spec = nullptr;
			if ((*PO)->TryGetObjectField(TEXT("spec"), Spec)) P.Spec = *Spec;
			(*PO)->TryGetStringArrayField(TEXT("needs"), P.Needs);
			if (P.Title.IsEmpty()) P.Title = P.Name;
			Page->Presets.Add(MoveTemp(P));
		}
		Page->Overrides.Reset();
		const TArray<TSharedPtr<FJsonValue>>* OList = nullptr;
		if (O->TryGetArrayField(TEXT("overrides"), OList))
		{
			for (const TSharedPtr<FJsonValue>& V : *OList)
			{
				const TSharedPtr<FJsonObject>* OO = nullptr;
				if (!V.IsValid() || !V->TryGetObject(OO)) continue;
				FOverride Ov;
				(*OO)->TryGetStringField(TEXT("name"), Ov.Name);
				(*OO)->TryGetStringField(TEXT("about"), Ov.About);
				(*OO)->TryGetStringField(TEXT("error"), Ov.Error);
				const TSharedPtr<FJsonObject>* Pl = nullptr;
				if ((*OO)->TryGetObjectField(TEXT("pipeline"), Pl)) Ov.Pipeline = *Pl;
				Page->Overrides.Add(MoveTemp(Ov));
			}
		}
		Page->ChosenOverrides.RemoveAll([Page](const FString& Name)
		{
			return !Page->Overrides.ContainsByPredicate([&Name](const FOverride& X) { return X.Name == Name && X.Error.IsEmpty(); });
		});
		if (Page->Presets.Num() == 0) Page->PresetsNote = TEXT("No presets in tools/sim_benchmark/specs.");
		Page->RebuildPresets();
		// Keep what is being edited; start from the first preset otherwise.
		if (!Page->Chosen() || !Page->bEdited)
		{
			const FString Pick = Page->Chosen() ? Page->PresetName : Page->Presets.Num() > 0 ? Page->Presets[0].Name : FString();
			if (!Pick.IsEmpty()) Page->PickPreset(Pick);
		}
		else
		{
			Page->RebuildForm();
		}
	});
}

void SFSDSBenchPage::FetchManifest()
{
	if (!Client.IsValid()) return;
	TWeakPtr<SFSDSBenchPage> Weak = SharedThis(this);
	Client->Get(TEXT("/api/bench/manifest"), [Weak](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSBenchPage> Self = Weak.Pin();
		if (!Self.IsValid()) return;
		const TSharedPtr<FJsonObject> O = R.Object();
		Self->bManifestKnown = true;
		if (!R.Ok() || !O.IsValid())
		{
			// An older backend has no /api/bench/manifest: 404.
			Self->ManifestError = R.Code == 404 ? TEXT("this backend is older than the page: START again to update it.")
				: Self->Unreachable(TEXT("couldn't read it"), R);
			Self->RebuildForm();
			return;
		}
		Self->ManifestError.Reset();
		O->TryGetStringField(TEXT("error"), Self->ManifestError);
		Self->Benches.Reset();
		const TArray<TSharedPtr<FJsonValue>>* List = nullptr;
		if (O->TryGetArrayField(TEXT("benchmarks"), List))
		{
			for (const TSharedPtr<FJsonValue>& V : *List)
			{
				const TSharedPtr<FJsonObject>* BO = nullptr;
				if (!V.IsValid() || !V->TryGetObject(BO)) continue;
				FBench B;
				(*BO)->TryGetStringField(TEXT("name"), B.Name);
				(*BO)->TryGetStringField(TEXT("title"), B.Title);
				(*BO)->TryGetStringField(TEXT("description"), B.Description);
				(*BO)->TryGetStringField(TEXT("bags"), B.Bags);   // null: takes no bags
				(*BO)->TryGetStringArrayField(TEXT("parts"), B.Parts);
				const TArray<TSharedPtr<FJsonValue>>* SList = nullptr;
				if ((*BO)->TryGetArrayField(TEXT("settings"), SList))
				{
					for (const TSharedPtr<FJsonValue>& SV : *SList)
					{
						const TSharedPtr<FJsonObject>* SO = nullptr;
						if (!SV.IsValid() || !SV->TryGetObject(SO)) continue;
						FSetting S;
						(*SO)->TryGetStringField(TEXT("name"), S.Name);
						(*SO)->TryGetStringField(TEXT("type"), S.Type);
						(*SO)->TryGetStringField(TEXT("help"), S.Help);
						(*SO)->TryGetStringArrayField(TEXT("choices"), S.Choices);
						B.Settings.Add(MoveTemp(S));
					}
				}
				if (B.Title.IsEmpty()) B.Title = B.Name;
				Self->Benches.Add(MoveTemp(B));
			}
		}
		Self->BagsByKind.Reset();
		const TSharedPtr<FJsonObject>* Bags = nullptr;
		if (O->TryGetObjectField(TEXT("bags"), Bags))
		{
			for (const auto& KV : (*Bags)->Values)
			{
				TArray<FString> Names;
				const TArray<TSharedPtr<FJsonValue>>* L = nullptr;
				if (KV.Value.IsValid() && KV.Value->TryGetArray(L))
				{
					for (const auto& BV : *L)
					{
						const TSharedPtr<FJsonObject>* BagO = nullptr;
						FString Name;
						if (BV.IsValid() && BV->TryGetObject(BagO) && (*BagO)->TryGetStringField(TEXT("name"), Name)) Names.Add(Name);
					}
				}
				Self->BagsByKind.Add(FString(*KV.Key), MoveTemp(Names));
			}
		}
		Self->Missions.Reset();
		Self->TrackNames.Reset();
		O->TryGetStringArrayField(TEXT("missions"), Self->Missions);
		O->TryGetStringArrayField(TEXT("tracks"), Self->TrackNames);
		Self->MissionTracks.Reset();
		const TSharedPtr<FJsonObject>* Own = nullptr;
		if (O->TryGetObjectField(TEXT("mission_tracks"), Own))
		{
			for (const auto& KV : (*Own)->Values)
			{
				TArray<FString> L;
				const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
				if (KV.Value.IsValid() && KV.Value->TryGetArray(A)) for (const auto& V : *A) L.Add(V->AsString());
				Self->MissionTracks.Add(FString(*KV.Key), MoveTemp(L));
			}
		}
		Self->RebuildForm();
	});
}

void SFSDSBenchPage::Check()
{
	CheckDueAt = 0.0;
	if (!Client.IsValid() || !HasBenchmarks())
	{
		bChecked = true;
		bCheckOk = false;
		CheckErrors = { TEXT("Pick what to test.") };
		CheckNeeds.Reset();
		return;
	}
	const int32 Serial = ++CheckSerial;
	const TSharedRef<FJsonObject> Body = MakeShared<FJsonObject>();
	Body->SetObjectField(TEXT("spec"), BuildSpec());
	TWeakPtr<SFSDSBenchPage> Weak = SharedThis(this);
	Client->Post(TEXT("/api/bench/check"), Body, [Weak, Serial](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSBenchPage> Self = Weak.Pin();
		if (!Self.IsValid() || Serial != Self->CheckSerial) return;   // an older check: the draft changed since
		const TSharedPtr<FJsonObject> O = R.Object();
		Self->bChecked = true;
		Self->bCheckOk = false;
		Self->CheckRuns = 0;
		Self->CheckErrors.Reset();
		Self->CheckLabels.Reset();
		Self->CheckLines.Reset();
		if (!R.Ok() || !O.IsValid())
		{
			Self->CheckErrors.Add(Self->Unreachable(TEXT("Couldn't check it"), R));
			return;
		}
		O->TryGetBoolField(TEXT("ok"), Self->bCheckOk);
		O->TryGetStringArrayField(TEXT("errors"), Self->CheckErrors);
		O->TryGetStringArrayField(TEXT("labels"), Self->CheckLabels);
		O->TryGetStringArrayField(TEXT("needs"), Self->CheckNeeds);
		O->TryGetStringArrayField(TEXT("lines"), Self->CheckLines);
		O->TryGetNumberField(TEXT("jobs"), Self->CheckJobs);
		if (!O->TryGetNumberField(TEXT("runs"), Self->CheckRuns)) Self->CheckRuns = Self->CheckJobs;   // an older backend
	});
}

void SFSDSBenchPage::RebuildPresets()
{
	if (!PresetBox.IsValid()) return;
	PresetBox->ClearChildren();
	for (const FPreset& P : Presets)
	{
		PresetBox->AddSlot()
		[
			BenchUi::Choice(FText::FromString(P.Title), [this, Name = P.Name]() { return PresetName == Name && !bEdited; },
				[this, Name = P.Name]() { PickPreset(Name); }, FText::FromString(P.About))
		];
	}
}

void SFSDSBenchPage::FetchRig()
{
	NextRig = FPlatformTime::Seconds() + BenchUi::RigEveryS;
	if (!Client.IsValid()) return;
	TWeakPtr<SFSDSBenchPage> Weak = SharedThis(this);
	Client->Get(TEXT("/api/bench/rig"), [Weak](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSBenchPage> Self = Weak.Pin();
		if (!Self.IsValid()) return;
		const TSharedPtr<FJsonObject> O = R.Object();
		if (!R.Ok() || !O.IsValid())
		{
			Self->bRigKnown = false;
			Self->RigReason = R.Reached() ? R.Error : TEXT("the backend isn't running");
			return;
		}
		Self->bRigKnown = false;
		O->TryGetBoolField(TEXT("known"), Self->bRigKnown);
		Self->bRigStale = true;
		O->TryGetBoolField(TEXT("stale"), Self->bRigStale);
		Self->RigReason.Reset();
		O->TryGetStringField(TEXT("reason"), Self->RigReason);
		Self->RigId.Reset();
		O->TryGetStringField(TEXT("id"), Self->RigId);
		Self->RigAge = 0.0;
		O->TryGetNumberField(TEXT("age_s"), Self->RigAge);
		Self->Code.Reset();
		const TSharedPtr<FJsonObject>* CodeObj = nullptr;
		if (O->TryGetObjectField(TEXT("code"), CodeObj))
		{
			for (const auto& KV : (*CodeObj)->Values)
			{
				const TSharedPtr<FJsonObject>* CO = nullptr;
				if (!KV.Value.IsValid() || !KV.Value->TryGetObject(CO)) continue;
				FCode C;
				(*CO)->TryGetStringField(TEXT("commit"), C.Commit);
				(*CO)->TryGetStringField(TEXT("branch"), C.Branch);   // null: a pinned submodule
				(*CO)->TryGetBoolField(TEXT("dirty"), C.bDirty);
				Self->Code.Add(FString(*KV.Key), C);
			}
		}
		Self->Parts.Reset();
		const TSharedPtr<FJsonObject>* PartsObj = nullptr;
		if (O->TryGetObjectField(TEXT("parts"), PartsObj))
		{
			for (const auto& KV : (*PartsObj)->Values)
			{
				const TSharedPtr<FJsonObject>* PO = nullptr;
				if (!KV.Value.IsValid() || !KV.Value->TryGetObject(PO)) continue;
				FPart P;
				(*PO)->TryGetStringField(TEXT("status"), P.Status);
				(*PO)->TryGetStringField(TEXT("reason"), P.Reason);
				Self->Parts.Add(FString(*KV.Key), MoveTemp(P));
			}
		}
	});
}

void SFSDSBenchPage::FetchQueue()
{
	NextQueue = FPlatformTime::Seconds() + BenchUi::QueueEveryS;
	if (!Client.IsValid()) return;
	TWeakPtr<SFSDSBenchPage> Weak = SharedThis(this);
	Client->Get(TEXT("/api/bench/queue"), [Weak](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSBenchPage> Self = Weak.Pin();
		if (!Self.IsValid()) return;
		const TSharedPtr<FJsonObject> O = R.Object();
		const TArray<TSharedPtr<FJsonValue>>* List = nullptr;
		if (!R.Ok() || !O.IsValid() || !O->TryGetArrayField(TEXT("jobs"), List)) return;   // keep what's shown; try again next poll

		Self->bQueueKnown = true;
		const TSharedPtr<FJsonObject>* W = nullptr;
		const bool bWasAlive = Self->bWorkerAlive;
		Self->bWorkerAlive = Self->bWorkerStopping = false;
		if (O->TryGetObjectField(TEXT("worker"), W))
		{
			(*W)->TryGetBoolField(TEXT("alive"), Self->bWorkerAlive);
			(*W)->TryGetBoolField(TEXT("stopping"), Self->bWorkerStopping);
			(*W)->TryGetStringField(TEXT("host"), Self->WorkerHost);
			(*W)->TryGetNumberField(TEXT("pid"), Self->WorkerPid);
		}
		if (Self->bWorkerAlive)
		{
			Self->WorkerStartingUntil = 0.0;
			if (!bWasAlive) Self->FetchRig();   // its first checks
		}
		TArray<FJob> Jobs;
		for (const TSharedPtr<FJsonValue>& V : *List)
		{
			const TSharedPtr<FJsonObject>* JO = nullptr;
			if (!V.IsValid() || !V->TryGetObject(JO)) continue;
			FJob J;
			(*JO)->TryGetStringField(TEXT("id"), J.Id);
			(*JO)->TryGetStringField(TEXT("name"), J.Name);
			(*JO)->TryGetStringField(TEXT("state"), J.State);
			(*JO)->TryGetStringField(TEXT("created"), J.Created);
			(*JO)->TryGetStringField(TEXT("requested_by"), J.RequestedBy);
			(*JO)->TryGetBoolField(TEXT("cancel_requested"), J.bCancelRequested);
			const TSharedPtr<FJsonObject>* St = nullptr;
			if ((*JO)->TryGetObjectField(TEXT("status"), St))
			{
				// null fields (current, waiting) read as empty
				(*St)->TryGetStringField(TEXT("current"), J.Current);
				(*St)->TryGetStringField(TEXT("waiting"), J.Waiting);
				(*St)->TryGetStringField(TEXT("error"), J.Error);
				(*St)->TryGetNumberField(TEXT("done"), J.Done);
				(*St)->TryGetNumberField(TEXT("total"), J.Total);
			}
			Jobs.Add(MoveTemp(J));
		}
		// Rebuild only on a change: a rebuild under the cursor eats a click.
		FString Shown;
		for (const FJob& J : Jobs)
		{
			Shown += FString::Printf(TEXT("%s|%s|%d/%d|%s|%s|%s|%d\n"), *J.Id, *J.State, J.Done, J.Total, *J.Current,
				*J.Waiting, *J.Error, J.bCancelRequested ? 1 : 0);
		}
		Self->Jobs = MoveTemp(Jobs);
		if (Shown != Self->JobsShown || Self->QueueList->NumSlots() == 0)
		{
			Self->JobsShown = Shown;
			Self->RebuildQueue();
		}
	});
}

void SFSDSBenchPage::Launch()
{
	if (!CanQueue()) return;
	bLaunching = true;
	SetNote(FString(), false);
	const FString Name = SpecName();
	const TSharedRef<FJsonObject> Body = MakeShared<FJsonObject>();
	Body->SetObjectField(TEXT("spec"), BuildSpec());
	Body->SetStringField(TEXT("target"), bRunner ? TEXT("runner") : TEXT("local"));
	Body->SetStringField(TEXT("name"), Name);
	Body->SetStringField(TEXT("requested_by"), FString::Printf(TEXT("%s@%s"), FPlatformProcess::UserName(), FPlatformProcess::ComputerName()));
	TWeakPtr<SFSDSBenchPage> Weak = SharedThis(this);
	Client->Post(TEXT("/api/bench/launch"), Body, [Weak, Name](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSBenchPage> Self = Weak.Pin();
		if (!Self.IsValid()) return;
		Self->bLaunching = false;
		const TSharedPtr<FJsonObject> O = R.Object();
		if (!R.Ok() || !O.IsValid())
		{
			Self->SetNote(Self->Unreachable(FString::Printf(TEXT("Couldn't queue %s"), *Name), R), true);
			return;
		}
		bool bWorker = false;
		O->TryGetBoolField(TEXT("worker_running"), bWorker);
		Self->SetNote(bWorker ? FString::Printf(TEXT("Queued %s."), *Name)
			: FString::Printf(TEXT("Queued %s. It waits for the worker: Start worker, on the right."), *Name), false);
		Self->FetchQueue();
	});
}

void SFSDSBenchPage::Cancel(const FString& Id)
{
	if (!Client.IsValid()) return;
	TWeakPtr<SFSDSBenchPage> Weak = SharedThis(this);
	Client->Post(FString::Printf(TEXT("/api/bench/cancel/%s"), *FFSDSMissionControlClient::EncodeSegment(Id)),
		MakeShared<FJsonObject>(), [Weak, Id](const FFSDSMcReply& R)
		{
			TSharedPtr<SFSDSBenchPage> Self = Weak.Pin();
			if (!Self.IsValid()) return;
			if (!R.Ok()) Self->SetNote(Self->Unreachable(FString::Printf(TEXT("Couldn't cancel %s"), *Id), R), true);
			Self->FetchQueue();
		});
}

// ---------------------------------------------------------------------------
// The worker

FString SFSDSBenchPage::RepoDir()
{
	return FFSDSStackStarter::FindComposeDir();
}

void SFSDSBenchPage::CheckMachine()
{
	const double T = FPlatformTime::Seconds();
	NextMachineCheck = T + 60.0;
#if PLATFORM_WINDOWS
	return;   // run_spec.py's checks need a Linux host (the worker runs in WSL there)
#else
	const FString Repo = RepoDir();
	if (Repo.IsEmpty() || T < MachineCheckUntil) return;
	// run_spec.py --rig saves its checks where the backend reads them (results/.bench/rig.json).
	const FString Bench = FPaths::Combine(Repo, TEXT("tools/sim_benchmark/results/.bench"));
	IFileManager::Get().MakeDirectory(*Bench, true);
	auto Quote = [](const FString& S) { return TEXT("'") + S.Replace(TEXT("'"), TEXT("'\\''")) + TEXT("'"); };
	const FString Script = FPaths::CreateTempFilename(FPlatformProcess::UserTempDir(), TEXT("fsds-rig-"), TEXT(".sh"));
	const FString Body = FString::Printf(TEXT("cd %s || exit 1\nexec python3 tools/sim_benchmark/run_spec.py --rig --json > /dev/null 2>> %s\n"),
		*Quote(Repo), *Quote(FPaths::Combine(Bench, TEXT("rig-check.out"))));
	if (!FFileHelper::SaveStringToFile(Body, *Script, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM)) return;
	FProcHandle H = FPlatformProcess::CreateProc(TEXT("/bin/bash"), *FString::Printf(TEXT("-l \"%s\""), *Script),
		/*bLaunchDetached=*/true, /*bLaunchHidden=*/true, /*bLaunchReallyHidden=*/true, nullptr, 0, nullptr, nullptr);
	if (!H.IsValid()) return;
	FPlatformProcess::CloseProc(H);
	// nvidia-smi, docker info and the LattePanda's ssh take a few seconds; read it back then.
	MachineCheckUntil = T + 12.0;
	TWeakPtr<SFSDSBenchPage> Weak = SharedThis(this);
	for (const float After : { 4.f, 8.f, 13.f })
	{
		RegisterActiveTimer(After, FWidgetActiveTimerDelegate::CreateLambda([Weak](double, float)
		{
			if (TSharedPtr<SFSDSBenchPage> Self = Weak.Pin()) Self->FetchRig();
			return EActiveTimerReturnType::Stop;
		}));
	}
#endif
}

void SFSDSBenchPage::StartWorker()
{
#if PLATFORM_WINDOWS
	SetNote(TEXT("Start the worker in a terminal (WSL): python3 tools/sim_benchmark/run_spec.py --work"), true);
#else
	const FString Repo = RepoDir();
	if (Repo.IsEmpty())
	{
		SetNote(TEXT("The IFSSIM folder isn't set: START once to set it up."), true);
		return;
	}
	// It runs on the host like the sim (a benchmark needs the host's Docker and paths),
	// and outlives the page: the queue keeps running when you leave it. Through a script,
	// as FFSDSStackStarter::Run does: CreateProc on Linux ignores the working folder and
	// splits its arguments on spaces.
	const FString Bench = FPaths::Combine(Repo, TEXT("tools/sim_benchmark/results/.bench"));
	IFileManager::Get().MakeDirectory(*Bench, true);
	const FString Log = FPaths::Combine(Bench, TEXT("worker.out"));
	auto Quote = [](const FString& S) { return TEXT("'") + S.Replace(TEXT("'"), TEXT("'\\''")) + TEXT("'"); };
	const FString Script = FPaths::CreateTempFilename(FPlatformProcess::UserTempDir(), TEXT("fsds-worker-"), TEXT(".sh"));
	const FString Body = FString::Printf(TEXT("cd %s || exit 1\nexec python3 tools/sim_benchmark/run_spec.py --work >> %s 2>&1\n"),
		*Quote(Repo), *Quote(Log));
	if (!FFileHelper::SaveStringToFile(Body, *Script, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
	{
		SetNote(FString::Printf(TEXT("Couldn't write %s."), *Script), true);
		return;
	}
	FProcHandle H = FPlatformProcess::CreateProc(TEXT("/bin/bash"), *FString::Printf(TEXT("-l \"%s\""), *Script),
		/*bLaunchDetached=*/true, /*bLaunchHidden=*/true, /*bLaunchReallyHidden=*/true, nullptr, 0, nullptr, nullptr);
	if (!H.IsValid())
	{
		SetNote(TEXT("Couldn't start the worker (/bin/bash)."), true);
		return;
	}
	FPlatformProcess::CloseProc(H);
	WorkerStartingUntil = FPlatformTime::Seconds() + BenchUi::WorkerStartS;
	SetNote(FString::Printf(TEXT("Starting the worker; its output goes to %s."), *Log), false);
	NextQueue = 0.0;
	UE_LOG(LogTemp, Log, TEXT("FSDS Benchmarks: starting the worker in %s (log %s)"), *Repo, *Log);

	// No heartbeat in time: say where to look.
	TWeakPtr<SFSDSBenchPage> Weak = SharedThis(this);
	RegisterActiveTimer(BenchUi::WorkerStartS + 1.f, FWidgetActiveTimerDelegate::CreateLambda([Weak, Log](double, float)
	{
		TSharedPtr<SFSDSBenchPage> Self = Weak.Pin();
		if (Self.IsValid() && !Self->bWorkerAlive)
		{
			Self->SetNote(FString::Printf(TEXT("The worker didn't start: see %s."), *Log), true);
		}
		return EActiveTimerReturnType::Stop;
	}));
#endif
}

void SFSDSBenchPage::StopWorker()
{
	if (!Client.IsValid()) return;
	TWeakPtr<SFSDSBenchPage> Weak = SharedThis(this);
	Client->Post(TEXT("/api/bench/worker/stop"), MakeShared<FJsonObject>(), [Weak](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSBenchPage> Self = Weak.Pin();
		if (!Self.IsValid()) return;
		if (!R.Ok()) Self->SetNote(Self->Unreachable(TEXT("Couldn't stop the worker"), R), true);
		else Self->bWorkerStopping = true;
		Self->FetchQueue();
	});
}

// ---------------------------------------------------------------------------
// The queue

FString SFSDSBenchPage::JobDir(const FString& Id)
{
	const FString Repo = RepoDir();
	if (Repo.IsEmpty()) return FString();
	const FString Dir = FPaths::Combine(Repo, TEXT("tools/sim_benchmark/results/.bench/queue"), Id);
	return FPaths::DirectoryExists(Dir) ? Dir : FString();
}

void SFSDSBenchPage::RebuildQueue()
{
	if (!QueueList.IsValid()) return;
	QueueList->ClearChildren();
	if (Jobs.Num() == 0)
	{
		QueueList->AddSlot().AutoHeight()
		[
			SNew(STextBlock).Text(LOCTEXT("Empty", "Nothing queued yet.")).Font(BenchUi::Font(10)).ColorAndOpacity(BenchUi::Muted)
		];
		return;
	}
	for (const FJob& J : Jobs)
	{
		TArray<FString> Meta;
		if (!J.Created.IsEmpty()) Meta.Add(BenchUi::When(J.Created));
		if (!J.RequestedBy.IsEmpty()) Meta.Add(J.RequestedBy);
		TArray<FString> Detail;
		if (J.Total > 0) Detail.Add(FString::Printf(TEXT("%d of %d jobs"), J.Done, J.Total));
		if (!J.Current.IsEmpty() && J.State == TEXT("running")) Detail.Add(J.Current);
		if (!J.Waiting.IsEmpty() && J.State == TEXT("queued")) Detail.Add(FString::Printf(TEXT("waiting: %s"), *J.Waiting));
		const bool bCancellable = J.State == TEXT("queued") || J.State == TEXT("running");
		const FString Dir = JobDir(J.Id);

		QueueList->AddSlot().AutoHeight().Padding(0, 0, 0, 6)
		[
			SNew(SBorder)
			.BorderImage(BenchUi::White())
			.BorderBackgroundColor(BenchUi::Card)
			.Padding(FMargin(10.f, 7.f))
			.ToolTipText(FText::FromString(J.Id))
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
					[
						SNew(STextBlock)
						.Text(FText::FromString(J.bCancelRequested && J.State == TEXT("running") ? TEXT("cancelling") : J.State))
						.Font(BenchUi::Font(10, TEXT("Bold"))).ColorAndOpacity(BenchUi::StateColor(J.State))
					]
					+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
					[
						SNew(STextBlock).Text(FText::FromString(J.Name.IsEmpty() ? J.Id : J.Name))
						.Font(BenchUi::Font(11, TEXT("Bold"))).ColorAndOpacity(BenchUi::Ink)
					]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
				[
					SNew(STextBlock).Text(FText::FromString(FString::Join(Meta, TEXT("  ·  "))))
					.Font(BenchUi::Font(9)).ColorAndOpacity(BenchUi::Muted)
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
				[
					SNew(STextBlock).Text(FText::FromString(FString::Join(Detail, TEXT("  ·  "))))
					.Visibility(Detail.Num() ? EVisibility::Visible : EVisibility::Collapsed)
					.Font(BenchUi::Font(9)).ColorAndOpacity(BenchUi::Ink).AutoWrapText(true)
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
				[
					SNew(STextBlock).Text(FText::FromString(J.Error))
					.Visibility(J.Error.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible)
					.Font(BenchUi::Font(9)).ColorAndOpacity(BenchUi::Bad).AutoWrapText(true)
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 5, 0, 0)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
					[
						SNew(SBox).Visibility(Dir.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible)
						[
							// worker.log and each job's folder
							BenchUi::Button(LOCTEXT("Folder", "Folder"), BenchUi::PlainBtn, [Dir]() { FPlatformProcess::ExploreFolder(*Dir); })
						]
					]
					+ SHorizontalBox::Slot().AutoWidth()
					[
						SNew(SBox).Visibility(bCancellable && !J.bCancelRequested ? EVisibility::Visible : EVisibility::Collapsed)
						[
							// A running spec stops mid-benchmark: a second click within 4 s.
							BenchUi::Button(TAttribute<FText>::CreateLambda([this, Id = J.Id]()
								{
									return FPlatformTime::Seconds() < CancelArmedUntil && CancelArmedFor == Id
										? LOCTEXT("CancelSure", "Click again to cancel") : LOCTEXT("Cancel", "Cancel");
								}),
								BenchUi::PlainBtn, [this, Id = J.Id]()
								{
									if (FPlatformTime::Seconds() >= CancelArmedUntil || CancelArmedFor != Id)
									{
										CancelArmedFor = Id;
										CancelArmedUntil = FPlatformTime::Seconds() + 4.0;
										return;
									}
									CancelArmedUntil = 0.0;
									Cancel(Id);
								})
						]
					]
				]
			]
		];
	}
}

#undef LOCTEXT_NAMESPACE
