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
#include "Widgets/Layout/SWrapBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "FSDSBenchPage"

// Always qualified: the UI files share unity blobs.
namespace BenchUi
{
	const FLinearColor Accent = FLinearColor::FromSRGBColor(FColor(255, 184, 28));
	const FLinearColor Ink = FLinearColor::FromSRGBColor(FColor(232, 232, 236));
	const FLinearColor Muted = FLinearColor::FromSRGBColor(FColor(150, 150, 160));
	const FLinearColor Good = FLinearColor::FromSRGBColor(FColor(74, 222, 128));
	const FLinearColor Bad = FLinearColor::FromSRGBColor(FColor(248, 113, 113));
	const FLinearColor Yellow = FLinearColor::FromSRGBColor(FColor(250, 204, 21));
	const FLinearColor Grey = FLinearColor::FromSRGBColor(FColor(110, 110, 120));
	const FLinearColor PlainBtn = FLinearColor::FromSRGBColor(FColor(64, 64, 72));
	const FLinearColor GoBtn = FLinearColor::FromSRGBColor(FColor(34, 197, 94));
	const FLinearColor Card(1.f, 1.f, 1.f, 0.04f);

	constexpr double QueueEveryS = 3.0;
	constexpr double RigEveryS = 15.0;     // the worker checks the rig once a minute
	constexpr double CheckAfterS = 0.5;    // after the last edit
	constexpr double WorkerStartS = 20.0;  // for its first heartbeat
	constexpr float LabelW = 190.f;

	const TCHAR* const PipelineOnSetting = TEXT("pipeline_on");
	const TCHAR* const OnThisPc = TEXT("bench_pc");
	const TCHAR* const OnPanda = TEXT("latte_panda");

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

	/** A choice button: on is the accent text on a lighter button, as the menu's. */
	TSharedRef<SWidget> Choice(TAttribute<FText> Label, TFunction<bool()> IsOn, TFunction<void()> OnPick, const FText& Tip = FText::GetEmpty())
	{
		return SNew(SButton)
			.IsFocusable(false)
			.ContentPadding(FMargin(9.f, 5.f))
			.HAlign(HAlign_Center)
			.ToolTipText(Tip)
			.ButtonColorAndOpacity_Lambda([IsOn]() { return FSlateColor(IsOn() ? PlainBtn : FLinearColor(1.f, 1.f, 1.f, 0.06f)); })
			.OnClicked_Lambda([OnPick]() { OnPick(); return FReply::Handled(); })
			[
				SNew(STextBlock).Text(Label).Font(Font(10, TEXT("Bold")))
				.ColorAndOpacity_Lambda([IsOn]() { return FSlateColor(IsOn() ? Accent : Muted); })
			];
	}

	TSharedRef<SWidget> Heading(const FText& Title)
	{
		return SNew(STextBlock).Text(Title).Font(Font(10, TEXT("Bold"))).ColorAndOpacity(Accent);
	}

	TSharedRef<SWidget> Small(const FText& Text, FLinearColor Color = Muted)
	{
		return SNew(STextBlock).Text(Text).Font(Font(9)).ColorAndOpacity(Color).AutoWrapText(true);
	}

	/** A form row: the label, then the control with the setting's help under it. */
	TSharedRef<SWidget> Row(const FString& Label, const FString& Help, TSharedRef<SWidget> Control)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Top).Padding(0, 4, 8, 0)
			[
				SNew(SBox).WidthOverride(LabelW)
				[
					SNew(STextBlock).Text(FText::FromString(Label)).Font(Font(10, TEXT("Bold"))).ColorAndOpacity(Ink)
					.ToolTipText(FText::FromString(Help))
				]
			]
			+ SHorizontalBox::Slot().FillWidth(1.f)
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight() [ Control ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
				[
					SNew(STextBlock).Text(FText::FromString(Help)).Font(Font(9)).ColorAndOpacity(Muted).AutoWrapText(true)
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
}

void SFSDSBenchPage::Construct(const FArguments& InArgs)
{
	IsActive = InArgs._IsActive;
	Client = InArgs._Client;
	Draft = MakeShared<FJsonObject>();

	ChildSlot
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.f)
			[
				SNew(STextBlock).Text(LOCTEXT("Title", "Benchmarks"))
				.Font(BenchUi::Font(20, TEXT("Bold"))).ColorAndOpacity(BenchUi::Ink)
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				BenchUi::Button(LOCTEXT("Refresh", "Refresh"), BenchUi::PlainBtn, [this]()
				{
					FetchPresets();
					FetchManifest();
					FetchRig();
					FetchQueue();
				})
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 6)
		[
			SNew(STextBlock)
			.Text(LOCTEXT("Hint", "Start from a preset and change what you need, then queue it. Local runs it on this "
				"machine, one spec at a time, once the worker takes it. The same specs run from a shell with "
				"tools/sim_benchmark/run_spec.py."))
			.Font(BenchUi::Font(10)).ColorAndOpacity(BenchUi::Muted).AutoWrapText(true)
		]
		+ SVerticalBox::Slot().FillHeight(1.f)
		[
			SNew(SScrollBox)
			+ SScrollBox::Slot()
			[
				SNew(SVerticalBox)
				// --- where to start from -----------------------------------------
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 6) [ BenchUi::Heading(LOCTEXT("Preset", "START FROM")) ]
				+ SVerticalBox::Slot().AutoHeight() [ SAssignNew(PresetBox, SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(4.f, 4.f)) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
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
					.Font(BenchUi::Font(10)).AutoWrapText(true)
				]
				// --- what to run: drawn from the manifest -------------------------
				+ SVerticalBox::Slot().AutoHeight() [ SAssignNew(FormBox, SVerticalBox) ]
				// --- where --------------------------------------------------------
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 16, 0, 6) [ BenchUi::Heading(LOCTEXT("Target", "TARGET")) ]
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 4, 0)
					[
						BenchUi::Choice(LOCTEXT("Local", "Local: this machine"), [this]() { return !bRunner; },
							[this]() { if (bRunner) { bRunner = false; FetchRig(); } })
					]
					+ SHorizontalBox::Slot().AutoWidth()
					[
						BenchUi::Choice(LOCTEXT("Runner", "Runner: the bench PC"), [this]() { return bRunner; },
							[this]() { bRunner = true; })
					]
				]
				// --- on what ------------------------------------------------------
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 16, 0, 6)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth() [ BenchUi::Heading(LOCTEXT("Parts", "COMPONENTS")) ]
					+ SHorizontalBox::Slot().FillWidth(1.f).Padding(10, 0, 0, 0)
					[
						SNew(STextBlock)
						.Text_Lambda([this]()
						{
							if (bRunner) return FText::GetEmpty();
							const FText Pick = CanPickPipeline()
								? LOCTEXT("PickHint", " Click a pipeline box to run the pipeline there.") : FText::GetEmpty();
							if (!bRigKnown || bRigStale)
							{
								return FText::Format(LOCTEXT("RigUnknown", "Not known: {0}.{1}"), FText::FromString(RigReason), Pick);
							}
							return FText::Format(LOCTEXT("RigKnown", "Checked on {0}. Outlined: what this run needs.{1}"),
								FText::FromString(RigId), Pick);
						})
						.Font(BenchUi::Font(10)).ColorAndOpacity(BenchUi::Muted).AutoWrapText(true)
					]
				]
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(6.f, 6.f))
					+ SWrapBox::Slot() [ MakeBox(TEXT("sim"), LOCTEXT("BoxSim", "Sim")) ]
					+ SWrapBox::Slot() [ MakeBox(TEXT("pipeline_bench_pc"), LOCTEXT("BoxPipeline", "Pipeline on this PC"), BenchUi::OnThisPc) ]
					+ SWrapBox::Slot() [ MakeBox(TEXT("latte_panda"), LOCTEXT("BoxPanda", "Pipeline on the LattePanda"), BenchUi::OnPanda) ]
					+ SWrapBox::Slot() [ MakeBox(TEXT("hil"), LOCTEXT("BoxHil", "ECU, AMS, uDV (HIL)")) ]
				]
				// --- go -----------------------------------------------------------
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 16, 0, 0)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth()
					[
						SNew(SButton)
						.IsFocusable(false)
						.ButtonColorAndOpacity(BenchUi::GoBtn)
						.ContentPadding(FMargin(16.f, 10.f))
						.IsEnabled_Lambda([this]() { return CanQueue(); })
						.OnClicked_Lambda([this]() { Launch(); return FReply::Handled(); })
						[
							SNew(STextBlock)
							.Text_Lambda([this]()
							{
								return bLaunching ? LOCTEXT("Queueing", "Queueing...")
									: bRunner ? LOCTEXT("QueueRunner", "QUEUE ON THE RUNNER") : LOCTEXT("QueueLocal", "QUEUE ON THIS MACHINE");
							})
							.Font(BenchUi::Font(13, TEXT("Bold"))).ColorAndOpacity(FLinearColor::White)
						]
					]
					+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center).Padding(12, 0, 0, 0)
					[
						SNew(STextBlock)
						.Text_Lambda([this]()
						{
							const FString Never = NeverHere();
							return FText::FromString(Never.IsEmpty() ? Note
								: FString::Printf(TEXT("This machine can't run it: %s."), *Never));
						})
						.ColorAndOpacity_Lambda([this]() { return FSlateColor(bNoteError || !NeverHere().IsEmpty() ? BenchUi::Bad : BenchUi::Accent); })
						.Font(BenchUi::Font(11, TEXT("Bold"))).AutoWrapText(true)
					]
				]
				// --- the queue and its worker -------------------------------------
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 20, 0, 6) [ BenchUi::Heading(LOCTEXT("Queue", "QUEUE ON THIS MACHINE")) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
					[
						SNew(STextBlock)
						.Text_Lambda([this]()
						{
							if (!bQueueKnown) return LOCTEXT("QueueUnknown", "Asking the backend...");
							if (bWorkerAlive && bWorkerStopping)
							{
								return LOCTEXT("WorkerStopping", "The worker stops once no spec is running (cancel one to stop sooner).");
							}
							if (bWorkerAlive)
							{
								return FText::Format(LOCTEXT("Worker", "The worker is running on {0} (pid {1}): it runs the queued specs one at a time."),
									FText::FromString(WorkerHost), FText::AsNumber(WorkerPid, &FNumberFormattingOptions::DefaultNoGrouping()));
							}
							if (FPlatformTime::Seconds() < WorkerStartingUntil) return LOCTEXT("WorkerStarting", "Starting the worker...");
#if PLATFORM_WINDOWS
							return LOCTEXT("NoWorkerWin", "No worker is running, so queued specs wait. Start one in a terminal (WSL), "
								"in the IFSSIM folder: python3 tools/sim_benchmark/run_spec.py --work");
#else
							return LOCTEXT("NoWorker", "No worker is running, so queued specs wait. Start worker runs one "
								"(python3 tools/sim_benchmark/run_spec.py --work in the IFSSIM folder).");
#endif
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
				+ SVerticalBox::Slot().AutoHeight() [ SAssignNew(QueueList, SVerticalBox) ]
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
	bEdited = false;
	bNameTouched = false;
	DraftName.Reset();
	CheckDueAt = FPlatformTime::Seconds();   // check the preset as it is
	bChecked = false;
	RebuildForm();
}

// ---------------------------------------------------------------------------
// The form

void SFSDSBenchPage::RebuildForm()
{
	if (!FormBox.IsValid()) return;
	FormBox->ClearChildren();
	FieldText.Reset();

	if (!bManifestKnown || !ManifestError.IsEmpty())
	{
		FormBox->AddSlot().AutoHeight().Padding(0, 12, 0, 0)
		[
			BenchUi::Small(ManifestError.IsEmpty() ? LOCTEXT("ManifestWait", "Asking the backend what a run can set...")
				: FText::Format(LOCTEXT("ManifestErr", "The preset runs as it is; changing it needs the backend's "
					"benchmark list: {0}"), FText::FromString(ManifestError)),
				ManifestError.IsEmpty() ? BenchUi::Muted : BenchUi::Bad)
		];
		return;
	}

	// Which benchmarks.
	TSharedRef<SWrapBox> Which = SNew(SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(4.f, 4.f));
	for (const FBench& B : Benches)
	{
		Which->AddSlot()
		[
			BenchUi::Choice(FText::FromString(B.Title), [this, N = B.Name]() { return HasBench(N); },
				[this, N = B.Name]() { SetBench(N, !HasBench(N)); }, FText::FromString(B.Description))
		];
	}
	FormBox->AddSlot().AutoHeight().Padding(0, 16, 0, 6) [ BenchUi::Heading(LOCTEXT("Benchmarks", "BENCHMARKS")) ];
	FormBox->AddSlot().AutoHeight() [ Which ];

	for (const FBench& B : Benches)
	{
		if (HasBench(B.Name)) FormBox->AddSlot().AutoHeight().Padding(0, 8, 0, 0) [ MakeBenchCard(B) ];
	}

	// Parameter overrides: files in specs/overrides.
	FormBox->AddSlot().AutoHeight().Padding(0, 16, 0, 6)
	[
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center) [ BenchUi::Heading(LOCTEXT("Overrides", "PARAMETER OVERRIDES")) ]
		+ SHorizontalBox::Slot().AutoWidth().Padding(10, 0, 0, 0)
		[
			BenchUi::Button(LOCTEXT("OpenOverrides", "Open the folder"), BenchUi::PlainBtn, []()
			{
				const FString Repo = RepoDir();
				if (Repo.IsEmpty()) return;
				const FString Dir = FPaths::Combine(Repo, TEXT("tools/sim_benchmark/specs/overrides"));
				IFileManager::Get().MakeDirectory(*Dir, true);
				FPlatformProcess::ExploreFolder(*Dir);
			}, TAttribute<bool>::CreateLambda([]() { return !RepoDir().IsEmpty(); }))
		]
	];
	if (Overrides.Num() == 0)
	{
		FormBox->AddSlot().AutoHeight()
		[
			BenchUi::Small(LOCTEXT("NoOverrides", "None yet: a YAML file in tools/sim_benchmark/specs/overrides with a pipeline: "
				"section (component, parameter, value) shows here, and Refresh picks it up."))
		];
	}
	else
	{
		TSharedRef<SWrapBox> Files = SNew(SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(4.f, 4.f));
		for (const FOverride& O : Overrides)
		{
			if (!O.Error.IsEmpty())
			{
				Files->AddSlot() [ BenchUi::Small(FText::FromString(O.Error), BenchUi::Bad) ];
				continue;
			}
			Files->AddSlot()
			[
				BenchUi::Choice(FText::FromString(O.Name), [this, N = O.Name]() { return ChosenOverrides.Contains(N); },
					[this, N = O.Name]()
					{
						if (ChosenOverrides.Remove(N) == 0) ChosenOverrides.Add(N);
						Edited();
					}, FText::FromString(O.About))
			];
		}
		FormBox->AddSlot().AutoHeight() [ Files ];
		FormBox->AddSlot().AutoHeight().Padding(0, 2, 0, 0)
		[
			BenchUi::Small(LOCTEXT("OverridesHint", "Merged over the preset's own, in the order you pick them: a later file wins."))
		];
	}

	// Name and the check's verdict.
	FormBox->AddSlot().AutoHeight().Padding(0, 16, 0, 0)
	[
		BenchUi::Row(TEXT("Name"), FString(),
			SNew(SBox).WidthOverride(360.f).HAlign(HAlign_Left)
			[
				SNew(SEditableTextBox)
				.Text_Lambda([this]() { return FText::FromString(SpecName()); })
				.OnTextChanged_Lambda([this](const FText& T) { DraftName = T.ToString(); bNameTouched = true; })
				.MinDesiredWidth(360.f)
				.Font(BenchUi::Font(10))
			])
	];
	FormBox->AddSlot().AutoHeight().Padding(0, 10, 0, 0)
	[
		SNew(STextBlock)
		.Text_Lambda([this]()
		{
			if (CheckDueAt > 0.0 || !bChecked) return LOCTEXT("Checking", "Checking...");
			if (bCheckOk)
			{
				FString S = FString::Printf(TEXT("%d job%s: %s"), CheckJobs, CheckJobs == 1 ? TEXT("") : TEXT("s"),
					*FString::Join(CheckLabels, TEXT(", ")));
				if (CheckJobs > CheckLabels.Num()) S += TEXT(", ...");
				return FText::FromString(S);
			}
			return FText::FromString(FString::Join(CheckErrors, TEXT("\n")));
		})
		.ColorAndOpacity_Lambda([this]() { return FSlateColor(CheckDueAt > 0.0 || !bChecked ? BenchUi::Muted : bCheckOk ? BenchUi::Good : BenchUi::Bad); })
		.Font(BenchUi::Font(10)).AutoWrapText(true)
	];
}

TSharedRef<SWidget> SFSDSBenchPage::MakeBenchCard(const FBench& B)
{
	TSharedRef<SVerticalBox> Rows = SNew(SVerticalBox);
	auto Add = [&Rows](TSharedRef<SWidget> W) { Rows->AddSlot().AutoHeight().Padding(0, 6, 0, 0) [ W ]; };
	const FString N = B.Name;

	// Bags: the ones there are, or all.
	if (!B.Bags.IsEmpty())
	{
		const TArray<FString>* Have = BagsByKind.Find(B.Bags);
		if (!Have || Have->Num() == 0)
		{
			Add(BenchUi::Row(TEXT("Bags"), FString(), BenchUi::Small(FText::Format(
				LOCTEXT("NoBags", "No {0} bags in the results yet: record one with tools/sim_benchmark/capture_benchmark_bag.py."),
				FText::FromString(B.Bags)), BenchUi::Bad)));
		}
		else
		{
			auto IsAll = [this, N]()
			{
				const TSharedPtr<FJsonObject> O = BenchObj(N);
				FString S;
				return O.IsValid() && O->TryGetStringField(TEXT("bags"), S) && S == TEXT("all");
			};
			auto Picked = [this, N]()
			{
				TArray<FString> Out;
				const TSharedPtr<FJsonObject> O = BenchObj(N);
				const TArray<TSharedPtr<FJsonValue>>* L = nullptr;
				if (O.IsValid() && O->TryGetArrayField(TEXT("bags"), L)) for (const auto& V : *L) Out.Add(V->AsString());
				return Out;
			};
			TSharedRef<SWrapBox> W = SNew(SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(4.f, 4.f));
			W->AddSlot()
			[
				BenchUi::Choice(LOCTEXT("AllBags", "All"), IsAll, [this, N]()
				{
					BenchObj(N, true)->SetStringField(TEXT("bags"), TEXT("all"));
					Edited();
				})
			];
			for (const FString& Bag : *Have)
			{
				W->AddSlot()
				[
					BenchUi::Choice(FText::FromString(Bag), [IsAll, Picked, Bag]() { return !IsAll() && Picked().Contains(Bag); },
						[this, IsAll, Picked, N, Bag]()
						{
							TArray<FString> L = IsAll() ? TArray<FString>() : Picked();
							if (L.Remove(Bag) == 0) L.Add(Bag);
							TArray<TSharedPtr<FJsonValue>> J;
							for (const FString& X : L) J.Add(MakeShared<FJsonValueString>(X));
							BenchObj(N, true)->SetArrayField(TEXT("bags"), J);
							Edited();
						})
				];
			}
			Add(BenchUi::Row(TEXT("Bags"), TEXT("Each picked bag is a job (times the repeats)."), W));
		}
	}

	// Parts: all lit means all of them (no "only").
	if (B.Parts.Num() > 0)
	{
		auto Only = [this, N]()
		{
			TArray<FString> Out;
			const TSharedPtr<FJsonObject> O = BenchObj(N);
			const TArray<TSharedPtr<FJsonValue>>* L = nullptr;
			FString One;
			if (O.IsValid() && O->TryGetArrayField(TEXT("only"), L)) for (const auto& V : *L) Out.Add(V->AsString());
			else if (O.IsValid() && O->TryGetStringField(TEXT("only"), One)) Out.Add(One);
			return Out;
		};
		TSharedRef<SWrapBox> W = SNew(SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(4.f, 4.f));
		for (const FString& Part : B.Parts)
		{
			W->AddSlot()
			[
				BenchUi::Choice(FText::FromString(Part), [Only, Part]() { const TArray<FString> L = Only(); return L.Num() == 0 || L.Contains(Part); },
					[this, Only, N, Part, All = B.Parts]()
					{
						TArray<FString> L = Only();
						if (L.Num() == 0) L = All;
						if (L.Remove(Part) == 0) L.Add(Part);
						const TSharedPtr<FJsonObject> O = BenchObj(N, true);
						if (L.Num() == 0 || L.Num() == All.Num()) O->RemoveField(TEXT("only"));
						else
						{
							TArray<TSharedPtr<FJsonValue>> J;
							for (const FString& X : L) J.Add(MakeShared<FJsonValueString>(X));
							O->SetArrayField(TEXT("only"), J);
						}
						Edited();
					})
			];
		}
		Add(BenchUi::Row(TEXT("Parts"), TEXT("What this benchmark measures; all lit runs them all."), W));
	}

	// Repeats.
	// The spec's repeats: the whole benchmark again, each time a job of its own. (The
	// mission benchmark's own "repeats" setting, runs per track, is its settings row.)
	Add(BenchUi::Row(TEXT("Times"), TEXT("Run the whole benchmark this many times, each a separate job."),
		SNew(SBox).WidthOverride(90.f).HAlign(HAlign_Left)
		[
			SNew(SSpinBox<int32>)
			.MinValue(1).MaxValue(100).MinSliderValue(1).MaxSliderValue(20)
			.MinDesiredWidth(90.f)
			.Value_Lambda([this, N]()
			{
				const TSharedPtr<FJsonObject> O = BenchObj(N);
				double R = 1.0;
				if (O.IsValid()) O->TryGetNumberField(TEXT("repeats"), R);
				return FMath::Max(1, (int32)R);
			})
			.OnValueChanged_Lambda([this, N](int32 V)
			{
				const TSharedPtr<FJsonObject> O = BenchObj(N, true);
				double Was = 1.0;
				O->TryGetNumberField(TEXT("repeats"), Was);
				if ((int32)Was == V) return;
				if (V <= 1) O->RemoveField(TEXT("repeats")); else O->SetNumberField(TEXT("repeats"), V);
				Edited();
			})
			.Font(BenchUi::Font(10))
		]));

	for (const FSetting& S : B.Settings)
	{
		// The LattePanda's commands matter only when the pipeline runs there.
		if (S.Name.StartsWith(TEXT("panda_")))
		{
			Add(SNew(SBox)
				.Visibility_Lambda([this]() { return PipelineOn() == BenchUi::OnPanda ? EVisibility::Visible : EVisibility::Collapsed; })
				[ MakeSettingRow(B, S) ]);
			continue;
		}
		Add(MakeSettingRow(B, S));
	}

	return SNew(SBorder)
		.BorderImage(BenchUi::White())
		.BorderBackgroundColor(BenchUi::Card)
		.Padding(FMargin(12.f, 8.f, 12.f, 10.f))
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight()
			[
				SNew(STextBlock).Text(FText::FromString(B.Title)).Font(BenchUi::Font(12, TEXT("Bold"))).ColorAndOpacity(BenchUi::Ink)
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 2) [ BenchUi::Small(FText::FromString(B.Description)) ]
			+ SVerticalBox::Slot().AutoHeight() [ Rows ]
		];
}

TSharedRef<SWidget> SFSDSBenchPage::MakeSettingRow(const FBench& B, const FSetting& S)
{
	const FString Label = BenchUi::Human(S.Name);
	const FString N = B.Name, Key = S.Name;

	// A list of known values, stored as "a,b": missions, tracks.
	const TArray<FString>* Known = S.Name == TEXT("missions") ? &Missions : S.Name == TEXT("tracks") ? &TrackNames : nullptr;
	if (Known && Known->Num() > 0 && S.Type == TEXT("string"))
	{
		const bool bAllowAll = S.Name == TEXT("missions");
		TSharedRef<SWrapBox> W = SNew(SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(4.f, 4.f));
		W->AddSlot()
		[
			BenchUi::Choice(bAllowAll ? LOCTEXT("AllMissions", "All") : LOCTEXT("OwnTracks", "Each mission's own"),
				[this, N, Key, bAllowAll]()
				{
					const TArray<FString> L = CsvSetting(N, Key);
					return L.Num() == 0 || (bAllowAll && L.Contains(TEXT("all")));
				},
				[this, N, Key]() { SetCsvSetting(N, Key, {}); })
		];
		for (const FString& V : *Known)
		{
			const FString Shown = V.EndsWith(TEXT(".csv")) ? V.LeftChop(4) : V;
			W->AddSlot()
			[
				BenchUi::Choice(FText::FromString(Shown), [this, N, Key, V]() { return CsvSetting(N, Key).Contains(V); },
					[this, N, Key, V]()
					{
						TArray<FString> L = CsvSetting(N, Key);
						L.Remove(TEXT("all"));
						if (L.Remove(V) == 0) L.Add(V);
						SetCsvSetting(N, Key, L);
					})
			];
		}
		return BenchUi::Row(Label, S.Help, W);
	}

	// Where the pipeline runs: also the component boxes.
	if (S.Name == BenchUi::PipelineOnSetting)
	{
		return BenchUi::Row(Label, S.Help,
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 4, 0)
			[
				BenchUi::Choice(LOCTEXT("OnPc", "This PC"), [this]() { return PipelineOn() == BenchUi::OnThisPc; },
					[this]() { SetPipelineOn(BenchUi::OnThisPc); })
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				BenchUi::Choice(LOCTEXT("OnPanda", "LattePanda"), [this]() { return PipelineOn() == BenchUi::OnPanda; },
					[this]() { SetPipelineOn(BenchUi::OnPanda); })
			]);
	}

	// On, off, or the benchmark's default.
	if (S.Type == TEXT("boolean"))
	{
		auto Is = [this, N, Key](int32 Want)   // 0 default, 1 on, 2 off
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
		const FText Names[] = { LOCTEXT("Default", "Default"), LOCTEXT("On", "On"), LOCTEXT("Off", "Off") };
		for (int32 I = 0; I < 3; ++I)
		{
			H->AddSlot().AutoWidth().Padding(0, 0, 4, 0) [ BenchUi::Choice(Names[I], [Is, I]() { return Is(I); }, [Set, I]() { Set(I); }) ];
		}
		return BenchUi::Row(Label, S.Help, H);
	}

	// One of the manifest's choices.
	if (S.Choices.Num() > 0)
	{
		TSharedRef<SWrapBox> W = SNew(SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(4.f, 4.f));
		W->AddSlot()
		[
			BenchUi::Choice(LOCTEXT("Default", "Default"), [this, N, Key]() { return BenchUi::FieldValue(SettingsObj(N), Key).IsEmpty(); },
				[this, N, Key]() { SettingsObj(N, true)->RemoveField(Key); Edited(); })
		];
		for (const FString& C : S.Choices)
		{
			W->AddSlot()
			[
				BenchUi::Choice(FText::FromString(C), [this, N, Key, C]() { return BenchUi::FieldValue(SettingsObj(N), Key) == C; },
					[this, N, Key, C, Type = S.Type]() { BenchUi::SetTyped(SettingsObj(N, true), Key, C, Type); Edited(); })
			];
		}
		return BenchUi::Row(Label, S.Help, W);
	}

	// Typed: a number or text; empty is the default.
	const FString FieldKey = N + TEXT(".") + Key;
	FieldText.Add(FieldKey, BenchUi::FieldValue(SettingsObj(N), Key));
	return BenchUi::Row(Label, S.Help,
		SNew(SBox).WidthOverride(S.Type == TEXT("string") ? 360.f : 120.f).HAlign(HAlign_Left)
		[
			SNew(SEditableTextBox)
			.HintText(LOCTEXT("DefaultHint", "default"))
			.MinDesiredWidth(S.Type == TEXT("string") ? 360.f : 120.f)
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
// The component boxes

TSharedRef<SWidget> SFSDSBenchPage::MakeBox(const FString& Part, const FText& Label, const FString& PipelineChoice)
{
	auto Color = [this, Part]()
	{
		switch (BoxState(Part))
		{
		case EBox::Ready: return BenchUi::Good;
		case EBox::NotNow: return BenchUi::Yellow;
		default: return BenchUi::Grey;
		}
	};
	auto Needed = [this, Part]() { return Needs().Contains(Part); };
	// A pipeline box picks where the pipeline runs; a box this machine never has can't be picked.
	auto Pickable = [this, Part, PipelineChoice]()
	{
		return !PipelineChoice.IsEmpty() && !bRunner && CanPickPipeline() && BoxState(Part) != EBox::Never;
	};
	// The outline marks what the run needs; the fill, the state.
	return SNew(SBorder)
		.BorderImage(BenchUi::White())
		.BorderBackgroundColor_Lambda([Color, Needed]() { return FSlateColor(Needed() ? Color() : FLinearColor(1, 1, 1, 0.05f)); })
		.Padding(2.f)
		.Cursor_Lambda([Pickable]() { return TOptional<EMouseCursor::Type>(Pickable() ? EMouseCursor::Hand : EMouseCursor::Default); })
		.OnMouseButtonDown_Lambda([this, Pickable, PipelineChoice](const FGeometry&, const FPointerEvent&)
		{
			if (!Pickable()) return FReply::Unhandled();
			SetPipelineOn(PipelineChoice);
			return FReply::Handled();
		})
		[
			SNew(SBox).WidthOverride(200.f).MinDesiredHeight(76.f)
			[
				SNew(SBorder)
				.BorderImage(BenchUi::White())
				.BorderBackgroundColor_Lambda([Color]() { return FSlateColor(Color() * FLinearColor(1, 1, 1, 0.16f)); })
				.Padding(FMargin(10.f, 8.f))
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[
						SNew(STextBlock).Text(Label).Font(BenchUi::Font(11, TEXT("Bold"))).ColorAndOpacity(BenchUi::Ink)
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
					[
						SNew(STextBlock)
						.Text_Lambda([this, Part]()
						{
							switch (BoxState(Part))
							{
							case EBox::Ready: return LOCTEXT("Ready", "Ready");
							case EBox::NotNow: return LOCTEXT("NotNow", "Not now");
							case EBox::Never: return LOCTEXT("Never", "Not on this machine");
							default: return LOCTEXT("Unknown", "Not known");
							}
						})
						.ColorAndOpacity_Lambda([Color]() { return FSlateColor(Color()); })
						.Font(BenchUi::Font(10, TEXT("Bold")))
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
					[
						SNew(STextBlock)
						.Text_Lambda([this, Part]()
						{
							FString Why;
							BoxState(Part, &Why);
							return FText::FromString(Why);
						})
						.Font(BenchUi::Font(9)).ColorAndOpacity(BenchUi::Muted).AutoWrapText(true)
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[
						SNew(STextBlock)
						.Text_Lambda([this, PipelineChoice, Pickable]()
						{
							if (PipelineChoice.IsEmpty() || !CanPickPipeline()) return FText::GetEmpty();
							if (PipelineOn() == PipelineChoice) return LOCTEXT("PipelineHere", "The pipeline runs here");
							return Pickable() ? LOCTEXT("PipelinePick", "Click to run the pipeline here") : FText::GetEmpty();
						})
						.ColorAndOpacity_Lambda([this, PipelineChoice]() { return FSlateColor(PipelineOn() == PipelineChoice ? BenchUi::Accent : BenchUi::Muted); })
						.Font(BenchUi::Font(9, TEXT("Bold")))
					]
				]
			]
		];
}

SFSDSBenchPage::EBox SFSDSBenchPage::BoxState(const FString& Part, FString* Why) const
{
	auto Say = [Why](EBox State, const FString& Text) { if (Why) *Why = Text; return State; };
	if (bRunner)
	{
		return Say(EBox::Unknown, TEXT("The bench PC doesn't publish its checks yet."));
	}
	if (!bRigKnown || bRigStale)
	{
		// Old or missing checks can't be trusted (bench_queue.rig).
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
		NextQueue = NextRig = 0.0;
	}
	bWasActive = bActive;
	if (!bActive) return EActiveTimerReturnType::Continue;
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
		const TSharedPtr<FJsonObject> O = R.Object();
		const TArray<TSharedPtr<FJsonValue>>* List = nullptr;
		if (!R.Ok() || !O.IsValid() || !O->TryGetArrayField(TEXT("presets"), List))
		{
			Self->PresetsNote = Self->Unreachable(TEXT("Couldn't read the presets"), R);
			return;
		}
		Self->PresetsNote.Reset();
		Self->Presets.Reset();
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
			Self->Presets.Add(MoveTemp(P));
		}
		Self->Overrides.Reset();
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
				Self->Overrides.Add(MoveTemp(Ov));
			}
		}
		SFSDSBenchPage* Page = Self.Get();
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
			Self->ManifestError = R.Code == 404 ? TEXT("this backend is older than the page: START again to update it")
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
		CheckErrors = { TEXT("Choose at least one benchmark.") };
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
		Self->CheckErrors.Reset();
		Self->CheckLabels.Reset();
		Self->bCheckOk = false;
		if (!R.Ok() || !O.IsValid())
		{
			Self->CheckErrors.Add(Self->Unreachable(TEXT("Couldn't check it"), R));
			return;
		}
		O->TryGetBoolField(TEXT("ok"), Self->bCheckOk);
		O->TryGetStringArrayField(TEXT("errors"), Self->CheckErrors);
		O->TryGetStringArrayField(TEXT("labels"), Self->CheckLabels);
		O->TryGetStringArrayField(TEXT("needs"), Self->CheckNeeds);
		O->TryGetNumberField(TEXT("jobs"), Self->CheckJobs);
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
			BenchUi::Choice(FText::FromString(P.Title), [this, Name = P.Name]() { return PresetName == Name; },
				[this, Name = P.Name]() { PickPreset(Name); })
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
			if (!bWasAlive) Self->FetchRig();   // its first checks: the boxes
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
			: FString::Printf(TEXT("Queued %s. It waits for the worker: Start worker below."), *Name), false);
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
		Meta.Add(J.Id);
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
			.Padding(FMargin(12.f, 8.f))
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 12, 0)
				[
					SNew(SBox).WidthOverride(76.f)
					[
						SNew(STextBlock)
						.Text(FText::FromString(J.bCancelRequested && J.State == TEXT("running") ? TEXT("cancelling") : J.State))
						.Font(BenchUi::Font(11, TEXT("Bold"))).ColorAndOpacity(BenchUi::StateColor(J.State))
					]
				]
				+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[
						SNew(STextBlock).Text(FText::FromString(J.Name.IsEmpty() ? J.Id : J.Name))
						.Font(BenchUi::Font(12, TEXT("Bold"))).ColorAndOpacity(BenchUi::Ink)
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
						.Font(BenchUi::Font(10)).ColorAndOpacity(BenchUi::Ink).AutoWrapText(true)
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
					[
						SNew(STextBlock).Text(FText::FromString(J.Error))
						.Visibility(J.Error.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible)
						.Font(BenchUi::Font(10)).ColorAndOpacity(BenchUi::Bad).AutoWrapText(true)
					]
				]
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(6, 0, 0, 0)
				[
					SNew(SBox).Visibility(Dir.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible)
					[
						// worker.log and each job's folder
						BenchUi::Button(LOCTEXT("Folder", "Folder"), BenchUi::PlainBtn, [Dir]() { FPlatformProcess::ExploreFolder(*Dir); })
					]
				]
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(6, 0, 0, 0)
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
		];
	}
}

#undef LOCTEXT_NAMESPACE
