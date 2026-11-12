#include "UI/SFSDSBenchPage.h"

#include "UI/FSDSMissionControlClient.h"
#include "UI/FSDSStackStarter.h"

#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/Paths.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
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
	constexpr double RigEveryS = 15.0;   // the worker checks the rig once a minute

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
	TSharedRef<SWidget> Choice(TAttribute<FText> Label, TFunction<bool()> IsOn, TFunction<void()> OnPick)
	{
		return SNew(SButton)
			.IsFocusable(false)
			.ContentPadding(FMargin(10.f, 7.f))
			.HAlign(HAlign_Center)
			.ButtonColorAndOpacity_Lambda([IsOn]() { return FSlateColor(IsOn() ? PlainBtn : FLinearColor(1.f, 1.f, 1.f, 0.06f)); })
			.OnClicked_Lambda([OnPick]() { OnPick(); return FReply::Handled(); })
			[
				SNew(STextBlock).Text(Label).Font(Font(11, TEXT("Bold")))
				.ColorAndOpacity_Lambda([IsOn]() { return FSlateColor(IsOn() ? Accent : Muted); })
			];
	}

	TSharedRef<SWidget> Heading(const FText& Title)
	{
		return SNew(STextBlock).Text(Title).Font(Font(10, TEXT("Bold"))).ColorAndOpacity(Accent);
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
}

void SFSDSBenchPage::Construct(const FArguments& InArgs)
{
	IsActive = InArgs._IsActive;
	Client = InArgs._Client;

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
					FetchRig();
					FetchQueue();
				})
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 6)
		[
			SNew(STextBlock)
			.Text(LOCTEXT("Hint", "Queue a benchmark spec. Local runs it on this machine, one spec at a time, once a worker "
				"takes it: python3 tools/sim_benchmark/run_spec.py --work in the IFSSIM folder. The same specs run from a "
				"shell with run_spec.py <preset>."))
			.Font(BenchUi::Font(10)).ColorAndOpacity(BenchUi::Muted).AutoWrapText(true)
		]
		+ SVerticalBox::Slot().FillHeight(1.f)
		[
			SNew(SScrollBox)
			+ SScrollBox::Slot()
			[
				SNew(SVerticalBox)
				// --- what to run ---------------------------------------------
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 6) [ BenchUi::Heading(LOCTEXT("Preset", "PRESET")) ]
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
				// --- where -----------------------------------------------------
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
				// --- on what ---------------------------------------------------
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
							if (!bRigKnown || bRigStale) return FText::Format(LOCTEXT("RigUnknown", "Not known: {0}."), FText::FromString(RigReason));
							return FText::Format(LOCTEXT("RigKnown", "Checked on {0}. Outlined: what the preset needs."), FText::FromString(RigId));
						})
						.Font(BenchUi::Font(10)).ColorAndOpacity(BenchUi::Muted)
					]
				]
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(6.f, 6.f))
					+ SWrapBox::Slot() [ MakeBox(TEXT("sim"), LOCTEXT("BoxSim", "Sim")) ]
					+ SWrapBox::Slot() [ MakeBox(TEXT("docker"), LOCTEXT("BoxDocker", "Docker")) ]
					+ SWrapBox::Slot() [ MakeBox(TEXT("pipeline_bench_pc"), LOCTEXT("BoxPipeline", "Pipeline on this PC")) ]
					+ SWrapBox::Slot() [ MakeBox(TEXT("latte_panda"), LOCTEXT("BoxPanda", "Pipeline on the LattePanda")) ]
					+ SWrapBox::Slot() [ MakeBox(TEXT("hil"), LOCTEXT("BoxHil", "ECU, AMS, uDV (HIL)")) ]
				]
				// --- go --------------------------------------------------------
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 16, 0, 0)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth()
					[
						SNew(SButton)
						.IsFocusable(false)
						.ButtonColorAndOpacity(BenchUi::GoBtn)
						.ContentPadding(FMargin(16.f, 10.f))
						.IsEnabled_Lambda([this]()
						{
							const FPreset* P = Chosen();
							return Client.IsValid() && !bLaunching && P && P->Spec.IsValid() && NeverHere().IsEmpty();
						})
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
				// --- the queue -------------------------------------------------
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 20, 0, 6) [ BenchUi::Heading(LOCTEXT("Queue", "QUEUE ON THIS MACHINE")) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
				[
					SNew(STextBlock)
					.Text_Lambda([this]()
					{
						if (!bQueueKnown) return LOCTEXT("QueueUnknown", "Asking the backend...");
						if (bWorkerAlive)
						{
							return FText::Format(LOCTEXT("Worker", "A worker is running on {0} (pid {1}): it runs the queued specs one at a time."),
								FText::FromString(WorkerHost), FText::AsNumber(WorkerPid, &FNumberFormattingOptions::DefaultNoGrouping()));
						}
						return LOCTEXT("NoWorker", "No worker is running, so queued specs wait. Start one in a terminal, in the IFSSIM "
							"folder: python3 tools/sim_benchmark/run_spec.py --work");
					})
					.ColorAndOpacity_Lambda([this]() { return FSlateColor(!bQueueKnown ? BenchUi::Muted : bWorkerAlive ? BenchUi::Good : BenchUi::Accent); })
					.Font(BenchUi::Font(10, TEXT("Bold"))).AutoWrapText(true)
				]
				+ SVerticalBox::Slot().AutoHeight() [ SAssignNew(QueueList, SVerticalBox) ]
			]
		]
	];

	RegisterActiveTimer(0.5f, FWidgetActiveTimerDelegate::CreateSP(this, &SFSDSBenchPage::Poll));
}

// ---------------------------------------------------------------------------
// The component boxes

TSharedRef<SWidget> SFSDSBenchPage::MakeBox(const FString& Part, const FText& Label)
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
	auto Needed = [this, Part]()
	{
		const FPreset* P = Chosen();
		return P && P->Needs.Contains(Part);
	};
	// The outline marks what the chosen preset needs; the fill, the state.
	return SNew(SBorder)
		.BorderImage(BenchUi::White())
		.BorderBackgroundColor_Lambda([Color, Needed]() { return FSlateColor(Needed() ? Color() : FLinearColor(1, 1, 1, 0.05f)); })
		.Padding(2.f)
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

FString SFSDSBenchPage::NeverHere() const
{
	const FPreset* P = Chosen();
	if (!P || bRunner) return FString();
	TArray<FString> Out;
	for (const FString& Part : P->Needs)
	{
		FString Why;
		if (BoxState(Part, &Why) == EBox::Never) Out.Add(Why.IsEmpty() ? Part : Why);
	}
	return FString::Join(Out, TEXT("; "));
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
		NextQueue = NextRig = 0.0;
	}
	bWasActive = bActive;
	if (!bActive) return EActiveTimerReturnType::Continue;
	if (T >= NextQueue) FetchQueue();
	if (T >= NextRig && !bRunner) FetchRig();
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
		if (!Self->Chosen() && Self->Presets.Num() > 0) Self->PresetName = Self->Presets[0].Name;
		if (Self->Presets.Num() == 0) Self->PresetsNote = TEXT("No presets in tools/sim_benchmark/specs.");
		Self->RebuildPresets();
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
				[this, Name = P.Name]() { PresetName = Name; })
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
		Self->bWorkerAlive = false;
		if (O->TryGetObjectField(TEXT("worker"), W))
		{
			(*W)->TryGetBoolField(TEXT("alive"), Self->bWorkerAlive);
			(*W)->TryGetStringField(TEXT("host"), Self->WorkerHost);
			(*W)->TryGetNumberField(TEXT("pid"), Self->WorkerPid);
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
	const FPreset* P = Chosen();
	if (!Client.IsValid() || !P || !P->Spec.IsValid() || bLaunching) return;
	bLaunching = true;
	SetNote(FString(), false);
	const TSharedRef<FJsonObject> Body = MakeShared<FJsonObject>();
	Body->SetObjectField(TEXT("spec"), P->Spec);
	Body->SetStringField(TEXT("target"), bRunner ? TEXT("runner") : TEXT("local"));
	Body->SetStringField(TEXT("name"), P->Title);
	Body->SetStringField(TEXT("requested_by"), FString::Printf(TEXT("%s@%s"), FPlatformProcess::UserName(), FPlatformProcess::ComputerName()));
	TWeakPtr<SFSDSBenchPage> Weak = SharedThis(this);
	Client->Post(TEXT("/api/bench/launch"), Body, [Weak, Title = P->Title](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSBenchPage> Self = Weak.Pin();
		if (!Self.IsValid()) return;
		Self->bLaunching = false;
		const TSharedPtr<FJsonObject> O = R.Object();
		if (!R.Ok() || !O.IsValid())
		{
			Self->SetNote(Self->Unreachable(FString::Printf(TEXT("Couldn't queue %s"), *Title), R), true);
			return;
		}
		FString Hint;
		O->TryGetStringField(TEXT("hint"), Hint);
		Self->SetNote(Hint.IsEmpty() ? FString::Printf(TEXT("Queued %s."), *Title)
			: FString::Printf(TEXT("Queued %s, but %s"), *Title, *Hint.Replace(TEXT("`"), TEXT(""))), false);
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
// The queue

FString SFSDSBenchPage::JobDir(const FString& Id)
{
	const FString Repo = FFSDSStackStarter::FindComposeDir();
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
