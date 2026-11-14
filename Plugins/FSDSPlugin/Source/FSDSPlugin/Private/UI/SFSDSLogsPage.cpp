#include "UI/SFSDSLogsPage.h"

#include "UI/FSDSStackStarter.h"

#include "HAL/PlatformProcess.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SMultiLineEditableTextBox.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "FSDSLogsPage"

// Always qualified: the UI files share unity blobs.
namespace LogsUi
{
	// The menu's palette.
	const FLinearColor Accent = FLinearColor::FromSRGBColor(FColor(255, 184, 28));
	const FLinearColor Ink = FLinearColor::FromSRGBColor(FColor(232, 232, 236));
	const FLinearColor Muted = FLinearColor::FromSRGBColor(FColor(150, 150, 160));
	const FLinearColor Good = FLinearColor::FromSRGBColor(FColor(74, 222, 128));
	const FLinearColor Bad = FLinearColor::FromSRGBColor(FColor(248, 113, 113));
	const FLinearColor PlainBtn = FLinearColor::FromSRGBColor(FColor(64, 64, 72));

	// Enough to see a run's start and its end; more makes the box slow.
	constexpr int32 TailLines = 1500;
	constexpr double FollowS = 3.0;

	FSlateFontInfo Font(int32 Size, const TCHAR* Weight = TEXT("Regular"))
	{
		return FCoreStyle::GetDefaultFontStyle(Weight, Size);
	}

	TSharedRef<SWidget> Choice(TAttribute<FText> Label, TFunction<bool()> IsOn, TFunction<void()> OnPick)
	{
		return SNew(SButton)
			.IsFocusable(false)
			.ContentPadding(FMargin(10.f, 5.f))
			.HAlign(HAlign_Center)
			.ButtonColorAndOpacity_Lambda([IsOn]() { return FSlateColor(IsOn() ? PlainBtn : FLinearColor(1, 1, 1, 0.06f)); })
			.OnClicked_Lambda([OnPick]() { OnPick(); return FReply::Handled(); })
			[
				SNew(STextBlock).Text(Label).Font(Font(11, TEXT("Bold")))
				.ColorAndOpacity_Lambda([IsOn]() { return FSlateColor(IsOn() ? Accent : Muted); })
			];
	}
}

SFSDSLogsPage::~SFSDSLogsPage() = default;

void SFSDSLogsPage::Construct(const FArguments& InArgs)
{
	IsActive = InArgs._IsActive;
	Runner = MakeShared<FFSDSStackStarter>();

	ChildSlot
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(STextBlock).Text(LOCTEXT("Title", "Logs"))
			.Font(LogsUi::Font(20, TEXT("Bold"))).ColorAndOpacity(LogsUi::Ink)
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 10)
		[
			SNew(STextBlock)
			.Text(LOCTEXT("Hint", "What the stack prints: the pipeline's nodes, or Mission Control's backend. "
				"The last lines, newest at the bottom; select text to copy it."))
			.Font(LogsUi::Font(10)).ColorAndOpacity(LogsUi::Muted).AutoWrapText(true)
		]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0)
			[
				LogsUi::Choice(LOCTEXT("Pipeline", "Pipeline"), [this]() { return Source == ESource::Pipeline; },
					[this]() { if (Source != ESource::Pipeline) { Source = ESource::Pipeline; Lines.Reset(); Fetch(); } })
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 14, 0)
			[
				LogsUi::Choice(LOCTEXT("Backend", "Backend"), [this]() { return Source == ESource::Backend; },
					[this]() { if (Source != ESource::Backend) { Source = ESource::Backend; Lines.Reset(); Fetch(); } })
			]
			+ SHorizontalBox::Slot().FillWidth(1.f).Padding(0, 0, 14, 0).VAlign(VAlign_Center)
			[
				SNew(SEditableTextBox)
				.Font(LogsUi::Font(11))
				.HintText(LOCTEXT("FilterHint", "filter, e.g. cone_slam or EBS"))
				.OnTextChanged_Lambda([this](const FText& T) { Filter = T.ToString().TrimStartAndEnd(); ApplyFilter(); })
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0)
			[
				LogsUi::Choice(LOCTEXT("Problems", "Problems only"), [this]() { return bProblemsOnly; },
					[this]() { bProblemsOnly = !bProblemsOnly; ApplyFilter(); })
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0)
			[
				LogsUi::Choice(LOCTEXT("Follow", "Live"), [this]() { return bFollow; }, [this]() { bFollow = !bFollow; })
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0)
			[
				LogsUi::Choice(LOCTEXT("Refresh", "Refresh"), []() { return false; }, [this]() { Fetch(); })
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				LogsUi::Choice(LOCTEXT("Save", "Save to a file"), []() { return false; }, [this]() { Save(); })
			]
		]
		+ SVerticalBox::Slot().FillHeight(1.f).Padding(0, 8, 0, 0)
		[
			SAssignNew(TextBox, SMultiLineEditableTextBox)
			.IsReadOnly(true)
			.AlwaysShowScrollbars(true)
			.Font(FCoreStyle::GetDefaultFontStyle("Mono", 9))
			.BackgroundColor(FLinearColor(0.02f, 0.02f, 0.025f, 1.f))
			.ForegroundColor(LogsUi::Ink)
			.AutoWrapText(false)
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			SNew(STextBlock)
			.Text_Lambda([this]() { return FText::FromString(Status); })
			.ColorAndOpacity_Lambda([this]() { return FSlateColor(bStatusError ? LogsUi::Bad : LogsUi::Muted); })
			.Font(LogsUi::Font(10)).AutoWrapText(true)
		]
	];

	RegisterActiveTimer(0.25f, FWidgetActiveTimerDelegate::CreateSP(this, &SFSDSLogsPage::Poll));
}

const TCHAR* SFSDSLogsPage::ServiceOf(ESource S)
{
	return S == ESource::Pipeline ? TEXT("dv_pipeline_stack") : TEXT("mission_control_backend");
}

void SFSDSLogsPage::Fetch()
{
	if (Runner->IsRunning()) return;
	LastFetch = FPlatformTime::Seconds();
	const FString Dir = FFSDSStackStarter::FindComposeDir();
	if (Dir.IsEmpty())
	{
		Status = TEXT("The IFSSIM repo isn't set: Settings, Pipeline code, or START once to set it up.");
		bStatusError = true;
		return;
	}
	FString Error;
	const FString Cmd = FString::Printf(TEXT("docker compose logs --no-color --no-log-prefix --tail %d %s"),
		LogsUi::TailLines, ServiceOf(Source));
	if (!Runner->Run(Dir, Cmd, Error))
	{
		Status = FString::Printf(TEXT("Couldn't ask docker: %s"), *Error);
		bStatusError = true;
	}
}

EActiveTimerReturnType SFSDSLogsPage::Poll(double Now, float Dt)
{
	const bool bActive = IsActive.Get(true);
	if (Runner->IsRunning())
	{
		if (!Runner->Poll())
		{
			const FString Out = Runner->Output();
			if (FFSDSStackStarter::SaysDockerIsDown(Out))
			{
				Status = TEXT("Docker isn't running: START brings it and the stack up.");
				bStatusError = true;
			}
			else if (Runner->ExitCode() != 0)
			{
				Status = FString::Printf(TEXT("docker compose logs failed: %s"), *Runner->LastLine());
				bStatusError = true;
			}
			else
			{
				Out.ParseIntoArrayLines(Lines, false);
				Status = FString::Printf(TEXT("%s: last %d lines, %s"), ServiceOf(Source), Lines.Num(),
					*FDateTime::Now().ToString(TEXT("%H:%M:%S")));
				if (Lines.Num() == 0) Status += TEXT(". Nothing yet: is the stack up?");
				bStatusError = false;
				ApplyFilter();
			}
		}
	}
	else if (bActive && (!bWasActive || (bFollow && FPlatformTime::Seconds() - LastFetch > LogsUi::FollowS)))
	{
		Fetch();   // on opening the page, and every few seconds while live
	}
	bWasActive = bActive;
	return EActiveTimerReturnType::Continue;
}

bool SFSDSLogsPage::IsProblem(const FString& Line)
{
	static const TCHAR* Words[] = { TEXT("ERROR"), TEXT("Error"), TEXT("WARN"), TEXT("Traceback"),
		TEXT("Exception"), TEXT("FATAL"), TEXT("has died"), TEXT("EBS") };
	for (const TCHAR* W : Words)
	{
		if (Line.Contains(W, ESearchCase::CaseSensitive)) return true;
	}
	return false;
}

void SFSDSLogsPage::ApplyFilter()
{
	FString Text;
	Text.Reserve(Lines.Num() * 80);
	for (const FString& L : Lines)
	{
		if (bProblemsOnly && !IsProblem(L)) continue;
		if (!Filter.IsEmpty() && !L.Contains(Filter, ESearchCase::IgnoreCase)) continue;
		Text += L;
		Text += LINE_TERMINATOR;
	}
	if (Text == Shown || !TextBox.IsValid()) return;
	Shown = Text;
	TextBox->SetText(FText::FromString(Text));
	TextBox->GoTo(ETextLocation::EndOfDocument);   // newest at the bottom, in view
}

void SFSDSLogsPage::Save()
{
	if (Lines.Num() == 0)
	{
		Status = TEXT("Nothing to save yet.");
		bStatusError = true;
		return;
	}
	FString File = FPaths::ConvertRelativePathToFull(FPaths::Combine(FPlatformProcess::UserDir(), TEXT("IFSSIM"), TEXT("Logs"),
		FString::Printf(TEXT("%s_%s.txt"), ServiceOf(Source), *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")))));
	// The whole tail, unfiltered: what someone else will want to read.
	const bool bOk = FFileHelper::SaveStringArrayToFile(Lines, *File, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
	FPaths::MakePlatformFilename(File);
	Status = bOk ? FString::Printf(TEXT("Saved %d lines to %s"), Lines.Num(), *File) : FString::Printf(TEXT("Couldn't write %s"), *File);
	bStatusError = !bOk;
}

#undef LOCTEXT_NAMESPACE
