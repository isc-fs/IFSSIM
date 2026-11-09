#include "UI/SFSDSBagsPage.h"

#include "UI/FSDSStackStarter.h"

#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/Paths.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "FSDSBagsPage"

// Always qualified: the UI files share unity blobs.
namespace BagsUi
{
	const FLinearColor Accent = FLinearColor::FromSRGBColor(FColor(255, 184, 28));
	const FLinearColor Ink = FLinearColor::FromSRGBColor(FColor(232, 232, 236));
	const FLinearColor Muted = FLinearColor::FromSRGBColor(FColor(150, 150, 160));
	const FLinearColor Bad = FLinearColor::FromSRGBColor(FColor(248, 113, 113));
	const FLinearColor PlainBtn = FLinearColor::FromSRGBColor(FColor(64, 64, 72));
	const FLinearColor GoBtn = FLinearColor::FromSRGBColor(FColor(34, 197, 94));
	const FLinearColor Card(1.f, 1.f, 1.f, 0.04f);

	FSlateFontInfo Font(int32 Size, const TCHAR* Weight = TEXT("Regular"))
	{
		return FCoreStyle::GetDefaultFontStyle(Weight, Size);
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

	int64 DirBytes(const FString& Dir)
	{
		int64 Total = 0;
		IFileManager::Get().IterateDirectoryStatRecursively(*Dir, [&Total](const TCHAR*, const FFileStatData& S)
		{
			if (!S.bIsDirectory && S.FileSize > 0) Total += S.FileSize;
			return true;
		});
		return Total;
	}
}

void SFSDSBagsPage::Construct(const FArguments& InArgs)
{
	IsActive = InArgs._IsActive;
	OnReplay = InArgs._OnReplay;
	Note = InArgs._Note;
	bNoteError = InArgs._NoteError;

	ChildSlot
	[
		SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.f)
			[
				SNew(STextBlock).Text(LOCTEXT("Title", "Bags"))
				.Font(BagsUi::Font(20, TEXT("Bold"))).ColorAndOpacity(BagsUi::Ink)
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 4, 0)
			[
				BagsUi::Button(LOCTEXT("Open", "Open the folder"), BagsUi::PlainBtn, []()
				{
					const FString Dir = BagsDir();
					if (!Dir.IsEmpty())
					{
						IFileManager::Get().MakeDirectory(*Dir, true);
						FPlatformProcess::ExploreFolder(*Dir);
					}
				})
			]
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				BagsUi::Button(LOCTEXT("Refresh", "Refresh"), BagsUi::PlainBtn, [this]() { Rescan(); })
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 10)
		[
			SNew(STextBlock)
			.Text(LOCTEXT("Hint", "Runs recorded with Record a bag (Drive, Autonomous), as the backend copies them out of "
				"the stack when the run stops. Replay puts the car back on the bag's track and moves it as it drove; the "
				"pipeline view shows what the pipeline saw at each moment. Esc stops the replay."))
			.Font(BagsUi::Font(10)).ColorAndOpacity(BagsUi::Muted).AutoWrapText(true)
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
		[
			SNew(STextBlock)
			.Text_Lambda([this]() { return FText::FromString(Note); })
			.Visibility_Lambda([this]() { return Note.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible; })
			.ColorAndOpacity_Lambda([this]() { return FSlateColor(bNoteError ? BagsUi::Bad : BagsUi::Accent); })
			.Font(BagsUi::Font(11, TEXT("Bold"))).AutoWrapText(true)
		]
		+ SVerticalBox::Slot().FillHeight(1.f)
		[
			SNew(SScrollBox)
			+ SScrollBox::Slot() [ SAssignNew(List, SVerticalBox) ]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			SNew(STextBlock)
			.Text_Lambda([this]() { return FText::FromString(Status); })
			.ColorAndOpacity_Lambda([this]() { return FSlateColor(bStatusError ? BagsUi::Bad : BagsUi::Muted); })
			.Font(BagsUi::Font(10)).AutoWrapText(true)
		]
	];

	RegisterActiveTimer(0.5f, FWidgetActiveTimerDelegate::CreateSP(this, &SFSDSBagsPage::Poll));
}

FString SFSDSBagsPage::BagsDir()
{
	const FString Repo = FFSDSStackStarter::FindComposeDir();
	return Repo.IsEmpty() ? FString() : FPaths::Combine(Repo, TEXT("bags"));
}

bool SFSDSBagsPage::ParseName(const FString& Name, FBag& Out)
{
	// <event>_<track>_<YYYYMMDD>_<HHMMSS> (bag_recorder.compose_bag_name); the
	// track may hold underscores, the event doesn't.
	TArray<FString> Parts;
	Name.ParseIntoArray(Parts, TEXT("_"));
	if (Parts.Num() < 4) return false;
	const FString& Date = Parts[Parts.Num() - 2];
	const FString& Time = Parts[Parts.Num() - 1];
	if (Date.Len() != 8 || Time.Len() != 6 || !Date.IsNumeric() || !Time.IsNumeric()) return false;
	Out.Name = Name;
	Out.Event = Parts[0];
	Out.Track = FString::Join(TArrayView<const FString>(Parts).Slice(1, Parts.Num() - 3), TEXT("_"));
	Out.bHasWhen = FDateTime::Parse(FString::Printf(TEXT("%s-%s-%s %s:%s:%s"),
		*Date.Left(4), *Date.Mid(4, 2), *Date.Right(2), *Time.Left(2), *Time.Mid(2, 2), *Time.Right(2)), Out.When);
	return !Out.Track.IsEmpty();
}

FString SFSDSBagsPage::SizeText(int64 Bytes)
{
	return Bytes >= (int64)1 << 30 ? FString::Printf(TEXT("%.1f GB"), Bytes / double((int64)1 << 30))
		: FString::Printf(TEXT("%.0f MB"), Bytes / double(1 << 20));
}

EActiveTimerReturnType SFSDSBagsPage::Poll(double Now, float Dt)
{
	const bool bActive = IsActive.Get(true);
	if (bActive && !bWasActive) Rescan();
	bWasActive = bActive;
	return EActiveTimerReturnType::Continue;
}

void SFSDSBagsPage::Rescan()
{
	Bags.Reset();
	const FString Dir = BagsDir();
	if (Dir.IsEmpty())
	{
		Status = TEXT("The IFSSIM repo isn't set: START once to set it up.");
		bStatusError = true;
		Rebuild();
		return;
	}
	TArray<FString> Names;
	IFileManager::Get().FindFiles(Names, *FPaths::Combine(Dir, TEXT("*")), false, true);
	for (const FString& N : Names)
	{
		if (N.EndsWith(TEXT("_carparity"))) continue;   // the LiDAR+IMU copy goes with its bag
		FBag B;
		if (!ParseName(N, B)) continue;
		B.Dir = FPaths::Combine(Dir, N);
		B.bComplete = FPaths::FileExists(FPaths::Combine(B.Dir, TEXT("metadata.yaml")));
		B.Bytes = BagsUi::DirBytes(B.Dir) + BagsUi::DirBytes(B.Dir + TEXT("_carparity"));
		Bags.Add(MoveTemp(B));
	}
	Bags.Sort([](const FBag& A, const FBag& B) { return A.Name.Right(15) > B.Name.Right(15); });   // newest first
	int64 Total = 0;
	for (const FBag& B : Bags) Total += B.Bytes;
	Status = Bags.Num() == 0 ? FString::Printf(TEXT("No bags in %s yet."), *FPaths::ConvertRelativePathToFull(Dir))
		: FString::Printf(TEXT("%d bags, %s in %s"), Bags.Num(), *SizeText(Total), *FPaths::ConvertRelativePathToFull(Dir));
	bStatusError = false;
	Rebuild();
}

void SFSDSBagsPage::Rebuild()
{
	if (!List.IsValid()) return;
	List->ClearChildren();
	for (const FBag& B : Bags)
	{
		const FString When = B.bHasWhen ? B.When.ToString(TEXT("%Y-%m-%d %H:%M")) : FString();
		const FString Meta = FString::Printf(TEXT("%s  ·  %s  ·  %s%s"), *B.Event, *When, *SizeText(B.Bytes),
			B.bComplete ? TEXT("") : TEXT("  ·  still being copied out, or cut short"));
		List->AddSlot().AutoHeight().Padding(0, 0, 0, 6)
		[
			SNew(SBorder)
			.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
			.BorderBackgroundColor(BagsUi::Card)
			.Padding(FMargin(12.f, 8.f))
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[
						SNew(STextBlock).Text(FText::FromString(B.Track)).Font(BagsUi::Font(12, TEXT("Bold"))).ColorAndOpacity(BagsUi::Ink)
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
					[
						SNew(STextBlock).Text(FText::FromString(Meta)).Font(BagsUi::Font(10)).ColorAndOpacity(BagsUi::Muted)
					]
				]
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 6, 0)
				[
					BagsUi::Button(LOCTEXT("Replay", "Replay in the sim"), BagsUi::GoBtn,
						[this, Dir = B.Dir, Track = B.Track]() { OnReplay.ExecuteIfBound(Dir, Track); },
						B.bComplete)
				]
				+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
				[
					BagsUi::Button(TAttribute<FText>::CreateLambda([this, Name = B.Name]()
						{
							return FPlatformTime::Seconds() < DeleteArmedUntil && DeleteArmedFor == Name
								? LOCTEXT("DeleteSure", "Click again to delete") : LOCTEXT("Delete", "Delete");
						}),
						BagsUi::PlainBtn, [this, Name = B.Name, Dir = B.Dir]()
						{
							if (FPlatformTime::Seconds() >= DeleteArmedUntil || DeleteArmedFor != Name)
							{
								DeleteArmedFor = Name;
								DeleteArmedUntil = FPlatformTime::Seconds() + 4.0;
								return;
							}
							DeleteArmedUntil = 0.0;
							const bool bOk = IFileManager::Get().DeleteDirectory(*Dir, false, true);
							IFileManager::Get().DeleteDirectory(*(Dir + TEXT("_carparity")), false, true);
							Rescan();
							if (!bOk)
							{
								Status = FString::Printf(TEXT("Couldn't delete %s (is it open somewhere?)."), *Name);
								bStatusError = true;
							}
						})
				]
			]
		];
	}
}

#undef LOCTEXT_NAMESPACE
