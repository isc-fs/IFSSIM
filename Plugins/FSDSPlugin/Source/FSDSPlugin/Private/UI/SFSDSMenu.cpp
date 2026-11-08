#include "UI/SFSDSMenu.h"

#include "UI/SFSDSTrackMap.h"
#include "UI/FSDSGraphics.h"
#include "UI/FSDSHud.h"
#include "UI/FSDSStackStarter.h"
#include "UI/FSDSMissionControlClient.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "HAL/PlatformProcess.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Layout/SSpacer.h"
#include "Widgets/Layout/SUniformGridPanel.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "Widgets/Layout/SWrapBox.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "FSDSMenu"

// Always qualified (MenuUi::...): other UI files in the same unity blob
// bring their own Font / Accent into scope.
namespace MenuUi
{
	// The Mission Control panel's palette, so the two read as one product.
	const FLinearColor Accent = FLinearColor::FromSRGBColor(FColor(255, 184, 28));
	const FLinearColor Ink = FLinearColor::FromSRGBColor(FColor(232, 232, 236));
	const FLinearColor Muted = FLinearColor::FromSRGBColor(FColor(150, 150, 160));
	const FLinearColor Good = FLinearColor::FromSRGBColor(FColor(74, 222, 128));
	const FLinearColor Bad = FLinearColor::FromSRGBColor(FColor(248, 113, 113));
	const FLinearColor Ground(0.010f, 0.010f, 0.013f, 0.94f);
	const FLinearColor Card(0.010f, 0.010f, 0.013f, 0.86f);
	const FLinearColor Dim(0.f, 0.f, 0.f, 0.35f);
	const FLinearColor PlainBtn = FLinearColor::FromSRGBColor(FColor(64, 64, 72));
	// Slate's button darkens its colour a lot: the big buttons start brighter
	// so they still read as green and grey.
	const FLinearColor GoBtn = FLinearColor::FromSRGBColor(FColor(34, 197, 94));
	const FLinearColor SecondBtn = FLinearColor::FromSRGBColor(FColor(130, 130, 145));
	const FLinearColor Off(0.f, 0.f, 0.f, 0.f);

	FSlateFontInfo MenuFont(int32 Size, const TCHAR* Weight = TEXT("Regular"))
	{
		return FCoreStyle::GetDefaultFontStyle(Weight, Size);
	}

	const FSlateBrush* White()
	{
		return FCoreStyle::Get().GetBrush("WhiteBrush");
	}

	FString Seconds(float S)
	{
		return FString::Printf(TEXT("%.2f s"), S);
	}
}

void SFSDSMenu::Construct(const FArguments& InArgs)
{
	Screen = InArgs._Screen;
	Tracks = InArgs._Tracks;
	Venues = InArgs._Venues;
	Results = InArgs._Results;
	Last = InArgs._Last;
	Referee = InArgs._Referee;
	LauncherKey = InArgs._LauncherKey;
	OnDrive = InArgs._OnDrive;
	OnResume = InArgs._OnResume;
	OnRestart = InArgs._OnRestart;
	OnMainMenu = InArgs._OnMainMenu;
	OnQuit = InArgs._OnQuit;
	OnTestDrive = InArgs._OnTestDrive;
	OnTracksChanged = InArgs._OnTracksChanged;
	OnStartStack = InArgs._OnStartStack;
	OnRebuildPipeline = InArgs._OnRebuildPipeline;
	RebuildStatus = InArgs._RebuildStatus;
	IsRebuilding = InArgs._IsRebuilding;
	RebuildFailed = InArgs._RebuildFailed;
	Client = InArgs._Client;
	EditorState = InArgs._EditorState;
	if (InArgs._StartInEditor) Page = EPage::Editor;

	// Start from the last run's choices, or the first track.
	if (Last.IsSet())
	{
		Event = Last->Event;
		bPractice = Last->bPractice;
		Laps = Last->Event == EFSDSEventType::Trackdrive && !Last->bPractice ? Last->Laps : Laps;
		bAutonomous = Last->bAutonomous;
		bRecordBag = Last->bRecordBag;
		Selected = Tracks.IndexOfByPredicate([this](const FFSDSMenuTrack& T) { return T.Name == Last->Track.Name; });
		Location = Last->Track.Venue;
	}

	ChildSlot
	[
		Screen == EScreen::Main ? BuildMain() : BuildRunBox()
	];

	// The box over an autonomous run starts from what the backend says.
	if (Screen == EScreen::Pause && Last.IsSet() && Last->bAutonomous && Client.IsValid())
	{
		TWeakPtr<SFSDSMenu> Weak = SharedThis(this);
		Client->Get(TEXT("/api/res/status"), [Weak](const FFSDSMcReply& R)
		{
			TSharedPtr<SFSDSMenu> M = Weak.Pin();
			const TSharedPtr<FJsonObject> O = R.Object();
			if (M && O.IsValid()) O->TryGetBoolField(TEXT("res_active"), M->bResActive);
		});
		Client->Get(TEXT("/api/sim/status"), [Weak](const FFSDSMcReply& R)
		{
			TSharedPtr<SFSDSMenu> M = Weak.Pin();
			const TSharedPtr<FJsonObject> O = R.Object();
			if (M && O.IsValid()) O->TryGetBoolField(TEXT("paused"), M->bSimPaused);
		});
	}

	if (Screen == EScreen::Main)
	{
		SetEvent(Event, bPractice);   // fills the list, keeps the last track if it fits
	}
}

void SFSDSMenu::SetRepoPrompt(const FString& Text, TFunction<void(int32)> OnChoice)
{
	RepoPrompt = Text;
	OnRepoChoice = MoveTemp(OnChoice);
}

void SFSDSMenu::SetMessage(const FString& Msg, bool bError)
{
	Message = Msg;
	bMessageError = bError;
}

FReply SFSDSMenu::OnKeyDown(const FGeometry&, const FKeyEvent& KeyEvent)
{
	if (KeyEvent.GetKey() == EKeys::Escape)
	{
		if (Screen == EScreen::Pause)
		{
			OnResume.ExecuteIfBound();
		}
		else if (Screen == EScreen::Main && Page != EPage::Editor)
		{
			Page = EPage::Drive;
		}
		return FReply::Handled();
	}
	return FReply::Unhandled();
}

// ---------------------------------------------------------------------------
// Building blocks

TSharedRef<SWidget> SFSDSMenu::MakeHeading(const FText& Title)
{
	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 16, 0, 6)
		[
			SNew(SSeparator).Thickness(1.f).ColorAndOpacity(FLinearColor(1, 1, 1, 0.08f))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 8)
		[
			SNew(STextBlock).Text(Title).Font(MenuUi::MenuFont(10, TEXT("Bold"))).ColorAndOpacity(MenuUi::Accent)
		];
}

TSharedRef<SWidget> SFSDSMenu::MakeNavButton(const FText& Label, EPage InPage)
{
	return SNew(SButton)
		.IsFocusable(false)
		.ContentPadding(FMargin(10.f, 8.f))
		.ButtonColorAndOpacity_Lambda([this, InPage]()
		{
			return FSlateColor(Page == InPage ? MenuUi::PlainBtn : MenuUi::Off);
		})
		.OnClicked_Lambda([this, InPage]()
		{
			Page = InPage;
			// Tracks may have come or gone outside the menu (Mission Control,
			// envgen, a file manager): read the folders again.
			if (InPage == EPage::Drive || InPage == EPage::Editor) OnTracksChanged.ExecuteIfBound();
			return FReply::Handled();
		})
		[
			SNew(STextBlock)
			.Text(Label)
			.Font(MenuUi::MenuFont(15, TEXT("Bold")))
			.ColorAndOpacity_Lambda([this, InPage]()
			{
				return FSlateColor(Page == InPage ? MenuUi::Accent : MenuUi::Ink);
			})
		];
}

TSharedRef<SWidget> SFSDSMenu::GraphicsRow(const FText& Label, const TArray<FGraphicsChoice>& Choices)
{
	TSharedRef<SHorizontalBox> Row = SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
		[
			SNew(SBox).WidthOverride(200.f)
			[
				SNew(STextBlock).Text(Label).Font(MenuUi::MenuFont(11, TEXT("Bold"))).ColorAndOpacity(MenuUi::Ink)
			]
		];
	for (const FGraphicsChoice& C : Choices)
	{
		Row->AddSlot().AutoWidth().Padding(0, 0, 3, 0) [ MakeChoice(C.Label, C.IsOn, C.OnPick) ];
	}
	return Row;
}

TSharedRef<SWidget> SFSDSMenu::SpeedChoice(const FText& Label, double Speed)
{
	return MakeChoice(Label,
		[Speed]() { return FMath::IsNearlyEqual(FFSDSRealTime::Get().SpeedFactor(), Speed, 1e-3); },
		[Speed]() { FFSDSRealTime::Get().SetSpeedFactor(Speed); });
}

TSharedRef<SWidget> SFSDSMenu::MakeChoice(TAttribute<FText> Label, TFunction<bool()> IsOn, TFunction<void()> OnPick)
{
	// Slate darkens a button's colour, so "on" is marked by the accent text
	// on a lighter button, as the panel's tabs are.
	return SNew(SButton)
		.IsFocusable(false)
		.ContentPadding(FMargin(10.f, 7.f))
		.HAlign(HAlign_Center)
		.ButtonColorAndOpacity_Lambda([IsOn]()
		{
			return FSlateColor(IsOn() ? MenuUi::PlainBtn : FLinearColor(1.f, 1.f, 1.f, 0.06f));
		})
		.OnClicked_Lambda([OnPick]()
		{
			OnPick();
			return FReply::Handled();
		})
		[
			SNew(STextBlock)
			.Text(Label)
			.Font(MenuUi::MenuFont(11, TEXT("Bold")))
			.ColorAndOpacity_Lambda([IsOn]()
			{
				return FSlateColor(IsOn() ? MenuUi::Accent : MenuUi::Muted);
			})
		];
}

TSharedRef<SWidget> SFSDSMenu::MakeBigButton(const FText& Label, FLinearColor Color, FSimpleDelegate OnClick,
	TAttribute<bool> Enabled)
{
	return SNew(SButton)
		.IsFocusable(false)
		.ButtonColorAndOpacity(Color)
		.ContentPadding(FMargin(16.f, 10.f))
		.HAlign(HAlign_Center)
		.IsEnabled(Enabled)
		.OnClicked_Lambda([OnClick]()
		{
			OnClick.ExecuteIfBound();
			return FReply::Handled();
		})
		[
			SNew(STextBlock).Text(Label).Font(MenuUi::MenuFont(13, TEXT("Bold"))).ColorAndOpacity(FLinearColor::White)
		];
}

// ---------------------------------------------------------------------------
// The front end

TSharedRef<SWidget> SFSDSMenu::BuildMain()
{
	return SNew(SOverlay)
		+ SOverlay::Slot()
		[
			SNew(SBorder).BorderImage(MenuUi::White()).BorderBackgroundColor(MenuUi::Dim)
		]
		+ SOverlay::Slot()
		[
			SNew(SHorizontalBox)
			// Title and navigation.
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SBorder)
				.BorderImage(MenuUi::White())
				.BorderBackgroundColor(MenuUi::Ground)
				.Padding(FMargin(32.f, 40.f))
				[
					SNew(SBox).WidthOverride(230.f)
					[
						SNew(SVerticalBox)
						+ SVerticalBox::Slot().AutoHeight()
						[
							SNew(STextBlock).Text(LOCTEXT("Title", "IFSSIM"))
							.Font(MenuUi::MenuFont(34, TEXT("Bold"))).ColorAndOpacity(MenuUi::Accent)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
						[
							SNew(STextBlock).Text(LOCTEXT("Subtitle", "Formula Student Driverless simulator"))
							.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 40)
						[
							SNew(STextBlock).Text(LOCTEXT("Team", "ISC Racing Team"))
							.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted)
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ MakeNavButton(LOCTEXT("NavDrive", "Drive"), EPage::Drive) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ MakeNavButton(LOCTEXT("NavEditor", "Track editor"), EPage::Editor) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ MakeNavButton(LOCTEXT("NavResults", "Results"), EPage::Results) ]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 2) [ MakeNavButton(LOCTEXT("NavSettings", "Settings"), EPage::Settings) ]
						+ SVerticalBox::Slot().FillHeight(1.f) [ SNew(SSpacer) ]
						+ SVerticalBox::Slot().AutoHeight()
						[
							SNew(SButton)
							.IsFocusable(false)
							.ContentPadding(FMargin(10.f, 8.f))
							.ButtonColorAndOpacity(MenuUi::Off)
							.OnClicked_Lambda([this]() { OnQuit.ExecuteIfBound(); return FReply::Handled(); })
							[
								SNew(STextBlock).Text(LOCTEXT("Quit", "Quit"))
								.Font(MenuUi::MenuFont(13, TEXT("Bold"))).ColorAndOpacity(MenuUi::Muted)
							]
						]
					]
				]
			]
			// The page.
			+ SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(40.f))
			[
				SNew(SBorder)
				.BorderImage(MenuUi::White())
				.BorderBackgroundColor(MenuUi::Card)
				.Padding(FMargin(28.f, 24.f))
				[
					SNew(SWidgetSwitcher)
					.WidgetIndex_Lambda([this]() { return static_cast<int32>(Page); })
					+ SWidgetSwitcher::Slot() [ BuildDrivePage() ]
					+ SWidgetSwitcher::Slot()
					[
						SAssignNew(EditorPage, SFSDSTrackEditor)
						.State(EditorState)
						.Client(Client)
						.Tracks(Tracks)
						.Venues(Venues)
						.OnTestDrive(OnTestDrive)
						.OnSaved(OnTracksChanged)
						.OnStartStack(OnStartStack)
					]
					+ SWidgetSwitcher::Slot() [ BuildResultsPage() ]
					+ SWidgetSwitcher::Slot() [ BuildSettingsPage() ]
				]
			]
		];
}

TSharedRef<SWidget> SFSDSMenu::BuildDrivePage()
{
	auto EventChoice = [this](const FText& Label, EFSDSEventType E, bool bP)
	{
		return MakeChoice(Label,
			[this, E, bP]() { return bPractice == bP && (bP || Event == E); },
			[this, E, bP]() { SetEvent(E, bP); });
	};

	return SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().AutoWidth()
		[
			SNew(SBox).WidthOverride(400.f)
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(STextBlock).Text(LOCTEXT("DriveTitle", "Drive"))
					.Font(MenuUi::MenuFont(20, TEXT("Bold"))).ColorAndOpacity(MenuUi::Ink)
				]
				// Where: a venue and its tracks, or anywhere (every track). Only
				// with venues to pick from.
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SVerticalBox)
					.Visibility_Lambda([this]() { return Venues.Num() > 0 ? EVisibility::Visible : EVisibility::Collapsed; })
					+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("Location", "LOCATION")) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(3, 0)
					[
						SAssignNew(LocationPicker, SFSDSLocationPicker)
						.NoneLabel(LOCTEXT("Anywhere", "Anywhere"))
						.Venues(Venues)
						.Selected_Lambda([this]() { return Location; })
						.CountOf([this](const FString& V) { return CountAt(V); })
						.OnPicked_Lambda([this](const FString& V) { SetLocation(V); })
					]
				]
				+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("Event", "EVENT")) ]
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SUniformGridPanel).SlotPadding(FMargin(3.f))
					+ SUniformGridPanel::Slot(0, 0) [ EventChoice(LOCTEXT("EvAccel", "Acceleration"), EFSDSEventType::Acceleration, false) ]
					+ SUniformGridPanel::Slot(1, 0) [ EventChoice(LOCTEXT("EvSkid", "Skidpad"), EFSDSEventType::Skidpad, false) ]
					+ SUniformGridPanel::Slot(2, 0) [ EventChoice(LOCTEXT("EvAuto", "Autocross"), EFSDSEventType::Autocross, false) ]
					+ SUniformGridPanel::Slot(0, 1) [ EventChoice(LOCTEXT("EvTrack", "Trackdrive"), EFSDSEventType::Trackdrive, false) ]
					+ SUniformGridPanel::Slot(1, 1) [ EventChoice(LOCTEXT("EvPractice", "Practice"), EFSDSEventType::Trackdrive, true) ]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(3, 6, 3, 0)
				[
					SNew(STextBlock)
					.Text_Lambda([this]() { return EventHint(); })
					.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
				]
				// Laps, Trackdrive only.
				+ SVerticalBox::Slot().AutoHeight().Padding(3, 6, 3, 0)
				[
					SNew(SHorizontalBox)
					.Visibility_Lambda([this]() { return Event == EFSDSEventType::Trackdrive && !bPractice ? EVisibility::Visible : EVisibility::Collapsed; })
					+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
					[
						SNew(STextBlock).Text(LOCTEXT("Laps", "Laps")).Font(MenuUi::MenuFont(11)).ColorAndOpacity(MenuUi::Muted)
					]
					+ SHorizontalBox::Slot().AutoWidth()
					[
						MakeChoice(LOCTEXT("LapsDown", "-"), []() { return false; }, [this]() { Laps = FMath::Max(1, Laps - 1); })
					]
					+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(12, 0)
					[
						SNew(SBox).WidthOverride(24.f).HAlign(HAlign_Center)
						[
							SNew(STextBlock).Text_Lambda([this]() { return FText::AsNumber(Laps); })
							.Font(MenuUi::MenuFont(13, TEXT("Bold"))).ColorAndOpacity(MenuUi::Ink)
						]
					]
					+ SHorizontalBox::Slot().AutoWidth()
					[
						MakeChoice(LOCTEXT("LapsUp", "+"), []() { return false; }, [this]() { Laps = FMath::Min(30, Laps + 1); })
					]
				]
				+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("Track", "TRACK")) ]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
				[
					SNew(SEditableTextBox)
					.Visibility_Lambda([this]() { return Tracks.Num() > 12 || !Search.IsEmpty() ? EVisibility::Visible : EVisibility::Collapsed; })
					.Font(MenuUi::MenuFont(11))
					.HintText(LOCTEXT("SearchHint", "search tracks"))
					.Text_Lambda([this]() { return FText::FromString(Search); })
					.OnTextChanged_Lambda([this](const FText& T)
					{
						Search = T.ToString().TrimStartAndEnd();
						SetEvent(Event, bPractice);
					})
				]
				+ SVerticalBox::Slot().FillHeight(1.f)
				[
					SAssignNew(DriveList, SScrollBox)
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
				[
					SNew(SHorizontalBox)
					.Visibility_Lambda([this]() { return Client.IsValid() ? EVisibility::Visible : EVisibility::Collapsed; })
					+ SHorizontalBox::Slot().FillWidth(1.f).Padding(3.f).VAlign(VAlign_Center)
					[
						SNew(STextBlock)
						.Text(LOCTEXT("NewTracksHint", "New tracks, drawn or random: Track editor."))
						.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
					]
					+ SHorizontalBox::Slot().FillWidth(1.f).Padding(3.f).VAlign(VAlign_Center)
					[
						SNew(STextBlock)
						.Visibility_Lambda([this]()
						{
							return Tracks.IsValidIndex(Selected) && !WhyKept(Tracks[Selected].Name).IsEmpty()
								? EVisibility::Visible : EVisibility::Collapsed;
						})
						.Text_Lambda([this]()
						{
							return Tracks.IsValidIndex(Selected) ? FText::FromString(WhyKept(Tracks[Selected].Name)) : FText::GetEmpty();
						})
						.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
					]
					+ SHorizontalBox::Slot().FillWidth(1.f).Padding(3.f)
					[
						SNew(SBox)
						.Visibility_Lambda([this]()
						{
							return Tracks.IsValidIndex(Selected) && WhyKept(Tracks[Selected].Name).IsEmpty()
								? EVisibility::Visible : EVisibility::Collapsed;
						})
						[
							MakeChoice(TAttribute<FText>::CreateLambda([this]()
								{
									const bool bArmed = FPlatformTime::Seconds() < DeleteArmedUntil
										&& Tracks.IsValidIndex(Selected) && DeleteArmedFor == Tracks[Selected].Name;
									return bArmed ? LOCTEXT("DeleteSure", "Click again to delete") : LOCTEXT("Delete", "Delete this track");
								}),
								[]() { return false; }, [this]() { DeleteTrack(); })
						]
					]
				]
				+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("Driver", "DRIVER")) ]
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().FillWidth(1.f).Padding(3.f)
					[
						MakeChoice(LOCTEXT("Manual", "Manual (keyboard)"),
							[this]() { return !bAutonomous; }, [this]() { bAutonomous = false; })
					]
					// Practice is manual: Mission Control has no practice mission.
					+ SHorizontalBox::Slot().FillWidth(1.f).Padding(3.f)
					[
						SNew(SBox)
						.Visibility_Lambda([this]() { return bPractice ? EVisibility::Collapsed : EVisibility::Visible; })
						[
							MakeChoice(LOCTEXT("Autonomous", "Autonomous"),
								[this]() { return bAutonomous; },
								[this]()
								{
									// Switching to the pipeline starts at real time; slow
									// motion is a choice made after.
									if (!bAutonomous) FFSDSRealTime::Get().SetSpeedFactor(1.0);
									bAutonomous = true;
								})
						]
					]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(3, 6, 3, 0)
				[
					SNew(STextBlock)
					.Text_Lambda([this]()
					{
						return bAutonomous
							? FText::Format(LOCTEXT("AutoHint",
								"The DV pipeline drives. START loads the track and starts the session through "
								"Mission Control's backend, starting the Docker stack first if it isn't running. Esc during the run: stop "
								"it or start again. ({0} opens the engineers' panel.)"), LauncherKey)
							: bPractice ? LOCTEXT("ManualHintPractice", "You drive with the keyboard or a gamepad. Esc pauses. "
								"Practice is for you; for the pipeline, pick an event.")
							: LOCTEXT("ManualHint", "You drive with the keyboard or a gamepad. Esc pauses.");
					})
					.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
				]
				// How fast the simulation runs while the pipeline drives.
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SVerticalBox)
					.Visibility_Lambda([this]() { return bAutonomous ? EVisibility::Visible : EVisibility::Collapsed; })
					+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("Timing", "TIMING")) ]
					+ SVerticalBox::Slot().AutoHeight()
					[
						// "Real time" needs more room than the slow motions.
						SNew(SHorizontalBox)
						+ SHorizontalBox::Slot().FillWidth(1.7f).Padding(3.f) [ SpeedChoice(LOCTEXT("RealTime", "Real time"), 1.0) ]
						+ SHorizontalBox::Slot().FillWidth(1.f).Padding(3.f) [ SpeedChoice(LOCTEXT("Slow50", "0.5x"), 0.5) ]
						+ SHorizontalBox::Slot().FillWidth(1.f).Padding(3.f) [ SpeedChoice(LOCTEXT("Slow25", "0.25x"), 0.25) ]
						+ SHorizontalBox::Slot().FillWidth(1.f).Padding(3.f) [ SpeedChoice(LOCTEXT("Slow10", "0.1x"), 0.1) ]
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(3, 6, 3, 0)
					[
						SNew(STextBlock)
						.Text_Lambda([]()
						{
							const double S = FFSDSRealTime::Get().SpeedFactor();
							return S >= 1.0
								? LOCTEXT("RealTimeHint", "For the car's computer: the sim keeps pace with the real "
									"world, as on track. The HUD warns if this PC can't keep up.")
								: FText::Format(LOCTEXT("SlowHint", "For comparing algorithms: the sim runs at {0} of real "
									"time, so the pipeline gets {1}x its real time per step. Physics and sensors are "
									"unchanged; the view is choppier."),
									FText::FromString(FString::Printf(TEXT("%gx"), S)), FText::AsNumber(FMath::RoundToInt(1.0 / S)));
						})
						.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
					]
					+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("Recording", "RECORDING")) ]
					+ SVerticalBox::Slot().AutoHeight()
					[
						SNew(SHorizontalBox)
						+ SHorizontalBox::Slot().FillWidth(1.f).Padding(3.f)
						[
							MakeChoice(LOCTEXT("NoBag", "No bag"), [this]() { return !bRecordBag; }, [this]() { bRecordBag = false; })
						]
						+ SHorizontalBox::Slot().FillWidth(1.f).Padding(3.f)
						[
							MakeChoice(LOCTEXT("Bag", "Record a bag"), [this]() { return bRecordBag; }, [this]() { bRecordBag = true; })
						]
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(3, 6, 3, 0)
					[
						SNew(STextBlock)
						.Text(LOCTEXT("BagHint", "A rosbag of the run's topics, for replaying it, in the bags folder of the "
							"team's code (tools/list-bags.sh). Heavy: it records every LiDAR scan (about 15 MB a second), "
							"so a busy PC can slow the pipeline down, and saving it after the run can take a few minutes."))
						.Visibility_Lambda([this]() { return bRecordBag ? EVisibility::Visible : EVisibility::Collapsed; })
						.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
					]
					+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("Scoring", "SCORING")) ]
					+ SVerticalBox::Slot().AutoHeight().Padding(3.f)
					[
						SNew(SHorizontalBox)
						+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(0, 0, 8, 0)
						[
							SNew(STextBlock).Text(LOCTEXT("RefLabel", "Reference time"))
							.Font(MenuUi::MenuFont(11)).ColorAndOpacity(MenuUi::Muted)
						]
						+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
						[
							SNew(SBox).WidthOverride(110.f)
							[
								SNew(SEditableTextBox)
								.Font(MenuUi::MenuFont(11))
								.HintText(LOCTEXT("RefHint", "none"))
								.Text_Lambda([this]() { return FText::FromString(Reference()); })
								.OnTextCommitted_Lambda([this](const FText& Text, ETextCommit::Type)
								{
									FString In = Text.ToString().TrimStartAndEnd().Replace(TEXT(","), TEXT("."));
									In.RemoveFromEnd(TEXT("s"));
									In.TrimEndInline();
									if (In.IsEmpty() || (In.IsNumeric() && FCString::Atod(*In) > 0.0))
									{
										SetReference(In);
										Message.Reset();
									}
									else
									{
										SetMessage(TEXT("The reference time is a number of seconds, like 62.5, or empty for none."), true);
									}
								})
							]
						]
						+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center).Padding(6, 0, 0, 0)
						[
							SNew(STextBlock).Text(LOCTEXT("RefUnit", "s")).Font(MenuUi::MenuFont(11)).ColorAndOpacity(MenuUi::Muted)
						]
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(3, 4, 3, 0)
					[
						SNew(STextBlock)
						.Text_Lambda([this]()
						{
							return Reference().IsEmpty()
								? LOCTEXT("RefNoneHint", "The fastest corrected time for this event on this track, the one FS "
									"scores against. Without it the run gets its time and penalties but no points.")
								: LOCTEXT("RefSetHint", "Points are scored against this time, as at FS. Saved for this event "
									"and track.");
						})
						.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
					]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(3, 14, 3, 0)
				[
					MakeBigButton(LOCTEXT("Start", "START"), MenuUi::GoBtn,
						FSimpleDelegate::CreateLambda([this]()
						{
							if (!Tracks.IsValidIndex(Selected)) return;
							FFSDSDriveRequest R;
							R.Track = Tracks[Selected];
							R.Event = Event;
							R.bPractice = bPractice;
							R.Laps = Event == EFSDSEventType::Trackdrive && !bPractice ? Laps : 1;
							R.bAutonomous = bAutonomous;
							R.bRecordBag = bAutonomous && bRecordBag;
							R.Reference = bAutonomous && !Reference().IsEmpty() ? FCString::Atod(*Reference()) : 0.0;
							OnDrive.ExecuteIfBound(R);
						}),
						TAttribute<bool>::CreateLambda([this]() { return Tracks.IsValidIndex(Selected); }))
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(3, 8, 3, 0)
				[
					SNew(STextBlock)
					.Text_Lambda([this]() { return FText::FromString(Message); })
					.Visibility_Lambda([this]() { return Message.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible; })
					.ColorAndOpacity_Lambda([this]() { return FSlateColor(bMessageError ? MenuUi::Bad : MenuUi::Good); })
					.Font(MenuUi::MenuFont(10)).AutoWrapText(true)
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(3, 8, 3, 0)
				[
					SNew(SVerticalBox)
					.Visibility_Lambda([this]() { return RepoPrompt.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible; })
					+ SVerticalBox::Slot().AutoHeight()
					[
						SNew(STextBlock)
						.Text_Lambda([this]() { return FText::FromString(RepoPrompt); })
						.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Ink).AutoWrapText(true)
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
					[
						SNew(SWrapBox).UseAllottedSize(true).InnerSlotPadding(FVector2D(3.f, 3.f))
						+ SWrapBox::Slot() [ MakeChoice(LOCTEXT("RepoGet", "Get it"), []() { return false; },
							[this]() { if (OnRepoChoice) OnRepoChoice(0); }) ]
						+ SWrapBox::Slot() [ MakeChoice(LOCTEXT("RepoElsewhere", "Somewhere else..."), []() { return false; },
							[this]() { if (OnRepoChoice) OnRepoChoice(1); }) ]
						+ SWrapBox::Slot() [ MakeChoice(LOCTEXT("RepoHave", "I have it..."), []() { return false; },
							[this]() { if (OnRepoChoice) OnRepoChoice(2); }) ]
						+ SWrapBox::Slot() [ MakeChoice(LOCTEXT("RepoNotNow", "Not now"), []() { return false; },
							[this]() { if (OnRepoChoice) OnRepoChoice(3); }) ]
					]
				]
			]
		]
		+ SHorizontalBox::Slot().FillWidth(1.f).Padding(28, 0, 0, 0)
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().FillHeight(1.f)
			[
				SAssignNew(DriveMap, SFSDSTrackMap)
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
			[
				SNew(STextBlock)
				.Text_Lambda([this]() { return TrackSummary(Selected); })
				.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted)
			]
		];
}

TSharedRef<SWidget> SFSDSMenu::BuildResultsPage()
{
	TSharedRef<SVerticalBox> List = SNew(SVerticalBox);
	const int32 N = Results.IsValid() ? Results->Num() : 0;
	for (int32 i = N - 1; i >= 0; --i)
	{
		const FFSDSRunResult& R = (*Results)[i];
		float Best = 0.f, Total = 0.f;
		for (float L : R.Laps)
		{
			Total += L;
			Best = Best > 0.f ? FMath::Min(Best, L) : L;
		}
		const FString Head = FString::Printf(TEXT("%s   %s  ·  %s"),
			*R.When.ToString(TEXT("%H:%M")), *R.Track,
			*(FFSDSMenu::EventName(R.Event, R.bPractice) + (R.bAutonomous ? TEXT(", autonomous") : TEXT(""))));
		FString Body = R.bPractice ? FString::Printf(TEXT("Laps %d"), R.Laps.Num())
			: FString::Printf(TEXT("Laps %d/%d"), R.Laps.Num(), R.RequiredLaps);
		if (R.Laps.Num() > 0)
		{
			Body += FString::Printf(TEXT("   best %s   total %s"), *MenuUi::Seconds(Best), *MenuUi::Seconds(Total));
		}
		Body += FString::Printf(TEXT("   cones down %d   off course %d"), R.ConesDown, R.OffCourse);
		if (R.bScored && R.bFinished) Body += TEXT("\n") + ScoreText(R);
		if (R.bBag) Body += TEXT("\nBag recorded (the bags folder of the team's code).");

		List->AddSlot().AutoHeight().Padding(0, 0, 0, 10)
		[
			SNew(SBorder)
			.BorderImage(MenuUi::White())
			.BorderBackgroundColor(FLinearColor(1.f, 1.f, 1.f, 0.04f))
			.Padding(FMargin(12.f, 8.f))
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().FillWidth(1.f)
					[
						SNew(STextBlock).Text(FText::FromString(Head)).Font(MenuUi::MenuFont(11, TEXT("Bold"))).ColorAndOpacity(MenuUi::Ink)
					]
					+ SHorizontalBox::Slot().AutoWidth()
					[
						SNew(STextBlock)
						.Text(R.bFinished ? LOCTEXT("Finished", "finished") : R.bPractice ? FText::GetEmpty() : LOCTEXT("NotFinished", "not finished"))
						.Font(MenuUi::MenuFont(10, TEXT("Bold")))
						.ColorAndOpacity(R.bFinished ? MenuUi::Good : MenuUi::Muted)
					]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
				[
					SNew(STextBlock).Text(FText::FromString(Body)).Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted)
				]
			]
		];
	}
	if (N == 0)
	{
		List->AddSlot().AutoHeight()
		[
			SNew(STextBlock).Text(LOCTEXT("NoRuns", "No runs yet. Your manual runs show up here."))
			.Font(MenuUi::MenuFont(11)).ColorAndOpacity(MenuUi::Muted)
		];
	}

	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(STextBlock).Text(LOCTEXT("ResultsTitle", "Results"))
			.Font(MenuUi::MenuFont(20, TEXT("Bold"))).ColorAndOpacity(MenuUi::Ink)
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
		[
			SNew(STextBlock)
			.Text(LOCTEXT("ResultsHint",
				"Runs since the sim started, newest first. Autonomous runs show Mission Control's FS score: the time "
				"with the cone and off-course penalties, and the points against a reference time when the event has one."))
			.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
		]
		+ SVerticalBox::Slot().FillHeight(1.f)
		[
			SNew(SScrollBox)
			+ SScrollBox::Slot() [ MakeHeading(LOCTEXT("Runs", "RUNS")) ]
			+ SScrollBox::Slot() [ List ]
			+ SScrollBox::Slot() [ MakeHeading(LOCTEXT("SessionLog", "SESSION LOG")) ]
			+ SScrollBox::Slot()
			[
				SNew(SHorizontalBox)
				.Visibility_Lambda([this]() { return Client.IsValid() ? EVisibility::Visible : EVisibility::Collapsed; })
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0)
				[
					MakeChoice(LOCTEXT("ShowLog", "Show the session log"), [this]() { return bLogLoading; }, [this]() { FetchLog(); })
				]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0)
				[
					MakeChoice(LOCTEXT("SaveLog", "Save it to a file"), []() { return false; }, [this]() { SaveLog(); })
				]
				+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center).Padding(9, 0, 0, 0)
				[
					SNew(STextBlock).Text_Lambda([this]() { return FText::FromString(LogNote); })
					.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
				]
			]
			+ SScrollBox::Slot().Padding(0, 8, 0, 0)
			[
				SAssignNew(LogBox, SVerticalBox)
			]
		];
}

TSharedRef<SWidget> SFSDSMenu::BuildSettingsPage()
{
	RepoShown = FFSDSStackStarter::FindComposeDir();
	TSharedRef<SVerticalBox> Keys = SNew(SVerticalBox);
	auto Row = [&Keys](const FText& Key, const FText& What)
	{
		Keys->AddSlot().AutoHeight().Padding(0, 3)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SBox).WidthOverride(200.f)
				[
					SNew(STextBlock).Text(Key).Font(MenuUi::MenuFont(11, TEXT("Bold"))).ColorAndOpacity(MenuUi::Ink)
				]
			]
			+ SHorizontalBox::Slot().FillWidth(1.f)
			[
				SNew(STextBlock).Text(What).Font(MenuUi::MenuFont(11)).ColorAndOpacity(MenuUi::Muted)
			]
		];
	};
	Row(LOCTEXT("KThrottle", "W  /  Up"), LOCTEXT("KThrottleW", "Throttle"));
	Row(LOCTEXT("KReverse", "S  /  Down"), LOCTEXT("KReverseW", "Reverse"));
	Row(LOCTEXT("KSteer", "A D  /  Left Right"), LOCTEXT("KSteerW", "Steer"));
	Row(LOCTEXT("KBrake", "Space"), LOCTEXT("KBrakeW", "Brake"));
	Row(LOCTEXT("KPause", "Esc"), LOCTEXT("KPauseW", "Pause the run"));
	Row(LauncherKey, LOCTEXT("KPanelW", "Mission Control panel (for engineers)"));
	Row(LOCTEXT("KPad", "Gamepad"), LOCTEXT("KPadW", "Right trigger throttle, left trigger brake, left stick steer"));

	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(STextBlock).Text(LOCTEXT("SettingsTitle", "Settings"))
			.Font(MenuUi::MenuFont(20, TEXT("Bold"))).ColorAndOpacity(MenuUi::Ink)
		]
		+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("Controls", "CONTROLS")) ]
		+ SVerticalBox::Slot().AutoHeight() [ Keys ]
		+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("Performance", "PERFORMANCE")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				SNew(SBox).WidthOverride(200.f)
				[
					SNew(STextBlock).Text(LOCTEXT("MenuFps", "Menu frame rate"))
					.Font(MenuUi::MenuFont(11, TEXT("Bold"))).ColorAndOpacity(MenuUi::Ink)
				]
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0)
			[
				MakeChoice(LOCTEXT("Fps60", "60"), []() { return FFSDSMenu::GetMenuFrameRate() == 60; }, []() { FFSDSMenu::SetMenuFrameRate(60); })
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0)
			[
				MakeChoice(LOCTEXT("Fps30", "30"), []() { return FFSDSMenu::GetMenuFrameRate() == 30; }, []() { FFSDSMenu::SetMenuFrameRate(30); })
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				MakeChoice(LOCTEXT("Fps15", "15"), []() { return FFSDSMenu::GetMenuFrameRate() == 15; }, []() { FFSDSMenu::SetMenuFrameRate(15); })
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			SNew(STextBlock)
			.Text(FText::Format(LOCTEXT("MenuFpsHint",
				"While this menu is up and nothing drives, the sim draws this many frames a second "
				"({0} when its window is minimised), so it costs less in the background. A run always "
				"goes at the sim's full rate: every frame is a fixed physics step, so fewer frames would "
				"slow the simulation down and give the pipeline more time than on the car. The HUD's "
				"\"Real time\" line shows how close a run is to real time."), FText::AsNumber(FFSDSMenu::MinimisedFrameRate)))
			.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
		]
		+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("Hud", "HUD")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				SNew(SBox).WidthOverride(200.f)
				[
					SNew(STextBlock).Text(LOCTEXT("HudDetail", "What it shows"))
					.Font(MenuUi::MenuFont(11, TEXT("Bold"))).ColorAndOpacity(MenuUi::Ink)
				]
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 3, 0)
			[
				MakeChoice(LOCTEXT("HudBasic", "Basic"), []() { return !FFSDSHud::IsDetailed(); }, []() { FFSDSHud::SetDetailed(false); })
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				MakeChoice(LOCTEXT("HudDetailed", "Detailed"), []() { return FFSDSHud::IsDetailed(); }, []() { FFSDSHud::SetDetailed(true); })
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			SNew(STextBlock)
			.Text(FText::Format(LOCTEXT("HudDetailHint",
				"Basic: speed, pedals, steering, laps and penalties. Detailed adds the motor's regen (torque, "
				"power and which cap binds) and the car's position and heading. {0} shows and hides the HUD."),
				FText::FromString(FFSDSHud::ToggleKey().GetDisplayName().ToString())))
			.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
		]
		+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("Graphics", "GRAPHICS")) ]
		+ SVerticalBox::Slot().AutoHeight() [ GraphicsRow(LOCTEXT("GQuality", "Quality"),
			{ { LOCTEXT("GLow", "Low"), []() { return FFSDSGraphics::Get().GetSettings().Quality == FFSDSGraphics::EQuality::Low; },
				  []() { FFSDSGraphics::Get().SetQuality(FFSDSGraphics::EQuality::Low); } },
			  { LOCTEXT("GMedium", "Medium"), []() { return FFSDSGraphics::Get().GetSettings().Quality == FFSDSGraphics::EQuality::Medium; },
				  []() { FFSDSGraphics::Get().SetQuality(FFSDSGraphics::EQuality::Medium); } },
			  { LOCTEXT("GHigh", "High"), []() { return FFSDSGraphics::Get().GetSettings().Quality == FFSDSGraphics::EQuality::High; },
				  []() { FFSDSGraphics::Get().SetQuality(FFSDSGraphics::EQuality::High); } } }) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0) [ GraphicsRow(LOCTEXT("GRes", "Resolution"),
			{ { LOCTEXT("G25", "25%"), []() { return FFSDSGraphics::Get().GetSettings().ResolutionPct == 25; }, []() { FFSDSGraphics::Get().SetResolution(25); } },
			  { LOCTEXT("G50", "50%"), []() { return FFSDSGraphics::Get().GetSettings().ResolutionPct == 50; }, []() { FFSDSGraphics::Get().SetResolution(50); } },
			  { LOCTEXT("G75", "75%"), []() { return FFSDSGraphics::Get().GetSettings().ResolutionPct == 75; }, []() { FFSDSGraphics::Get().SetResolution(75); } },
			  { LOCTEXT("G100", "100%"), []() { return FFSDSGraphics::Get().GetSettings().ResolutionPct == 100; }, []() { FFSDSGraphics::Get().SetResolution(100); } } }) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0) [ GraphicsRow(LOCTEXT("GView", "View while the pipeline drives"),
			{ { LOCTEXT("GShown", "Shown"), []() { return !FFSDSGraphics::Get().GetSettings().bHideViewWhileAutonomous; },
				  []() { FFSDSGraphics::Get().SetHideViewWhileAutonomous(false); } },
			  { LOCTEXT("GHidden", "Hidden"), []() { return FFSDSGraphics::Get().GetSettings().bHideViewWhileAutonomous; },
				  []() { FFSDSGraphics::Get().SetHideViewWhileAutonomous(true); } } }) ]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth()
			[
				MakeChoice(LOCTEXT("GFind", "Find the best for this PC"),
					[]() { return FFSDSGraphics::Get().IsRecommending(); },
					[this]()
					{
						RecommendResult.Reset();
						TWeakPtr<SFSDSMenu> Weak = SharedThis(this);
						FFSDSGraphics::Get().Recommend([Weak](const FString& Msg)
						{
							if (TSharedPtr<SFSDSMenu> M = Weak.Pin()) M->RecommendResult = Msg;
						});
					})
			]
			+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center).Padding(12, 0, 0, 0)
			[
				SNew(STextBlock)
				.Text_Lambda([this]()
				{
					const FFSDSGraphics& G = FFSDSGraphics::Get();
					return FText::FromString(G.IsRecommending() ? G.RecommendProgress() : RecommendResult);
				})
				.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Ink).AutoWrapText(true)
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			SNew(STextBlock)
			.Text(LOCTEXT("GraphicsHint",
				"These only change the view you watch: the LiDAR renders its own images of the scene, and its "
				"scans stay the same at every setting. Low at 25% is the cheapest. Hiding the view while the "
				"pipeline drives gives its GPU and CPU time back (the HUD and the Esc box still show). Find the "
				"best tries each setting for a few seconds and keeps the best looking one that leaves the GPU "
				"room within each 1/60 s frame."))
			.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
		]
		+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("PipelineCode", "PIPELINE CODE")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				MakeChoice(LOCTEXT("RepoChange", "Change..."), []() { return false; }, [this]()
				{
					FString Picked;
					if (!FFSDSStackStarter::PickFolder(TEXT("The IFSSIM folder (with docker-compose.yml in it)"),
						RepoShown.IsEmpty() ? FFSDSStackStarter::SuggestedRepoDir() : RepoShown, Picked)) return;
					if (!FFSDSStackStarter::IsRepo(Picked))
					{
						RepoNote = FString::Printf(TEXT("%s has no docker-compose.yml: pick the IFSSIM folder itself."), *Picked);
						return;
					}
					FFSDSStackStarter::SaveRepoDir(Picked);
					RepoShown = FFSDSStackStarter::FindComposeDir();
					RepoNote = RepoShown == Picked ? FString()
						: TEXT("Saved, but settings.json's MissionControl.ComposeDir or IFSSIM_ROOT points elsewhere and wins.");
					OnTracksChanged.ExecuteIfBound();   // its tracks
				})
			]
			+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center).Padding(12, 0, 0, 0)
			[
				SNew(STextBlock)
				.Text_Lambda([this]()
				{
					if (RepoShown.IsEmpty()) return FText::FromString(TEXT("Not on this PC. START with Autonomous offers to get it."));
					FString Shown = RepoShown;
					FPaths::MakePlatformFilename(Shown);   // C:\Users\..., as Windows shows it
					return FText::FromString(Shown);
				})
				.Font(MenuUi::MenuFont(11)).ColorAndOpacity(MenuUi::Ink).AutoWrapText(true)
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			SNew(STextBlock)
			.Text_Lambda([this]()
			{
				return FText::FromString(RepoNote.IsEmpty()
					? FString(TEXT("The team's code (the IFSSIM repo): the pipeline and Mission Control's backend run from it in Docker, "
						"and START with Autonomous starts them."))
					: RepoNote);
			})
			.Font(MenuUi::MenuFont(10)).ColorAndOpacity_Lambda([this]() { return FSlateColor(RepoNote.IsEmpty() ? MenuUi::Muted : MenuUi::Bad); })
			.AutoWrapText(true)
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				MakeChoice(LOCTEXT("Rebuild", "Rebuild pipeline"),
					[this]() { return IsRebuilding && IsRebuilding(); },
					[this]() { OnRebuildPipeline.ExecuteIfBound(); })
			]
			+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center).Padding(12, 0, 0, 0)
			[
				SNew(STextBlock)
				.Text_Lambda([this]() { return FText::FromString(RebuildStatus ? RebuildStatus() : FString()); })
				.ColorAndOpacity_Lambda([this]() { return FSlateColor(RebuildFailed && RebuildFailed() ? MenuUi::Bad : MenuUi::Good); })
				.Font(MenuUi::MenuFont(10)).AutoWrapText(true)
			]
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
		[
			SNew(STextBlock)
			.Text(LOCTEXT("RebuildHint",
				"Python changes in pipeline/ need nothing: START copies your pipeline code into the container before "
				"every run. Rebuild after changing C++, messages, launch files or a setup.py, or after a git pull: it "
				"builds the pipeline and the backend from your code (seconds to minutes) and restarts them."))
			.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
		]
		+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("Config", "CONFIGURATION")) ]
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(STextBlock)
			.Text(LOCTEXT("ConfigHint",
				"The car, the sensors and the keys live in settings.json. Start the sim with -NoMenu "
				"to skip this menu, or set \"Menu\": {\"Enabled\": false}."))
			.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
		];
}

// ---------------------------------------------------------------------------
// Pause and Finished

TSharedRef<SWidget> SFSDSMenu::BuildRunBox()
{
	const bool bPause = Screen == EScreen::Pause;
	const FString What = Last.IsSet()
		? FString::Printf(TEXT("%s  ·  %s"), *Last->Track.Name, *FFSDSMenu::EventName(Last->Event, Last->bPractice))
		: FString();

	auto Stat = [this](const FText& Label, TFunction<FString(const FFSDSRefereeState&)> Value) -> TSharedRef<SWidget>
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SBox).WidthOverride(130.f)
				[
					SNew(STextBlock).Text(Label).Font(MenuUi::MenuFont(11)).ColorAndOpacity(MenuUi::Muted)
				]
			]
			+ SHorizontalBox::Slot().FillWidth(1.f)
			[
				SNew(STextBlock)
				.Text_Lambda([this, Value]()
				{
					const AFSDSReferee* R = Referee.Get();
					return R ? FText::FromString(Value(R->GetState())) : FText::GetEmpty();
				})
				.Font(MenuUi::MenuFont(11, TEXT("Bold"))).ColorAndOpacity(MenuUi::Ink)
			];
	};

	TSharedRef<SVerticalBox> Buttons = SNew(SVerticalBox);
	if (bPause)
	{
		Buttons->AddSlot().AutoHeight().Padding(0, 4)
		[
			MakeBigButton(Last.IsSet() && Last->bAutonomous ? LOCTEXT("BackToRun", "Back to the run") : LOCTEXT("Resume", "Resume"),
			MenuUi::GoBtn, OnResume)
		];
	}
	Buttons->AddSlot().AutoHeight().Padding(0, 4)
	[
		MakeBigButton(bPause ? LOCTEXT("Restart", "Restart run") : LOCTEXT("Again", "Drive again"),
			bPause ? MenuUi::SecondBtn : MenuUi::GoBtn, OnRestart)
	];
	Buttons->AddSlot().AutoHeight().Padding(0, 4)
	[
		MakeBigButton(LOCTEXT("MainMenu", "Main menu"), MenuUi::SecondBtn, OnMainMenu)
	];

	return SNew(SOverlay)
		+ SOverlay::Slot()
		[
			SNew(SBorder).BorderImage(MenuUi::White()).BorderBackgroundColor(FLinearColor(0.f, 0.f, 0.f, 0.45f))
		]
		+ SOverlay::Slot().HAlign(HAlign_Center).VAlign(VAlign_Center)
		[
			SNew(SBox).WidthOverride(440.f)
			[
				SNew(SBorder)
				.BorderImage(MenuUi::White())
				.BorderBackgroundColor(MenuUi::Ground)
				.Padding(FMargin(28.f, 24.f))
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[
						SNew(STextBlock)
						.Text(!bPause && Last.IsSet() && Last->bAutonomous && Results.IsValid() && Results->Num() > 0 && !Results->Last().bFinished
							? LOCTEXT("RunStopped", "RUN STOPPED")
							: !bPause ? LOCTEXT("RunDone", "RUN FINISHED")
							: Last.IsSet() && Last->bAutonomous ? LOCTEXT("AutoRunning", "THE PIPELINE IS DRIVING")
							: LOCTEXT("Paused", "PAUSED"))
						.Font(MenuUi::MenuFont(22, TEXT("Bold"))).ColorAndOpacity(MenuUi::Accent)
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
					[
						SNew(STextBlock).Text(FText::FromString(What)).Font(MenuUi::MenuFont(11)).ColorAndOpacity(MenuUi::Muted)
					]
					+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("ThisRun", "THIS RUN")) ]
					+ SVerticalBox::Slot().AutoHeight()
					[
						Stat(LOCTEXT("SLaps", "Laps"), [bP = Last.IsSet() && Last->bPractice](const FFSDSRefereeState& S)
						{
							return bP ? FString::FromInt(S.Laps.Num())
								: FString::Printf(TEXT("%d / %d"), S.Laps.Num(), S.RequiredLaps);
						})
					]
					+ SVerticalBox::Slot().AutoHeight()
					[
						Stat(LOCTEXT("SBest", "Best lap"), [](const FFSDSRefereeState& S)
						{
							if (S.Laps.Num() == 0) return FString(TEXT("--"));
							return MenuUi::Seconds(FMath::Min(S.Laps));
						})
					]
					+ SVerticalBox::Slot().AutoHeight()
					[
						Stat(LOCTEXT("STotal", "Total"), [](const FFSDSRefereeState& S)
						{
							if (S.Laps.Num() == 0) return FString(TEXT("--"));
							float T = 0.f;
							for (float L : S.Laps) T += L;
							return MenuUi::Seconds(T);
						})
					]
					+ SVerticalBox::Slot().AutoHeight()
					[
						Stat(LOCTEXT("SCones", "Cones down"), [](const FFSDSRefereeState& S)
						{
							return FString::FromInt(S.DooCounter);
						})
					]
					+ SVerticalBox::Slot().AutoHeight()
					[
						Stat(LOCTEXT("SOff", "Off course"), [](const FFSDSRefereeState& S)
						{
							return FString::FromInt(S.OffTrackCounter);
						})
					]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
						[
							SNew(STextBlock)
							.Visibility_Lambda([this, bPause]()
							{
								return !bPause && Last.IsSet() && Last->bAutonomous ? EVisibility::Visible : EVisibility::Collapsed;
							})
							.Text_Lambda([this]()
							{
								if (!Results.IsValid() || Results->Num() == 0) return FText::GetEmpty();
								const FFSDSRunResult& R = Results->Last();
								// A run stopped short has no FS time worth showing.
								FString S = !R.bFinished
									? FString(TEXT("The car was stopped before the finish (EBS). RES shows in the session log "
										"(Results); otherwise the pipeline stopped it itself, its safety watchdog or a failure: "
										"its own log says why (docker compose logs dv_pipeline_stack)."))
									: R.bScored ? ScoreText(R) : FString(TEXT("Asking Mission Control for the score..."));
								if (R.bBag) S += TEXT("\nBag recorded (the bags folder of the team's code).");
								return FText::FromString(S);
							})
							.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Ink).AutoWrapText(true)
						]
						// The box over an autonomous run: the emergency stop and the sim's pause.
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 14, 0, 0)
						[
							SNew(SVerticalBox)
							.Visibility_Lambda([this, bPause]()
							{
								return bPause && Last.IsSet() && Last->bAutonomous && Client.IsValid() ? EVisibility::Visible : EVisibility::Collapsed;
							})
							+ SVerticalBox::Slot().AutoHeight()
							[
								SNew(SHorizontalBox)
								+ SHorizontalBox::Slot().FillWidth(1.f).Padding(0, 0, 3, 0)
								[
									MakeChoice(TAttribute<FText>::CreateLambda([this]()
										{
											return bResActive ? LOCTEXT("ResRelease", "Release RES") : LOCTEXT("ResStop", "RES: stop the car");
										}),
										[this]() { return bResActive; },
										[this]()
										{
											const bool bRelease = bResActive;
											TWeakPtr<SFSDSMenu> Weak = SharedThis(this);
											Client->Post(bRelease ? TEXT("/api/res/release") : TEXT("/api/res/activate"), nullptr,
												[Weak, bRelease](const FFSDSMcReply& R)
											{
												TSharedPtr<SFSDSMenu> M = Weak.Pin();
												if (!M) return;
												if (R.Ok()) M->bResActive = !bRelease;
												M->RunNote = R.Ok() ? (bRelease ? TEXT("RES released.") : TEXT("RES active: the car is held."))
													: FString::Printf(TEXT("RES failed: %s"), *R.Error);
											}, 10.f);
										})
								]
								+ SHorizontalBox::Slot().FillWidth(1.f)
								[
									MakeChoice(TAttribute<FText>::CreateLambda([this]()
										{
											return bSimPaused ? LOCTEXT("SimResume", "Resume the sim") : LOCTEXT("SimPause", "Pause the sim");
										}),
										[this]() { return bSimPaused; },
										[this]()
										{
											const bool bResume = bSimPaused;
											TWeakPtr<SFSDSMenu> Weak = SharedThis(this);
											Client->Post(bResume ? TEXT("/api/sim/resume") : TEXT("/api/sim/pause"), nullptr,
												[Weak, bResume](const FFSDSMcReply& R)
											{
												TSharedPtr<SFSDSMenu> M = Weak.Pin();
												if (!M) return;
												if (R.Ok()) M->bSimPaused = !bResume;
												M->RunNote = R.Ok() ? (bResume ? TEXT("Resumed.") : TEXT("Paused: the sim and the pipeline's clock stand still."))
													: FString::Printf(TEXT("Failed: %s"), *R.Error);
											}, 10.f);
										})
								]
							]
							+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
							[
								SNew(STextBlock).Text_Lambda([this]() { return FText::FromString(RunNote); })
								.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
							]
						]
						+ SVerticalBox::Slot().AutoHeight().Padding(0, 18, 0, 0)
						[
							Buttons
						]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
					[
						SNew(STextBlock)
						.Text(!bPause ? FText::GetEmpty()
							: Last.IsSet() && Last->bAutonomous ? LOCTEXT("EscBack", "The run goes on behind this box. Main menu stops it.")
							: LOCTEXT("EscResume", "Esc: resume"))
						.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted)
					]
				]
			]
		];
}

// ---------------------------------------------------------------------------
// Tracks

void SFSDSMenu::SetLocation(const FString& InLocation)
{
	Location = InLocation;
	// A venue has no acceleration straight or skidpad: go to one of its laps.
	if (!Location.IsEmpty() && !bPractice
		&& (Event == EFSDSEventType::Acceleration || Event == EFSDSEventType::Skidpad))
	{
		Event = EFSDSEventType::Autocross;
	}
	SetEvent(Event, bPractice);
}

TSharedPtr<const FSDSVenue::FVenue> SFSDSMenu::FindVenue(const FString& Name) const
{
	for (const FSDSVenue::FVenueRef& V : Venues)
	{
		if (V->Name == Name) return V;
	}
	return nullptr;
}

bool SFSDSMenu::FitsEvent(const FFSDSMenuTrack& Track) const
{
	const FString Name = Track.Name.ToLower();
	const bool bAccel = Name.Contains(TEXT("acceleration"));
	const bool bSkid = Name.Contains(TEXT("skidpad"));
	if (bPractice) return true;
	switch (Event)
	{
	case EFSDSEventType::Acceleration: return bAccel;
	case EFSDSEventType::Skidpad:      return bSkid;
	// One run: a lap of a loop, or an open track from its start gate to its
	// finish gate (the referee times both).
	case EFSDSEventType::Autocross:    return !bAccel && !bSkid;
	default:                           return !bAccel && !bSkid && Track.bClosed;   // laps need a loop
	}
}

bool SFSDSMenu::Fits(const FFSDSMenuTrack& Track) const
{
	if (!Location.IsEmpty() && Track.Venue != Location) return false;
	if (!Search.IsEmpty() && !Track.Name.Contains(Search)) return false;   // ignores case
	return FitsEvent(Track);
}

TArray<int32> SFSDSMenu::ShownTracks() const
{
	TArray<int32> Out;
	if (!Location.IsEmpty() || Venues.Num() == 0)
	{
		for (int32 i = 0; i < Tracks.Num(); ++i)
		{
			if (Fits(Tracks[i])) Out.Add(i);
		}
		return Out;
	}
	// Anywhere: the open floor's tracks, then each venue's, in the venues' order.
	TArray<FString> Groups = { FString() };
	for (const FSDSVenue::FVenueRef& V : Venues) Groups.Add(V->Name);
	for (const FString& G : Groups)
	{
		for (int32 i = 0; i < Tracks.Num(); ++i)
		{
			if (Tracks[i].Venue == G && Fits(Tracks[i])) Out.Add(i);
		}
	}
	return Out;
}

int32 SFSDSMenu::CountAt(const FString& Venue) const
{
	int32 N = 0;
	for (const FFSDSMenuTrack& T : Tracks)
	{
		if ((Venue.IsEmpty() || T.Venue == Venue) && FitsEvent(T)) ++N;
	}
	return N;
}

FText SFSDSMenu::EventHint() const
{
	if (bPractice) return LOCTEXT("HPractice", "Any track. Open tracks end at the finish gate; on a loop, laps are counted until you stop.");
	switch (Event)
	{
	case EFSDSEventType::Acceleration: return LOCTEXT("HAccel", "A 75 m straight from a standing start.");
	case EFSDSEventType::Skidpad:      return LOCTEXT("HSkid", "The figure of eight: two laps right, two left.");
	case EFSDSEventType::Autocross:    return LOCTEXT("HAuto", "One timed run: a lap of a closed track, or an open track from start to finish.");
	case EFSDSEventType::Trackdrive:   return LOCTEXT("HTrack", "Several laps of a closed track.");
	default:                           return FText::GetEmpty();
	}
}

void SFSDSMenu::RebuildTrackList()
{
	if (!DriveList.IsValid()) return;
	DriveList->ClearChildren();
	const TArray<int32> Shown = ShownTracks();
	const bool bGroups = Location.IsEmpty() && Venues.Num() > 0;
	const FString* Group = nullptr;
	for (const int32 i : Shown)
	{
		const FFSDSMenuTrack& T = Tracks[i];
		if (bGroups && (!Group || *Group != T.Venue))
		{
			Group = &T.Venue;
			const TSharedPtr<const FSDSVenue::FVenue> V = FindVenue(T.Venue);
			const FString Title = V.IsValid() ? V->Title : TEXT("Open floor");
			DriveList->AddSlot().Padding(4, i == Shown[0] ? 0 : 10, 0, 3)
			[
				SNew(STextBlock).Text(FText::FromString(Title.ToUpper()))
				.Font(MenuUi::MenuFont(9, TEXT("Bold"))).ColorAndOpacity(MenuUi::Muted)
			];
		}
		const FString Counts = FString::Printf(TEXT("%d cones"), T.Blue + T.Yellow + T.Orange);
		DriveList->AddSlot().Padding(0, 1)
		[
			SNew(SButton)
			.IsFocusable(false)
			.ContentPadding(FMargin(10.f, 5.f))
			.ButtonColorAndOpacity_Lambda([this, i]()
			{
				return FSlateColor(Selected == i ? MenuUi::PlainBtn : MenuUi::Off);
			})
			.OnClicked_Lambda([this, i]()
			{
				SelectTrack(i);
				return FReply::Handled();
			})
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(1.f).Padding(0, 0, 12, 0)
				[
					SNew(STextBlock)
					.Text(FText::FromString(T.Name))
					.OverflowPolicy(ETextOverflowPolicy::Ellipsis)
					.Font(MenuUi::MenuFont(11, TEXT("Bold")))
					.ColorAndOpacity_Lambda([this, i]() { return FSlateColor(Selected == i ? MenuUi::Accent : MenuUi::Ink); })
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[
					SNew(STextBlock).Text(FText::FromString(Counts)).Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted)
				]
			]
		];
	}
	if (Shown.Num() == 0)
	{
		DriveList->AddSlot()
		[
			SNew(STextBlock).Text(Search.IsEmpty() ? LOCTEXT("NoTracks", "No track for this event in the tracks folders.")
				: FText::Format(LOCTEXT("NoMatch", "No track for this event matches \"{0}\"."), FText::FromString(Search)))
			.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Bad)
		];
	}
}

FString SFSDSMenu::ScoreText(const FFSDSRunResult& R)
{
	FString S = FString::Printf(TEXT("FS time %.2f s (cones +%.1f s, off course +%.1f s)"),
		R.Corrected, R.DooPenalty, R.OcPenalty);
	S += R.bHasReference
		? FString::Printf(TEXT("   %.1f of %.0f points (reference %.2f s)"), R.Points, R.MaxPoints, R.Reference)
		: FString(TEXT("   no points: set a reference time on the Drive page"));
	return S;
}

namespace MenuScoring { static const TCHAR* Section = TEXT("FSDSScoring"); }

FString SFSDSMenu::ReferenceKey() const
{
	if (!Tracks.IsValidIndex(Selected)) return FString();
	// Per event and track: T_best is the fastest time on that course.
	return FString::Printf(TEXT("Ref_%d_%s"), (int32)Event, *Tracks[Selected].Name);
}

const FString& SFSDSMenu::Reference() const
{
	const FString Key = ReferenceKey();
	if (Key != ReferenceFor)
	{
		ReferenceFor = Key;
		ReferenceText.Reset();
		if (GConfig && !Key.IsEmpty())
		{
			GConfig->GetString(MenuScoring::Section, *Key, ReferenceText, GGameUserSettingsIni);
		}
	}
	return ReferenceText;
}

void SFSDSMenu::SetReference(const FString& Seconds)
{
	const FString Key = ReferenceKey();
	if (Key.IsEmpty() || !GConfig) return;
	ReferenceFor = Key;
	ReferenceText = Seconds;
	if (Seconds.IsEmpty()) GConfig->RemoveKey(MenuScoring::Section, *Key, GGameUserSettingsIni);
	else GConfig->SetString(MenuScoring::Section, *Key, *Seconds, GGameUserSettingsIni);
	GConfig->Flush(false, GGameUserSettingsIni);
}

FString SFSDSMenu::WhyKept(const FString& Name)
{
	// Same rule as protected_reason in tools/mission_control/backend/track_validators.py.
	if (Name == TEXT("acceleration") || Name == TEXT("skidpad"))
		return TEXT("Official event track: it can't be deleted.");
	if (Name.StartsWith(TEXT("stress_")))
		return TEXT("Test track kept in the repo: it can't be deleted.");
	return FString();
}

void SFSDSMenu::DeleteTrack()
{
	if (bTrackBusy || !Client.IsValid() || !Tracks.IsValidIndex(Selected)) return;
	const FString Name = Tracks[Selected].Name;
	if (!WhyKept(Name).IsEmpty()) return;
	// Two clicks: the first arms it for 4 s.
	if (FPlatformTime::Seconds() >= DeleteArmedUntil || DeleteArmedFor != Name)
	{
		DeleteArmedFor = Name;
		DeleteArmedUntil = FPlatformTime::Seconds() + 4.0;
		return;
	}
	DeleteArmedUntil = 0.0;
	bTrackBusy = true;
	SetMessage(FString::Printf(TEXT("Deleting %s..."), *Name), false);
	TWeakPtr<SFSDSMenu> Weak = SharedThis(this);
	Client->Delete(FString::Printf(TEXT("/api/track/%s"), *FFSDSMissionControlClient::EncodeSegment(Name + TEXT(".csv"))),
		[Weak, Name](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSMenu> M = Weak.Pin();
		if (!M) return;
		M->bTrackBusy = false;
		if (!R.Ok())
		{
			// The backend says "Cannot delete: <why>"; drop its prefix so it doesn't repeat ours.
			FString Why = R.Error;
			Why.RemoveFromStart(TEXT("Cannot delete: "));
			M->SetMessage(FString::Printf(TEXT("Couldn't delete %s: %s"), *Name, *Why), true);
			return;
		}
		M->OnTracksChanged.ExecuteIfBound();
		M->SetMessage(FString::Printf(TEXT("Deleted %s."), *Name), false);
	});
}

void SFSDSMenu::FetchLog()
{
	if (bLogLoading || !Client.IsValid()) return;
	bLogLoading = true;
	LogNote = TEXT("Loading...");
	TWeakPtr<SFSDSMenu> Weak = SharedThis(this);
	Client->Get(TEXT("/api/session/log"), [Weak](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSMenu> M = Weak.Pin();
		if (!M) return;
		M->bLogLoading = false;
		const TArray<TSharedPtr<FJsonValue>>* Items = nullptr;
		if (!R.Ok() || !R.Body.IsValid() || !R.Body->TryGetArray(Items))
		{
			M->LogNote = FString::Printf(TEXT("Couldn't get the log: %s"), R.Error.IsEmpty() ? TEXT("malformed reply") : *R.Error);
			return;
		}
		M->LogLines.Reset();
		for (const TSharedPtr<FJsonValue>& Item : *Items)
		{
			const TSharedPtr<FJsonObject>* O = nullptr;
			if (!Item.IsValid() || !Item->TryGetObject(O)) continue;
			FLogLine L;
			FString Stamp;
			(*O)->TryGetStringField(TEXT("timestamp"), Stamp);
			(*O)->TryGetStringField(TEXT("type"), L.Type);
			(*O)->TryGetStringField(TEXT("message"), L.Text);
			int32 T = INDEX_NONE;
			L.Time = Stamp.FindChar(TEXT('T'), T) ? Stamp.Mid(T + 1, 8) : Stamp;   // as the web UI shows it
			M->LogLines.Add(MoveTemp(L));
		}
		M->LogNote = FString::Printf(TEXT("%d events in this session."), M->LogLines.Num());
		M->RebuildLogBox();
	});
}

void SFSDSMenu::RebuildLogBox()
{
	if (!LogBox.IsValid()) return;
	LogBox->ClearChildren();
	// Newest first, as the runs.
	for (int32 i = LogLines.Num() - 1; i >= 0; --i)
	{
		const FLogLine& L = LogLines[i];
		LogBox->AddSlot().AutoHeight().Padding(0, 1)
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SBox).WidthOverride(70.f)
				[
					SNew(STextBlock).Text(FText::FromString(L.Time)).Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted)
				]
			]
			+ SHorizontalBox::Slot().AutoWidth()
			[
				SNew(SBox).WidthOverride(110.f)
				[
					SNew(STextBlock).Text(FText::FromString(L.Type)).Font(MenuUi::MenuFont(10, TEXT("Bold")))
					.ColorAndOpacity(L.Type.Contains(TEXT("error")) ? MenuUi::Bad : MenuUi::Ink)
				]
			]
			+ SHorizontalBox::Slot().FillWidth(1.f)
			[
				SNew(STextBlock).Text(FText::FromString(L.Text)).Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Ink).AutoWrapText(true)
			]
		];
	}
}

void SFSDSMenu::SaveLog()
{
	if (!Client.IsValid()) return;
	LogNote = TEXT("Saving...");
	TWeakPtr<SFSDSMenu> Weak = SharedThis(this);
	Client->Get(TEXT("/api/session/export"), [Weak](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSMenu> M = Weak.Pin();
		if (!M) return;
		const TArray<TSharedPtr<FJsonValue>>* Items = nullptr;
		if (!R.Ok() || !R.Body.IsValid() || !R.Body->TryGetArray(Items))
		{
			M->LogNote = FString::Printf(TEXT("Couldn't save the log: %s"), R.Error.IsEmpty() ? TEXT("malformed reply") : *R.Error);
			return;
		}
		// Where the panel saves it too: Documents/IFSSIM/SessionLogs.
		FString Json;
		const TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer =
			TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&Json);
		FJsonSerializer::Serialize(*Items, Writer);
		FString File = FPaths::ConvertRelativePathToFull(FPaths::Combine(FPlatformProcess::UserDir(), TEXT("IFSSIM"), TEXT("SessionLogs"),
			FString::Printf(TEXT("session_%s.json"), *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")))));
		const bool bOk = FFileHelper::SaveStringToFile(Json, *File, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
		FPaths::MakePlatformFilename(File);
		M->LogNote = bOk ? FString::Printf(TEXT("Saved %d events to %s"), Items->Num(), *File)
			: FString::Printf(TEXT("Couldn't write %s"), *File);
	});
}

void SFSDSMenu::SetTracks(const TArray<FFSDSMenuTrack>& InTracks, const TArray<FSDSVenue::FVenueRef>& InVenues)
{
	// Keep the selected track by name: indices shift when one is added.
	FString Keep = Tracks.IsValidIndex(Selected) ? Tracks[Selected].Name : FString();
	Tracks = InTracks;
	Venues = InVenues;
	if (!Location.IsEmpty() && !FindVenue(Location).IsValid()) Location.Reset();
	if (LocationPicker.IsValid()) LocationPicker->SetVenues(Venues);
	Selected = Tracks.IndexOfByPredicate([&](const FFSDSMenuTrack& T) { return T.Name == Keep; });
	if (EditorPage.IsValid()) EditorPage->SetTracks(Tracks, Venues);
	SetEvent(Event, bPractice);   // redraws the list, and picks a track if the old one went
}

void SFSDSMenu::SelectTrack(int32 Index)
{
	Selected = Tracks.IsValidIndex(Index) ? Index : INDEX_NONE;
	Message.Reset();

	TArray<SFSDSTrackMap::FCone> Cones;
	if (Selected != INDEX_NONE)
	{
		SFSDSTrackMap::LoadCsv(Tracks[Selected].Path, Cones);
	}
	if (DriveMap.IsValid())
	{
		// A venue track shows its surroundings, placed by its sidecar, and
		// the clutter of its own the sidecar adds near its cones.
		TSharedPtr<const FSDSVenue::FVenue> V = Selected != INDEX_NONE ? FindVenue(Tracks[Selected].Venue) : nullptr;
		FVector2D Shift = FVector2D::ZeroVector;
		TArray<FSDSVenue::FProp> Own;
		if (V.IsValid() && !FSDSVenue::TrackShift(*V, Tracks[Selected].Path, Shift, &Own)) V.Reset();
		if (V.IsValid() && Own.Num() > 0)
		{
			TSharedRef<FSDSVenue::FVenue> WithOwn = MakeShared<FSDSVenue::FVenue>(*V);
			WithOwn->Props.Append(Own);
			V = WithOwn;
		}
		DriveMap->SetVenue(V, Shift);
		DriveMap->SetCones(MoveTemp(Cones));
	}
}

void SFSDSMenu::SetEvent(EFSDSEventType InEvent, bool bInPractice)
{
	Event = InEvent;
	bPractice = bInPractice;
	if (bPractice) bAutonomous = false;   // Practice is manual
	Message.Reset();
	RebuildTrackList();

	// Keep the track if it is still shown, else the first one that is.
	if (!Tracks.IsValidIndex(Selected) || !Fits(Tracks[Selected]))
	{
		const TArray<int32> Shown = ShownTracks();
		Selected = Shown.Num() > 0 ? Shown[0] : INDEX_NONE;
	}
	SelectTrack(Selected);
}

FText SFSDSMenu::TrackSummary(int32 Index) const
{
	if (!Tracks.IsValidIndex(Index)) return FText::GetEmpty();
	const FFSDSMenuTrack& T = Tracks[Index];
	FString S = T.Name;   // the counts are in the map's legend
	if (const TSharedPtr<const FSDSVenue::FVenue> V = FindVenue(T.Venue)) S += TEXT("   ·   ") + V->Title;
	if (T.bDrawn) S += TEXT("   ·   made with the track creator");
	if (!T.bClosed) S += TEXT("   ·   open: Autocross or Practice");
	return FText::FromString(S);
}

#undef LOCTEXT_NAMESPACE
