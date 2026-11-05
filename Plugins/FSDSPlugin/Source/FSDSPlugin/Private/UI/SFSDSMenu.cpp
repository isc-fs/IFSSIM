#include "UI/SFSDSMenu.h"

#include "UI/SFSDSTrackMap.h"

#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Layout/SSpacer.h"
#include "Widgets/Layout/SUniformGridPanel.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
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
		Selected = Tracks.IndexOfByPredicate([this](const FFSDSMenuTrack& T) { return T.Name == Last->Track.Name; });
		Location = Last->Track.Venue;
	}

	ChildSlot
	[
		Screen == EScreen::Main ? BuildMain() : BuildRunBox()
	];

	if (Screen == EScreen::Main)
	{
		SetEvent(Event, bPractice);   // fills the list, keeps the last track if it fits
	}
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

TSharedRef<SWidget> SFSDSMenu::SpeedChoice(const FText& Label, double Speed)
{
	return MakeChoice(Label,
		[Speed]() { return FMath::IsNearlyEqual(FFSDSRealTime::Get().SpeedFactor(), Speed, 1e-3); },
		[Speed]() { FFSDSRealTime::Get().SetSpeedFactor(Speed); });
}

TSharedRef<SWidget> SFSDSMenu::MakeChoice(const FText& Label, TFunction<bool()> IsOn, TFunction<void()> OnPick)
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
								[this]() { return bAutonomous; }, [this]() { bAutonomous = true; })
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
								"The DV pipeline drives. START opens Mission Control ({0}), where the session starts; "
								"it needs the Docker stack."), LauncherKey)
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
			*R.When.ToString(TEXT("%H:%M")), *R.Track, *FFSDSMenu::EventName(R.Event, R.bPractice));
		FString Body = R.bPractice ? FString::Printf(TEXT("Laps %d"), R.Laps.Num())
			: FString::Printf(TEXT("Laps %d/%d"), R.Laps.Num(), R.RequiredLaps);
		if (R.Laps.Num() > 0)
		{
			Body += FString::Printf(TEXT("   best %s   total %s"), *MenuUi::Seconds(Best), *MenuUi::Seconds(Total));
		}
		Body += FString::Printf(TEXT("   cones down %d   off course %d"), R.ConesDown, R.OffCourse);

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
				"Manual runs since the sim started, newest first. Manual runs get no FS score yet; "
				"autonomous sessions are scored in Mission Control."))
			.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
		]
		+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("Runs", "RUNS")) ]
		+ SVerticalBox::Slot().FillHeight(1.f)
		[
			SNew(SScrollBox) + SScrollBox::Slot() [ List ]
		];
}

TSharedRef<SWidget> SFSDSMenu::BuildSettingsPage()
{
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
			MakeBigButton(LOCTEXT("Resume", "Resume"), MenuUi::GoBtn, OnResume)
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
						.Text(bPause ? LOCTEXT("Paused", "PAUSED") : LOCTEXT("RunDone", "RUN FINISHED"))
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
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 18, 0, 0)
					[
						Buttons
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
					[
						SNew(STextBlock)
						.Text(bPause ? LOCTEXT("EscResume", "Esc: resume") : FText::GetEmpty())
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

void SFSDSMenu::SetTracks(const TArray<FFSDSMenuTrack>& InTracks, const TArray<FSDSVenue::FVenueRef>& InVenues)
{
	// Keep the selected track by name: indices shift when one is added.
	const FString Keep = Tracks.IsValidIndex(Selected) ? Tracks[Selected].Name : FString();
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
