#include "UI/SFSDSMenu.h"

#include "UI/SFSDSTrackMap.h"

#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
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
						SNew(SFSDSTrackEditor)
						.State(EditorState)
						.Client(Client)
						.Tracks(Tracks)
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
				+ SVerticalBox::Slot().FillHeight(1.f)
				[
					SAssignNew(DriveList, SScrollBox)
				]
				+ SVerticalBox::Slot().AutoHeight() [ MakeHeading(LOCTEXT("Driver", "DRIVER")) ]
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SUniformGridPanel).SlotPadding(FMargin(3.f))
					+ SUniformGridPanel::Slot(0, 0)
					[
						MakeChoice(LOCTEXT("Manual", "Manual (keyboard)"),
							[this]() { return !bAutonomous; }, [this]() { bAutonomous = false; })
					]
					+ SUniformGridPanel::Slot(1, 0)
					[
						MakeChoice(LOCTEXT("Autonomous", "Autonomous"),
							[this]() { return bAutonomous; }, [this]() { bAutonomous = true; })
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
							: LOCTEXT("ManualHint", "You drive with the keyboard or a gamepad. Esc pauses.");
					})
					.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Muted).AutoWrapText(true)
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

bool SFSDSMenu::Fits(const FFSDSMenuTrack& Track) const
{
	const FString Name = Track.Name.ToLower();
	const bool bAccel = Name.Contains(TEXT("acceleration"));
	const bool bSkid = Name.Contains(TEXT("skidpad"));
	if (bPractice) return true;
	switch (Event)
	{
	case EFSDSEventType::Acceleration: return bAccel;
	case EFSDSEventType::Skidpad:      return bSkid;
	default:                           return !bAccel && !bSkid && Track.bClosed;   // a lap needs a loop
	}
}

FText SFSDSMenu::EventHint() const
{
	if (bPractice) return LOCTEXT("HPractice", "Any track. Open tracks end at the finish gate; on a loop, laps are counted until you stop.");
	switch (Event)
	{
	case EFSDSEventType::Acceleration: return LOCTEXT("HAccel", "A 75 m straight from a standing start.");
	case EFSDSEventType::Skidpad:      return LOCTEXT("HSkid", "The figure of eight: two laps right, two left.");
	case EFSDSEventType::Autocross:    return LOCTEXT("HAuto", "One timed lap of a closed track.");
	case EFSDSEventType::Trackdrive:   return LOCTEXT("HTrack", "Several laps of a closed track.");
	default:                           return FText::GetEmpty();
	}
}

void SFSDSMenu::RebuildTrackList()
{
	if (!DriveList.IsValid()) return;
	DriveList->ClearChildren();
	int32 Shown = 0;
	for (int32 i = 0; i < Tracks.Num(); ++i)
	{
		const FFSDSMenuTrack& T = Tracks[i];
		if (!Fits(T)) continue;
		++Shown;
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
	if (Shown == 0)
	{
		DriveList->AddSlot()
		[
			SNew(STextBlock).Text(LOCTEXT("NoTracks", "No track for this event in the tracks folders."))
			.Font(MenuUi::MenuFont(10)).ColorAndOpacity(MenuUi::Bad)
		];
	}
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
	if (DriveMap.IsValid()) DriveMap->SetCones(MoveTemp(Cones));
}

void SFSDSMenu::SetEvent(EFSDSEventType InEvent, bool bInPractice)
{
	Event = InEvent;
	bPractice = bInPractice;
	Message.Reset();
	RebuildTrackList();

	// Keep the track if it suits the event, else the first one that does.
	if (!Tracks.IsValidIndex(Selected) || !Fits(Tracks[Selected]))
	{
		Selected = Tracks.IndexOfByPredicate([this](const FFSDSMenuTrack& T) { return Fits(T); });
	}
	SelectTrack(Selected);
}

FText SFSDSMenu::TrackSummary(int32 Index) const
{
	if (!Tracks.IsValidIndex(Index)) return FText::GetEmpty();
	const FFSDSMenuTrack& T = Tracks[Index];
	FString S = T.Name;   // the counts are in the map's legend
	if (T.bDrawn) S += TEXT("   ·   made with the track creator");
	if (!T.bClosed) S += TEXT("   ·   open, Practice only");
	return FText::FromString(S);
}

#undef LOCTEXT_NAMESPACE
