#include "UI/SFSDSTrackEditor.h"

#include "UI/FSDSMissionControlClient.h"
#include "UI/SFSDSTrackCanvas.h"

#include "Misc/Paths.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SCheckBox.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SSeparator.h"
#include "Widgets/Layout/SUniformGridPanel.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "FSDSTrackEditor"

// Always qualified (EditorUi::...): other UI files share the unity blob.
namespace EditorUi
{
	// The menu's palette.
	const FLinearColor Accent = FLinearColor::FromSRGBColor(FColor(255, 184, 28));
	const FLinearColor Ink = FLinearColor::FromSRGBColor(FColor(232, 232, 236));
	const FLinearColor Muted = FLinearColor::FromSRGBColor(FColor(150, 150, 160));
	const FLinearColor Good = FLinearColor::FromSRGBColor(FColor(74, 222, 128));
	const FLinearColor Warn = FLinearColor::FromSRGBColor(FColor(251, 191, 36));
	const FLinearColor Bad = FLinearColor::FromSRGBColor(FColor(248, 113, 113));
	const FLinearColor Box(1.f, 1.f, 1.f, 0.04f);
	const FLinearColor PlainBtn = FLinearColor::FromSRGBColor(FColor(64, 64, 72));
	const FLinearColor SecondBtn = FLinearColor::FromSRGBColor(FColor(130, 130, 145));
	const FLinearColor GoBtn = FLinearColor::FromSRGBColor(FColor(34, 197, 94));
	const FLinearColor SaveBtn = FLinearColor::FromSRGBColor(FColor(255, 184, 28));
	const FLinearColor Off(0.f, 0.f, 0.f, 0.f);

	constexpr double DebounceS = 0.25;
	constexpr double ArmS = 4.0;

	FSlateFontInfo EdFont(int32 Size, const TCHAR* Weight = TEXT("Regular"))
	{
		return FCoreStyle::GetDefaultFontStyle(Weight, Size);
	}

	TSharedRef<SWidget> Heading(const FText& Title)
	{
		return SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 12, 0, 5)
			[
				SNew(SSeparator).Thickness(1.f).ColorAndOpacity(FLinearColor(1, 1, 1, 0.08f))
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 0, 0, 6)
			[
				SNew(STextBlock).Text(Title).Font(EdFont(10, TEXT("Bold"))).ColorAndOpacity(Accent)
			];
	}

	TSharedRef<SWidget> Button(TAttribute<FText> Label, FLinearColor Color, TFunction<void()> OnClick,
		TAttribute<bool> Enabled = true, int32 Size = 10)
	{
		return SNew(SButton)
			.IsFocusable(false)
			.ButtonColorAndOpacity(Color)
			.ContentPadding(FMargin(10.f, 5.f))
			.HAlign(HAlign_Center)
			.IsEnabled(Enabled)
			.OnClicked_Lambda([OnClick]() { OnClick(); return FReply::Handled(); })
			[
				SNew(STextBlock).Text(Label).Font(EdFont(Size, TEXT("Bold"))).ColorAndOpacity(FLinearColor::White)
			];
	}

	/** A toggle that reads as on (accent text) or off, like the menu's choices. */
	TSharedRef<SWidget> Choice(const FText& Label, TFunction<bool()> IsOn, TFunction<void()> OnPick)
	{
		return SNew(SButton)
			.IsFocusable(false)
			.ContentPadding(FMargin(8.f, 4.f))
			.HAlign(HAlign_Center)
			.ButtonColorAndOpacity_Lambda([IsOn]() { return FSlateColor(IsOn() ? PlainBtn : FLinearColor(1, 1, 1, 0.06f)); })
			.OnClicked_Lambda([OnPick]() { OnPick(); return FReply::Handled(); })
			[
				SNew(STextBlock).Text(Label).Font(EdFont(10, TEXT("Bold")))
				.ColorAndOpacity_Lambda([IsOn]() { return FSlateColor(IsOn() ? Accent : Muted); })
			];
	}

	/** A labelled number box. */
	template <typename T>
	TSharedRef<SWidget> Num(const FText& Label, TFunction<T()> Get, TFunction<void(T)> Set, T Min, T Max, T Delta, float Width = 76.f)
	{
		return SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight()
			[
				SNew(STextBlock).Text(Label).Font(EdFont(8, TEXT("Bold"))).ColorAndOpacity(Muted)
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
			[
				SNew(SBox).WidthOverride(Width)
				[
					SNew(SSpinBox<T>)
					.MinValue(Min).MaxValue(Max).Delta(Delta)
					.Font(EdFont(10))
					.Value_Lambda([Get]() { return Get(); })
					.OnValueChanged_Lambda([Set](T V) { Set(V); })
				]
			];
	}

	/** A number that may be unset (empty means "the track's own"). */
	TSharedRef<SWidget> OptNum(const FText& Label, TFunction<TOptional<float>()> Get, TFunction<void(TOptional<float>)> Set,
		float Width = 64.f)
	{
		return SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight()
			[
				SNew(STextBlock).Text(Label).Font(EdFont(8, TEXT("Bold"))).ColorAndOpacity(Muted)
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
			[
				SNew(SBox).WidthOverride(Width)
				[
					SNew(SEditableTextBox)
					.Font(EdFont(10))
					.HintText(LOCTEXT("Own", "track's"))
					.Text_Lambda([Get]()
					{
						const TOptional<float> V = Get();
						return V.IsSet() ? FText::AsNumber(*V) : FText::GetEmpty();
					})
					.OnTextCommitted_Lambda([Set](const FText& T, ETextCommit::Type)
					{
						const FString S = T.ToString().TrimStartAndEnd();
						if (S.IsEmpty()) Set(TOptional<float>());
						else if (S.IsNumeric()) Set(FCString::Atof(*S));
					})
				]
			];
	}
}

void SFSDSTrackEditor::Construct(const FArguments& InArgs)
{
	State = InArgs._State.IsValid() ? InArgs._State : MakeShared<FSDSTrackSpec::FEditorState>();
	Client = InArgs._Client;
	Tracks = InArgs._Tracks;
	OnTestDrive = InArgs._OnTestDrive;
	OnSaved = InArgs._OnSaved;
	OnStartStack = InArgs._OnStartStack;

	ChildSlot
	[
		SNew(SHorizontalBox)
		+ SHorizontalBox::Slot().FillWidth(1.f) [ BuildLeft() ]
		+ SHorizontalBox::Slot().AutoWidth().Padding(20, 0, 0, 0)
		[
			SNew(SBox).WidthOverride(380.f) [ BuildRight() ]
		]
	];
	RebuildSections();

	// Is the backend there? The first preview tells too, but a blank page has none.
	if (Client.IsValid())
	{
		TWeakPtr<SFSDSTrackEditor> Weak = StaticCastSharedRef<SFSDSTrackEditor>(AsShared());
		Client->Get(TEXT("/api/track/list"), [Weak](const FFSDSMcReply& R)
		{
			if (TSharedPtr<SFSDSTrackEditor> E = Weak.Pin())
			{
				E->bBackendUp = R.Reached();
				E->bBackendKnown = true;
			}
		});
	}
	bDirty = State->Spec->Enough();   // a track left open: preview it again
	ChangedAt = 0.0;
	RegisterActiveTimer(0.1f, FWidgetActiveTimerDelegate::CreateSP(this, &SFSDSTrackEditor::Poll));
}

// ---------------------------------------------------------------------------
// Layout

TSharedRef<SWidget> SFSDSTrackEditor::BuildLeft()
{
	return SNew(SVerticalBox)
		+ SVerticalBox::Slot().AutoHeight()
		[
			SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
			[
				SNew(STextBlock).Text(LOCTEXT("Title", "Track editor"))
				.Font(EditorUi::EdFont(20, TEXT("Bold"))).ColorAndOpacity(EditorUi::Ink)
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(3, 0)
			[
				EditorUi::Choice(LOCTEXT("ModeClick", "Click points"),
					[this]() { return Canvas.IsValid() && !Canvas->IsFreehand(); },
					[this]() { if (Canvas) Canvas->SetFreehand(false); })
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(3, 0)
			[
				EditorUi::Choice(LOCTEXT("ModeFree", "Freehand"),
					[this]() { return Canvas.IsValid() && Canvas->IsFreehand(); },
					[this]() { if (Canvas) Canvas->SetFreehand(true); })
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(12, 0, 3, 0)
			[
				EditorUi::Button(LOCTEXT("Fit", "Fit view"), EditorUi::PlainBtn, [this]() { if (Canvas) Canvas->FitView(); })
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(3, 0)
			[
				EditorUi::Button(LOCTEXT("Clear", "Clear"), EditorUi::PlainBtn, [this]()
				{
					State->Spec->Points.Reset();
					State->Spec->Sections.Reset();
					if (Canvas) Canvas->ClearSelection();
					RebuildSections();
					SpecChanged();
				})
			]
			+ SHorizontalBox::Slot().AutoWidth().Padding(3, 0)
			[
				EditorUi::Button(LOCTEXT("New", "New track"), EditorUi::PlainBtn, [this]() { NewTrack(); })
			]
		]
		+ SVerticalBox::Slot().FillHeight(1.f).Padding(0, 10, 0, 0)
		[
			SAssignNew(Canvas, SFSDSTrackCanvas)
			.Spec(State->Spec)
			.OnChanged(FSimpleDelegate::CreateSP(this, &SFSDSTrackEditor::SpecChanged))
		]
		+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
		[
			SNew(STextBlock)
			.Text_Lambda([this]()
			{
				return Canvas.IsValid() && Canvas->IsFreehand()
					? LOCTEXT("HintFree", "Drag to draw the whole path in one stroke; it becomes control points you can then move in Click points.")
					: LOCTEXT("HintClick", "Click to add a point (after the selected one). Drag a point to move it, drag the background "
						"or right-drag to pan, scroll to zoom. Delete removes the selected point. Point 0 (green) is the start; "
						"the car drives toward point 1. The green square is the floor.");
			})
			.Font(EditorUi::EdFont(9)).ColorAndOpacity(EditorUi::Muted).AutoWrapText(true)
		];
}

TSharedRef<SWidget> SFSDSTrackEditor::BuildOpenList()
{
	TSharedRef<SVerticalBox> List = SNew(SVerticalBox);
	int32 N = 0;
	for (const FFSDSMenuTrack& T : Tracks)
	{
		if (!T.bDrawn) continue;
		++N;
		List->AddSlot().AutoHeight().Padding(0, 1)
		[
			SNew(SButton)
			.IsFocusable(false)
			.ContentPadding(FMargin(8.f, 3.f))
			.ButtonColorAndOpacity_Lambda([this, Name = T.Name]()
			{
				return FSlateColor(State->Name == Name ? EditorUi::PlainBtn : EditorUi::Off);
			})
			.OnClicked_Lambda([this, T]() { Open(T); return FReply::Handled(); })
			[
				SNew(STextBlock).Text(FText::FromString(T.Name)).Font(EditorUi::EdFont(10, TEXT("Bold")))
				.OverflowPolicy(ETextOverflowPolicy::Ellipsis)
				.ColorAndOpacity_Lambda([this, Name = T.Name]()
				{
					return FSlateColor(State->Name == Name ? EditorUi::Accent : EditorUi::Ink);
				})
			]
		];
	}
	if (N == 0)
	{
		List->AddSlot().AutoHeight()
		[
			SNew(STextBlock).Text(LOCTEXT("NoDrawn", "No drawn tracks yet.")).Font(EditorUi::EdFont(10)).ColorAndOpacity(EditorUi::Muted)
		];
	}
	return SNew(SBox).MaxDesiredHeight(130.f)
		[
			SNew(SScrollBox) + SScrollBox::Slot() [ List ]
		];
}

TSharedRef<SWidget> SFSDSTrackEditor::BuildRight()
{
	TSharedRef<FSDSTrackSpec::FSpec> SpecRef = State->Spec;

	return SNew(SScrollBox)
		+ SScrollBox::Slot()
		[
			SNew(SVerticalBox)
			+ SVerticalBox::Slot().AutoHeight() [ EditorUi::Heading(LOCTEXT("OpenH", "OPEN A DRAWN TRACK")) ]
			+ SVerticalBox::Slot().AutoHeight() [ BuildOpenList() ]

			+ SVerticalBox::Slot().AutoHeight() [ EditorUi::Heading(LOCTEXT("TrackH", "TRACK")) ]
			+ SVerticalBox::Slot().AutoHeight()
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 8, 0)
				[
					EditorUi::Num<float>(LOCTEXT("Width", "WIDTH (M)"), [SpecRef]() { return SpecRef->Width; },
						[this, SpecRef](float V) { SpecRef->Width = V; SpecChanged(); }, 2.f, 8.f, 0.1f)
				]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 8, 0)
				[
					EditorUi::Num<float>(LOCTEXT("Spacing", "SPACING (M)"), [SpecRef]() { return SpecRef->SpacingStraight; },
						[this, SpecRef](float V) { SpecRef->SpacingStraight = V; SpecChanged(); }, 1.f, 10.f, 0.5f)
				]
				+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 8, 0)
				[
					EditorUi::Num<float>(LOCTEXT("CornerMin", "CORNER MIN"), [SpecRef]() { return SpecRef->SpacingCornerMin; },
						[this, SpecRef](float V) { SpecRef->SpacingCornerMin = V; SpecChanged(); }, 0.5f, 5.f, 0.1f)
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[
					EditorUi::Num<int32>(LOCTEXT("Seed", "SEED"), [SpecRef]() { return SpecRef->Seed; },
						[this, SpecRef](int32 V) { SpecRef->Seed = FMath::Max(0, V); SpecChanged(); }, 0, 9999, 1, 60.f)
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
			[
				SNew(SCheckBox)
				.IsChecked_Lambda([SpecRef]() { return SpecRef->bClosed ? ECheckBoxState::Checked : ECheckBoxState::Unchecked; })
				.OnCheckStateChanged_Lambda([this, SpecRef](ECheckBoxState C)
				{
					SpecRef->bClosed = C == ECheckBoxState::Checked;
					SpecChanged();
				})
				[
					SNew(STextBlock).Text(LOCTEXT("Closed", "Closed loop (off: an open track, start to finish gate)"))
					.Font(EditorUi::EdFont(10)).ColorAndOpacity(EditorUi::Ink)
				]
			]

			+ SVerticalBox::Slot().AutoHeight() [ EditorUi::Heading(LOCTEXT("CheckH", "CHECK")) ]
			+ SVerticalBox::Slot().AutoHeight()
			[
				SNew(SBorder)
				.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
				.BorderBackgroundColor(EditorUi::Box)
				.Padding(FMargin(10.f, 8.f))
				[
					SNew(SVerticalBox)
					+ SVerticalBox::Slot().AutoHeight()
					[
						SNew(STextBlock)
						.Text_Lambda([this]() { return CheckText(); })
						.ColorAndOpacity_Lambda([this]() { return CheckColor(); })
						.Font(EditorUi::EdFont(10)).AutoWrapText(true)
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
					[
						SNew(SBox)
						.Visibility_Lambda([this]() { return bBackendKnown && !bBackendUp ? EVisibility::Visible : EVisibility::Collapsed; })
						[
							EditorUi::Button(LOCTEXT("StartStack", "Start the Docker stack"), EditorUi::PlainBtn,
								[this]() { OnStartStack.ExecuteIfBound(); Message = TEXT("Starting the stack: the editor connects when the backend is up (a minute or two)."); bMessageError = false; bBackendKnown = false; bDirty = true; })
						]
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0, 4, 0, 0)
					[
						SAssignNew(IssuesBox, SVerticalBox)
					]
				]
			]

			+ SVerticalBox::Slot().AutoHeight() [ EditorUi::Heading(LOCTEXT("SectionsH", "TEST SECTIONS")) ]
			+ SVerticalBox::Slot().AutoHeight()
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
				[
					SNew(STextBlock)
					.Text(LOCTEXT("SectionsHint", "One stretch, by point number: narrower, sparser, missing, wrong-coloured or misplaced cones."))
					.Font(EditorUi::EdFont(9)).ColorAndOpacity(EditorUi::Muted).AutoWrapText(true)
				]
				+ SHorizontalBox::Slot().AutoWidth().Padding(8, 0, 0, 0).VAlign(VAlign_Center)
				[
					EditorUi::Button(LOCTEXT("AddSection", "Add"), EditorUi::PlainBtn, [this, SpecRef]()
					{
						const int32 N = SpecRef->Points.Num();
						FSDSTrackSpec::FSection Sec;
						Sec.From = Canvas.IsValid() && Canvas->GetSelected() != INDEX_NONE ? Canvas->GetSelected() : 1;
						Sec.To = SpecRef->bClosed ? FMath::Min(Sec.From + 2, N) : FMath::Min(Sec.From + 2, N - 1);
						Sec.Drop = 0.5f;
						Sec.Side = TEXT("left");
						SpecRef->Sections.Add(Sec);
						RebuildSections();
						SpecChanged();
					}, TAttribute<bool>::CreateLambda([SpecRef]() { return SpecRef->Points.Num() >= 3; }))
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
			[
				SAssignNew(SectionsBox, SVerticalBox)
			]

			+ SVerticalBox::Slot().AutoHeight() [ EditorUi::Heading(LOCTEXT("SaveH", "SAVE")) ]
			+ SVerticalBox::Slot().AutoHeight()
			[
				SNew(SHorizontalBox)
				+ SHorizontalBox::Slot().FillWidth(1.f).Padding(0, 0, 8, 0)
				[
					SNew(SEditableTextBox)
					.Font(EditorUi::EdFont(11))
					.HintText(LOCTEXT("NameHint", "track name, e.g. hairpin_missing_blue"))
					.Text_Lambda([this]() { return FText::FromString(State->Name); })
					.OnTextChanged_Lambda([this](const FText& T) { State->Name = T.ToString().TrimStartAndEnd(); OverwriteArmedUntil = 0.0; })
				]
				+ SHorizontalBox::Slot().AutoWidth()
				[
					SNew(SButton)
					.IsFocusable(false)
					.ButtonColorAndOpacity(EditorUi::SaveBtn)
					.ContentPadding(FMargin(14.f, 5.f))
					.IsEnabled_Lambda([this]() { return CanSave(); })
					.OnClicked_Lambda([this]()
					{
						Save(FPlatformTime::Seconds() < OverwriteArmedUntil);
						return FReply::Handled();
					})
					[
						SNew(STextBlock)
						.Text_Lambda([this]()
						{
							return bSaving ? LOCTEXT("Saving", "Saving...")
								: FPlatformTime::Seconds() < OverwriteArmedUntil ? LOCTEXT("Replace", "Replace?")
								: LOCTEXT("Save", "Save track");
						})
						.Font(EditorUi::EdFont(11, TEXT("Bold"))).ColorAndOpacity(FLinearColor::Black)
					]
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 10, 0, 0)
			[
				SNew(SButton)
				.IsFocusable(false)
				.ButtonColorAndOpacity(EditorUi::GoBtn)
				.ContentPadding(FMargin(14.f, 8.f))
				.HAlign(HAlign_Center)
				.IsEnabled_Lambda([this]() { return !State->SavedName.IsEmpty(); })
				.OnClicked_Lambda([this]() { OnTestDrive.ExecuteIfBound(State->SavedName); return FReply::Handled(); })
				[
					SNew(STextBlock)
					.Text_Lambda([this]()
					{
						return State->SavedName.IsEmpty() ? LOCTEXT("TestDriveOff", "Test drive (save first)")
							: FText::Format(LOCTEXT("TestDrive", "Test drive {0}"), FText::FromString(State->SavedName));
					})
					.Font(EditorUi::EdFont(12, TEXT("Bold"))).ColorAndOpacity(FLinearColor::White)
				]
			]
			+ SVerticalBox::Slot().AutoHeight().Padding(0, 8, 0, 0)
			[
				SNew(STextBlock)
				.Text_Lambda([this]() { return FText::FromString(Message); })
				.Visibility_Lambda([this]() { return Message.IsEmpty() ? EVisibility::Collapsed : EVisibility::Visible; })
				.ColorAndOpacity_Lambda([this]() { return FSlateColor(bMessageError ? EditorUi::Bad : EditorUi::Good); })
				.Font(EditorUi::EdFont(10)).AutoWrapText(true)
			]
		];
}

void SFSDSTrackEditor::RebuildSections()
{
	if (!SectionsBox.IsValid()) return;
	SectionsBox->ClearChildren();
	TSharedRef<FSDSTrackSpec::FSpec> SpecRef = State->Spec;

	for (int32 k = 0; k < SpecRef->Sections.Num(); ++k)
	{
		// Every accessor checks the index: a section can go away under it.
		auto Sec = [SpecRef, k]() -> FSDSTrackSpec::FSection* { return SpecRef->Sections.IsValidIndex(k) ? &SpecRef->Sections[k] : nullptr; };
		auto Cycle = [this, Sec](const TArray<FString>& Options, FString FSDSTrackSpec::FSection::* Field, const FText& Label)
		{
			return SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(STextBlock).Text(Label).Font(EditorUi::EdFont(8, TEXT("Bold"))).ColorAndOpacity(EditorUi::Muted)
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 2, 0, 0)
				[
					SNew(SButton)
					.IsFocusable(false)
					.ButtonColorAndOpacity(EditorUi::PlainBtn)
					.ContentPadding(FMargin(8.f, 3.f))
					.OnClicked_Lambda([this, Sec, Options, Field]()
					{
						if (FSDSTrackSpec::FSection* S = Sec())
						{
							const int32 i = Options.IndexOfByKey(S->*Field);
							S->*Field = Options[(i + 1) % Options.Num()];
							SpecChanged();
						}
						return FReply::Handled();
					})
					[
						SNew(STextBlock).Font(EditorUi::EdFont(10, TEXT("Bold"))).ColorAndOpacity(FLinearColor::White)
						.Text_Lambda([Sec, Field]()
						{
							const FSDSTrackSpec::FSection* S = Sec();
							const FString V = S ? S->*Field : FString();
							return FText::FromString(V.IsEmpty() ? TEXT("-") : V);
						})
					]
				];
		};

		SectionsBox->AddSlot().AutoHeight().Padding(0, 0, 0, 6)
		[
			SNew(SBorder)
			.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
			.BorderBackgroundColor(EditorUi::Box)
			.Padding(FMargin(8.f, 6.f))
			[
				SNew(SVerticalBox)
				+ SVerticalBox::Slot().AutoHeight()
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
					[
						EditorUi::Num<int32>(LOCTEXT("From", "FROM"), [Sec]() { return Sec() ? Sec()->From : 0; },
							[this, Sec](int32 V) { if (Sec()) { Sec()->From = V; SpecChanged(); } }, 0, 499, 1, 52.f)
					]
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
					[
						EditorUi::Num<int32>(LOCTEXT("To", "TO"), [Sec]() { return Sec() ? Sec()->To : 0; },
							[this, Sec](int32 V) { if (Sec()) { Sec()->To = V; SpecChanged(); } }, 0, 499, 1, 52.f)
					]
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
					[
						EditorUi::OptNum(LOCTEXT("SWidth", "WIDTH"), [Sec]() { return Sec() ? Sec()->Width : TOptional<float>(); },
							[this, Sec](TOptional<float> V) { if (Sec()) { Sec()->Width = V; SpecChanged(); } })
					]
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
					[
						EditorUi::OptNum(LOCTEXT("SSpacing", "SPACING"), [Sec]() { return Sec() ? Sec()->Spacing : TOptional<float>(); },
							[this, Sec](TOptional<float> V) { if (Sec()) { Sec()->Spacing = V; SpecChanged(); } })
					]
					+ SHorizontalBox::Slot().FillWidth(1.f).HAlign(HAlign_Right).VAlign(VAlign_Bottom)
					[
						EditorUi::Button(LOCTEXT("Remove", "Remove"), EditorUi::PlainBtn, [this, SpecRef, k]()
						{
							if (SpecRef->Sections.IsValidIndex(k)) SpecRef->Sections.RemoveAt(k);
							RebuildSections();
							SpecChanged();
						})
					]
				]
				+ SVerticalBox::Slot().AutoHeight().Padding(0, 6, 0, 0)
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
					[
						EditorUi::Num<float>(LOCTEXT("Drop", "DROP 0-1"), [Sec]() { return Sec() ? Sec()->Drop : 0.f; },
							[this, Sec](float V) { if (Sec()) { Sec()->Drop = V; SpecChanged(); } }, 0.f, 1.f, 0.1f, 64.f)
					]
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
					[
						EditorUi::Num<float>(LOCTEXT("Jitter", "JITTER (M)"), [Sec]() { return Sec() ? Sec()->Jitter : 0.f; },
							[this, Sec](float V) { if (Sec()) { Sec()->Jitter = V; SpecChanged(); } }, 0.f, 2.f, 0.1f, 64.f)
					]
					+ SHorizontalBox::Slot().AutoWidth().Padding(0, 0, 6, 0)
					[
						Cycle({ TEXT("both"), TEXT("left"), TEXT("right") }, &FSDSTrackSpec::FSection::Side, LOCTEXT("Side", "SIDE"))
					]
					+ SHorizontalBox::Slot().AutoWidth()
					[
						Cycle({ TEXT(""), TEXT("swap"), TEXT("blue"), TEXT("yellow"), TEXT("small_orange") },
							&FSDSTrackSpec::FSection::Recolor, LOCTEXT("Colours", "COLOURS"))
					]
				]
			]
		];
	}
}

// ---------------------------------------------------------------------------
// Behaviour

void SFSDSTrackEditor::SpecChanged()
{
	bDirty = true;
	ChangedAt = FPlatformTime::Seconds();
	State->SavedName.Reset();   // what's drawn is no longer what was saved
	OverwriteArmedUntil = 0.0;
	if (!State->Spec->Enough())
	{
		Preview.Reset();
		PreviewError.Reset();
		if (Canvas) Canvas->SetPreview(nullptr);
		if (IssuesBox) IssuesBox->ClearChildren();
	}
}

EActiveTimerReturnType SFSDSTrackEditor::Poll(double, float)
{
	if (bDirty && !bInFlight && FPlatformTime::Seconds() - ChangedAt >= EditorUi::DebounceS)
	{
		RequestPreview();
	}
	return EActiveTimerReturnType::Continue;
}

void SFSDSTrackEditor::RequestPreview()
{
	bDirty = false;
	if (!Client.IsValid() || !State->Spec->Enough()) return;

	TSharedRef<FJsonObject> Body = MakeShared<FJsonObject>();
	Body->SetObjectField(TEXT("spec"), FSDSTrackSpec::ToJson(*State->Spec));
	bInFlight = true;
	TWeakPtr<SFSDSTrackEditor> Weak = StaticCastSharedRef<SFSDSTrackEditor>(AsShared());
	Client->Post(TEXT("/api/track/preview_spec"), Body, [Weak](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSTrackEditor> E = Weak.Pin();
		if (!E) return;
		E->bInFlight = false;
		E->bBackendKnown = true;
		E->bBackendUp = R.Reached();
		if (E->bDirty || !E->State->Spec->Enough()) return;   // stale: a newer one is coming

		TSharedPtr<FSDSTrackSpec::FPreview> P = MakeShared<FSDSTrackSpec::FPreview>();
		if (R.Reached() && FSDSTrackSpec::ParsePreview(R.Object(), *P))
		{
			E->Preview = P;
			E->PreviewError.Reset();
		}
		else
		{
			E->Preview.Reset();
			FString Err = R.Error;
			if (const TSharedPtr<FJsonObject> O = R.Object()) O->TryGetStringField(TEXT("error"), Err);
			E->PreviewError = R.Reached() ? Err : FString();
		}
		if (E->Canvas) E->Canvas->SetPreview(E->Preview);

		// The issues, errors first.
		if (E->IssuesBox)
		{
			E->IssuesBox->ClearChildren();
			if (E->Preview)
			{
				for (int32 Pass = 0; Pass < 2; ++Pass)
				{
					for (const FSDSTrackSpec::FIssue& I : E->Preview->Issues)
					{
						if (I.bError != (Pass == 0)) continue;
						FString Line = FString::Printf(TEXT("%s  %s"), I.bError ? TEXT("ERROR") : TEXT("RULE"), *I.Message);
						if (I.S.IsSet()) Line += FString::Printf(TEXT(" (%.0f m in)"), *I.S);
						E->IssuesBox->AddSlot().AutoHeight().Padding(0, 2)
						[
							SNew(STextBlock).Text(FText::FromString(Line)).Font(EditorUi::EdFont(9)).AutoWrapText(true)
							.ColorAndOpacity(I.bError ? EditorUi::Bad : EditorUi::Warn)
						];
					}
				}
			}
		}
	}, 10.f);
}

FText SFSDSTrackEditor::CheckText() const
{
	if (bBackendKnown && !bBackendUp)
	{
		return LOCTEXT("NoBackend", "Mission Control's backend isn't running. The editor needs it: it builds the cones and checks "
			"the rules with the track creator (tools/envgen).");
	}
	const FSDSTrackSpec::FSpec& S = *State->Spec;
	if (!S.Enough())
	{
		return FText::Format(LOCTEXT("Enough", "Add at least {0} points."), FText::AsNumber(S.bClosed ? 3 : 2));
	}
	if (!PreviewError.IsEmpty()) return FText::FromString(PreviewError);
	if (!Preview.IsValid()) return LOCTEXT("Building", "Building...");
	FString T = FString::Printf(TEXT("%.0f m   %d blue   %d yellow   %d orange"), Preview->LengthM, Preview->Blue, Preview->Yellow, Preview->Orange);
	if (Preview->Issues.Num() == 0) T += TEXT("\nMeets the FS layout rules.");
	return FText::FromString(T);
}

FSlateColor SFSDSTrackEditor::CheckColor() const
{
	if ((bBackendKnown && !bBackendUp) || !PreviewError.IsEmpty()) return EditorUi::Bad;
	if (Preview.IsValid() && Preview->Issues.Num() == 0) return EditorUi::Good;
	return EditorUi::Muted;
}

bool SFSDSTrackEditor::CanSave() const
{
	return !bSaving && bBackendUp && Preview.IsValid() && Preview->NumErrors() == 0 && !bDirty && !bInFlight
		&& !State->Name.IsEmpty();
}

void SFSDSTrackEditor::Save(bool bOverwrite)
{
	if (!Client.IsValid() || State->Name.IsEmpty()) return;
	TSharedRef<FJsonObject> Body = MakeShared<FJsonObject>();
	Body->SetStringField(TEXT("name"), State->Name);
	Body->SetObjectField(TEXT("spec"), FSDSTrackSpec::ToJson(*State->Spec));
	Body->SetBoolField(TEXT("overwrite"), bOverwrite);
	bSaving = true;
	OverwriteArmedUntil = 0.0;
	const FString Name = State->Name;

	TWeakPtr<SFSDSTrackEditor> Weak = StaticCastSharedRef<SFSDSTrackEditor>(AsShared());
	Client->Post(TEXT("/api/track/create"), Body, [Weak, Name](const FFSDSMcReply& R)
	{
		TSharedPtr<SFSDSTrackEditor> E = Weak.Pin();
		if (!E) return;
		E->bSaving = false;
		const TSharedPtr<FJsonObject> O = R.Object();
		FString Err = R.Error;
		if (O.IsValid()) O->TryGetStringField(TEXT("error"), Err);

		if (R.Code == 409)
		{
			E->OverwriteArmedUntil = FPlatformTime::Seconds() + EditorUi::ArmS;
			E->Message = FString::Printf(TEXT("%s.csv already exists: press Replace? to overwrite it."), *Name);
			E->bMessageError = true;
			return;
		}
		if (R.Code == 401 || R.Code == 403)
		{
			E->Message = TEXT("The backend wants an API key: set MissionControl.ApiKey in settings.json (or IFSSIM_MC_API_KEY).");
			E->bMessageError = true;
			return;
		}
		if (!R.Ok())
		{
			E->Message = FString::Printf(TEXT("Not saved: %s"), *Err);
			E->bMessageError = true;
			return;
		}
		double Cones = 0, Len = 0;
		FString File = Name + TEXT(".csv");
		if (O.IsValid())
		{
			O->TryGetNumberField(TEXT("cones"), Cones);
			O->TryGetNumberField(TEXT("length_m"), Len);
			O->TryGetStringField(TEXT("name"), File);
		}
		E->State->SavedName = FPaths::GetBaseFilename(File);
		E->Message = FString::Printf(TEXT("Saved %s: %d cones, %.0f m."), *File, (int32)Cones, Len);
		E->bMessageError = false;
		E->OnSaved.ExecuteIfBound();
	}, 20.f);
}

void SFSDSTrackEditor::Open(const FFSDSMenuTrack& Track)
{
	const FString SpecPath = FPaths::Combine(FPaths::GetPath(Track.Path), Track.Name + TEXT(".track.json"));
	FSDSTrackSpec::FSpec Loaded;
	if (!FSDSTrackSpec::LoadFile(SpecPath, Loaded))
	{
		Message = FString::Printf(TEXT("Couldn't read %s."), *SpecPath);
		bMessageError = true;
		return;
	}
	*State->Spec = MoveTemp(Loaded);   // in place: the canvas holds the same spec
	State->Name = Track.Name;
	if (Canvas) { Canvas->ClearSelection(); Canvas->FitView(); }
	RebuildSections();
	SpecChanged();
	State->SavedName = Track.Name;     // as on disk: it can be test driven at once
	Message = FString::Printf(TEXT("Opened %s."), *Track.Name);
	bMessageError = false;
}

void SFSDSTrackEditor::NewTrack()
{
	*State->Spec = FSDSTrackSpec::FSpec();
	State->Name.Reset();
	if (Canvas) { Canvas->ClearSelection(); Canvas->FitView(); }
	RebuildSections();
	SpecChanged();
	Message.Reset();
}

#undef LOCTEXT_NAMESPACE
