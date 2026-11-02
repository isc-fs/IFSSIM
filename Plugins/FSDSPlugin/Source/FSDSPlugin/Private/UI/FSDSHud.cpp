#include "UI/FSDSHud.h"

#include "FSDSReferee.h"
#include "FSDSSettings.h"
#include "FSDSVehiclePawn.h"

#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "Framework/Application/SlateApplication.h"
#include "Misc/App.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Brushes/SlateColorBrush.h"
#include "Styling/CoreStyle.h"
#include "Styling/SlateTypes.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Notifications/SProgressBar.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "FSDSHud"

namespace
{
	// Below the Mission Control panel (50) and its hint (49).
	constexpr int32 HudZOrder = 40;

	const FLinearColor TextColor(0.92f, 0.92f, 0.94f, 1.f);
	const FLinearColor DimColor(0.62f, 0.64f, 0.68f, 1.f);
	const FLinearColor Backdrop(0.f, 0.f, 0.f, 0.55f);
	const FLinearColor ThrottleColor(0.20f, 0.78f, 0.35f, 1.f);
	const FLinearColor BrakeColor(0.90f, 0.25f, 0.22f, 1.f);
	const FLinearColor SteerColor(0.30f, 0.62f, 0.95f, 1.f);
	const FLinearColor EbsColor(0.85f, 0.10f, 0.10f, 1.f);

	FSlateFontInfo Font(int32 Size, const TCHAR* Weight = TEXT("Regular"))
	{
		return FCoreStyle::GetDefaultFontStyle(Weight, Size);
	}

	/** Flat bars: the default style's fill image is blue and ignores the colour. */
	const FProgressBarStyle& BarStyle()
	{
		static const FSlateColorBrush Track(FLinearColor(1.f, 1.f, 1.f, 0.12f));
		static const FSlateColorBrush Fill(FLinearColor::White);
		static const FProgressBarStyle Style = FProgressBarStyle()
			.SetBackgroundImage(Track)
			.SetFillImage(Fill)
			.SetMarqueeImage(Fill);
		return Style;
	}

	FText EventName(EFSDSEventType Event)
	{
		switch (Event)
		{
		case EFSDSEventType::Trackdrive:   return LOCTEXT("Trackdrive", "Trackdrive");
		case EFSDSEventType::Acceleration: return LOCTEXT("Acceleration", "Acceleration");
		case EFSDSEventType::Skidpad:      return LOCTEXT("Skidpad", "Skidpad");
		case EFSDSEventType::Autocross:    return LOCTEXT("Autocross", "Autocross");
		default:                           return LOCTEXT("NoEvent", "No event");
		}
	}

	FText LapTime(float Seconds)
	{
		if (Seconds <= 0.f) return LOCTEXT("NoLap", "--");
		const int32 Min = FMath::FloorToInt(Seconds / 60.f);
		return FText::FromString(FString::Printf(TEXT("%d:%06.3f"), Min, Seconds - Min * 60.f));
	}
}

/**
 * The overlay itself. Every value is an attribute read from the pawn and the
 * referee when Slate paints, so there is nothing to push and nothing to go
 * stale; with the pawn or referee gone it shows dashes.
 */
class SFSDSHud : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SFSDSHud) {}
		SLATE_ARGUMENT(TWeakObjectPtr<AFSDSVehiclePawn>, Pawn)
		SLATE_ARGUMENT(TWeakObjectPtr<AFSDSReferee>, Referee)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs)
	{
		Pawn = InArgs._Pawn;
		Referee = InArgs._Referee;

		ChildSlot
		[
			SNew(SBorder)
			.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
			.BorderBackgroundColor(Backdrop)
			.Padding(FMargin(14.f, 10.f))
			[
				SNew(SBox)
				.WidthOverride(250.f)
				[
					SNew(SVerticalBox)

					// Who is driving, and the EBS.
					+ SVerticalBox::Slot().AutoHeight()
					[
						SNew(SHorizontalBox)
						+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
						[
							SNew(STextBlock)
							.Font(Font(10, TEXT("Bold")))
							.ColorAndOpacity(DimColor)
							.Text(this, &SFSDSHud::ModeText)
						]
						+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
						[
							SNew(SBorder)
							.Visibility(this, &SFSDSHud::EbsVisibility)
							.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
							.BorderBackgroundColor(EbsColor)
							.Padding(FMargin(6.f, 1.f))
							[
								SNew(STextBlock)
								.Font(Font(10, TEXT("Bold")))
								.ColorAndOpacity(FLinearColor::White)
								.Text(LOCTEXT("Ebs", "EBS"))
							]
						]
					]

					// Speed.
					+ SVerticalBox::Slot().AutoHeight().Padding(0.f, 2.f, 0.f, 0.f)
					[
						SNew(SHorizontalBox)
						+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Bottom)
						[
							SNew(STextBlock)
							.Font(Font(30, TEXT("Bold")))
							.ColorAndOpacity(TextColor)
							.Text(this, &SFSDSHud::SpeedText)
						]
						+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Bottom).Padding(6.f, 0.f, 0.f, 6.f)
						[
							SNew(STextBlock)
							.Font(Font(11))
							.ColorAndOpacity(DimColor)
							.Text(LOCTEXT("Kmh", "km/h"))
						]
						+ SHorizontalBox::Slot().FillWidth(1.f).HAlign(HAlign_Right).VAlign(VAlign_Bottom).Padding(0.f, 0.f, 0.f, 6.f)
						[
							SNew(STextBlock)
							.Font(Font(10))
							.ColorAndOpacity(DimColor)
							.Text(this, &SFSDSHud::DetailText)
						]
					]

					// Pedals and steering.
					+ SVerticalBox::Slot().AutoHeight().Padding(0.f, 6.f, 0.f, 0.f)
					[
						Bar(LOCTEXT("Throttle", "Throttle"), ThrottleColor,
							TAttribute<TOptional<float>>(this, &SFSDSHud::Throttle))
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.f, 3.f, 0.f, 0.f)
					[
						Bar(LOCTEXT("Brake", "Brake"), BrakeColor,
							TAttribute<TOptional<float>>(this, &SFSDSHud::Brake))
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.f, 3.f, 0.f, 0.f)
					[
						SteeringBar()
					]

					// The mission.
					+ SVerticalBox::Slot().AutoHeight().Padding(0.f, 8.f, 0.f, 0.f)
					[
						SNew(STextBlock)
						.Font(Font(10, TEXT("Bold")))
						.ColorAndOpacity(TextColor)
						.Text(this, &SFSDSHud::MissionText)
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.f, 2.f, 0.f, 0.f)
					[
						SNew(STextBlock)
						.Font(Font(10))
						.ColorAndOpacity(DimColor)
						.Text(this, &SFSDSHud::LapText)
					]
					+ SVerticalBox::Slot().AutoHeight().Padding(0.f, 2.f, 0.f, 0.f)
					[
						SNew(STextBlock)
						.Font(Font(10))
						.ColorAndOpacity(this, &SFSDSHud::PenaltyColor)
						.Text(this, &SFSDSHud::PenaltyText)
					]
				]
			]
		];
	}

private:
	TSharedRef<SWidget> Bar(const FText& Label, const FLinearColor& Color, TAttribute<TOptional<float>> Percent)
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				SNew(SBox).WidthOverride(62.f)
				[
					SNew(STextBlock).Font(Font(9)).ColorAndOpacity(DimColor).Text(Label)
				]
			]
			+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
			[
				SNew(SBox).HeightOverride(8.f)
				[
					SNew(SProgressBar).Style(&BarStyle()).Percent(Percent).FillColorAndOpacity(Color)
				]
			];
	}

	/** Centred: left of the middle for a left turn, right for a right turn. */
	TSharedRef<SWidget> SteeringBar()
	{
		return SNew(SHorizontalBox)
			+ SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Center)
			[
				SNew(SBox).WidthOverride(62.f)
				[
					SNew(STextBlock).Font(Font(9)).ColorAndOpacity(DimColor).Text(LOCTEXT("Steering", "Steering"))
				]
			]
			+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
			[
				SNew(SBox).HeightOverride(8.f)
				[
					SNew(SProgressBar).Style(&BarStyle()).BarFillType(EProgressBarFillType::RightToLeft)
					.FillColorAndOpacity(SteerColor)
					.Percent(this, &SFSDSHud::SteerLeft)
				]
			]
			+ SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center).Padding(1.f, 0.f, 0.f, 0.f)
			[
				SNew(SBox).HeightOverride(8.f)
				[
					SNew(SProgressBar).Style(&BarStyle()).BarFillType(EProgressBarFillType::LeftToRight)
					.FillColorAndOpacity(SteerColor)
					.Percent(this, &SFSDSHud::SteerRight)
				]
			];
	}

	AFSDSVehiclePawn* GetPawn() const { return Pawn.Get(); }

	FText ModeText() const
	{
		const AFSDSVehiclePawn* P = GetPawn();
		if (!P) return LOCTEXT("NoCar", "NO CAR");
		return P->IsApiControlEnabled() ? LOCTEXT("Autonomous", "AUTONOMOUS") : LOCTEXT("Manual", "MANUAL");
	}

	EVisibility EbsVisibility() const
	{
		const AFSDSVehiclePawn* P = GetPawn();
		return P && P->IsEbsLatched() ? EVisibility::HitTestInvisible : EVisibility::Collapsed;
	}

	FText SpeedText() const
	{
		const AFSDSVehiclePawn* P = GetPawn();
		if (!P) return LOCTEXT("NoSpeed", "--");
		return FText::AsNumber(FMath::RoundToInt(P->GetCarState().Speed * 3.6f));
	}

	FText DetailText() const
	{
		const AFSDSVehiclePawn* P = GetPawn();
		if (!P) return FText::GetEmpty();
		const AFSDSVehiclePawn::FCarState S = P->GetCarState();
		return FText::FromString(FString::Printf(TEXT("%.1f m/s  %d rpm"), S.Speed, FMath::RoundToInt(S.RPM)));
	}

	TOptional<float> Throttle() const
	{
		const AFSDSVehiclePawn* P = GetPawn();
		return P ? FMath::Clamp(P->GetCarControls().Throttle, 0.f, 1.f) : 0.f;
	}

	/** Regen, or full when the mechanical (EBS) brake is on. */
	TOptional<float> Brake() const
	{
		const AFSDSVehiclePawn* P = GetPawn();
		if (!P) return 0.f;
		const AFSDSVehiclePawn::FCarControls C = P->GetCarControls();
		return C.bHandbrake ? 1.f : FMath::Clamp(C.Regen, 0.f, 1.f);
	}

	// The car turns right for positive steering.
	TOptional<float> SteerLeft() const
	{
		const AFSDSVehiclePawn* P = GetPawn();
		return P ? FMath::Clamp(-P->GetCarControls().Steering, 0.f, 1.f) : 0.f;
	}

	TOptional<float> SteerRight() const
	{
		const AFSDSVehiclePawn* P = GetPawn();
		return P ? FMath::Clamp(P->GetCarControls().Steering, 0.f, 1.f) : 0.f;
	}

	FText MissionText() const
	{
		const AFSDSReferee* R = Referee.Get();
		if (!R) return LOCTEXT("NoReferee", "No mission");
		const FFSDSRefereeState S = R->GetState();
		return S.bFinished
			? FText::Format(LOCTEXT("Finished", "{0}: finished"), EventName(S.EventType))
			: EventName(S.EventType);
	}

	FText LapText() const
	{
		const AFSDSReferee* R = Referee.Get();
		if (!R) return FText::GetEmpty();
		const FFSDSRefereeState S = R->GetState();
		float Best = 0.f;
		for (float L : S.Laps)
		{
			if (L > 0.f && (Best <= 0.f || L < Best)) Best = L;
		}
		return FText::Format(LOCTEXT("Laps", "Lap {0}/{1}   last {2}   best {3}"),
			FText::AsNumber(S.Laps.Num()), FText::AsNumber(S.RequiredLaps),
			LapTime(S.Laps.Num() ? S.Laps.Last() : 0.f), LapTime(Best));
	}

	FText PenaltyText() const
	{
		const AFSDSReferee* R = Referee.Get();
		if (!R) return FText::GetEmpty();
		const FFSDSRefereeState S = R->GetState();
		return FText::Format(LOCTEXT("Penalties", "Cones down {0}   Off course {1}"),
			FText::AsNumber(S.DooCounter), FText::AsNumber(S.OffTrackCounter));
	}

	FSlateColor PenaltyColor() const
	{
		const AFSDSReferee* R = Referee.Get();
		const bool bAny = R && (R->GetState().DooCounter > 0 || R->GetState().OffTrackCounter > 0);
		return bAny ? FSlateColor(FLinearColor(1.f, 0.62f, 0.25f, 1.f)) : FSlateColor(DimColor);
	}

	TWeakObjectPtr<AFSDSVehiclePawn> Pawn;
	TWeakObjectPtr<AFSDSReferee> Referee;
};

FFSDSHud::FFSDSHud() = default;

FFSDSHud::~FFSDSHud()
{
	Shutdown();
}

FKey FFSDSHud::ToggleKey()
{
	const FKey Key(*FFSDSSettings::Get().HudKey);
	return Key.IsValid() ? Key : EKeys::H;
}

void FFSDSHud::Init(UWorld* InWorld, AFSDSVehiclePawn* InPawn, AFSDSReferee* InReferee)
{
	World = InWorld;
	Pawn = InPawn;
	Referee = InReferee;

	// Same rule as the Mission Control panel: no viewport, no HUD.
	bAvailable = FApp::CanEverRender()
		&& !FParse::Param(FCommandLine::Get(), TEXT("NoHUD"))
		&& InWorld && InWorld->GetGameViewport() != nullptr
		&& FSlateApplication::IsInitialized();
	if (!bAvailable)
	{
		UE_LOG(LogTemp, Log, TEXT("FSDS HUD: off (no viewport or -NoHUD)"));
		return;
	}
	if (FFSDSSettings::Get().bHudEnabled)
	{
		Show();
	}
	UE_LOG(LogTemp, Log, TEXT("FSDS HUD: %s, %s shows and hides it"),
		IsShown() ? TEXT("on") : TEXT("off (HUD.Enabled=false)"), *ToggleKey().ToString());
}

void FFSDSHud::Shutdown()
{
	Hide();
	bAvailable = false;
}

void FFSDSHud::Toggle()
{
	if (!bAvailable) return;
	if (IsShown()) Hide();
	else Show();
}

void FFSDSHud::Show()
{
	UWorld* W = World.Get();
	UGameViewportClient* Viewport = W ? W->GetGameViewport() : nullptr;
	if (!Viewport || IsShown()) return;

	Root = SNew(SBox)
		.HAlign(HAlign_Right)
		.VAlign(VAlign_Bottom)
		.Padding(FMargin(24.f))
		.Visibility(EVisibility::HitTestInvisible)
		[
			SNew(SFSDSHud).Pawn(Pawn).Referee(Referee)
		];
	Viewport->AddViewportWidgetContent(Root.ToSharedRef(), HudZOrder);
}

void FFSDSHud::Hide()
{
	if (!Root.IsValid()) return;
	if (UWorld* W = World.Get())
	{
		if (UGameViewportClient* Viewport = W->GetGameViewport())
		{
			Viewport->RemoveViewportWidgetContent(Root.ToSharedRef());
		}
	}
	Root.Reset();
}

#undef LOCTEXT_NAMESPACE
