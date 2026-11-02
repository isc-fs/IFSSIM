#include "UI/FSDSLauncher.h"

#include "FSDSReferee.h"
#include "FSDSSettings.h"
#include "UI/FSDSMissionControlClient.h"
#include "UI/FSDSStackStarter.h"
#include "UI/SFSDSLauncherPanel.h"

#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "Framework/Application/SlateApplication.h"
#include "GameFramework/PlayerController.h"
#include "HAL/PlatformMisc.h"
#include "Misc/App.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "FSDSLauncher"

namespace
{
	// Above the game's own viewport widgets, below debug overlays.
	constexpr int32 PanelZOrder = 50;
	constexpr double HintSeconds = 10.0;
}

FFSDSLauncher::FFSDSLauncher() = default;

FFSDSLauncher::~FFSDSLauncher()
{
	Shutdown();
}

FKey FFSDSLauncher::ToggleKey()
{
	const FKey Key(*FFSDSSettings::Get().LauncherKey);
	return Key.IsValid() ? Key : EKeys::Tab;
}

void FFSDSLauncher::Init(UWorld* InWorld, AFSDSReferee* InReferee)
{
	World = InWorld;
	Referee = InReferee;

	const FFSDSSettings& S = FFSDSSettings::Get();
	// No viewport, no panel: headless runs (-RenderOffscreen, -nullrhi),
	// dedicated servers, and anyone who turned it off.
	bEnabled = S.bLauncherPanel
		&& FApp::CanEverRender()
		&& !FParse::Param(FCommandLine::Get(), TEXT("NoLauncher"))
		&& InWorld && InWorld->GetGameViewport() != nullptr
		&& FSlateApplication::IsInitialized();
	if (!bEnabled)
	{
		UE_LOG(LogTemp, Log, TEXT("FSDS Launcher: off (no viewport, -NoLauncher, or MissionControl.Panel=false)"));
		return;
	}

	FString ApiKey = S.MissionControlApiKey;
	if (ApiKey.IsEmpty())
	{
		ApiKey = FPlatformMisc::GetEnvironmentVariable(TEXT("IFSSIM_MC_API_KEY"));
	}
	Client = MakeShared<FFSDSMissionControlClient>(S.MissionControlUrl, ApiKey);
	Stack = MakeShared<FFSDSStackStarter>();
	ComposeDir = FFSDSStackStarter::FindComposeDir();

	TickHandle = FTSTicker::GetCoreTicker().AddTicker(
		FTickerDelegate::CreateRaw(this, &FFSDSLauncher::Tick), 0.25f);

	ShowHint();
	UE_LOG(LogTemp, Log, TEXT("FSDS Launcher: %s opens Mission Control (%s); compose dir: %s"),
		*ToggleKey().ToString(), *S.MissionControlUrl, ComposeDir.IsEmpty() ? TEXT("not found") : *ComposeDir);
}

void FFSDSLauncher::Shutdown()
{
	if (TickHandle.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(TickHandle);
		TickHandle.Reset();
	}
	Close();
	HideHint();
	Panel.Reset();
	Stack.Reset();
	Client.Reset();
	bEnabled = false;
}

void FFSDSLauncher::Toggle(APlayerController* PC)
{
	if (!bEnabled) return;
	if (IsOpen()) Close();
	else Open(PC);
}

void FFSDSLauncher::Open(APlayerController* PC)
{
	UWorld* W = World.Get();
	UGameViewportClient* Viewport = W ? W->GetGameViewport() : nullptr;
	if (!Viewport || IsOpen()) return;
	HideHint();

	SAssignNew(Panel, SFSDSLauncherPanel)
		.Client(Client)
		.Stack(Stack)
		.Referee(Referee)
		.ComposeDir(ComposeDir)
		.ToggleKey(ToggleKey())
		.OnClose(FSimpleDelegate::CreateRaw(this, &FFSDSLauncher::Close))
		.OnStartStack(FSimpleDelegate::CreateRaw(this, &FFSDSLauncher::StartStack));

	Root = SNew(SBox)
		.HAlign(HAlign_Left)
		.VAlign(VAlign_Top)
		.Padding(FMargin(24.f))
		[
			Panel.ToSharedRef()
		];
	Viewport->AddViewportWidgetContent(Root.ToSharedRef(), PanelZOrder);

	Controller = PC;
	if (PC)
	{
		// Game and UI: the panel takes clicks, the car keeps its keys.
		FInputModeGameAndUI Mode;
		Mode.SetWidgetToFocus(Panel);
		Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
		Mode.SetHideCursorDuringCapture(false);
		PC->SetInputMode(Mode);
		PC->SetShowMouseCursor(true);
	}
	else
	{
		FSlateApplication::Get().SetKeyboardFocus(Panel, EFocusCause::SetDirectly);
	}
}

void FFSDSLauncher::Close()
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
	Panel.Reset();   // drops its poll timer with it

	if (APlayerController* PC = Controller.Get())
	{
		PC->SetInputMode(FInputModeGameOnly());
		PC->SetShowMouseCursor(false);
	}
	if (FSlateApplication::IsInitialized())
	{
		FSlateApplication::Get().SetAllUserFocusToGameViewport();
	}
	Controller.Reset();
}

void FFSDSLauncher::StartStack()
{
	if (!Stack.IsValid()) return;
	FString Error;
	if (!Stack->Start(ComposeDir, Error))
	{
		UE_LOG(LogTemp, Warning, TEXT("FSDS Launcher: can't start the stack: %s"), *Error);
	}
}

bool FFSDSLauncher::Tick(float)
{
	// Drain docker's output even while the panel is closed: a full pipe
	// would stall `docker compose` part-way.
	if (Stack.IsValid() && Stack->IsRunning())
	{
		Stack->Poll();
	}
	if (Hint.IsValid() && FPlatformTime::Seconds() > HintUntil)
	{
		HideHint();
	}
	return true;
}

void FFSDSLauncher::ShowHint()
{
	UWorld* W = World.Get();
	UGameViewportClient* Viewport = W ? W->GetGameViewport() : nullptr;
	if (!Viewport) return;

	const FText Text = FText::Format(LOCTEXT("Hint", "{0}: Mission Control"), ToggleKey().GetDisplayName());
	Hint = SNew(SBox)
		.HAlign(HAlign_Left)
		.VAlign(VAlign_Bottom)
		.Padding(FMargin(24.f))
		.Visibility(EVisibility::HitTestInvisible)
		[
			SNew(SBorder)
			.BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
			.BorderBackgroundColor(FLinearColor(0.f, 0.f, 0.f, 0.55f))
			.Padding(FMargin(10.f, 5.f))
			[
				SNew(STextBlock)
				.Text(Text)
				.Font(FCoreStyle::GetDefaultFontStyle("Regular", 10))
				.ColorAndOpacity(FLinearColor(0.9f, 0.9f, 0.92f, 1.f))
			]
		];
	Viewport->AddViewportWidgetContent(Hint.ToSharedRef(), PanelZOrder - 1);
	HintUntil = FPlatformTime::Seconds() + HintSeconds;
}

void FFSDSLauncher::HideHint()
{
	if (!Hint.IsValid()) return;
	if (UWorld* W = World.Get())
	{
		if (UGameViewportClient* Viewport = W->GetGameViewport())
		{
			Viewport->RemoveViewportWidgetContent(Hint.ToSharedRef());
		}
	}
	Hint.Reset();
}

#undef LOCTEXT_NAMESPACE
