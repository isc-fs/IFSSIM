#pragma once

#include "CoreMinimal.h"
#include "Containers/Ticker.h"
#include "InputCoreTypes.h"

class AFSDSReferee;
class APlayerController;
class FFSDSMissionControlClient;
class FFSDSStackStarter;
class SFSDSLauncherPanel;
class SWidget;
class UWorld;

/**
 * The in-game Mission Control panel's owner (one per game mode).
 *
 * Opens and closes the panel (the toggle key, Tab by default), switches the
 * mouse cursor and input mode with it, and keeps the "Start stack" process
 * alive while the panel is closed. Off in headless runs, where there is no
 * viewport, and when settings.json says "MissionControl": {"Panel": false}.
 */
class FSDSPLUGIN_API FFSDSLauncher
{
public:
	FFSDSLauncher();
	~FFSDSLauncher();

	void Init(UWorld* InWorld, AFSDSReferee* InReferee);
	void Shutdown();

	void Toggle(APlayerController* PC);
	bool IsEnabled() const { return bEnabled; }
	bool IsOpen() const { return Root.IsValid(); }

	/** The key from settings ("Key"), or Tab when that isn't a key name. */
	static FKey ToggleKey();

private:
	void Open(APlayerController* PC);
	void Close();
	void StartStack();
	bool Tick(float DeltaTime);
	void ShowHint();
	void HideHint();

	TWeakObjectPtr<UWorld> World;
	TWeakObjectPtr<AFSDSReferee> Referee;
	TWeakObjectPtr<APlayerController> Controller;
	TSharedPtr<FFSDSMissionControlClient> Client;
	TSharedPtr<FFSDSStackStarter> Stack;
	TSharedPtr<SFSDSLauncherPanel> Panel;
	TSharedPtr<SWidget> Root;
	TSharedPtr<SWidget> Hint;
	FString ComposeDir;
	FTSTicker::FDelegateHandle TickHandle;
	double HintUntil = 0.0;
	bool bEnabled = false;
};
