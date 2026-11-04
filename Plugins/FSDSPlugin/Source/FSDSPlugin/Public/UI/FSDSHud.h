#pragma once

#include "CoreMinimal.h"
#include "InputCoreTypes.h"

class AFSDSReferee;
class AFSDSVehiclePawn;
class SWidget;
class UWorld;

/**
 * The in-game HUD: a small overlay of the car's state (speed, pedals,
 * steering, who is driving, EBS) and the mission's (event, laps, cones hit,
 * off-course), in a corner of the viewport. One per game mode.
 *
 * Read-only: it never takes the mouse or keyboard, so driving and the
 * Mission Control panel work the same with it on. Shown at start unless
 * settings.json says "HUD": {"Enabled": false}; the key ("Key", H by
 * default) shows and hides it. Off in headless runs, where there is no
 * viewport, and with -NoHUD.
 */
class FSDSPLUGIN_API FFSDSHud
{
public:
	FFSDSHud();
	~FFSDSHud();

	void Init(UWorld* InWorld, AFSDSVehiclePawn* InPawn, AFSDSReferee* InReferee);
	void Shutdown();

	void Toggle();
	bool IsAvailable() const { return bAvailable; }
	bool IsShown() const { return Root.IsValid(); }

	/** The key from settings ("Key"), or H when that isn't a key name. */
	static FKey ToggleKey();

private:
	void Show();
	void Hide();

	TWeakObjectPtr<UWorld> World;
	TWeakObjectPtr<AFSDSVehiclePawn> Pawn;
	TWeakObjectPtr<AFSDSReferee> Referee;
	TSharedPtr<SWidget> Root;
	bool bAvailable = false;
};
