#pragma once

#include "CoreMinimal.h"
#include "InputCoreTypes.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Input/SComboBox.h"

class FFSDSMissionControlClient;
class FFSDSStackStarter;
class AFSDSReferee;
class FJsonObject;
class SVerticalBox;
struct FFSDSMcReply;

/**
 * The in-game Mission Control panel: load a track, start and stop a
 * session, reset the car, RES (the Session tab), and the session log with
 * its export (the Log tab). A client of the backend, like the web UI; the
 * backend owns the session sequence.
 *
 * Built from code in Slate (no widget Blueprint to keep in sync with the C++).
 * Nothing in it takes keyboard focus except the panel itself, so the
 * driving keys keep driving while it is open: a focused button would
 * otherwise eat Space, which is the brake.
 */
class SFSDSLauncherPanel : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SFSDSLauncherPanel) {}
		SLATE_ARGUMENT(TSharedPtr<FFSDSMissionControlClient>, Client)
		SLATE_ARGUMENT(TSharedPtr<FFSDSStackStarter>, Stack)
		SLATE_ARGUMENT(TWeakObjectPtr<AFSDSReferee>, Referee)
		SLATE_ARGUMENT(FString, ComposeDir)
		SLATE_ARGUMENT(FKey, ToggleKey)
		SLATE_EVENT(FSimpleDelegate, OnClose)
		SLATE_EVENT(FSimpleDelegate, OnStartStack)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	virtual bool SupportsKeyboardFocus() const override { return true; }
	virtual FReply OnKeyDown(const FGeometry& Geometry, const FKeyEvent& KeyEvent) override;

private:
	struct FMission
	{
		FString Name;
		FString Label;
		FString Kind;
		bool bLaps = false;
	};
	using FMissionPtr = TSharedPtr<FMission>;

	/** One /api/session/log entry. */
	struct FLogEntry
	{
		FString Time;     // local HH:MM:SS
		FString Type;
		FString Text;
	};

	/** The tabs, in the order of the tab bar. */
	enum class ETab : int32 { Session = 0, Log = 1 };
	using FTrackPtr = TSharedPtr<FString>;

	// --- backend -------------------------------------------------------
	EActiveTimerReturnType Poll(double Time, float DeltaTime);
	void FetchLists();
	/** Run one action: mark busy, POST, report the outcome on the message line. */
	void Act(const FString& What, const FString& Path, const TSharedPtr<FJsonObject>& Body, float TimeoutS,
		TFunction<FString(const FFSDSMcReply&)> Describe = nullptr);
	/** Two-click confirm for actions the web UI also confirms. */
	bool Confirmed(FName Action);
	bool IsArmed(FName Action) const;
	void Say(const FString& Msg, bool bError);

	/** The session log, while the Log tab is open. */
	void FetchLog();
	void RebuildLogList();

	// --- actions -------------------------------------------------------
	FReply OnLoadTrack();
	FReply OnStartSession();
	FReply OnStopSession();
	FReply OnReset();
	FReply OnPause();
	FReply OnRes();
	FReply OnExportLog();

	// --- layout --------------------------------------------------------
	TSharedRef<SWidget> MakeStatusRow(const FText& Label, TAttribute<FText> Value, TAttribute<FSlateColor> Color);
	TSharedRef<SWidget> MakeHeading(const FText& Title);
	TSharedRef<SWidget> MakeButton(TAttribute<FText> Label, FLinearColor Color, FOnClicked OnClicked,
		TAttribute<bool> Enabled);
	TSharedRef<SWidget> BuildStackBlock();
	TSharedRef<SWidget> BuildTrackBlock();
	TSharedRef<SWidget> BuildSessionBlock();
	TSharedRef<SWidget> BuildCarBlock();
	TSharedRef<SWidget> BuildTabBar();
	TSharedRef<SWidget> MakeTab(const FText& Label, ETab Tab);
	TSharedRef<SWidget> BuildSessionTab();
	TSharedRef<SWidget> BuildLogTab();

	bool CanAct() const;
	FText BackendText() const;
	FSlateColor BackendColor() const;
	FText SimText() const;
	FSlateColor SimColor() const;
	FText TrackText() const;
	FText LapText() const;
	FText PipelineText() const;
	FSlateColor PipelineColor() const;
	FText ResText() const;
	FSlateColor ResColor() const;
	FText BagText() const;
	FText StackText() const;

	TSharedPtr<FFSDSMissionControlClient> Client;
	TSharedPtr<FFSDSStackStarter> Stack;
	TWeakObjectPtr<AFSDSReferee> Referee;
	FString ComposeDir;
	FKey ToggleKey;
	FSimpleDelegate OnClose;
	FSimpleDelegate OnStartStack;

	// Last poll. Each field is what the backend said, not a guess.
	bool bPolled = false;
	bool bBackendUp = false;
	bool bSimConnected = false;
	bool bPaused = false;
	bool bPipeline = false;
	bool bResActive = false;
	int32 Fps = 0;
	FString BagState;
	FString BagName;
	FString BackendError;
	int32 PollsInFlight = 0;

	bool bListsLoaded = false;
	bool bListsLoading = false;
	TArray<FTrackPtr> Tracks;
	FTrackPtr SelectedTrack;
	TArray<FMissionPtr> Missions;
	FMissionPtr SelectedMission;
	TSharedPtr<SComboBox<FTrackPtr>> TrackCombo;
	TSharedPtr<SComboBox<FMissionPtr>> MissionCombo;
	int32 NumLaps = 10;
	bool bRecordBag = false;

	ETab ActiveTab = ETab::Session;

	TArray<FLogEntry> LogEntries;
	/** The last log reply as text, so the list is rebuilt only when it changes. */
	FString LogSignature;
	FString LogError;
	bool bLogInFlight = false;
	TSharedPtr<SVerticalBox> LogList;

	/** Non-empty while an action is in flight: what it is doing. */
	FString Busy;
	FString Message;
	bool bMessageError = false;
	FName Armed;
	double ArmedUntil = 0.0;
};
