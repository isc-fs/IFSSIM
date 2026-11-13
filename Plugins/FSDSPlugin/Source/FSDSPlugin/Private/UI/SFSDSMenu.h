#pragma once

#include "CoreMinimal.h"
#include "UI/FSDSMenu.h"
#include "UI/FSDSTrackSpec.h"
#include "UI/FSDSVenue.h"
#include "UI/SFSDSLocationPicker.h"
#include "UI/SFSDSTrackEditor.h"
#include "Widgets/SCompoundWidget.h"
#include "UI/SFSDSBagsPage.h"
#include "UI/SFSDSBenchPage.h"

class SFSDSTrackMap;
class SScrollBox;
class SVerticalBox;

DECLARE_DELEGATE_OneParam(FOnFSDSDrive, const FFSDSDriveRequest&);

/**
 * The menu's screens. Main: the full-screen front end (Drive, Tracks,
 * Results, Settings). Pause and Finished: a box over the paused or ended
 * run. FFSDSMenu owns it and does the work; this only draws and asks.
 */
class SFSDSMenu : public SCompoundWidget
{
public:
	enum class EScreen : uint8 { Main, Pause, Finished };

	SLATE_BEGIN_ARGS(SFSDSMenu) {}
		SLATE_ARGUMENT(EScreen, Screen)
		SLATE_ARGUMENT(TArray<FFSDSMenuTrack>, Tracks)
		SLATE_ARGUMENT(TArray<FSDSVenue::FVenueRef>, Venues)
		SLATE_ARGUMENT(TSharedPtr<const TArray<FFSDSRunResult>>, Results)
		SLATE_ARGUMENT(TOptional<FFSDSDriveRequest>, Last)
		SLATE_ARGUMENT(TWeakObjectPtr<AFSDSReferee>, Referee)
		SLATE_ARGUMENT(FText, LauncherKey)
		SLATE_ARGUMENT(TSharedPtr<FFSDSMissionControlClient>, Client)
		SLATE_ARGUMENT(TSharedPtr<FSDSTrackSpec::FEditorState>, EditorState)
		SLATE_ARGUMENT(bool, StartInEditor)
		SLATE_ARGUMENT(bool, StartInBags)
		SLATE_ARGUMENT(FString, BagsNote)
		SLATE_ARGUMENT(bool, BagsNoteError)
		SLATE_EVENT(FOnFSDSReplayBag, OnReplay)
		SLATE_EVENT(FOnFSDSTestDrive, OnTestDrive)
		SLATE_EVENT(FSimpleDelegate, OnTracksChanged)
		SLATE_EVENT(FSimpleDelegate, OnStartStack)
		SLATE_EVENT(FSimpleDelegate, OnRebuildPipeline)
		SLATE_ARGUMENT(TFunction<FString()>, RebuildStatus)
		SLATE_ARGUMENT(TFunction<bool()>, IsRebuilding)
		SLATE_ARGUMENT(TFunction<bool()>, RebuildFailed)
		SLATE_EVENT(FOnFSDSDrive, OnDrive)
		SLATE_EVENT(FSimpleDelegate, OnResume)
		SLATE_EVENT(FSimpleDelegate, OnRestart)
		SLATE_EVENT(FSimpleDelegate, OnMainMenu)
		SLATE_EVENT(FSimpleDelegate, OnQuit)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	/** A line under START, e.g. why a track would not load. */
	void SetMessage(const FString& Msg, bool bError);
	/** START's question when the team's code isn't on this PC: Text and the
	 *  answers (FFSDSMenu::OnRepoChoice). An empty Text hides it. */
	void SetRepoPrompt(const FString& Text, TFunction<void(int32)> OnChoice);

	/** The tracks folders changed, e.g. a track was saved in the editor. */
	void SetTracks(const TArray<FFSDSMenuTrack>& InTracks, const TArray<FSDSVenue::FVenueRef>& InVenues);
	/** A note on the Bags page (a replay that couldn't start). */
	void SetBagsMessage(const FString& Text, bool bError) { if (BagsPage.IsValid()) BagsPage->SetNote(Text, bError); }

	virtual bool SupportsKeyboardFocus() const override { return true; }
	virtual FReply OnKeyDown(const FGeometry& Geometry, const FKeyEvent& KeyEvent) override;

private:
	enum class EPage : int32 { Drive = 0, Editor = 1, Results = 2, Settings = 3, Logs = 4, Bags = 5, Bench = 6 };

	TSharedRef<SWidget> BuildMain();
	TSharedRef<SWidget> BuildRunBox();
	TSharedRef<SWidget> BuildDrivePage();
	TSharedRef<SWidget> BuildResultsPage();
	TSharedRef<SWidget> BuildSettingsPage();

	TSharedRef<SWidget> MakeNavButton(const FText& Label, EPage Page);
	TSharedRef<SWidget> MakeChoice(TAttribute<FText> Label, TFunction<bool()> IsOn, TFunction<void()> OnPick);
	/** A TIMING choice: the speed while the pipeline drives (FFSDSRealTime). */
	TSharedRef<SWidget> SpeedChoice(const FText& Label, double Speed);
	/** A Settings, Graphics row: a label, then one button per choice (FFSDSGraphics). */
	struct FGraphicsChoice { FText Label; TFunction<bool()> IsOn; TFunction<void()> OnPick; };
	TSharedRef<SWidget> GraphicsRow(const FText& Label, const TArray<FGraphicsChoice>& Choices);
	/** What Find the best settled on, shown beside the button. */
	FString RecommendResult;
	TSharedRef<SWidget> MakeBigButton(const FText& Label, FLinearColor Color, FSimpleDelegate OnClick,
		TAttribute<bool> Enabled = true);
	TSharedRef<SWidget> MakeHeading(const FText& Title);

	void RebuildTrackList();
	/** Where to drive: a venue's name, or empty for every track. */
	void SetLocation(const FString& InLocation);
	TSharedPtr<const FSDSVenue::FVenue> FindVenue(const FString& Name) const;
	void SelectTrack(int32 Index);
	/** Why a track can't be deleted, for the Drive page; empty if it can. */
	static FString WhyKept(const FString& Name);
	void SetEvent(EFSDSEventType Event, bool bInPractice);
	/** Whether a track suits the chosen event, wherever it is. */
	bool FitsEvent(const FFSDSMenuTrack& Track) const;
	/** ...and the location and the search: whether the list shows it. */
	bool Fits(const FFSDSMenuTrack& Track) const;
	/** The shown tracks in list order: by venue (open floor first) on Anywhere. */
	TArray<int32> ShownTracks() const;
	/** The tracks at a venue (empty: all) that suit the event. */
	int32 CountAt(const FString& Venue) const;
	FText EventHint() const;
	FText TrackSummary(int32 Index) const;

	EScreen Screen = EScreen::Main;
	EPage Page = EPage::Drive;

	TArray<FFSDSMenuTrack> Tracks;
	TArray<FSDSVenue::FVenueRef> Venues;
	FString Location;
	FString Search;
	TSharedPtr<SFSDSLocationPicker> LocationPicker;
	TSharedPtr<class SFSDSTrackEditor> EditorPage;
	TSharedPtr<const TArray<FFSDSRunResult>> Results;
	TOptional<FFSDSDriveRequest> Last;
	TWeakObjectPtr<AFSDSReferee> Referee;
	FText LauncherKey;

	FOnFSDSDrive OnDrive;
	FOnFSDSTestDrive OnTestDrive;
	FSimpleDelegate OnTracksChanged, OnStartStack;
	TSharedPtr<FFSDSMissionControlClient> Client;
	TSharedPtr<FSDSTrackSpec::FEditorState> EditorState;
	FSimpleDelegate OnResume, OnRestart, OnMainMenu, OnQuit;
	FOnFSDSReplayBag OnReplay;
	TSharedPtr<SFSDSBagsPage> BagsPage;
	FString BagsNote;
	bool bBagsNoteError = false;

	int32 Selected = INDEX_NONE;
	EFSDSEventType Event = EFSDSEventType::Autocross;
	int32 Laps = 10;
	bool bPractice = false;
	bool bAutonomous = false;

	FString Message;
	bool bMessageError = false;

	// --- what the Tab panel did, in the menu -----------------------------
	/** Drive: record a bag of an autonomous run. */
	bool bRecordBag = false;
	/** The reference time typed for the chosen event and track, as saved
	 *  ([FSDSScoring] in GameUserSettings); reloaded when either changes. */
	const FString& Reference() const;
	void SetReference(const FString& Seconds);
	FString ReferenceKey() const;
	mutable FString ReferenceFor;
	mutable FString ReferenceText;
	/** Drive: generate a track (/api/track/generate), then select it. */
	/** Drive: delete the selected track, on a second click within 4 s. */
	void DeleteTrack();
	double DeleteArmedUntil = 0.0;
	FString DeleteArmedFor;
	bool bTrackBusy = false;
	/** The run box over an autonomous run: RES and the sim's pause. */
	bool bResActive = false;
	bool bSimPaused = false;
	FString RunNote;
	/** Results: the session log (/api/session/log), and its export. */
	struct FLogLine { FString Time; FString Type; FString Text; };
	TArray<FLogLine> LogLines;
	TSharedPtr<SVerticalBox> LogBox;
	FString LogNote;
	bool bLogLoading = false;
	void FetchLog();
	void SaveLog();
	void RebuildLogBox();
	/** An autonomous run's score, for the results box and the Results page. */
	static FString ScoreText(const FFSDSRunResult& R);
	FString RepoPrompt;
	TFunction<void(int32)> OnRepoChoice;
	/** Settings, PIPELINE CODE: the folder, and what the last Change... said. */
	FString RepoShown;
	FString RepoNote;
	/** Settings, Rebuild pipeline (FFSDSMenu::RebuildPipeline). */
	FSimpleDelegate OnRebuildPipeline;
	TFunction<FString()> RebuildStatus;
	TFunction<bool()> IsRebuilding;
	TFunction<bool()> RebuildFailed;

	TSharedPtr<SScrollBox> DriveList;
	TSharedPtr<SFSDSTrackMap> DriveMap;
};
