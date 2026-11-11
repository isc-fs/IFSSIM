#pragma once

#include "CoreMinimal.h"
#include "UI/FSDSMenu.h"
#include "UI/FSDSTrackSpec.h"
#include "UI/FSDSVenue.h"
#include "UI/SFSDSLocationPicker.h"
#include "Widgets/SCompoundWidget.h"

class FFSDSMissionControlClient;
class SFSDSTrackCanvas;
class SVerticalBox;
class SBox;

DECLARE_DELEGATE_OneParam(FOnFSDSTestDrive, const FString& /*TrackName*/);

/**
 * The menu's Track editor page: Raúl's web Draw Track in the sim. Draw a path
 * (points or freehand); the backend's track creator (tools/envgen) builds the
 * cones, checks the FS layout rules and places the start, live as you draw.
 * Test sections, Save, and Test drive (save, load, drive it in Practice).
 *
 * Needs Mission Control's backend: the geometry is envgen's, so there is one
 * implementation of it. Without the backend the page says so and offers to
 * start the stack.
 */
class SFSDSTrackEditor : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SFSDSTrackEditor) {}
		SLATE_ARGUMENT(TSharedPtr<FSDSTrackSpec::FEditorState>, State)
		SLATE_ARGUMENT(TSharedPtr<FFSDSMissionControlClient>, Client)
		SLATE_ARGUMENT(TArray<FFSDSMenuTrack>, Tracks)
		SLATE_ARGUMENT(TArray<FSDSVenue::FVenueRef>, Venues)
		SLATE_EVENT(FOnFSDSTestDrive, OnTestDrive)
		SLATE_EVENT(FSimpleDelegate, OnSaved)
		SLATE_EVENT(FSimpleDelegate, OnStartStack)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	/** The tracks folders changed, e.g. after a save: redraw the open list. */
	void SetTracks(const TArray<FFSDSMenuTrack>& InTracks, const TArray<FSDSVenue::FVenueRef>& InVenues);

private:
	TSharedRef<SWidget> BuildLeft();
	TSharedRef<SWidget> BuildRight();
	TSharedRef<SWidget> BuildOpenList();
	/** Draw on a venue (or none): its backdrop, its lot checks, its name prefix. */
	void SetVenue(const FString& Name);
	TSharedPtr<const FSDSVenue::FVenue> FindVenue(const FString& Name) const;
	/** The name to save under: with the venue's prefix, and no other venue's. */
	FString NameForVenue(const FString& Name) const;
	void RebuildSections();

	void SpecChanged();
	void Open(const FFSDSMenuTrack& Track);
	void NewTrack();
	void Save(bool bOverwrite);
	EActiveTimerReturnType Poll(double Now, float Dt);
	void RequestPreview();

	FText CheckText() const;
	FSlateColor CheckColor() const;
	bool CanSave() const;

	TSharedPtr<FSDSTrackSpec::FEditorState> State;
	TSharedPtr<FFSDSMissionControlClient> Client;
	TArray<FFSDSMenuTrack> Tracks;
	TArray<FSDSVenue::FVenueRef> Venues;
	FOnFSDSTestDrive OnTestDrive;
	FSimpleDelegate OnSaved, OnStartStack;

	TSharedPtr<SFSDSTrackCanvas> Canvas;
	TSharedPtr<SBox> OpenListBox;
	TSharedPtr<SFSDSLocationPicker> VenuePicker;
	TSharedPtr<SVerticalBox> SectionsBox;
	TSharedPtr<SVerticalBox> IssuesBox;

	TSharedPtr<FSDSTrackSpec::FPreview> Preview;
	FString PreviewError;
	bool bBackendUp = true;
	bool bBackendKnown = false;
	bool bDirty = false;
	bool bInFlight = false;
	double ChangedAt = 0.0;

	bool bSaving = false;
	double OverwriteArmedUntil = 0.0;
	FString Message;
	bool bMessageError = false;

	/** OR START FROM A RANDOM TRACK: the random generator's loop as control
	 *  points (POST /api/track/random_spec), a new unsaved drawing. */
	TSharedRef<SWidget> BuildRandom();
	void GenerateRandom();
	int32 GenPoints = 50;
	int32 GenRegions = 30;
	// 140 m: a lap of about 390 m, most loops inside FS's 200-500 m.
	int32 GenMaxBound = 140;
	bool bGenerating = false;
};
