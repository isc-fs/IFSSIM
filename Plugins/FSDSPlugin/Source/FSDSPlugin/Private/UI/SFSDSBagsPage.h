#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

class SVerticalBox;

DECLARE_DELEGATE_TwoParams(FOnFSDSReplayBag, const FString& /*BagDir*/, const FString& /*TrackName*/);

/**
 * The menu's Bags page: the rosbags recorded with "Record a bag", as the
 * backend copies them out of the stack (the repo's bags folder). Each with
 * its event, track, date and size; Replay in the sim (the car follows the
 * recorded run and the pipeline view shows what the pipeline saw), Delete,
 * and Open the folder.
 */
class SFSDSBagsPage : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SFSDSBagsPage) {}
		/** Whether the page is on screen: it rescans the folder when it opens. */
		SLATE_ATTRIBUTE(bool, IsActive)
		SLATE_EVENT(FOnFSDSReplayBag, OnReplay)
		/** A note to show first, e.g. how the last replay ended. */
		SLATE_ARGUMENT(FString, Note)
		SLATE_ARGUMENT(bool, NoteError)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);
	void SetNote(const FString& Text, bool bError) { Note = Text; bNoteError = bError; }

	struct FBag
	{
		FString Name;      // folder name: <event>_<track>_<YYYYMMDD>_<HHMMSS>
		FString Dir;       // absolute
		FString Event;
		FString Track;
		FDateTime When;
		bool bHasWhen = false;
		int64 Bytes = 0;   // with its _carparity companion
		bool bComplete = false;   // metadata.yaml written: the copy-out finished
	};
	/** Event, track and time from a bag folder's name; false when it isn't one. */
	static bool ParseName(const FString& Name, FBag& Out);

private:
	void Rescan();
	void Rebuild();
	EActiveTimerReturnType Poll(double Now, float Dt);
	static FString BagsDir();
	static FString SizeText(int64 Bytes);

	TAttribute<bool> IsActive;
	FOnFSDSReplayBag OnReplay;
	TSharedPtr<SVerticalBox> List;
	TArray<FBag> Bags;
	FString Status;
	bool bStatusError = false;
	FString Note;
	bool bNoteError = false;
	bool bWasActive = false;
	FString DeleteArmedFor;
	double DeleteArmedUntil = 0.0;
};
