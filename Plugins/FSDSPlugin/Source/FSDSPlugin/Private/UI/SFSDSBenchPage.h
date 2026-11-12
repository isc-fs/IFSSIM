#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"
#include "Widgets/SCompoundWidget.h"

class SVerticalBox;
class SWrapBox;
class FFSDSMissionControlClient;
struct FFSDSMcReply;

/**
 * The menu's Benchmarks page: queue a run spec and watch the queue, through
 * Mission Control's backend (bench_queue.py):
 *
 *   GET  /api/bench/presets     the specs in tools/sim_benchmark/specs, as JSON
 *   POST /api/bench/launch      queue one: Local, or Runner (the bench PC; 501 for now)
 *   GET  /api/bench/queue       the worker and the jobs, newest first
 *   POST /api/bench/cancel/{id}
 *   GET  /api/bench/rig         this machine's checks, for the component boxes
 *
 * A Local spec runs once a worker on this machine takes it
 * (python3 tools/sim_benchmark/run_spec.py --work). The boxes: green ready,
 * yellow not now (a queued spec waits), grey not on this machine or not known.
 */
class SFSDSBenchPage : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SFSDSBenchPage) {}
		/** Whether the page is on screen: it polls the backend only then. */
		SLATE_ATTRIBUTE(bool, IsActive)
		SLATE_ARGUMENT(TSharedPtr<FFSDSMissionControlClient>, Client)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	/** What a component box shows. */
	enum class EBox : uint8 { Ready, NotNow, Never, Unknown };

private:
	struct FPreset
	{
		FString Name, Title, About, Error;
		TSharedPtr<FJsonObject> Spec;
		TArray<FString> Needs;
	};
	struct FJob
	{
		FString Id, Name, State, Created, RequestedBy, Current, Waiting, Error;
		int32 Done = 0, Total = 0;
		bool bCancelRequested = false;
	};
	struct FPart { FString Status, Reason; };

	EActiveTimerReturnType Poll(double Now, float Dt);
	void FetchPresets();
	void FetchQueue();
	void FetchRig();
	void Launch();
	void Cancel(const FString& Id);

	void RebuildPresets();
	void RebuildQueue();
	TSharedRef<SWidget> MakeBox(const FString& Part, const FText& Label);

	const FPreset* Chosen() const;
	bool IsRunner() const { return bRunner; }
	/** A box's state and why, for the chosen target. */
	EBox BoxState(const FString& Part, FString* Why = nullptr) const;
	/** What the chosen preset needs that this machine never has; empty if nothing. */
	FString NeverHere() const;
	/** <repo>/tools/sim_benchmark/results/.bench/queue/<id>, or empty. */
	static FString JobDir(const FString& Id);
	void SetNote(const FString& Text, bool bError) { Note = Text; bNoteError = bError; }
	FString Unreachable(const FString& What, const FFSDSMcReply& R) const;

	TAttribute<bool> IsActive;
	TSharedPtr<FFSDSMissionControlClient> Client;
	bool bWasActive = false;
	double NextQueue = 0.0, NextRig = 0.0;

	TArray<FPreset> Presets;
	FString PresetName;
	FString PresetsNote;
	bool bRunner = false;
	bool bLaunching = false;
	TSharedPtr<SWrapBox> PresetBox;

	bool bRigKnown = false, bRigStale = true;
	FString RigReason, RigId;
	TMap<FString, FPart> Parts;

	bool bQueueKnown = false;
	bool bWorkerAlive = false;
	FString WorkerHost;
	int32 WorkerPid = 0;
	TArray<FJob> Jobs;
	FString JobsShown;   // what the list was last built from, so a poll that changes nothing keeps it
	TSharedPtr<SVerticalBox> QueueList;
	FString CancelArmedFor;
	double CancelArmedUntil = 0.0;

	FString Note;
	bool bNoteError = false;
};
