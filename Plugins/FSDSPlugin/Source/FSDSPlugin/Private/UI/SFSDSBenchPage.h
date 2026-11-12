#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"
#include "Widgets/SCompoundWidget.h"

class SVerticalBox;
class SWrapBox;
class FFSDSMissionControlClient;
struct FFSDSMcReply;

/**
 * The menu's Benchmarks page: make a run spec, check it, queue it and watch the
 * queue, through Mission Control's backend (bench_queue.py):
 *
 *   GET  /api/bench/presets      starting points (tools/sim_benchmark/specs) and override files
 *   GET  /api/bench/manifest     what a spec can set: benchmarks and their settings, bags, missions, tracks
 *   POST /api/bench/check        the worker's validation: the spec's problems, or its jobs
 *   POST /api/bench/launch       queue it: Local, or Runner (the bench PC; 501 for now)
 *   GET  /api/bench/queue        the worker and the jobs, newest first
 *   POST /api/bench/cancel/{id}
 *   POST /api/bench/worker/stop  the worker stops once no spec is running
 *   GET  /api/bench/rig          this machine's checks, for the component boxes
 *
 * A preset fills the form; the form edits a copy of its spec (the draft), so what
 * the form doesn't show (notes, a sweep) rides along. The form is drawn from the
 * manifest, so a setting added to bench.yaml appears without touching the sim.
 *
 * A Local spec runs once a worker on this machine takes it
 * (tools/sim_benchmark/run_spec.py --work), which Start worker launches. The
 * boxes: green ready, yellow not now (a queued spec waits), grey not on this
 * machine or not known; Pipeline and LattePanda pick where the pipeline runs.
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
	// --- what the backend offers ---------------------------------------------
	struct FPreset
	{
		FString Name, Title, About, Error;
		TSharedPtr<FJsonObject> Spec;
		TArray<FString> Needs;
	};
	struct FOverride { FString Name, About, Error; TSharedPtr<FJsonObject> Pipeline; };
	struct FSetting { FString Name, Type, Help; TArray<FString> Choices; };
	struct FBench
	{
		FString Name, Title, Description, Bags;
		TArray<FString> Parts;
		TArray<FSetting> Settings;
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
	void FetchManifest();
	void FetchQueue();
	void FetchRig();
	void Check();
	void Launch();
	void Cancel(const FString& Id);
	void StartWorker();
	void StopWorker();

	// --- the draft spec ------------------------------------------------------
	void PickPreset(const FString& Name);
	/** The draft's benchmarks.<Bench>, made when bCreate. */
	TSharedPtr<FJsonObject> BenchObj(const FString& Bench, bool bCreate = false) const;
	TSharedPtr<FJsonObject> SettingsObj(const FString& Bench, bool bCreate = false) const;
	bool HasBench(const FString& Bench) const { return BenchObj(Bench).IsValid(); }
	/** Whether the draft runs any benchmark. */
	bool HasBenchmarks() const;
	void SetBench(const FString& Bench, bool bOn);
	/** A list-valued choice stored as "a,b" (settings.missions, .tracks) or "all". */
	TArray<FString> CsvSetting(const FString& Bench, const FString& Setting) const;
	void SetCsvSetting(const FString& Bench, const FString& Setting, const TArray<FString>& Values);
	/** The draft changed: check it again shortly. */
	void Edited();
	/** What is queued: the draft, its name and the chosen override files merged in. */
	TSharedRef<FJsonObject> BuildSpec() const;
	FString SpecName() const;
	/** settings.pipeline_on of the benchmarks that have it: bench_pc unless set. */
	FString PipelineOn() const;
	void SetPipelineOn(const FString& Where);
	bool CanPickPipeline() const;

	// --- drawing -------------------------------------------------------------
	void RebuildPresets();
	void RebuildForm();
	void RebuildQueue();
	TSharedRef<SWidget> MakeBenchCard(const FBench& B);
	TSharedRef<SWidget> MakeSettingRow(const FBench& B, const FSetting& S);
	TSharedRef<SWidget> MakeBox(const FString& Part, const FText& Label, const FString& PipelineChoice = FString());

	const FPreset* Chosen() const;
	/** A box's state and why, for the chosen target. */
	EBox BoxState(const FString& Part, FString* Why = nullptr) const;
	/** What the draft needs that this machine never has; empty if nothing. */
	FString NeverHere() const;
	const TArray<FString>& Needs() const;
	bool CanQueue() const;
	/** <repo>/tools/sim_benchmark/results/.bench/queue/<id>, or empty. */
	static FString JobDir(const FString& Id);
	static FString RepoDir();
	void SetNote(const FString& Text, bool bError) { Note = Text; bNoteError = bError; }
	FString Unreachable(const FString& What, const FFSDSMcReply& R) const;

	TAttribute<bool> IsActive;
	TSharedPtr<FFSDSMissionControlClient> Client;
	bool bWasActive = false;
	double NextQueue = 0.0, NextRig = 0.0;

	TArray<FPreset> Presets;
	TArray<FOverride> Overrides;
	FString PresetName;
	FString PresetsNote;
	TSharedPtr<SWrapBox> PresetBox;

	bool bManifestKnown = false;
	FString ManifestError;
	TArray<FBench> Benches;
	TMap<FString, TArray<FString>> BagsByKind;
	TArray<FString> Missions, TrackNames;

	TSharedPtr<FJsonObject> Draft;
	FString DraftName;          // what the name box says, once typed in
	bool bNameTouched = false;
	bool bEdited = false;       // the draft differs from the preset
	TArray<FString> ChosenOverrides;
	TMap<FString, FString> FieldText;   // what each number/text field shows, as typed
	TSharedPtr<SVerticalBox> FormBox;

	double CheckDueAt = 0.0;    // 0: nothing to check
	int32 CheckSerial = 0;
	bool bChecked = false, bCheckOk = false;
	int32 CheckJobs = 0;
	TArray<FString> CheckErrors, CheckLabels, CheckNeeds;

	bool bRunner = false;
	bool bLaunching = false;

	bool bRigKnown = false, bRigStale = true;
	FString RigReason, RigId;
	TMap<FString, FPart> Parts;

	bool bQueueKnown = false;
	bool bWorkerAlive = false, bWorkerStopping = false;
	FString WorkerHost;
	int32 WorkerPid = 0;
	double WorkerStartingUntil = 0.0;   // Start worker pressed: waiting for its heartbeat
	TArray<FJob> Jobs;
	FString JobsShown;   // what the list was last built from, so a poll that changes nothing keeps it
	TSharedPtr<SVerticalBox> QueueList;
	FString CancelArmedFor;
	double CancelArmedUntil = 0.0;

	FString Note;
	bool bNoteError = false;
};
