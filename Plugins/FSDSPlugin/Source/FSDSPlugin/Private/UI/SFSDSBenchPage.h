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
 * The form asks for decisions, not the spec format: what to test (drive the
 * missions, or replay recorded bags), the few choices that matter for it in
 * plain words, and everything else bench.yaml declares under More settings
 * (drawn from the manifest, so a new setting appears without touching the sim).
 * A preset fills it in; the form edits a copy of its spec (the draft), so what it
 * doesn't show (notes, a sweep) rides along. A bar under the form says what will
 * run, or what's wrong, next to Queue.
 *
 * The right column shows this machine (green ready, yellow not now, grey not here
 * or not known) and the queue with its worker
 * (tools/sim_benchmark/run_spec.py --work), which Start worker launches.
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
	/** What to test: this benchmark alone. The others' settings are kept for switching back. */
	void PickBench(const FString& Bench);
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
	/** The Setup tab: the components and this machine. */
	void RebuildSetup();
	void RebuildQueue();
	/** The choices that matter for a benchmark, then its other settings under More settings. */
	void AddBenchRows(const FBench& B);
	TSharedRef<SWidget> MakeSettingRow(const FBench& B, const FSetting& S);
	TSharedRef<SWidget> MakeMissionsRow(const FBench& B);
	TSharedRef<SWidget> MakeTracksRow(const FBench& B);
	TSharedRef<SWidget> MakeBagsRow(const FBench& B);
	TSharedRef<SWidget> MakePartsRow(const FBench& B);
	TSharedRef<SWidget> MakePipelineChoice();
	/** The components a run uses: Sim, Pipeline (and where it runs), boards; each with the
	 *  version a run would use and whether this machine can run it now. */
	TSharedRef<SWidget> MakeComponents();
	TSharedRef<SWidget> MakeComponent(const FText& Title, TFunction<FString()> Part, TFunction<FString()> Version,
		TSharedPtr<SWidget> Extra, const FText& Note, float Width = 270.f);
	/** The boards box's choices: none, real (HIL) or emulated (vHIL), the system, each board's firmware. */
	TSharedRef<SWidget> MakeBoardsChoices();
	/** The version picker under the boxes: a repository's PRs and branches, or a typed ref. */
	TSharedRef<SWidget> MakePicker();
	/** Current: what is chosen now ("branch", "#88", "branch@commit"), to start from. */
	void OpenPicker(const FString& Title, const FString& Repo, const FString& Current, TFunction<void(const FString&)> OnPick);
	void FetchRefs(const FString& Repo);
	/** The latest commits of the picker's branch or PR, for its commit field. */
	void FetchCommits();
	/** The draft's code.<Repo> (a branch, #PR or commit); empty: this checkout. */
	FString CodeRef(const FString& Repo) const;
	void SetCodeRef(const FString& Repo, const FString& Ref);
	/** The draft's boards: (mode: hil or vhil, system, firmware), null when none. */
	TSharedPtr<FJsonObject> BoardsObj(bool bCreate = false) const;
	FString BoardsMode() const;
	void SetBoardsMode(const FString& Mode);
	TArray<FString> BoardsInRun() const;
	/** run_spec.py --rig on the host: checks this machine (and its code) for the boxes. */
	void CheckMachine();
	TSharedRef<SWidget> MakeOverridesRow();
	/** A count: settings.<Setting>, or the spec's repeats when Setting is empty. */
	TSharedRef<SWidget> MakeCountRow(const FBench& B, const FString& Setting, const FString& Label, const FString& Help);


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
	TMap<FString, TArray<FString>> MissionTracks;   // each mission's own tracks (missions.yaml)
	FString PipelineRepo;   // owner/name on GitHub
	struct FFirmware { FString Repo, Default; };
	TMap<FString, FFirmware> Firmware;   // per board (bench.yaml boards.firmware)
	TArray<FString> HilBoards;
	TArray<TPair<FString, TArray<FString>>> VhilSystems;   // id, its boards

	// the version picker
	struct FPull { int32 Number = 0; FString Title, Branch; };
	FString PickerTitle, PickerRepo, PickerError;
	FString PickerBranch, PickerCommit;   // what the two fields hold
	struct FCommit { FString Sha, Date, Message; };
	TArray<FCommit> PickerCommits;
	FString CommitsFor, CommitsError;   // the branch PickerCommits are of
	bool bCommitsLoading = false;
	TFunction<void(const FString&)> PickerSet;
	TArray<FString> PickerBranches;
	TArray<FPull> PickerPulls;
	FString PickerDefault;
	bool bPickerLoading = false;
	TMap<FString, FString> SetupText;   // the Setup tab's typed fields

	TSharedPtr<FJsonObject> Draft;
	FString DraftName;          // what the name box says, once typed in
	bool bNameTouched = false;
	bool bEdited = false;       // the draft differs from the preset
	TArray<FString> ChosenOverrides;
	TMap<FString, FString> FieldText;   // what each number/text field shows, as typed
	TMap<FString, TSharedPtr<FJsonObject>> Stash;   // a benchmark's settings while another is picked
	TSet<FString> MoreOpen;     // benchmarks whose More settings are open
	bool bTracksOpen = false, bOverridesOpen = false;
	TSharedPtr<SVerticalBox> FormBox;
	TSharedPtr<SVerticalBox> SetupBox;
	int32 Tab = 0;   // 0 Run, 1 Setup

	double CheckDueAt = 0.0;    // 0: nothing to check
	int32 CheckSerial = 0;
	bool bChecked = false, bCheckOk = false;
	int32 CheckJobs = 0;
	int32 CheckRuns = 0;
	TArray<FString> CheckErrors, CheckLabels, CheckNeeds, CheckLines;

	bool bRunner = false;
	bool bLaunching = false;

	bool bRigKnown = false, bRigStale = true;
	FString RigReason, RigId;
	TMap<FString, FPart> Parts;
	struct FCode { FString Commit, Branch; bool bDirty = false; };
	TMap<FString, FCode> Code;   // the code a local run uses (rig.json's code)
	double RigAge = 0.0;
	double NextMachineCheck = 0.0, MachineCheckUntil = 0.0;

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
