#pragma once

#include "CoreMinimal.h"
#include "HAL/PlatformProcess.h"

/**
 * Starts the Docker stack (`docker compose up -d`) from inside the game, for
 * when Mission Control's backend isn't running yet.
 *
 * Runs through the user's login shell on macOS/Linux, so `docker` and its
 * credential helpers resolve on the same PATH as in a terminal (an app
 * launched from Finder gets a bare PATH otherwise). Output is collected
 * from a pipe so the panel can show the last line.
 */
class FSDSPLUGIN_API FFSDSStackStarter
{
public:
	~FFSDSStackStarter();

	/** The folder holding docker-compose.yml: settings, then $IFSSIM_ROOT,
	 *  then searched upward from the project, the settings file and the
	 *  executable. Empty when none was found. */
	static FString FindComposeDir();

	/** The command a person would type, for the panel's hint line. */
	static FString CommandLine() { return TEXT("docker compose up -d"); }

	/** `docker compose up -d`, then Extra (services, flags) when given. */
	bool Start(const FString& ComposeDir, FString& OutError, const FString& Extra = FString());
	/** Any command line, through the shell, in WorkDir; its output is collected as Start's. */
	bool Run(const FString& WorkDir, const FString& Command, FString& OutError);

	// --- the team's code (the IFSSIM repo), for a sim that came as a zip ----

	/** Whether Dir holds docker-compose.yml: the repo's root. */
	static bool IsRepo(const FString& Dir);
	/** The folder chosen or cloned from the menu (GameUserSettings.ini); empty when none. */
	static FString SavedRepoDir();
	static void SaveRepoDir(const FString& Dir);
	/** Where to clone it when asked: Documents\IFSSIM-repo (Documents\IFSSIM
	 *  holds the sim's session logs), asked of Windows (so a
	 *  Spanish "Documentos" or a moved Documents is found), or the user's
	 *  home when Documents is in OneDrive, which would sync gigabytes and lock
	 *  files under git and Docker. A free name: IFSSIM-repo, IFSSIM-repo-2... */
	static FString SuggestedRepoDir();
	/** The clone, with its submodule (the pipeline). Git Credential Manager
	 *  opens the browser for the GitHub login when it needs one. */
	static FString CloneCommand(const FString& Dir);
	/** Whether an executable (git, docker, winget) is on the PATH, or where
	 *  its installer puts it (added to the PATH: an install after the sim
	 *  started isn't on the PATH it was given). */
	static bool HasTool(const FString& Exe);
	/** A folder picker (Windows); false when cancelled or unavailable. */
	static bool PickFolder(const FString& Title, const FString& StartDir, FString& OutDir);
	/** Docker Desktop's "Enable host networking" is off (the stack's
	 *  containers then listen inside Docker's VM, out of the sim's reach). */
	static bool HostNetworkingOff();

	/** Whether docker's output says its engine isn't running (Docker Desktop closed). */
	static bool SaysDockerIsDown(const FString& Output);
	/** Open Docker Desktop (Windows, macOS); on Linux the service is the user's to start. */
	static bool StartDocker(FString& OutError);
	/** Drain the pipe; call every tick or so. Returns true while running. */
	bool Poll();

	bool IsRunning() const { return bRunning; }
	bool HasFinished() const { return bFinished; }
	int32 ExitCode() const { return ReturnCode; }
	/** Last non-empty output line. */
	const FString& LastLine() const { return Last; }
	/** Everything printed, for the hint when it failed. */
	const FString& Output() const { return All; }

private:
	void Close();

	FProcHandle Proc;
	void* ReadPipe = nullptr;
	void* WritePipe = nullptr;
	bool bRunning = false;
	bool bFinished = false;
	int32 ReturnCode = 0;
	FString Pending;
	FString Last;
	FString All;
};
