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
