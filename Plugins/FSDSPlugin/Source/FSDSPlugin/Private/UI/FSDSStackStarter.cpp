#include "UI/FSDSStackStarter.h"

#include "FSDSSettings.h"
#include "HAL/PlatformMisc.h"
#include "Misc/Paths.h"

namespace
{
	bool HasCompose(const FString& Dir)
	{
		return !Dir.IsEmpty() && FPaths::FileExists(FPaths::Combine(Dir, TEXT("docker-compose.yml")));
	}

	/** Walk up from Start looking for docker-compose.yml (a staged build sits
	 *  a few levels under the repo, e.g. Saved/StagedBuilds/Mac/IFSSIM.app/...). */
	FString SearchUp(FString Start)
	{
		if (Start.IsEmpty()) return FString();
		Start = FPaths::ConvertRelativePathToFull(Start);
		FPaths::NormalizeDirectoryName(Start);
		for (int32 Depth = 0; Depth < 10 && !Start.IsEmpty(); ++Depth)
		{
			if (HasCompose(Start)) return Start;
			const FString Parent = FPaths::GetPath(Start);
			if (Parent == Start) break;
			Start = Parent;
		}
		return FString();
	}
}

FFSDSStackStarter::~FFSDSStackStarter()
{
	// Leave docker running: `compose up -d` returns once the containers are
	// up, and killing it half-way would leave the stack half-started.
	Close();
}

FString FFSDSStackStarter::FindComposeDir()
{
	const FFSDSSettings& S = FFSDSSettings::Get();
	if (HasCompose(S.MissionControlComposeDir)) return S.MissionControlComposeDir;

	const FString Env = FPlatformMisc::GetEnvironmentVariable(TEXT("IFSSIM_ROOT"));
	if (HasCompose(Env)) return Env;

	for (const FString& From : { FPaths::ProjectDir(), FPaths::GetPath(S.LoadedFromPath),
		FPaths::LaunchDir(), FString(FPlatformProcess::BaseDir()) })
	{
		const FString Found = SearchUp(From);
		if (!Found.IsEmpty()) return Found;
	}
	return FString();
}

bool FFSDSStackStarter::SaysDockerIsDown(const FString& Output)
{
	// The engine's pipe or socket isn't there. Matched on docker's own
	// English words, not the OS's (localised) error text after them.
	return Output.Contains(TEXT("error during connect"))
		|| Output.Contains(TEXT("dockerDesktopLinuxEngine"))
		|| Output.Contains(TEXT("Cannot connect to the Docker daemon"))
		|| Output.Contains(TEXT("docker daemon is not running"));
}

bool FFSDSStackStarter::StartDocker(FString& OutError)
{
#if PLATFORM_WINDOWS
	FString Exe = FPaths::Combine(FPlatformMisc::GetEnvironmentVariable(TEXT("ProgramFiles")),
		TEXT("Docker"), TEXT("Docker"), TEXT("Docker Desktop.exe"));
	if (!FPaths::FileExists(Exe))
	{
		OutError = TEXT("Docker isn't running and Docker Desktop isn't installed where expected: start Docker");
		return false;
	}
	// Hidden: only its engine is wanted, not its window over the sim.
	FProcHandle H = FPlatformProcess::CreateProc(*Exe, TEXT(""), /*bLaunchDetached=*/true,
		/*bLaunchHidden=*/true, /*bLaunchReallyHidden=*/true, nullptr, 0, nullptr, nullptr);
#elif PLATFORM_MAC
	FProcHandle H = FPlatformProcess::CreateProc(TEXT("/usr/bin/open"), TEXT("-g -a Docker"), true,
		false, false, nullptr, 0, nullptr, nullptr);
#else
	OutError = TEXT("Docker isn't running: start it (sudo systemctl start docker)");
	return false;
#endif
#if PLATFORM_WINDOWS || PLATFORM_MAC
	if (!H.IsValid())
	{
		OutError = TEXT("couldn't open Docker Desktop: start it yourself");
		return false;
	}
	FPlatformProcess::CloseProc(H);
	UE_LOG(LogTemp, Log, TEXT("FSDS Launcher: Docker wasn't running; opening Docker Desktop"));
	return true;
#endif
}

bool FFSDSStackStarter::Start(const FString& ComposeDir, FString& OutError, const FString& Extra)
{
	if (bRunning) return true;
	if (!HasCompose(ComposeDir))
	{
		OutError = TEXT("no docker-compose.yml found; set MissionControl.ComposeDir in settings.json");
		return false;
	}
	Close();
	Pending.Reset();
	Last.Reset();
	All.Reset();
	bFinished = false;
	ReturnCode = 0;

	if (!FPlatformProcess::CreatePipe(ReadPipe, WritePipe))
	{
		OutError = TEXT("could not open a pipe for docker's output");
		return false;
	}

	const FString Command = Extra.IsEmpty() ? CommandLine() : CommandLine() + TEXT(" ") + Extra;
#if PLATFORM_WINDOWS
	const FString Exe = TEXT("cmd.exe");
	const FString Args = FString::Printf(TEXT("/c %s 2>&1"), *Command);
#elif PLATFORM_MAC
	const FString Exe = TEXT("/bin/zsh");
	const FString Args = FString::Printf(TEXT("-lc \"%s 2>&1\""), *Command);
#else
	const FString Exe = TEXT("/bin/bash");
	const FString Args = FString::Printf(TEXT("-lc \"%s 2>&1\""), *Command);
#endif

	Proc = FPlatformProcess::CreateProc(*Exe, *Args, /*bLaunchDetached=*/false,
		/*bLaunchHidden=*/true, /*bLaunchReallyHidden=*/true, nullptr, 0,
		*ComposeDir, WritePipe, nullptr);
	if (!Proc.IsValid())
	{
		Close();
		OutError = FString::Printf(TEXT("could not run %s"), *Exe);
		return false;
	}
	bRunning = true;
	UE_LOG(LogTemp, Log, TEXT("FSDS Launcher: '%s' in %s"), *Command, *ComposeDir);
	return true;
}

bool FFSDSStackStarter::Poll()
{
	if (!bRunning) return false;

	auto Drain = [this]()
	{
		const FString Chunk = FPlatformProcess::ReadPipe(ReadPipe);
		if (Chunk.IsEmpty()) return;
		All += Chunk;
		Pending += Chunk;
		TArray<FString> Lines;
		Pending.ParseIntoArrayLines(Lines, /*CullEmpty=*/true);
		// Keep a trailing partial line for next time.
		if (!Pending.EndsWith(TEXT("\n")) && Lines.Num() > 0)
		{
			Pending = Lines.Pop();
		}
		else
		{
			Pending.Reset();
		}
		for (int32 i = Lines.Num() - 1; i >= 0; --i)
		{
			const FString Line = Lines[i].TrimStartAndEnd();
			if (!Line.IsEmpty()) { Last = Line; break; }
		}
	};

	Drain();
	if (FPlatformProcess::IsProcRunning(Proc)) return true;

	Drain();
	if (!Pending.TrimStartAndEnd().IsEmpty()) Last = Pending.TrimStartAndEnd();
	FPlatformProcess::GetProcReturnCode(Proc, &ReturnCode);
	bRunning = false;
	bFinished = true;
	UE_LOG(LogTemp, Log, TEXT("FSDS Launcher: docker compose exited %d: %s"), ReturnCode, *Last);
	Close();
	return false;
}

void FFSDSStackStarter::Close()
{
	if (Proc.IsValid())
	{
		FPlatformProcess::CloseProc(Proc);
	}
	if (ReadPipe || WritePipe)
	{
		FPlatformProcess::ClosePipe(ReadPipe, WritePipe);
		ReadPipe = WritePipe = nullptr;
	}
}
