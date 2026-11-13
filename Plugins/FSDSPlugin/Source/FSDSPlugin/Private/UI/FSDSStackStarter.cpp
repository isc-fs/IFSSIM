#include "UI/FSDSStackStarter.h"

#include "FSDSSettings.h"
#include "Dom/JsonObject.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

#if PLATFORM_WINDOWS
#include "Windows/AllowWindowsPlatformTypes.h"
#include <shobjidl.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace
{
	const TCHAR* StackSection = TEXT("FSDSStack");
	const TCHAR* RepoUrl = TEXT("https://github.com/isc-fs/IFSSIM.git");

	/** Where the installers put git and docker: added to this process's PATH,
	 *  which its children inherit. */
	void AddToolDirsToPath()
	{
#if PLATFORM_WINDOWS
		FString Path = FPlatformMisc::GetEnvironmentVariable(TEXT("PATH"));
		const FString Pf = FPlatformMisc::GetEnvironmentVariable(TEXT("ProgramFiles"));
		const FString Local = FPlatformMisc::GetEnvironmentVariable(TEXT("LOCALAPPDATA"));
		bool bChanged = false;
		for (const FString& Dir : { FPaths::Combine(Pf, TEXT("Git"), TEXT("cmd")),
			FPaths::Combine(Pf, TEXT("Docker"), TEXT("Docker"), TEXT("resources"), TEXT("bin")),
			FPaths::Combine(Local, TEXT("Microsoft"), TEXT("WindowsApps")) })
		{
			FString Win = Dir;
			FPaths::MakePlatformFilename(Win);
			if (!FPaths::DirectoryExists(Dir) || Path.Contains(Win)) continue;
			Path += TEXT(";") + Win;
			bChanged = true;
		}
		if (bChanged) FPlatformMisc::SetEnvironmentVar(TEXT("PATH"), *Path);
#endif
	}

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

	const FString Saved = SavedRepoDir();
	if (HasCompose(Saved)) return Saved;

	for (const FString& From : { FPaths::ProjectDir(), FPaths::GetPath(S.LoadedFromPath),
		FPaths::LaunchDir(), FString(FPlatformProcess::BaseDir()) })
	{
		const FString Found = SearchUp(From);
		if (!Found.IsEmpty()) return Found;
	}
	return FString();
}

bool FFSDSStackStarter::IsRepo(const FString& Dir)
{
	return HasCompose(Dir);
}

FString FFSDSStackStarter::SavedRepoDir()
{
	FString Dir;
	if (GConfig) GConfig->GetString(StackSection, TEXT("RepoDir"), Dir, GGameUserSettingsIni);
	return Dir;
}

void FFSDSStackStarter::SaveRepoDir(const FString& Dir)
{
	if (!GConfig) return;
	GConfig->SetString(StackSection, TEXT("RepoDir"), *Dir, GGameUserSettingsIni);
	GConfig->Flush(false, GGameUserSettingsIni);
	UE_LOG(LogTemp, Log, TEXT("FSDS Launcher: the team's code is at %s"), *Dir);
}

FString FFSDSStackStarter::SuggestedRepoDir()
{
	// UserDir is Documents as Windows has it (FOLDERID_Documents): wherever it
	// was moved, in whatever language it is shown.
	FString Base = FPlatformProcess::UserDir();
	if (Base.IsEmpty() || Base.Contains(TEXT("OneDrive"), ESearchCase::IgnoreCase))
	{
		Base = FPlatformMisc::GetEnvironmentVariable(TEXT("USERPROFILE"));
		if (Base.IsEmpty()) Base = FPlatformProcess::UserHomeDir();
	}
	FPaths::NormalizeDirectoryName(Base);
	for (int32 i = 1; i < 20; ++i)
	{
		// Not IFSSIM: the sim keeps its session logs in Documents\\IFSSIM.
		const FString Dir = FPaths::Combine(Base, i == 1 ? FString(TEXT("IFSSIM-repo")) : FString::Printf(TEXT("IFSSIM-repo-%d"), i));
		if (!FPaths::DirectoryExists(Dir) || HasCompose(Dir)) return Dir;
		TArray<FString> Inside;
		IFileManager::Get().FindFiles(Inside, *FPaths::Combine(Dir, TEXT("*")), true, true);
		if (Inside.Num() == 0) return Dir;   // there, but empty
	}
	return FPaths::Combine(Base, TEXT("IFSSIM-repo"));
}

FString FFSDSStackStarter::CloneCommand(const FString& Dir)
{
	FString Win = Dir;
	FPaths::MakePlatformFilename(Win);
	// dev: main is empty. The Unreal assets (Git LFS) aren't needed to run the
	// stack, and are fetched with `git lfs pull` by whoever opens the project.
	// Long paths: the pipeline's tree is deep.
	return FString::Printf(TEXT("git -c core.longpaths=true clone --recursive --branch dev --progress %s \"%s\""),
		RepoUrl, *Win);
}

bool FFSDSStackStarter::HasTool(const FString& Exe)
{
	AddToolDirsToPath();
	const FString Path = FPlatformMisc::GetEnvironmentVariable(TEXT("PATH"));
	TArray<FString> Dirs;
#if PLATFORM_WINDOWS
	Path.ParseIntoArray(Dirs, TEXT(";"), true);
	const FString Name = Exe + TEXT(".exe");
#else
	Path.ParseIntoArray(Dirs, TEXT(":"), true);
	const FString Name = Exe;
#endif
	for (const FString& D : Dirs)
	{
		if (FPaths::FileExists(FPaths::Combine(D.TrimQuotes(), Name))) return true;
	}
	return false;
}

bool FFSDSStackStarter::PickFolder(const FString& Title, const FString& StartDir, FString& OutDir)
{
#if PLATFORM_WINDOWS
	bool bPicked = false;
	const HRESULT Init = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
	IFileOpenDialog* Dialog = nullptr;
	if (SUCCEEDED(CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&Dialog))))
	{
		DWORD Options = 0;
		Dialog->GetOptions(&Options);
		Dialog->SetOptions(Options | FOS_PICKFOLDERS | FOS_FORCEFILESYSTEM | FOS_PATHMUSTEXIST);
		Dialog->SetTitle(*Title);
		FString Start = StartDir;
		FPaths::MakePlatformFilename(Start);
		IShellItem* StartItem = nullptr;
		if (!Start.IsEmpty() && SUCCEEDED(SHCreateItemFromParsingName(*Start, nullptr, IID_PPV_ARGS(&StartItem))))
		{
			Dialog->SetFolder(StartItem);
			StartItem->Release();
		}
		if (SUCCEEDED(Dialog->Show(GetActiveWindow())))
		{
			IShellItem* Result = nullptr;
			if (SUCCEEDED(Dialog->GetResult(&Result)))
			{
				PWSTR Picked = nullptr;
				if (SUCCEEDED(Result->GetDisplayName(SIGDN_FILESYSPATH, &Picked)))
				{
					OutDir = Picked;
					FPaths::NormalizeDirectoryName(OutDir);
					CoTaskMemFree(Picked);
					bPicked = true;
				}
				Result->Release();
			}
		}
		Dialog->Release();
	}
	if (SUCCEEDED(Init)) CoUninitialize();
	return bPicked;
#else
	return false;
#endif
}

bool FFSDSStackStarter::HostNetworkingOff()
{
#if PLATFORM_WINDOWS
	const FString File = FPaths::Combine(FPlatformMisc::GetEnvironmentVariable(TEXT("APPDATA")),
		TEXT("Docker"), TEXT("settings-store.json"));
	FString Text;
	if (!FFileHelper::LoadFileToString(Text, *File)) return false;   // can't tell: don't claim it
	TSharedPtr<FJsonObject> Obj;
	if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Obj) || !Obj.IsValid()) return false;
	bool bOn = false;
	return Obj->TryGetBoolField(TEXT("HostNetworkingEnabled"), bOn) ? !bOn : true;   // missing: the default, off
#else
	return false;
#endif
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
	// Hidden: only its engine is wanted, not its window over the sim. Started
	// in the user's home, not the sim's folder: it outlives the sim, and a
	// process sitting in Binaries/Win64 stops the next build or update from
	// replacing it.
	const FString Home = FPlatformMisc::GetEnvironmentVariable(TEXT("USERPROFILE"));
	FProcHandle H = FPlatformProcess::CreateProc(*Exe, TEXT(""), /*bLaunchDetached=*/true,
		/*bLaunchHidden=*/true, /*bLaunchReallyHidden=*/true, nullptr, 0, Home.IsEmpty() ? nullptr : *Home, nullptr);
#elif PLATFORM_MAC
	FProcHandle H = FPlatformProcess::CreateProc(TEXT("/usr/bin/open"), TEXT("-g -a Docker"), true,
		false, false, nullptr, 0, FPlatformProcess::UserHomeDir(), nullptr);
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
	return Run(ComposeDir, Extra.IsEmpty() ? CommandLine() : CommandLine() + TEXT(" ") + Extra, OutError);
}

bool FFSDSStackStarter::Run(const FString& WorkDir, const FString& Command, FString& OutError)
{
	if (bRunning) return false;
	AddToolDirsToPath();
	// Docker's and git's own words, whatever the PC's language: the menu
	// reads some of them (SaysDockerIsDown, a refused clone).
	FPlatformMisc::SetEnvironmentVar(TEXT("LC_ALL"), TEXT("C"));
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

#if PLATFORM_WINDOWS
	// docker-compose.yml builds paths from ${PWD} (the backend's UE5_TRACKS_DIR). A
	// shell sets it, cmd.exe doesn't: compose left it blank, the backend asked the sim
	// for /Content/tracks/<name>, and the sim loaded its own copy of the track instead
	// of the repository's (or none, for a track only the repository has).
	if (!WorkDir.IsEmpty())
	{
		FPlatformMisc::SetEnvironmentVar(TEXT("PWD"), *FPaths::ConvertRelativePathToFull(WorkDir));
	}
	// /s: cmd strips just the outer quotes, so quoted paths inside (spaces,
	// accents) reach the command as they are.
	const FString Exe = TEXT("cmd.exe");
	const FString Args = FString::Printf(TEXT("/s /c \"%s 2>&1\""), *Command);
#else
	// The command goes through a script file, not -c. CreateProc on Mac and
	// Linux splits Parms on spaces itself and only knows plain "..." pairs, so
	// any quote inside the command (a quoted path for docker compose cp) broke
	// the wrapper: zsh got "unmatched" quotes and the code copy never ran.
#if PLATFORM_MAC
	const FString Exe = TEXT("/bin/zsh");
#else
	const FString Exe = TEXT("/bin/bash");
#endif
	ScriptPath = FPaths::CreateTempFilename(FPlatformProcess::UserTempDir(), TEXT("fsds-run-"), TEXT(".sh"));
	if (!FFileHelper::SaveStringToFile(TEXT("exec 2>&1\n") + Command + TEXT("\n"), *ScriptPath,
		FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
	{
		Close();
		OutError = FString::Printf(TEXT("could not write %s"), *ScriptPath);
		return false;
	}
	const FString Args = FString::Printf(TEXT("-l \"%s\""), *ScriptPath);
#endif

	Proc = FPlatformProcess::CreateProc(*Exe, *Args, /*bLaunchDetached=*/false,
		/*bLaunchHidden=*/true, /*bLaunchReallyHidden=*/true, nullptr, 0,
		*WorkDir, WritePipe, nullptr);
	if (!Proc.IsValid())
	{
		Close();
		OutError = FString::Printf(TEXT("could not run %s"), *Exe);
		return false;
	}
	bRunning = true;
	UE_LOG(LogTemp, Log, TEXT("FSDS Launcher: '%s' in %s"), *Command, *WorkDir);
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
	UE_LOG(LogTemp, Log, TEXT("FSDS Launcher: exited %d: %s"), ReturnCode, *Last);
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
	if (!ScriptPath.IsEmpty())
	{
		IFileManager::Get().Delete(*ScriptPath, /*RequireExists=*/false, /*EvenReadOnly=*/true, /*Quiet=*/true);
		ScriptPath.Reset();
	}
}
