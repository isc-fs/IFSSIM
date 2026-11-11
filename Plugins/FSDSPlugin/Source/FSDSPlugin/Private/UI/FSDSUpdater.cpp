#include "UI/FSDSUpdater.h"

#include "UI/FSDSStackStarter.h"

#include "Dom/JsonObject.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformProcess.h"
#include "HttpModule.h"
#include "Interfaces/IHttpRequest.h"
#include "Interfaces/IHttpResponse.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

// Qualified everywhere (UpdaterUi::...): other UI files share the unity blob.
namespace UpdaterUi
{
	const TCHAR* Section = TEXT("FSDSUpdates");
	const TCHAR* Api = TEXT("https://api.github.com/repos/isc-fs/IFSSIM");
	const TCHAR* LatestTag = TEXT("latest-dev");
	// Each OS takes its own zip from a release (tools/publish_build.sh names them).
#if PLATFORM_WINDOWS
	const TCHAR* AssetSuffix = TEXT("-Windows-x64.zip");
	const TCHAR* PlatformName = TEXT("Windows");
#elif PLATFORM_MAC
	const TCHAR* AssetSuffix = TEXT("-macOS-arm64.zip");
	const TCHAR* PlatformName = TEXT("macOS");
#else
	const TCHAR* AssetSuffix = TEXT("-Linux-x64.zip");
	const TCHAR* PlatformName = TEXT("Linux");
#endif

	FString Mb(int64 Bytes) { return FString::Printf(TEXT("%.0f MB"), Bytes / (1024.0 * 1024.0)); }

	FString Date(const FDateTime& T) { return T.ToString(TEXT("%Y-%m-%d")); }

	void SetHeaders(const TSharedRef<IHttpRequest, ESPMode::ThreadSafe>& Req, const FString& Token, const TCHAR* Accept)
	{
		Req->SetHeader(TEXT("User-Agent"), TEXT("IFSSIM-updater"));
		Req->SetHeader(TEXT("Accept"), Accept);
		if (!Token.IsEmpty()) Req->SetHeader(TEXT("Authorization"), TEXT("Bearer ") + Token);
	}

	/** Small GETs (the release, a checksum), following a redirect without the token. */
	void GetText(const FString& Url, const FString& Token, const TCHAR* Accept, TWeakPtr<bool> Alive,
		TFunction<void(int32, const FString&)> Done)
	{
		const TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Req = FHttpModule::Get().CreateRequest();
		Req->SetURL(Url);
		Req->SetVerb(TEXT("GET"));
		Req->SetTimeout(30.f);
		SetHeaders(Req, Token, Accept);
		Req->OnProcessRequestComplete().BindLambda(
			[Accept, Alive, Done = MoveTemp(Done)](FHttpRequestPtr, FHttpResponsePtr Resp, bool bOk) mutable
			{
				if (!Alive.IsValid()) return;
				if (!bOk || !Resp.IsValid()) { Done(0, FString()); return; }
				const int32 Code = Resp->GetResponseCode();
				const FString Location = Resp->GetHeader(TEXT("Location"));
				if (Code >= 300 && Code < 400 && !Location.IsEmpty())
				{
					GetText(Location, FString(), Accept, Alive, MoveTemp(Done));
					return;
				}
				Done(Code, Resp->GetContentAsString());
			});
		Req->ProcessRequest();
	}
}

FFSDSUpdater& FFSDSUpdater::Get()
{
	static FFSDSUpdater Instance;
	return Instance;
}

FString FFSDSUpdater::SimDir()
{
	// The folder with IFSSIM.exe, Engine/ and IFSSIM/ in a packaged build.
	FString Dir = FPaths::ConvertRelativePathToFull(FPaths::RootDir());
	FPaths::NormalizeDirectoryName(Dir);
	return Dir;
}

void FFSDSUpdater::Init()
{
	Alive = MakeShared<bool>(true);
	if (GConfig)
	{
		int32 C = 0;
		GConfig->GetInt(UpdaterUi::Section, TEXT("Channel"), C, GGameUserSettingsIni);
		Channel = C == 1 ? EChannel::Latest : EChannel::Stable;
	}

	Build = FBuildInfo();
	FString Text;
	TSharedPtr<FJsonObject> Obj;
	if (FFileHelper::LoadFileToString(Text, *FPaths::Combine(SimDir(), TEXT("build.json")))
		&& FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Obj) && Obj.IsValid())
	{
		FString When;
		Obj->TryGetStringField(TEXT("version"), Build.Version);
		Obj->TryGetStringField(TEXT("commit"), Build.Commit);
		Obj->TryGetStringField(TEXT("commit_time"), When);
		Build.bValid = !Build.Commit.IsEmpty() && FDateTime::ParseIso8601(*When, Build.CommitTime);
	}
	if (!Build.bValid)
	{
		State = EState::NoBuildInfo;
		Status = TEXT("This build has no build.json (it wasn't published), so it doesn't update itself.");
		return;
	}

	// How the last update went (the helper's log), once.
	const FString Log = SimDir() + TEXT("-update/update.log");
	TArray<FString> Lines;
	if (FFileHelper::LoadFileToStringArray(Lines, *Log) && Lines.Num() > 0)
	{
		const FString& Last = Lines.Last();
		// Kept in front of the first check's result (OnRelease).
		Note = Last.Contains(TEXT("failed")) ? TEXT("The last update failed: ") + Last.Mid(Last.Find(TEXT("failed")) + 8) + TEXT(".")
			: Last.Contains(TEXT("rolled back")) ? FString(TEXT("Back on the previous build."))
			: FString(TEXT("Updated."));
		Status = Note;
		IFileManager::Get().Delete(*Log);
	}

	TickHandle = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateRaw(this, &FFSDSUpdater::Tick));
	Check(/*bInteractive=*/false);
	CheckStack(/*bFetch=*/true);
}

void FFSDSUpdater::Shutdown()
{
	if (TickHandle.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(TickHandle);
		TickHandle.Reset();
	}
	Alive.Reset();
	Proc.Reset();
	StackProc.Reset();
	OnStackMoved = nullptr;
	if (StackState == EStack::Checking || StackState == EStack::Moving) StackState = EStack::Unknown;
	if (State == EState::SigningIn || State == EState::Checking || State == EState::Downloading) State = EState::Idle;
}

void FFSDSUpdater::SetChannel(EChannel InChannel)
{
	if (Channel == InChannel) return;
	Channel = InChannel;
	if (GConfig)
	{
		GConfig->SetInt(UpdaterUi::Section, TEXT("Channel"), (int32)Channel, GGameUserSettingsIni);
		GConfig->Flush(false, GGameUserSettingsIni);
	}
	if (State != EState::NoBuildInfo && State != EState::Downloading) Check(/*bInteractive=*/false);
}

FString FFSDSUpdater::CurrentText() const
{
	if (!Build.bValid) return TEXT("Built here (no build.json)");
	return FString::Printf(TEXT("%s, %s (%s)"), *Build.Version, *Build.Commit, *UpdaterUi::Date(Build.CommitTime));
}

void FFSDSUpdater::Fail(const FString& Msg)
{
	State = EState::Failed;
	Status = Msg;
	UE_LOG(LogTemp, Warning, TEXT("FSDS Updater: %s"), *Msg);
}

// ---------------------------------------------------------------------------
// Check

void FFSDSUpdater::Check(bool bInteractive)
{
	if (State == EState::NoBuildInfo || State == EState::SigningIn || State == EState::Checking || State == EState::Downloading) return;
	// The repo is public: no sign-in. A token (IFSSIM_GITHUB_TOKEN, or git's
	// GitHub login after a refusal) only lifts GitHub's 60 checks an hour.
	if (Token.IsEmpty()) Token = FPlatformMisc::GetEnvironmentVariable(TEXT("IFSSIM_GITHUB_TOKEN"));
	bTokenInteractive = bInteractive;
	bTriedSignIn = false;
	FetchRelease();
}

void FFSDSUpdater::AskToken(bool bInteractive)
{
	if (!FFSDSStackStarter::HasTool(TEXT("git")))
	{
		State = EState::Idle;
		Status = TEXT("Not checked: GitHub refused (too many checks from this network?) and git, which would sign in, "
			"isn't installed. Try again in an hour, or set IFSSIM_GITHUB_TOKEN.");
		return;
	}
	if (!Proc.IsValid()) Proc = MakeShared<FFSDSStackStarter>();
	// The login git keeps for github.com (Git Credential Manager). Quiet
	// checks never prompt; Check now may open the browser to sign in.
	// The query from a file: cmd's echo would add stray spaces, which git
	// rejects ("invalid credential line").
	const FString Query = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("ifssim-credential-query.txt"));
	FFileHelper::SaveStringToFile(FString(TEXT("protocol=https\nhost=github.com\n\n")), *Query, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
	FString QueryWin = Query;
	FPaths::MakePlatformFilename(QueryWin);
	const FString Ask = FString::Printf(TEXT("git credential fill < \"%s\""), *QueryWin);
	const FString Cmd = bInteractive ? Ask : TEXT("set GCM_INTERACTIVE=never&& set GIT_TERMINAL_PROMPT=0&& ") + Ask;
	FString Error;
	if (!Proc->Run(FPlatformProcess::UserTempDir(), Cmd, Error))
	{
		Fail(FString::Printf(TEXT("Couldn't ask git for the GitHub login: %s"), *Error));
		return;
	}
	bTokenInteractive = bInteractive;
	State = EState::SigningIn;
	Status = bInteractive ? TEXT("Signing in to GitHub (your browser may open)...") : TEXT("Checking...");
}

bool FFSDSUpdater::Tick(float)
{
	if (State == EState::SigningIn && Proc.IsValid())
	{
		if (Proc->Poll()) return true;
		TArray<FString> Lines;
		Proc->Output().ParseIntoArrayLines(Lines);
		for (const FString& L : Lines)
		{
			if (L.StartsWith(TEXT("password="))) Token = L.Mid(9).TrimStartAndEnd();
		}
		if (Token.IsEmpty())
		{
			State = bTokenInteractive ? EState::Failed : EState::Idle;
			Status = bTokenInteractive
				? FString(TEXT("GitHub refused the check and signing in didn't work. Try again later, or set IFSSIM_GITHUB_TOKEN."))
				: FString(TEXT("Not checked: GitHub refused (too many checks from this network?). Check now signs in."));
			return true;
		}
		FetchRelease();
	}
	if ((StackState == EStack::Checking || StackState == EStack::Moving) && StackProc.IsValid() && !StackProc->Poll())
	{
		if (StackState == EStack::Checking)
		{
			OnStackChecked();
		}
		else if (StackProc->ExitCode() == 0)
		{
			StackState = EStack::InStep;
			StackStatus = FString::Printf(TEXT("Moved to %s%s: in step once Rebuild pipeline, above, has finished."), *Build.Commit,
				bMovePipeline ? TEXT(", the pipeline too") : TEXT("; your pipeline work is as it was"));
			if (OnStackMoved) OnStackMoved();
		}
		else
		{
			StackState = EStack::Failed;
			StackStatus = FString::Printf(TEXT("Couldn't move the team's code: %s. Nothing else changed."), *StackProc->LastLine().Left(140));
		}
	}
	if (State == EState::Downloading)
	{
		Status = Found.AssetSize > 0
			? FString::Printf(TEXT("Downloading %s: %s of %s..."), *Found.Tag, *UpdaterUi::Mb(Received), *UpdaterUi::Mb(Found.AssetSize))
			: FString::Printf(TEXT("Downloading %s: %s..."), *Found.Tag, *UpdaterUi::Mb(Received));
	}
	return true;
}

void FFSDSUpdater::FetchRelease()
{
	State = EState::Checking;
	Status = TEXT("Checking...");
	const FString Url = Channel == EChannel::Latest
		? FString::Printf(TEXT("%s/releases/tags/%s"), UpdaterUi::Api, UpdaterUi::LatestTag)
		: FString::Printf(TEXT("%s/releases/latest"), UpdaterUi::Api);
	UpdaterUi::GetText(Url, Token, TEXT("application/vnd.github+json"), Alive,
		[this](int32 Code, const FString& Body) { OnRelease(Body, Code); });
}

void FFSDSUpdater::OnRelease(const FString& Body, int32 Code)
{
	if (Code == 0) { Fail(TEXT("Couldn't reach GitHub.")); return; }
	if (Code == 401 || Code == 403)
	{
		// Without a token: GitHub's limit for anonymous checks. Sign in once
		// with git's login and ask again.
		if (!bTriedSignIn)
		{
			bTriedSignIn = true;
			Token.Reset();
			AskToken(bTokenInteractive);
			return;
		}
		Token.Reset();
		Fail(TEXT("GitHub refused the check. Try again later, or set IFSSIM_GITHUB_TOKEN."));
		return;
	}
	if (Code == 404)
	{
		State = EState::UpToDate;
		Status = Channel == EChannel::Latest ? TEXT("No Latest build published yet.") : TEXT("No release published yet.");
		return;
	}
	TSharedPtr<FJsonObject> Obj;
	if (Code != 200 || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Body), Obj) || !Obj.IsValid())
	{
		Fail(FString::Printf(TEXT("GitHub answered %d."), Code));
		return;
	}

	FRelease R;
	FString When;
	Obj->TryGetStringField(TEXT("tag_name"), R.Tag);
	Obj->TryGetStringField(TEXT("name"), R.Name);
	Obj->TryGetStringField(TEXT("published_at"), When);
	FDateTime::ParseIso8601(*When, R.Published);
	const TArray<TSharedPtr<FJsonValue>>* Assets = nullptr;
	if (Obj->TryGetArrayField(TEXT("assets"), Assets))
	{
		for (const TSharedPtr<FJsonValue>& V : *Assets)
		{
			const TSharedPtr<FJsonObject> A = V->AsObject();
			FString Name, Url;
			if (!A.IsValid() || !A->TryGetStringField(TEXT("name"), Name) || !A->TryGetStringField(TEXT("url"), Url)) continue;
			if (Name.EndsWith(UpdaterUi::AssetSuffix))
			{
				R.AssetName = Name;
				R.AssetUrl = Url;
				A->TryGetNumberField(TEXT("size"), R.AssetSize);
			}
		}
		for (const TSharedPtr<FJsonValue>& V : *Assets)
		{
			const TSharedPtr<FJsonObject> A = V->AsObject();
			FString Name, Url;
			if (A.IsValid() && A->TryGetStringField(TEXT("name"), Name) && A->TryGetStringField(TEXT("url"), Url)
				&& !R.AssetName.IsEmpty() && Name == R.AssetName + TEXT(".sha256"))
			{
				R.ChecksumUrl = Url;
			}
		}
	}
	if (R.AssetName.IsEmpty())
	{
		State = EState::UpToDate;
		Status = FString::Printf(TEXT("%s has no %s build yet."), *R.Tag, UpdaterUi::PlatformName);
		return;
	}

	// Newer: published after this build's commit, and not this build.
	const bool bSame = Channel == EChannel::Latest
		? R.AssetName.Contains(Build.Commit)
		: R.Tag == TEXT("v") + Build.Version;
	Found = R;
	if (R.Published > Build.CommitTime && !bSame)
	{
		State = EState::Available;
		// The tag, not the release's title: a title can name another platform.
		Status = FString::Printf(TEXT("New build: %s, published %s (%s)."),
			Channel == EChannel::Latest ? *R.AssetName : *R.Tag, *UpdaterUi::Date(R.Published), *UpdaterUi::Mb(R.AssetSize));
	}
	else
	{
		State = EState::UpToDate;
		Status = FString::Printf(TEXT("Up to date (the newest %s build: %s, %s)."),
			Channel == EChannel::Latest ? TEXT("Latest") : TEXT("Stable"), *R.Tag, *UpdaterUi::Date(R.Published));
	}
	if (!Note.IsEmpty())
	{
		Status = Note + TEXT(" ") + Status;
		Note.Reset();
	}
	UE_LOG(LogTemp, Log, TEXT("FSDS Updater: %s"), *Status);
}

// ---------------------------------------------------------------------------
// Update

void FFSDSUpdater::Update()
{
#if PLATFORM_WINDOWS
	if (State != EState::Available) return;
	const FString Work = SimDir() + TEXT("-update");
	IFileManager::Get().MakeDirectory(*Work, true);
	ZipPath = FPaths::Combine(Work, Found.AssetName);
	Received = 0;
	State = EState::Downloading;
	Download(Found.AssetUrl, /*bWithAuth=*/true);
#else
	Fail(TEXT("Updates install themselves on Windows only: download the build from the repo's Releases."));
#endif
}

void FFSDSUpdater::Download(const FString& Url, bool bWithAuth)
{
	// Straight to disk: the zip is hundreds of MB.
	FArchive* File = IFileManager::Get().CreateFileWriter(*ZipPath);
	if (!File)
	{
		Fail(FString::Printf(TEXT("Couldn't write %s."), *ZipPath));
		return;
	}
	const TSharedRef<FArchive> Stream = MakeShareable(File);
	const TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Req = FHttpModule::Get().CreateRequest();
	Req->SetURL(Url);
	Req->SetVerb(TEXT("GET"));
	Req->SetTimeout(3600.f);
	Req->SetActivityTimeout(120.f);
	UpdaterUi::SetHeaders(Req, bWithAuth ? Token : FString(), TEXT("application/octet-stream"));
	Req->SetResponseBodyReceiveStream(Stream);
	const TWeakPtr<bool> Token_ = Alive;
	Req->OnRequestProgress64().BindLambda([this, Token_](FHttpRequestPtr, uint64, uint64 Got)
	{
		if (Token_.IsValid()) Received = (int64)Got;
	});
	Req->OnProcessRequestComplete().BindLambda([this, Token_, Stream](FHttpRequestPtr, FHttpResponsePtr Resp, bool bOk)
	{
		Stream->Close();
		if (!Token_.IsValid()) return;
		if (!bOk || !Resp.IsValid()) { Fail(TEXT("The download stopped: check the connection, then Update again.")); return; }
		const int32 Code = Resp->GetResponseCode();
		const FString Location = Resp->GetHeader(TEXT("Location"));
		if (Code >= 300 && Code < 400 && !Location.IsEmpty())
		{
			// GitHub hands the file over from storage: no token there.
			Download(Location, /*bWithAuth=*/false);
			return;
		}
		if (Code != 200) { Fail(FString::Printf(TEXT("The download failed: GitHub answered %d."), Code)); return; }
		FetchChecksumThenHandOver();
	});
	Req->ProcessRequest();
}

void FFSDSUpdater::FetchChecksumThenHandOver()
{
	if (Found.ChecksumUrl.IsEmpty())
	{
		HandOver(FString());
		return;
	}
	Status = TEXT("Checking the download...");
	UpdaterUi::GetText(Found.ChecksumUrl, Token, TEXT("application/octet-stream"), Alive,
		[this](int32 Code, const FString& Body)
		{
			// "<64 hex>  <name>", as sha256sum writes it.
			const FString Sha = Body.TrimStartAndEnd().Left(64);
			if (Code != 200 || Sha.Len() != 64)
			{
				Fail(TEXT("Couldn't get the build's checksum: Update again."));
				return;
			}
			HandOver(Sha);
		});
}

bool FFSDSUpdater::CanRollBack() const
{
#if PLATFORM_WINDOWS
	return Build.bValid && FPaths::FileExists(FPaths::Combine(SimDir() + TEXT("-previous"), TEXT("IFSSIM.exe")));
#else
	return false;
#endif
}

bool FFSDSUpdater::WriteHelper(FString& OutPath) const
{
	// Runs after the sim has closed: Windows can't replace a running exe.
	static const TCHAR* Script = TEXT(R"PS(param([int]$ProcId, [string]$Zip = '', [string]$Sha = '', [string]$Target, [switch]$Rollback)
$ErrorActionPreference = 'Stop'
$Work = "$Target-update"
New-Item -ItemType Directory -Force -Path $Work | Out-Null
$Log = Join-Path $Work 'update.log'
function Log([string]$m) { Add-Content -LiteralPath $Log -Value ((Get-Date -Format s) + ' ' + $m) -Encoding UTF8 }
# A folder can stay busy for a moment after the sim exits (its launcher, antivirus).
function Retry([scriptblock]$b) { for ($i = 0; $i -lt 20; $i++) { try { & $b; return } catch { Start-Sleep -Milliseconds 500 } }; & $b }
$Prev = "$Target-previous"
try { Wait-Process -Id $ProcId -Timeout 120 -ErrorAction SilentlyContinue } catch {}
Start-Sleep -Seconds 2
try {
  if ($Rollback) {
    $Tmp = "$Target-rollback"
    if (Test-Path -LiteralPath $Tmp) { Remove-Item -LiteralPath $Tmp -Recurse -Force }
    Retry { Rename-Item -LiteralPath $Target -NewName (Split-Path $Tmp -Leaf) }
    Retry { Rename-Item -LiteralPath $Prev -NewName (Split-Path $Target -Leaf) }
    Retry { Rename-Item -LiteralPath $Tmp -NewName (Split-Path $Prev -Leaf) }
    Log 'rolled back'
  } else {
    if ($Sha) {
      $h = (Get-FileHash -LiteralPath $Zip -Algorithm SHA256).Hash
      if ($h -ne $Sha.ToUpper()) { throw "the download's checksum doesn't match" }
    }
    $New = Join-Path $Work 'new'
    if (Test-Path -LiteralPath $New) { Remove-Item -LiteralPath $New -Recurse -Force }
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    [System.IO.Compression.ZipFile]::ExtractToDirectory($Zip, $New)
    if (-not (Test-Path -LiteralPath (Join-Path $New 'IFSSIM.exe'))) { throw 'the zip has no IFSSIM.exe' }
    # The user's own: a settings.json they changed is kept beside the new one,
    # and tracks they added come along.
    $os = Join-Path $Target 'settings.json'; $ns = Join-Path $New 'settings.json'
    if ((Test-Path -LiteralPath $os) -and (Test-Path -LiteralPath $ns) -and
        ((Get-FileHash -LiteralPath $os).Hash -ne (Get-FileHash -LiteralPath $ns).Hash)) {
      Copy-Item -LiteralPath $os -Destination (Join-Path $New 'settings.previous.json') -Force
    }
    $ot = Join-Path $Target 'tracks'; $nt = Join-Path $New 'tracks'
    if (Test-Path -LiteralPath $ot) {
      Get-ChildItem -LiteralPath $ot -Recurse -File | ForEach-Object {
        $dst = Join-Path $nt ($_.FullName.Substring($ot.Length).TrimStart('\'))
        if (-not (Test-Path -LiteralPath $dst)) {
          New-Item -ItemType Directory -Force -Path (Split-Path $dst) | Out-Null
          Copy-Item -LiteralPath $_.FullName -Destination $dst
        }
      }
    }
    if (Test-Path -LiteralPath $Prev) { Remove-Item -LiteralPath $Prev -Recurse -Force }
    Retry { Rename-Item -LiteralPath $Target -NewName (Split-Path $Prev -Leaf) }
    Retry { Move-Item -LiteralPath $New -Destination $Target }
    Remove-Item -LiteralPath $Zip -Force -ErrorAction SilentlyContinue
    Log 'updated'
  }
} catch {
  Log ('failed: ' + $_.Exception.Message)
  if ((-not (Test-Path -LiteralPath $Target)) -and (Test-Path -LiteralPath $Prev)) {
    Rename-Item -LiteralPath $Prev -NewName (Split-Path $Target -Leaf)
  }
}
Start-Process -FilePath (Join-Path $Target 'IFSSIM.exe') -WorkingDirectory $Target
)PS");
	OutPath = FPaths::Combine(SimDir() + TEXT("-update"), TEXT("update.ps1"));
	IFileManager::Get().MakeDirectory(*FPaths::GetPath(OutPath), true);
	return FFileHelper::SaveStringToFile(FString(Script), *OutPath, FFileHelper::EEncodingOptions::ForceUTF8);
}

void FFSDSUpdater::HandOver(const FString& Sha256)
{
#if PLATFORM_WINDOWS
	FString ScriptPath;
	if (!WriteHelper(ScriptPath)) { Fail(TEXT("Couldn't write the update helper.")); return; }
	auto Win = [](FString P) { FPaths::MakePlatformFilename(P); return P; };
	FString Args = FString::Printf(TEXT("-NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File \"%s\" -ProcId %u -Target \"%s\""),
		*Win(ScriptPath), FPlatformProcess::GetCurrentProcessId(), *Win(SimDir()));
	if (!ZipPath.IsEmpty()) Args += FString::Printf(TEXT(" -Zip \"%s\""), *Win(ZipPath));
	if (!Sha256.IsEmpty()) Args += FString::Printf(TEXT(" -Sha %s"), *Sha256);
	if (ZipPath.IsEmpty()) Args += TEXT(" -Rollback");
	const FString Home = FPlatformMisc::GetEnvironmentVariable(TEXT("USERPROFILE"));
	FProcHandle H = FPlatformProcess::CreateProc(TEXT("powershell.exe"), *Args, /*bLaunchDetached=*/true,
		/*bLaunchHidden=*/true, /*bLaunchReallyHidden=*/true, nullptr, 0, *Home, nullptr);
	if (!H.IsValid()) { Fail(TEXT("Couldn't start the update helper.")); return; }
	FPlatformProcess::CloseProc(H);
	Status = TEXT("Restarting...");
	UE_LOG(LogTemp, Log, TEXT("FSDS Updater: handing over to %s"), *ScriptPath);
	FPlatformMisc::RequestExit(false);
#endif
}

// ---------------------------------------------------------------------------
// The stack in step with the sim

bool FFSDSUpdater::RunStackHelper(const FString& Mode, bool bFetch, bool bPipeline)
{
	static const TCHAR* Script = TEXT(R"PS(param([string]$Mode, [string]$Repo, [string]$Commit, [switch]$Fetch, [switch]$MovePipeline)
$ErrorActionPreference = 'Continue'
Set-Location -LiteralPath $Repo
if ($Mode -eq 'apply') {
  # Only what the sim offered: dev fast-forwards, no branch moves to the commit.
  $branch = (git rev-parse --abbrev-ref HEAD).Trim()
  if ($branch -eq 'HEAD') { git checkout --quiet --detach $Commit } else { git merge --quiet --ff-only $Commit }
  if ($LASTEXITCODE -ne 0) { Write-Output 'git refused to move it'; exit 1 }
  if ($MovePipeline) {
    git submodule update --init pipeline
    if ($LASTEXITCODE -ne 0) { Write-Output 'git could not move the pipeline'; exit 1 }
  }
  Write-Output 'ok'
  exit 0
}
$r = [ordered]@{}
if ($Fetch) { git fetch --quiet origin 2>$null; $r.fetched = ($LASTEXITCODE -eq 0) }
$r.head = (git rev-parse --short HEAD).Trim()
$r.branch = (git rev-parse --abbrev-ref HEAD).Trim()
$r.dirty = [bool](git status --porcelain --untracked-files=no --ignore-submodules=all)
git cat-file -e "$Commit^{commit}" 2>$null
$r.known = ($LASTEXITCODE -eq 0)
$r.behind = $false; $r.contains = $false
if ($r.known) {
  git merge-base --is-ancestor $Commit HEAD; $r.contains = ($LASTEXITCODE -eq 0)
  git merge-base --is-ancestor HEAD $Commit; $r.behind = ($LASTEXITCODE -eq 0)
}
$p = Join-Path $Repo 'pipeline'
git -C $p symbolic-ref -q HEAD 2>$null | Out-Null; $r.pipelineBranch = ($LASTEXITCODE -eq 0)
$r.pipelineDirty = [bool](git -C $p status --porcelain --untracked-files=no 2>$null)
$s = [string](git submodule status pipeline 2>$null)
$r.pipelineMoved = $s.StartsWith('+')
Write-Output ('STACK ' + ($r | ConvertTo-Json -Compress))
)PS");
	StackRepo = FFSDSStackStarter::FindComposeDir();
	if (StackRepo.IsEmpty() || !FFSDSStackStarter::HasTool(TEXT("git"))) return false;
	const FString Path = FPaths::Combine(FPlatformProcess::UserTempDir(), TEXT("ifssim-stack.ps1"));
	if (!FFileHelper::SaveStringToFile(FString(Script), *Path, FFileHelper::EEncodingOptions::ForceUTF8)) return false;
	auto Win = [](FString P) { FPaths::MakePlatformFilename(P); return P; };
	FString Cmd = FString::Printf(TEXT("set GCM_INTERACTIVE=never&& set GIT_TERMINAL_PROMPT=0&& powershell -NoProfile -ExecutionPolicy Bypass -File \"%s\" -Mode %s -Repo \"%s\" -Commit %s"),
		*Win(Path), *Mode, *Win(StackRepo), *Build.Commit);
	if (bFetch) Cmd += TEXT(" -Fetch");
	if (bPipeline) Cmd += TEXT(" -MovePipeline");
	if (!StackProc.IsValid()) StackProc = MakeShared<FFSDSStackStarter>();
	FString Error;
	return StackProc->Run(StackRepo, Cmd, Error);
}

void FFSDSUpdater::CheckStack(bool bFetch)
{
	if (!Build.bValid || StackState == EStack::Checking || StackState == EStack::Moving) return;
	if (!RunStackHelper(TEXT("check"), bFetch, false))
	{
		StackState = EStack::Unknown;
		StackStatus = FString();
		return;
	}
	StackState = EStack::Checking;
	StackStatus = TEXT("Comparing the team's code with this build...");
}

void FFSDSUpdater::OnStackChecked()
{
	FString Json;
	TArray<FString> Lines;
	StackProc->Output().ParseIntoArrayLines(Lines);
	for (const FString& L : Lines)
	{
		if (L.StartsWith(TEXT("STACK "))) Json = L.Mid(6);
	}
	TSharedPtr<FJsonObject> R;
	if (Json.IsEmpty() || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Json), R) || !R.IsValid())
	{
		StackState = EStack::Failed;
		StackStatus = TEXT("Couldn't compare the team's code with this build.");
		return;
	}
	auto B = [&R](const TCHAR* K) { bool V = false; R->TryGetBoolField(K, V); return V; };
	FString Head, Branch;
	R->TryGetStringField(TEXT("head"), Head);
	R->TryGetStringField(TEXT("branch"), Branch);
	const bool bPipelineWork = B(TEXT("pipelineBranch")) || B(TEXT("pipelineDirty")) || B(TEXT("pipelineMoved"));
	// If the pipeline is kept: the bridge's messages live in the pipeline too.
	const FString MsgNote = TEXT(" If the pipeline won't start after the rebuild, bring your pipeline branch up to date: "
		"the team may have changed fs_msgs or dv_msgs.");

	if (!B(TEXT("known")))
	{
		StackState = EStack::Differs;
		StackStatus = FString::Printf(TEXT("This build's commit (%s) isn't in your clone (not pushed, or no network): "
			"the stack is built from %s, which may not match the sim."), *Build.Commit, *Head);
	}
	else if (B(TEXT("contains")))
	{
		StackState = Head.StartsWith(Build.Commit) || Build.Commit.StartsWith(Head) ? EStack::InStep : EStack::Newer;
		StackStatus = StackState == EStack::InStep
			? FString::Printf(TEXT("In step: the stack is built from %s, as this build."), *Head)
			: FString::Printf(TEXT("Your code (%s) is newer than this build (%s): fine, unless the sim side changed since."), *Head, *Build.Commit);
	}
	else if (B(TEXT("behind")) && !B(TEXT("dirty")) && (Branch == TEXT("dev") || Branch == TEXT("HEAD")))
	{
		StackState = EStack::Behind;
		bMovePipeline = !bPipelineWork;
		StackStatus = FString::Printf(TEXT("Your code (%s) is older than this build (%s). Bring the stack in step moves it "
			"to %s%s, then rebuilds."), *Head, *Build.Commit, *Build.Commit,
			bMovePipeline ? TEXT(", the pipeline too") : TEXT(" and leaves your pipeline work as it is"));
		if (!bMovePipeline) StackStatus += MsgNote;
	}
	else
	{
		StackState = EStack::Differs;
		const FString Why = B(TEXT("dirty")) ? TEXT("it has changes")
			: (Branch != TEXT("dev") && Branch != TEXT("HEAD")) ? FString::Printf(TEXT("it's on your branch %s"), *Branch)
			: FString(TEXT("it has commits this build doesn't"));
		StackStatus = FString::Printf(TEXT("The stack is built from %s, not this build's %s, and %s, so nothing was moved. "
			"Runs may misbehave if the bridge or the backend changed in between."), *Head, *Build.Commit, *Why);
	}
	UE_LOG(LogTemp, Log, TEXT("FSDS Updater: stack: %s"), *StackStatus);
}

void FFSDSUpdater::MatchStack()
{
	if (StackState != EStack::Behind) return;
	if (!RunStackHelper(TEXT("apply"), false, bMovePipeline))
	{
		StackState = EStack::Failed;
		StackStatus = TEXT("Couldn't run git to move the team's code.");
		return;
	}
	StackState = EStack::Moving;
	StackStatus = FString::Printf(TEXT("Moving the team's code to %s..."), *Build.Commit);
}

void FFSDSUpdater::RollBack()
{
	if (!CanRollBack() || State == EState::Downloading) return;
	ZipPath.Reset();
	HandOver(FString());
}
