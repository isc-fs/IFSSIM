#pragma once

#include "CoreMinimal.h"
#include "Containers/Ticker.h"

class FFSDSStackStarter;

/**
 * The sim updates itself from the builds published on the repo's GitHub
 * Releases (tools/publish_windows_build.sh): Settings, UPDATES.
 *
 * Two channels: Stable, the tagged releases (vX.Y.Z), and Latest, the
 * rolling `latest-dev` pre-release of the last build of dev. The repo is
 * private, so the API needs a GitHub token: the one git already keeps from
 * cloning the team's code (`git credential fill`), or $IFSSIM_GITHUB_TOKEN.
 *
 * A build knows what it is from build.json next to IFSSIM.exe (written by
 * package_windows.sh). A newer build is one published after this build's
 * commit, and with another version (Stable) or commit (Latest).
 *
 * Windows only: an update downloads the zip next to the sim's folder, then
 * a PowerShell helper waits for the sim to close, checks the checksum,
 * unzips, keeps the old folder as <folder>-previous (Back to the previous
 * build), carries over the user's settings.json and tracks, and starts the
 * new sim.
 */
class FSDSPLUGIN_API FFSDSUpdater
{
public:
	enum class EChannel : uint8 { Stable = 0, Latest = 1 };
	enum class EState : uint8 { Idle, NoBuildInfo, SigningIn, Checking, UpToDate, Available, Downloading, Failed };

	static FFSDSUpdater& Get();

	/** Reads build.json and the channel, and checks quietly (no login prompt). */
	void Init();
	void Shutdown();

	EChannel GetChannel() const { return Channel; }
	void SetChannel(EChannel InChannel);

	/** Ask GitHub. Interactive: git may open the browser to sign in. */
	void Check(bool bInteractive);
	/** Download the newer build, then hand over to the helper and quit. */
	void Update();
	bool CanRollBack() const;
	/** Swap back to <folder>-previous and restart. */
	void RollBack();

	EState GetState() const { return State; }
	bool HasUpdate() const { return State == EState::Available; }
	/** This build, as shown in Settings. */
	FString CurrentText() const;
	/** What the updater is doing or found. */
	FString StatusText() const { return Status; }

private:
	struct FBuildInfo
	{
		bool bValid = false;
		FString Version;
		FString Commit;
		FDateTime CommitTime;
	};
	struct FRelease
	{
		FString Name;
		FString Tag;
		FDateTime Published;
		FString AssetName;
		FString AssetUrl;
		int64 AssetSize = 0;
		FString ChecksumUrl;
	};

	bool Tick(float DeltaTime);
	void AskToken(bool bInteractive);
	void FetchRelease();
	void OnRelease(const FString& Body, int32 Code);
	void Download(const FString& Url, bool bWithAuth);
	void FetchChecksumThenHandOver();
	void HandOver(const FString& Sha256);
	/** The helper (update.ps1), written next to the sim's folder. */
	bool WriteHelper(FString& OutPath) const;
	void Fail(const FString& Msg);

	/** The sim's folder (the one with IFSSIM.exe). */
	static FString SimDir();

	FBuildInfo Build;
	EChannel Channel = EChannel::Stable;
	EState State = EState::Idle;
	FString Status;
	FString Token;
	FRelease Found;
	FString ZipPath;
	/** How the last update or rollback went, shown with the first check. */
	FString Note;
	int64 Received = 0;

	TSharedPtr<FFSDSStackStarter> Proc;
	bool bTokenInteractive = false;
	FTSTicker::FDelegateHandle TickHandle;
	/** Gone with Shutdown: HTTP replies check it. */
	TSharedPtr<bool> Alive;
};
