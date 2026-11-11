#pragma once

#include "CoreMinimal.h"
#include "Containers/Ticker.h"

class UWorld;

/**
 * How the sim draws the view you watch: a quality level, the view's
 * resolution, and whether the view is drawn at all while the pipeline drives.
 * Settings, Graphics in the menu; kept in the user's GameUserSettings.ini.
 *
 * Only what the LiDAR doesn't see. The LiDAR renders its own depth and
 * colour images of the scene, and these settings were checked to leave its
 * scans unchanged (points, ranges and intensity, car at rest on Parking lot
 * A): r.ScreenPercentage, and the shadow, post-process, anti-aliasing,
 * shading, texture and effects quality groups. View distance, foliage and
 * level-of-detail settings are left alone: they decide which geometry the
 * LiDAR's own render holds. See docs/sim_timing.md.
 *
 * Low is the project's own (Config/DefaultEngine.ini: 25 % resolution, every
 * group low), the cheapest. Measured on a GTX 1660 SUPER with the LiDAR at
 * 1.74 M points/s: 51 % GPU as is, 72 % at 100 % resolution, 85 % at High
 * and 100 %, all at 60 fps; the view off takes the sim from 15 % to 6 % of a
 * 12-thread CPU.
 */
class FSDSPLUGIN_API FFSDSGraphics
{
public:
	enum class EQuality : uint8 { Low = 0, Medium = 1, High = 2 };

	struct FSettings
	{
		EQuality Quality = EQuality::Low;
		int32 ResolutionPct = 25;          // 25, 50, 75 or 100
		bool bHideViewWhileAutonomous = false;
	};

	static FFSDSGraphics& Get();

	/** Read the saved settings and apply them. */
	void Init(UWorld* InWorld);
	void Shutdown();

	const FSettings& GetSettings() const { return Settings; }
	/** Apply and save. */
	void SetQuality(EQuality Q);
	void SetResolution(int32 Pct);
	void SetHideViewWhileAutonomous(bool bHide);

	/** Draw the world in the view or not (the HUD and the menu stay). The LiDAR is unaffected. */
	void SetViewDrawn(bool bDrawn);

	/**
	 * Find the best settings for this PC: try the combinations from the best
	 * looking down, each for a couple of seconds, and keep the first whose GPU
	 * time per frame leaves a margin under the 1/60 s frame (RecommendBudgetMs),
	 * so a run holds real time with the pipeline's load on top. Done is called
	 * on the game thread with a line to show.
	 */
	void Recommend(TFunction<void(const FString&)> Done);
	bool IsRecommending() const { return TestIndex != INDEX_NONE; }
	/** While Recommend runs: what it is trying, for the menu to show. */
	FString RecommendProgress() const;

	static FText QualityName(EQuality Q);

	/** The GPU time per frame a recommendation keeps under: three quarters of a 60 fps frame. */
	static constexpr double RecommendBudgetMs = 12.5;

private:
	void Apply() const;
	void Save() const;
	bool TickRecommend(float DeltaTime);

	struct FTry { EQuality Quality; int32 ResolutionPct; };

	TWeakObjectPtr<UWorld> World;
	FSettings Settings;
	bool bViewDrawn = true;

	// Recommend's state.
	TArray<FTry> Tries;
	int32 TestIndex = INDEX_NONE;
	double TryStartS = 0.0;
	TArray<double> GpuMs;
	FSettings Before;
	TFunction<void(const FString&)> OnDone;
	FTSTicker::FDelegateHandle TestTicker;
};
