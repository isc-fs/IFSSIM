#include "FSDSPlugin.h"

#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "ShaderCore.h"

#if PLATFORM_WINDOWS
// Windows scheduler tick is 15.6 ms by default — so FPlatformProcess::Sleep
// (and SleepNoStats) round any sub-15.6ms request up to 15.6 ms. That
// silently wrecks two paced loops in this plugin:
//
//   - FSDSRpcServer's sensor stream targets Sleep(2.5ms) = 400 Hz, but
//     gets ~12 Hz on Windows (matches a 15.6 ms scheduler quantum plus
//     TCP send + scheduling jitter), so /imu publishes ~33× too slow.
//   - FSDSUdpBroadcaster's chunked-LiDAR pacer targets SleepNoStats(100µs)
//     between ~24 chunks per scan, but each yield costs 15.6 ms — total
//     ~374 ms per scan vs a 100 ms scan budget. The bridge's reassembly
//     window times out and every scan drops, so /lidar/Lidar1 is silent.
//
// timeBeginPeriod(1) is the Win32 multimedia-timer API call that lowers
// the system scheduler tick to 1 ms process-wide. Paired with
// timeEndPeriod(1) on shutdown so we don't leave the system in a
// higher-resolution state.
//
// This is the same fix Chrome, ffmpeg, and most game engines apply on
// Windows when they need sub-frame timing. UE5 itself does NOT do this
// globally — the RHI/render-thread paths use FEvent/spinlocks instead —
// so any plugin that relies on sub-15.6ms Sleep() must opt in.
#include "Windows/AllowWindowsPlatformTypes.h"
#include <timeapi.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

#define LOCTEXT_NAMESPACE "FFSDSPluginModule"

void FFSDSPluginModule::StartupModule()
{
#if PLATFORM_WINDOWS
	// Raise system timer resolution to 1 ms so the plugin's paced
	// Sleep() / SleepNoStats() calls actually achieve sub-2ms cadences.
	// See the header comment above for why this is the difference
	// between 12 Hz and 400 Hz on the /imu topic on Windows. The log
	// line below makes it trivial to confirm at runtime — grep the
	// IFSSIM.log for "FSDS Plugin: timer resolution raised" to
	// distinguish a missing call from a non-effective one.
	const MMRESULT TimerRes = timeBeginPeriod(1);
	UE_LOG(LogTemp, Log,
		TEXT("FSDS Plugin: timer resolution raised to 1 ms (timeBeginPeriod result=%u, 0=TIMERR_NOERROR)"),
		(uint32)TimerRes);
#endif

	// Register the plugin's Shaders/ directory under the virtual path
	// "/Plugin/FSDSPlugin" so .usf files can be referenced by
	// IMPLEMENT_GLOBAL_SHADER without absolute paths. Mirrors the
	// canonical UE5 plugin-shader bootstrap pattern; see #223 Phase 3
	// (FSDSLidarDecode.usf is the first shader to live here).
	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("FSDSPlugin"));
	if (Plugin.IsValid())
	{
		const FString ShaderDir = FPaths::Combine(Plugin->GetBaseDir(), TEXT("Shaders"));
		AddShaderSourceDirectoryMapping(TEXT("/Plugin/FSDSPlugin"), ShaderDir);
	}
}

void FFSDSPluginModule::ShutdownModule()
{
#if PLATFORM_WINDOWS
	// Release the elevated timer resolution. Paired with the
	// timeBeginPeriod(1) call in StartupModule().
	timeEndPeriod(1);
#endif
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FFSDSPluginModule, FSDSPlugin)
