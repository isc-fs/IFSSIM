#pragma once

#include "CoreMinimal.h"
#include <memory>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>

class FSocket;
class AFSDSVehiclePawn;
class AFSDSReferee;
class UWorld;

/**
 * FSDS RPC Server — TCP server on port 41451.
 * Protocol: "method_name [args]\n" -> "json_response\n"
 *
 * Simulation control: simPause, simResume, simStep, simSetVehiclePose
 *   (the legacy `reset` method was removed in fix/12 — it tried to
 *    OpenLevel and crashed the editor. `simSetVehiclePose` plus the
 *    plugin's ReleaseEbs is the supported soft-reset path.)
 * Object APIs: listSceneObjects, getObjectPose, setObjectPose
 */
class FSDSPLUGIN_API FFSDSRpcServer
{
public:
	FFSDSRpcServer();
	~FFSDSRpcServer();

	void Start(uint16 Port = 41451);
	void Stop();

	/**
	 * Optional AF_UNIX (UDS) listener for high-throughput LiDAR / sensor
	 * streaming. The TCP listener (Start) stays up regardless — UDS is
	 * opt-in for localhost consumers that want to bypass the macOS TCP
	 * loopback throughput cap (~7 MB/s) and get the full kernel pipe
	 * bandwidth (typically 5+ GB/s).
	 *
	 * Path is the filesystem socket path; create the parent directory
	 * if it doesn't exist (and ensure it's mountable into the bridge's
	 * Docker container if the bridge runs containerised).
	 */
	void StartUds(const FString& SocketPath);
	void StopUds();

	void SetVehiclePawn(AFSDSVehiclePawn* Pawn) { VehiclePawn = Pawn; }
	void SetReferee(AFSDSReferee* Ref) { Referee = Ref; }
	void SetSettingsString(const FString& Settings) { SettingsString = Settings; }
	void SetWorld(UWorld* InWorld) { World = InWorld; }
	void SetUdpBroadcaster(class FFSDSUdpBroadcaster* Broadcaster) { UdpBroadcaster = Broadcaster; }

	bool IsRunning() const { return bRunning; }

private:
	void BindMethods();
	void ServerThreadFunc();
	void HandleClient(FSocket* ClientSocket);
	FString ProcessRequest(const FString& Request);
	bool ProcessBinaryRequest(const FString& Request, FSocket* ClientSocket);

	/** Streaming modes — hold connection open and push data continuously.
	 *  StreamLidar removed in #322 (TCP LiDAR retired in favour of
	 *  UDP via FSDSUdpBroadcaster::BroadcastLidarFrame). */
	void StreamSensors(FSocket* ClientSocket);
	uint32 StreamFrameCounter = 0;

	// Cached binary data for thread-safe transfer
	TArray<uint8> CachedImageData;
	TArray<float> CachedLidarData;
	FCriticalSection BinaryDataLock;

	// UDS support (opt-in via StartUds). Mirrors the TCP path but using
	// raw POSIX sockets — UE5's FSocket framework doesn't expose AF_UNIX.
	// StreamLidarUds removed in #322. StreamSensorsUds remains as a
	// no-op stub pending a real sensor-over-UDS implementation; the UDS
	// listener is otherwise idle.
	void UdsServerThreadFunc();
	void HandleUdsClient(int ClientFd);
	void StreamSensorsUds(int ClientFd);
	std::unique_ptr<std::thread> UdsServerThread;
	std::vector<std::thread> UdsClientThreads;
	std::mutex UdsClientThreadsMutex;
	std::atomic<int> UdsListenFd{-1};
	FString UdsSocketPath;
	std::atomic<bool> bUdsRunning{false};

	std::unique_ptr<std::thread> ServerThread;
	// Active per-connection worker threads. HandleClient runs in one of
	// these; Stop() joins all of them so they cannot outlive the server
	// object and dereference freed `this` in `HandleClient`'s bRunning
	// check or in any AsyncTask lambda they dispatched.
	std::vector<std::thread> ClientThreads;
	std::mutex ClientThreadsMutex;
	std::atomic<bool> bRunning{false};
	std::atomic<bool> bApiControlEnabled{false};
	std::atomic<bool> bSimPaused{false};
	uint16 ServerPort = 41451;

	AFSDSVehiclePawn* VehiclePawn = nullptr;
	AFSDSReferee* Referee = nullptr;
	UWorld* World = nullptr;
	FString SettingsString;
	class FFSDSUdpBroadcaster* UdpBroadcaster = nullptr;

	// Start-gate pose set by loadTrack when car_aligned=true. Returned by
	// getStartGatePose so the ROS bridge always resets to the right gate
	// (position AND track-aligned heading) even after the car has driven
	// away from it. Without the rotation, /reset put the car at the right
	// xy but facing whatever direction it had ended up in.
	FVector LastStartGateLoc_UE = FVector::ZeroVector;
	FQuat   LastStartGateRot_UE = FQuat::Identity;
	bool bHasStartGate = false;

	// Cached car controls for immediate readback (set from TCP thread
	// before game thread applies). Field names mirror the canonical
	// FCarControls in FSDSVehiclePawn.h: `Regen` is the rear-axle motor
	// regen demand (the only retarding channel folded into the EMRAX
	// motor command — there is no hydraulic friction brake on the IFS-08).
	struct FCachedControls {
		float Throttle = 0.f;
		float Steering = 0.f;
		float Regen = 0.f;
		bool bHandbrake = false;
		bool bIsManualGear = false;
		int32 ManualGear = 0;
		bool bGearImmediate = true;
	};
	FCachedControls CachedControls;
};
