#pragma once

#include "CoreMinimal.h"
#include "Tickable.h"
#include <atomic>
#include <thread>

class AFSDSVehiclePawn;
class AFSDSReferee;
class FSocket;

/**
 * Binary frame format for UDP sensor broadcast.
 * Packed struct — same layout on UE5 side and ROS2 bridge side.
 */
#pragma pack(push, 1)

struct FFSDSSensorFrame
{
	// Header
	uint32 Magic = 0x49465353; // "IFSS"
	uint32 FrameID = 0;
	uint64 Timestamp = 0;

	// GPS (28 bytes)
	double Latitude = 0.0;
	double Longitude = 0.0;
	float Altitude = 0.f;

	// IMU (40 bytes) — body frame
	float AccelX = 0.f, AccelY = 0.f, AccelZ = 0.f;       // m/s² (divided by 100 from cm/s²)
	float GyroX = 0.f, GyroY = 0.f, GyroZ = 0.f;          // rad/s
	float OrientX = 0.f, OrientY = 0.f, OrientZ = 0.f, OrientW = 1.f; // quaternion

	// GSS (12 bytes) — body frame, m/s
	float GssVelX = 0.f, GssVelY = 0.f, GssVelZ = 0.f;

	// Odom/Pose (40 bytes) — ENU meters
	float PosX = 0.f, PosY = 0.f, PosZ = 0.f;
	float PoseOrientX = 0.f, PoseOrientY = 0.f, PoseOrientZ = 0.f, PoseOrientW = 1.f;
	float Speed = 0.f;
	float RPM = 0.f;

	// Referee (12 bytes)
	int32 DooCounter = 0;
	int32 OffTrackCounter = 0;
	int32 LapCount = 0;

	// Controls echo (12 bytes)
	float Throttle = 0.f;
	float Steering = 0.f;
	float Brake = 0.f;

	// Ground-truth body-frame velocity, clean (no GSS sensor noise) — #315.
	// Populated in PackSensorFrame from VehiclePawn->GetVelocity() rotated
	// into body frame, identical axis convention to the GSS fields above
	// (X forward, Y left, Z up — UE5/ENU). Bridge sources
	// /testing_only/odom's twist.linear from these so the odom topic is
	// fully ground-truth on both pose and twist; /fsds/gss keeps carrying
	// the noisy GSS sensor values where the noise model belongs.
	float GtVelBodyX = 0.f, GtVelBodyY = 0.f, GtVelBodyZ = 0.f; // m/s

	// Ground-truth body-frame angular velocity, clean (no IMU bias / noise).
	// Source: RootComponent->GetPhysicsAngularVelocityInRadians() rotated
	// into body frame, identical math to FSDSImuSensor::Tick (so the GT
	// gyro and the noisy IMU gyro live in exactly the same body-frame
	// convention; downstream diagnostics can diff them directly to see
	// the bias/noise the filter has to handle). Bridge sources
	// /testing_only/odom's twist.angular from these — closes the gap left
	// by #315 (which only populated twist.linear).
	float GtAngVelBodyX = 0.f, GtAngVelBodyY = 0.f, GtAngVelBodyZ = 0.f; // rad/s
};

struct FFSDSLidarChunkHeader
{
	uint32 Magic = 0x4C494452; // "LIDR"
	uint16 ChunkIndex = 0;
	uint16 TotalChunks = 0;
	uint32 FrameID = 0;
	int32 PointsInChunk = 0;
	int32 TotalPoints = 0;
	int32 Channels = 0;
	// Capture-to-send lag in nanoseconds (how long ago this scan was
	// captured, computed at packing time as `now_cycles - LidarSensor->
	// LastTimestamp` × SecondsPerCycle × 1e9). Bridge stamps the message
	// at `node_->now() - LagNs` so the ROS header.stamp reflects the
	// physical capture moment of the scan, regardless of GPU readback
	// latency or transport jitter. Self-correcting (no anchor needed).
	// Issue #238.
	int64 LagNs = 0;
	// Followed by PointsInChunk * 4 * sizeof(float) bytes of point data:
	// (x, y, z, intensity) per point — intensity ∈ [0, 1] in #255's
	// physically-grounded model. Stride was 3 floats pre-#255.
};

// Wire header for the TCP LiDAR stream (PR-#482). One header per scan,
// sent over the streaming TCP connection the bridge opens by sending
// "streamLidar\n" on port IFSSIM_PORT. Followed immediately by
// `TotalPoints * 4 * sizeof(float)` bytes of (x, y, z, intensity)
// floats — no chunking, no fragmentation handling at the application
// level (TCP is a byte stream; SendAll/readExact handle partial
// progress on either side).
//
// Why a separate struct from FFSDSLidarChunkHeader: the chunked-UDP
// header carried ChunkIndex / TotalChunks / PointsInChunk fields the
// bridge needed for reassembly. TCP doesn't need any of that — one
// header is the whole scan. Carrying the unused fields would bloat
// the wire format and confuse future readers into thinking TCP also
// chunks. Same Magic ("LIDR") so the bridge's first-bytes-magic check
// is recognisable from either path; FrameID / Channels / TotalPoints
// / LagNs semantics match the chunked header so all downstream code
// (lidar_cb_, onLidarFrame, ROS stamp recovery via LagNs) is
// transport-agnostic.
struct FFSDSLidarStreamHeader
{
	uint32 Magic = 0x4C494452; // "LIDR"
	uint32 FrameID = 0;
	int32  Channels = 0;
	int32  TotalPoints = 0;
	int64  LagNs = 0;
};
static_assert(sizeof(FFSDSLidarStreamHeader) == 24,
	"FFSDSLidarStreamHeader is the on-wire LiDAR-stream header — "
	"its size is part of the bridge↔sim ABI. If you grow or shrink "
	"this struct, bump a wire-version field instead of expecting "
	"old bridges to keep parsing.");

#pragma pack(pop)

/**
 * UDP Broadcaster — pushes sensor data from UE5 to the ROS2 bridge.
 * Broadcasts sensor frame at engine tick rate (~120Hz) on port 41452.
 * Broadcasts LiDAR point cloud at 10Hz on port 41453.
 */
class FSDSPLUGIN_API FFSDSUdpBroadcaster : public FTickableGameObject
{
public:
	FFSDSUdpBroadcaster();
	virtual ~FFSDSUdpBroadcaster();

	void Start(const FString& TargetIP = TEXT("255.255.255.255"),
		uint16 SensorPort = 41452, uint16 LidarPort = 41453);
	void Stop();

	void SetVehiclePawn(AFSDSVehiclePawn* Pawn) { VehiclePawn = Pawn; }
	void SetReferee(AFSDSReferee* Ref) { Referee = Ref; }

	/** Update target IP at runtime (called when bridge registers via TCP) */
	void SetTargetIP(const FString& IP);

	/** Toggle the UDP LiDAR broadcast at runtime.
	 *
	 *  Default OFF since PR-#482: the primary LiDAR transport is now TCP
	 *  via FFSDSRpcServer::StreamLidar, so sending UDP unconditionally
	 *  wastes ~1.5 MB/s of game-thread CPU + loopback bandwidth on packets
	 *  the bridge isn't listening for. Bridges using the legacy chunked-
	 *  UDP path (IFSSIM_LIDAR_TRANSPORT=udp) re-enable this via the
	 *  `enableLidarUdpBroadcast` RPC command at connection time.
	 *
	 *  Sensor UDP fanout (BroadcastSensorFrame) is unaffected — its
	 *  per-tick cost is two orders of magnitude lower and it's kept on
	 *  unconditionally so any future consumer of the UDP sensor stream
	 *  doesn't need a similar opt-in.
	 */
	void SetLidarBroadcastEnabled(bool bEnabled) { bLidarBroadcastEnabled.store(bEnabled); }
	bool IsLidarBroadcastEnabled() const { return bLidarBroadcastEnabled.load(); }

	// FTickableGameObject — engine ticks us automatically while bRunning.
	// Editor-only ticking is disabled (the broadcaster only does anything
	// useful when a vehicle pawn is possessed, which only happens in PIE).
	virtual void Tick(float DeltaTime) override;
	virtual bool IsTickable() const override { return bRunning && VehiclePawn != nullptr; }
	virtual bool IsTickableInEditor() const override { return false; }
	virtual bool IsTickableWhenPaused() const override { return false; }
	virtual TStatId GetStatId() const override
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FFSDSUdpBroadcaster, STATGROUP_Tickables);
	}

	bool IsRunning() const { return bRunning; }

private:
	void PackSensorFrame(FFSDSSensorFrame& Frame);
	void BroadcastSensorFrame();
	void BroadcastLidarFrame();

	AFSDSVehiclePawn* VehiclePawn = nullptr;
	AFSDSReferee* Referee = nullptr;

	FSocket* SensorSocket = nullptr;
	FSocket* LidarSocket = nullptr;

	TSharedPtr<FInternetAddr> SensorAddr;
	TSharedPtr<FInternetAddr> LidarAddr;

	std::atomic<bool> bRunning{false};

	// Runtime gate on LiDAR-over-UDP broadcasting. Default OFF (PR-#482);
	// see SetLidarBroadcastEnabled() header doc for the rationale.
	std::atomic<bool> bLidarBroadcastEnabled{false};
	// Count of LiDAR send AsyncTasks dispatched but not yet finished.
	// BroadcastLidarFrame moves the per-chunk send loop onto a background
	// worker thread; the lambda captures LidarSocket by raw pointer. If
	// Stop() races ahead and destroys the socket while a task is still
	// running, the lambda dereferences a dangling pointer (silent failure
	// at best, freed-memory access at worst — and on the next session the
	// new broadcaster's first dispatches can land in a worker pool that
	// hasn't fully drained, producing the "LiDAR goes silent after PIE
	// Stop+Play" symptom we kept hitting). Stop() spins on this counter
	// before tearing the sockets down.
	std::atomic<int32> LidarSendInFlight{0};
	uint32 FrameCounter = 0;

	// LiDAR rate limiting (broadcast every N ticks to achieve ~10Hz)
	float LidarAccumulator = 0.f;
	float LidarInterval = 0.1f; // 10Hz
};
