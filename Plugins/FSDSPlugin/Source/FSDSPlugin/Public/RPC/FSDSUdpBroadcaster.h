#pragma once

#include "CoreMinimal.h"
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
	// Followed by PointsInChunk * 3 * sizeof(float) bytes of point data
};

#pragma pack(pop)

/**
 * UDP Broadcaster — pushes sensor data from UE5 to the ROS2 bridge.
 * Broadcasts sensor frame at engine tick rate (~120Hz) on port 41452.
 * Broadcasts LiDAR point cloud at 10Hz on port 41453.
 */
class FSDSPLUGIN_API FFSDSUdpBroadcaster
{
public:
	FFSDSUdpBroadcaster();
	~FFSDSUdpBroadcaster();

	void Start(const FString& TargetIP = TEXT("255.255.255.255"),
		uint16 SensorPort = 41452, uint16 LidarPort = 41453);
	void Stop();

	void SetVehiclePawn(AFSDSVehiclePawn* Pawn) { VehiclePawn = Pawn; }
	void SetReferee(AFSDSReferee* Ref) { Referee = Ref; }

	/** Update target IP at runtime (called when bridge registers via TCP) */
	void SetTargetIP(const FString& IP);

	/** Called each game tick to broadcast sensor data */
	void Tick(float DeltaTime);

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
	uint32 FrameCounter = 0;

	// LiDAR rate limiting (broadcast every N ticks to achieve ~10Hz)
	float LidarAccumulator = 0.f;
	float LidarInterval = 0.1f; // 10Hz
};
