#include "RPC/FSDSUdpBroadcaster.h"
#include "FSDSVehiclePawn.h"
#include "FSDSReferee.h"
#include "FSDSCoordinates.h"
#include "Sensors/FSDSGpsSensor.h"
#include "Sensors/FSDSImuSensor.h"
#include "Sensors/FSDSGssSensor.h"
#include "Sensors/FSDSLidarSensor.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Common/UdpSocketBuilder.h"
#include "Async/Async.h"
#include "HAL/PlatformProcess.h"

FFSDSUdpBroadcaster::FFSDSUdpBroadcaster()
{
}

FFSDSUdpBroadcaster::~FFSDSUdpBroadcaster()
{
	Stop();
}

void FFSDSUdpBroadcaster::Start(const FString& TargetIP, uint16 SensorPort, uint16 LidarPort)
{
	if (bRunning) return;

	ISocketSubsystem* SocketSub = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);

	// Create sensor UDP socket
	SensorSocket = FUdpSocketBuilder(TEXT("SensorBroadcast"))
		.AsReusable()
		.WithBroadcast()
		.Build();

	if (!SensorSocket)
	{
		UE_LOG(LogTemp, Error, TEXT("FSDS UDP: Failed to create sensor socket"));
		return;
	}
	SensorSocket->SetBroadcast(true);

	// Create LiDAR UDP socket
	LidarSocket = FUdpSocketBuilder(TEXT("LidarBroadcast"))
		.AsReusable()
		.WithBroadcast()
		.Build();

	if (!LidarSocket)
	{
		UE_LOG(LogTemp, Error, TEXT("FSDS UDP: Failed to create LiDAR socket"));
		return;
	}
	LidarSocket->SetBroadcast(true);

	// Bump SO_SNDBUF on the LiDAR socket. macOS's default UDP send buffer
	// is tiny (~9 KB on Sequoia / Sonoma — same as net.inet.udp.maxdgram);
	// without this, BroadcastLidarFrame's tight loop of ~250 sendto's per
	// scan saturates the buffer and the kernel silently drops every send
	// after the first. Granted size is min(requested, kern.ipc.maxsockbuf
	// = 8 MB by default). 4 MB lets a full 1.74 M pts/s scan (~2 MB at
	// PointsPerChunk=700) sit in the buffer at once.
	int32 GrantedSndBuf = 0;
	LidarSocket->SetSendBufferSize(4 * 1024 * 1024, GrantedSndBuf);
	UE_LOG(LogTemp, Log, TEXT("FSDS UDP: LiDAR SO_SNDBUF granted = %d bytes"), GrantedSndBuf);

	// Set target addresses
	SensorAddr = SocketSub->CreateInternetAddr();
	bool bIsValid = false;
	SensorAddr->SetIp(*TargetIP, bIsValid);
	if (!bIsValid)
	{
		// Fallback to broadcast
		SensorAddr->SetBroadcastAddress();
	}
	SensorAddr->SetPort(SensorPort);

	LidarAddr = SocketSub->CreateInternetAddr();
	LidarAddr->SetIp(*TargetIP, bIsValid);
	if (!bIsValid)
	{
		LidarAddr->SetBroadcastAddress();
	}
	LidarAddr->SetPort(LidarPort);

	bRunning = true;
	FrameCounter = 0;

	UE_LOG(LogTemp, Log, TEXT("FSDS UDP: Broadcasting sensors on port %d, LiDAR on port %d (target: %s)"),
		SensorPort, LidarPort, *TargetIP);
}

void FFSDSUdpBroadcaster::SetTargetIP(const FString& IP)
{
	if (!SensorAddr.IsValid() || !LidarAddr.IsValid()) return;

	ISocketSubsystem* SocketSub = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	bool bIsValid = false;

	uint16 SensorPort = SensorAddr->GetPort();
	uint16 LidarPort = LidarAddr->GetPort();

	SensorAddr = SocketSub->CreateInternetAddr();
	SensorAddr->SetIp(*IP, bIsValid);
	SensorAddr->SetPort(SensorPort);

	LidarAddr = SocketSub->CreateInternetAddr();
	LidarAddr->SetIp(*IP, bIsValid);
	LidarAddr->SetPort(LidarPort);

	UE_LOG(LogTemp, Log, TEXT("FSDS UDP: Target updated to %s (sensors:%d, lidar:%d)"),
		*IP, SensorPort, LidarPort);
}

void FFSDSUdpBroadcaster::Stop()
{
	bRunning = false;

	if (SensorSocket)
	{
		SensorSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(SensorSocket);
		SensorSocket = nullptr;
	}

	if (LidarSocket)
	{
		LidarSocket->Close();
		ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(LidarSocket);
		LidarSocket = nullptr;
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS UDP: Broadcaster stopped"));
}

void FFSDSUdpBroadcaster::Tick(float DeltaTime)
{
	if (!bRunning || !VehiclePawn) return;

	// Always broadcast sensor frame (every tick = engine frame rate)
	BroadcastSensorFrame();

	// LiDAR at ~10Hz
	LidarAccumulator += DeltaTime;
	if (LidarAccumulator >= LidarInterval)
	{
		LidarAccumulator -= LidarInterval;
		BroadcastLidarFrame();
	}
}

void FFSDSUdpBroadcaster::PackSensorFrame(FFSDSSensorFrame& Frame)
{
	Frame.Magic = 0x49465353;
	Frame.FrameID = FrameCounter++;
	Frame.Timestamp = FPlatformTime::Cycles64();

	// GPS
	if (VehiclePawn->GpsSensor)
	{
		auto Gps = VehiclePawn->GpsSensor->GetOutput();
		Frame.Latitude = Gps.Latitude;
		Frame.Longitude = Gps.Longitude;
		Frame.Altitude = Gps.Altitude;
	}

	// IMU
	if (VehiclePawn->ImuSensor)
	{
		auto Imu = VehiclePawn->ImuSensor->GetOutput();
		// Acceleration: body frame, convert cm/s² to m/s²
		Frame.AccelX = Imu.LinearAcceleration.X / 100.f;
		Frame.AccelY = Imu.LinearAcceleration.Y / 100.f;
		Frame.AccelZ = Imu.LinearAcceleration.Z / 100.f;
		Frame.GyroX = Imu.AngularVelocity.X;
		Frame.GyroY = Imu.AngularVelocity.Y;
		Frame.GyroZ = Imu.AngularVelocity.Z;
		// Orientation: ENU quaternion
		FQuat EnuQuat = FSDSCoord::UEQuatToENU(Imu.Orientation);
		Frame.OrientX = EnuQuat.X;
		Frame.OrientY = EnuQuat.Y;
		Frame.OrientZ = EnuQuat.Z;
		Frame.OrientW = EnuQuat.W;
	}

	// GSS
	if (VehiclePawn->GssSensor)
	{
		auto Gss = VehiclePawn->GssSensor->GetOutput();
		// Body frame velocity (already m/s from sensor): UE5 X=forward, Y=right → ROS X=forward, Y=left
		Frame.GssVelX = Gss.LinearVelocity.X;   // forward
		Frame.GssVelY = -Gss.LinearVelocity.Y;  // left (negate right→left)
		Frame.GssVelZ = Gss.LinearVelocity.Z;
	}

	// Odom/Pose — ENU meters
	FVector Pos = VehiclePawn->GetActorLocation();
	FQuat Quat = VehiclePawn->GetActorQuat();
	Frame.PosX = Pos.Y / 100.f;  // ENU east = UE Y
	Frame.PosY = Pos.X / 100.f;  // ENU north = UE X
	Frame.PosZ = Pos.Z / 100.f;
	FQuat EnuQ = FSDSCoord::UEQuatToENU(Quat);
	Frame.PoseOrientX = EnuQ.X;
	Frame.PoseOrientY = EnuQ.Y;
	Frame.PoseOrientZ = EnuQ.Z;
	Frame.PoseOrientW = EnuQ.W;

	auto CarState = VehiclePawn->GetCarState();
	Frame.Speed = CarState.Speed;
	Frame.RPM = CarState.RPM;

	// Referee
	if (Referee)
	{
		auto RefState = Referee->GetState();
		Frame.DooCounter = RefState.DooCounter;
		Frame.OffTrackCounter = RefState.OffTrackCounter;
		Frame.LapCount = RefState.Laps.Num();
	}

	// Controls echo. Frame.Brake stays as the wire-format name (binary
	// struct, breaking it would invalidate downstream consumers); the
	// payload is the regen demand — see FCarControls in FSDSVehiclePawn.h.
	auto Controls = VehiclePawn->GetCarControls();
	Frame.Throttle = Controls.Throttle;
	Frame.Steering = Controls.Steering;
	Frame.Brake = Controls.Regen;
}

void FFSDSUdpBroadcaster::BroadcastSensorFrame()
{
	if (!SensorSocket || !SensorAddr.IsValid()) return;

	FFSDSSensorFrame Frame;
	PackSensorFrame(Frame);

	int32 BytesSent = 0;
	SensorSocket->SendTo((const uint8*)&Frame, sizeof(Frame), BytesSent, *SensorAddr);
}

void FFSDSUdpBroadcaster::BroadcastLidarFrame()
{
	if (!LidarSocket || !LidarAddr.IsValid() || !VehiclePawn || !VehiclePawn->LidarSensor) return;

	TArray<float> Points = VehiclePawn->LidarSensor->GetPointCloud();
	int32 TotalPoints = Points.Num() / 3;
	if (TotalPoints <= 0) return;

	// macOS's UDP loopback path drops ~33 % of datagrams when the plugin
	// emits 250+ sendto's in a sub-millisecond burst on the game thread.
	// The kernel's per-output-queue limit can't drain that fast and the
	// excess is silently discarded (no errno surfaced — UE5's FSocket
	// reports BytesSent == PacketSize anyway). Moving the send loop to a
	// background thread with a tiny inter-chunk yield lets the kernel
	// keep up: 100 µs × 250 chunks ≈ 25 ms, well within the 100 ms
	// inter-scan budget at 10 Hz, and the 4 MB SO_SNDBUF set in Start()
	// absorbs any residual jitter. Without this, /lidar published at
	// ~1.8 Hz with mostly-zero point clouds; with it, full scans land
	// at the LiDAR's native rate.
	uint32 LocalFrameID = FrameCounter;
	int32  Channels = VehiclePawn->LidarSensor->NumberOfChannels;
	TArray<float> PointsCopy = MoveTemp(Points);
	FSocket* SocketRef = LidarSocket;
	TSharedPtr<FInternetAddr> AddrRef = LidarAddr;

	AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask,
	    [SocketRef, AddrRef, LocalFrameID, Channels, PointsCopy = MoveTemp(PointsCopy), TotalPoints]()
	{
		if (!SocketRef || !AddrRef.IsValid()) return;

	// Chunk size MUST stay under macOS's default `net.inet.udp.maxdgram`
	// of 9216 bytes — datagrams above that limit are silently dropped by
	// the kernel before they hit the wire (the small sensor frames at
	// 148 B succeed, but a 60 KB LiDAR chunk vanishes with no error).
	// Linux defaults are far higher (≥64 KB) so the same code works
	// without tuning. PointsPerChunk=700 → 700×12 + 24 = 8424 B per
	// datagram, ~9 % below the macOS cap.
	//
	// MUST match LIDAR_UDP_POINTS_PER_CHUNK in the bridge's
	// udp_receiver.cpp (chunk_index → start-offset arithmetic depends on
	// it). Both sides recompile from source, so a hardcoded constant is
	// fine; the alternative (carrying start_index in the header) bloats
	// every datagram and would break wire compatibility on the existing
	// `streamSensors` UDP path that shares the binary frame format.
	const int32 PointsPerChunk = 700;
	int32 TotalChunks = (TotalPoints + PointsPerChunk - 1) / PointsPerChunk;

	for (int32 ChunkIdx = 0; ChunkIdx < TotalChunks; ChunkIdx++)
	{
		int32 StartPoint = ChunkIdx * PointsPerChunk;
		int32 ChunkPoints = FMath::Min(PointsPerChunk, TotalPoints - StartPoint);

		// Build chunk: header + point data
		int32 DataSize = ChunkPoints * 3 * sizeof(float);
		int32 PacketSize = sizeof(FFSDSLidarChunkHeader) + DataSize;

		TArray<uint8> Packet;
		Packet.SetNumUninitialized(PacketSize);

		// Fill header
		FFSDSLidarChunkHeader* Header = (FFSDSLidarChunkHeader*)Packet.GetData();
		Header->Magic = 0x4C494452;
		Header->ChunkIndex = (uint16)ChunkIdx;
		Header->TotalChunks = (uint16)TotalChunks;
		Header->FrameID = LocalFrameID;
		Header->PointsInChunk = ChunkPoints;
		Header->TotalPoints = TotalPoints;
		Header->Channels = Channels;

		// Copy point data (UE5 local frame: X=forward, Y=right — SLAM uses this convention)
		FMemory::Memcpy(
			Packet.GetData() + sizeof(FFSDSLidarChunkHeader),
			PointsCopy.GetData() + StartPoint * 3,
			DataSize);

		int32 BytesSent = 0;
		SocketRef->SendTo(Packet.GetData(), PacketSize, BytesSent, *AddrRef);

		// Inter-chunk yield to give the kernel's UDP output queue time to
		// drain. SleepNoStats(0.0001) ≈ 100 µs on macOS — granularity is
		// kernel-tick-bound (~1 ms in practice), but even one yield per
		// chunk is enough to cut the burst-induced drop rate from ~33 %
		// to negligible. Empirically zero pacing → 1.8 Hz publish; 100 µs
		// → 9.7 Hz at 1.74 M pts/s on a clean Apple-Silicon Mac.
		FPlatformProcess::SleepNoStats(0.0001f);
	}
	});
}
