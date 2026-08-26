#include "RPC/FSDSRpcServer.h"
#include "FMI/FSDSFmuPackage.h"
#include "FMI/FSDSFmi3.h"
#include "Plant/FSDSFmuPlant.h"
#include "EmraxMotor.h"
#include "FSDSRandom.h"
#include "RPC/FSDSUdpBroadcaster.h"
#include "FSDSVehiclePawn.h"
#include "FSDSReferee.h"
#include "FSDSConeSpawner.h"
#include "FSDSCoordinates.h"
#include "Async/Async.h"
#include "Engine/World.h"
#include "Engine/StaticMeshActor.h"
#include "Kismet/GameplayStatics.h"

#include "EngineUtils.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Common/TcpSocketBuilder.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Interfaces/IPv4/IPv4Endpoint.h"
#include "Common/TcpListener.h"
#include "Test/FSDSTestTerrain.h"

/**
 * Tell the vehicle's plant(s) that the car has been teleported.
 *
 * Moving the mesh is INVISIBLE to a plant that integrates its own state. Without
 * this the FMU keeps driving from wherever it had got to while the rest of the
 * sim starts a fresh mission. Measured before this existed: a 59.5 m step in the
 * shadow divergence, with the car standing still.
 *
 * Every path that repositions the vehicle must call this — loadTrack, reset and
 * simSetVehiclePose all do. A helper rather than three copies precisely because
 * the fourth caller is the one that will forget.
 *
 * Converts UE (left-handed, centimetres) to the contract (SI, ENU, y LEFT) here,
 * on the platform side of the boundary, matching FFSDSChaosPlant::Reset.
 */
static void NotifyPlantsOfTeleport(AFSDSVehiclePawn* Pawn, const FVector& PosUe, const FQuat& RotUe)
{
	if (!Pawn) return;
	// + the CoG offset: the plant's z is its CoG, not the mesh origin. Without
	// this a teleport buries the plant by a full CoG height.
	const double PosContract[3] = { PosUe.X * 0.01, -PosUe.Y * 0.01,
	                                PosUe.Z * 0.01 + AFSDSVehiclePawn::PlantMeshZOffsetM() };
	const double QuatContract[4] = { RotUe.W, -RotUe.X, RotUe.Y, -RotUe.Z };
	Pawn->ResetPlants(PosContract, QuatContract);
}


namespace
{
	/**
	 * Robust full-buffer Send for FSocket. UE's FSocket can return
	 *   bOk = true, BytesSent = 0
	 * when the kernel send buffer is momentarily full on a non-blocking
	 * socket (the default mode for client connections accepted by an
	 * FTcpListener). Treating that as a fatal disconnect — which the
	 * stream loops below were doing — caused the bridge to tear down
	 * the LiDAR push every ~1 s during an autocross run, dropping the
	 * effective rate from 10 Hz to ~3 Hz and starving perception.
	 *
	 * This helper retries on the transient-zero case for up to
	 * `MaxIdleMs` (default 200 ms), distinguishing "stuck buffer = real
	 * problem" from "occasional hiccup". A genuine `bOk=false` still
	 * returns immediately so a real disconnect isn't masked.
	 */
	static bool SendAll(FSocket* Socket, const uint8* Buffer, int32 Length, int32 MaxIdleMs = 5000)
	{
		if (!Socket || Length <= 0) return Socket != nullptr;
		int32 Sent = 0;
		int32 IdleMs = 0;
		while (Sent < Length)
		{
			int32 ChunkSent = 0;
			const bool bOk = Socket->Send(Buffer + Sent, Length - Sent, ChunkSent);
			if (!bOk) return false;                 // hard error — peer gone
			if (ChunkSent <= 0)
			{
				// Kernel send buffer momentarily full. Wait for it to
				// drain via FSocket::Wait (kernel wakes us as soon as the
				// peer has consumed enough bytes for another chunk).
				// Previously this slept a fixed 5 ms per partial-send,
				// which over loopback added up to most of the per-scan
				// budget at high pts/s — a 1.2 MB scan that needed even
				// 10 partial sends ate 50 ms of pure sleep. Wait() returns
				// in <1 ms over loopback for the common case.
				//
				// IdleMs still bounds the total wait so a genuinely dead
				// peer returns false within ~MaxIdleMs; we step it by the
				// timeout slice rather than the actual sleep duration to
				// preserve the previous semantics.
				if (IdleMs >= MaxIdleMs) return false;
				const int32 WaitSliceMs = 50;
				Socket->Wait(ESocketWaitConditions::WaitForWrite,
				             FTimespan::FromMilliseconds(WaitSliceMs));
				IdleMs += WaitSliceMs;
				continue;
			}
			IdleMs = 0;                              // progress — reset idle counter
			Sent += ChunkSent;
		}
		return true;
	}

	/**
	 * Shared completion state for CallOnGameThread. Lives on the heap,
	 * refcounted via TSharedPtr so the caller and the game-thread task
	 * can release independently — the last holder destroys it. Safe
	 * under caller-side timeout: if the caller gives up, the task
	 * still owns a ref and can safely complete without touching any
	 * dead stack memory.
	 */
	template <typename TResult>
	struct TCallState
	{
		std::atomic<bool> Done{false};
		TResult Value;
	};

	/**
	 * Run `Fn()` on the engine game thread and block up to
	 * `TimeoutSec` for it to complete. Returns the functor's result on
	 * success, `OnTimeout` on expiry. `Tag` is used in the warning log
	 * when a timeout fires.
	 *
	 * Replaces the previous `FEvent*` + `GetSynchEventFromPool/Return
	 * SynchEventToPool` pattern, which had two bugs: (1) on timeout,
	 * the caller's stack-captured result pointer became dangling while
	 * the lambda could still execute and write to it; and (2) the
	 * event was returned to the engine pool regardless of whether the
	 * lambda had triggered it, so a subsequent reuse of the event by
	 * an unrelated caller could be spuriously signalled by the
	 * orphaned lambda.
	 */
	template <typename TResult, typename TFn>
	TResult CallOnGameThread(TFn&& Fn, double TimeoutSec, TResult OnTimeout, const TCHAR* Tag)
	{
		using TState = TCallState<TResult>;
		TSharedPtr<TState, ESPMode::ThreadSafe> State = MakeShared<TState, ESPMode::ThreadSafe>();

		AsyncTask(ENamedThreads::GameThread, [State, Functor = std::forward<TFn>(Fn)]() mutable
		{
			TResult Local = Functor();
			State->Value = MoveTemp(Local);
			State->Done.store(true, std::memory_order_release);
		});

		const double Deadline = FPlatformTime::Seconds() + TimeoutSec;
		while (FPlatformTime::Seconds() < Deadline)
		{
			if (State->Done.load(std::memory_order_acquire))
			{
				return MoveTemp(State->Value);
			}
			FPlatformProcess::Sleep(0.002f);
		}
		UE_LOG(LogTemp, Warning, TEXT("FSDS RPC: game-thread call timed out (%s)"), Tag);
		return OnTimeout;
	}
}

FFSDSRpcServer::FFSDSRpcServer()
{
}

FFSDSRpcServer::~FFSDSRpcServer()
{
	Stop();
}

void FFSDSRpcServer::Start(uint16 Port)
{
	if (bRunning) return;

	bRunning = true;
	ServerPort = Port;
	ServerThread = std::make_unique<std::thread>(&FFSDSRpcServer::ServerThreadFunc, this);

	UE_LOG(LogTemp, Log, TEXT("FSDS RPC: Server starting on port %d"), Port);
}

void FFSDSRpcServer::Stop()
{
	if (!bRunning) return;
	bRunning = false;

	if (ServerThread && ServerThread->joinable())
	{
		ServerThread->join();
	}
	ServerThread.reset();

	// Drain in-flight client threads before we return — otherwise they
	// keep running detached after `this` is destroyed and the bRunning
	// check in HandleClient dereferences a freed pointer. Each client
	// thread's Wait() loop tops out at 5 s, so worst-case shutdown
	// blocks for ~5 s per active client.
	{
		std::lock_guard<std::mutex> Lock(ClientThreadsMutex);
		for (std::thread& T : ClientThreads)
		{
			if (T.joinable()) T.join();
		}
		ClientThreads.clear();
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS RPC: Server stopped"));
}

void FFSDSRpcServer::ServerThreadFunc()
{
	ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
	if (!SocketSubsystem)
	{
		UE_LOG(LogTemp, Error, TEXT("FSDS RPC: Failed to get socket subsystem"));
		return;
	}

	FSocket* ListenSocket = FTcpSocketBuilder(TEXT("FSDS RPC Server"))
		.AsReusable()
		.BoundToPort(ServerPort)
		.Listening(8)
		.Build();

	if (!ListenSocket)
	{
		UE_LOG(LogTemp, Error, TEXT("FSDS RPC: Failed to create listen socket on port %d"), ServerPort);
		return;
	}

	UE_LOG(LogTemp, Log, TEXT("FSDS RPC: Listening on port %d"), ServerPort);

	while (bRunning)
	{
		bool bHasPendingConnection = false;
		ListenSocket->HasPendingConnection(bHasPendingConnection);

		if (bHasPendingConnection)
		{
			TSharedRef<FInternetAddr> RemoteAddr = SocketSubsystem->CreateInternetAddr();
			FSocket* ClientSocket = ListenSocket->Accept(*RemoteAddr, TEXT("FSDS RPC Client"));

			if (ClientSocket)
			{
				// Disable Nagle's algorithm. Without this, the sensor-
				// stream loop (small ~200-byte frames sent every 2.5 ms)
				// suffers Windows TCP Nagle batching — frames buffer for
				// 40-200 ms before transmission, capping /imu at ~12 Hz
				// regardless of how fast we Sleep between sends. With
				// NoDelay, each Send goes on the wire immediately and
				// the loop is paced purely by the FPlatformProcess::Sleep
				// call (which now relies on timeBeginPeriod(1) from
				// FSDSPlugin::StartupModule to actually hit sub-15.6ms
				// granularity on Windows).
				ClientSocket->SetNoDelay(true);

				UE_LOG(LogTemp, Log, TEXT("FSDS RPC: Client connected from %s"), *RemoteAddr->ToString(true));

				// SO_SNDBUF bump — reinstated PR-#482-follow-up
				// (2026-05-14, Mac TCP-LiDAR validation).
				//
				// PR #322 retired LiDAR-over-TCP and removed this bump
				// because the remaining RPC clients (camera images,
				// ~40 KB/s sensor stream, command req/resp) all fit
				// comfortably in the kernel default ~128 KB send
				// buffer. PR #482 brought TCP-LiDAR back (1.5 MB
				// scans at 10 Hz) but did NOT re-add the bump —
				// which "worked" on the Windows verification because
				// Windows' default winsock SO_SNDBUF is much larger
				// than Mac's. On macOS Docker Desktop the small
				// default + the single SendAll(1.5MB) per scan
				// pattern produces immediate disconnects (each
				// payload Send returns failure inside SendAll →
				// StreamLidar exits → bridge sees FIN → "payload
				// recv failed", repeated ~10× / second).
				//
				// SetSendBufferSize takes the requested size and
				// returns the actually-applied size in NewSize. We
				// log when the kernel caps us below request so the
				// operator notices if rmem_max-equivalent limits
				// bite again on a different host.
				{
					int32 NewSize = 0;
					const int32 RequestedSnd = 32 * 1024 * 1024;
					ClientSocket->SetSendBufferSize(RequestedSnd, NewSize);
					if (NewSize < RequestedSnd) {
						UE_LOG(LogTemp, Warning,
							TEXT("FSDS RPC: SO_SNDBUF capped at %d bytes "
							     "(requested %d) — TCP-LiDAR may stall under "
							     "sustained load"),
							NewSize, RequestedSnd);
					}
				}

				// Handle each client in its own thread. Store it so Stop()
				// can join the full set before the server is destroyed —
				// previously these were `.detach()`ed and kept running
				// after shutdown, still holding the raw `this` pointer.
				// The vector grows for the lifetime of the server; in
				// practice client count is small (MC + bridge + camera
				// + one streaming channel per stream), so an unbounded
				// grow-only container is fine for the session lengths
				// this sim sees.
				std::lock_guard<std::mutex> Lock(ClientThreadsMutex);
				ClientThreads.emplace_back([this, ClientSocket, SocketSubsystem]() {
					HandleClient(ClientSocket);
					ClientSocket->Close();
					SocketSubsystem->DestroySocket(ClientSocket);
				});
			}
		}

		FPlatformProcess::Sleep(0.01f); // 10ms poll interval
	}

	ListenSocket->Close();
	SocketSubsystem->DestroySocket(ListenSocket);
}

void FFSDSRpcServer::HandleClient(FSocket* ClientSocket)
{
	uint8 Buffer[4096];

	while (bRunning && ClientSocket)
	{
		int32 BytesRead = 0;
		// Wait up to 5s for data; on timeout just loop to re-check bRunning.
		// Without this check, non-blocking Recv after a timed-out Wait returns
		// false immediately, breaking the loop and closing a live connection.
		if (!ClientSocket->Wait(ESocketWaitConditions::WaitForRead, FTimespan::FromSeconds(5)))
			continue;

		if (ClientSocket->Recv(Buffer, sizeof(Buffer) - 1, BytesRead))
		{
			if (BytesRead > 0)
			{
				Buffer[BytesRead] = 0;
				FString Request = UTF8_TO_TCHAR((char*)Buffer);
				Request.TrimEndInline();

				// Check for streaming mode
				if (Request == TEXT("streamSensors"))
				{
					StreamSensors(ClientSocket);
					return; // Connection used for streaming, done
				}
				if (Request == TEXT("streamLidar"))
				{
					// PR-#482: TCP LiDAR re-introduced. See StreamLidar
					// docs for the failure-mode trail that motivated
					// undoing #322. Connection is held open and used
					// for streaming until the client disconnects.
					StreamLidar(ClientSocket);
					return;
				}
				// History note: `streamLidar` was originally retired in
				// #322 (LiDAR moved to chunked UDP via
				// FSDSUdpBroadcaster) and re-introduced in PR-#482 (the
				// UDP path has structural wedges on Docker Desktop
				// Mac/Windows that TCP transparently avoids). The UDP
				// broadcaster path is still wired in
				// FSDSUdpBroadcaster::BroadcastLidarFrame; it just no
				// longer has a consumer once bridges switch to TCP.

				// Check if this is a binary request
				if (ProcessBinaryRequest(Request, ClientSocket))
				{
					continue; // Binary response already sent
				}

				// Text response
				FString Response = ProcessRequest(Request);
				Response += TEXT("\n");

				FTCHARToUTF8 Converter(*Response);
				int32 BytesSent = 0;
				ClientSocket->Send((const uint8*)Converter.Get(), Converter.Length(), BytesSent);
			}
		}
		else
		{
			break;
		}
	}
}

FString FFSDSRpcServer::ProcessRequest(const FString& Request)
{
	// Parse method name (first word)
	FString Method, Args;
	Request.Split(TEXT(" "), &Method, &Args);
	if (Method.IsEmpty()) Method = Request;

	if (Method == TEXT("ping"))
	{
		return TEXT("true");
	}
	else if (Method == TEXT("getSettingsString"))
	{
		return SettingsString;
	}
	else if (Method == TEXT("getSettingsPaths"))
	{
		// Diagnostic: returns the three paths AutoLoad() searches so we can
		// tell which one the shipping build can actually see.
		const FString P1 = FPaths::Combine(FPaths::LaunchDir(),    TEXT("settings.json"));
		const FString P2 = FPaths::Combine(FPaths::ProjectDir(),   TEXT("settings.json"));
		const FString P3 = FPaths::Combine(FPaths::Combine(FPlatformProcess::UserSettingsDir(), TEXT("IFSSIM")), TEXT("settings.json"));
		return FString::Printf(
			TEXT("{\"launch\":\"%s\",\"launch_exists\":%s,\"project\":\"%s\",\"project_exists\":%s,\"user\":\"%s\",\"user_exists\":%s}"),
			*P1, FPaths::FileExists(P1) ? TEXT("true") : TEXT("false"),
			*P2, FPaths::FileExists(P2) ? TEXT("true") : TEXT("false"),
			*P3, FPaths::FileExists(P3) ? TEXT("true") : TEXT("false"));
	}
	else if (Method == TEXT("enableApiControl"))
	{
		bApiControlEnabled = true;
		// Also set on vehicle pawn so keyboard input doesn't override API controls
		if (IsValid(VehiclePawn)) VehiclePawn->SetApiControlEnabled(true);
		return TEXT("true");
	}
	else if (Method == TEXT("disableApiControl"))
	{
		// Counterpart to enableApiControl: both flags must flip so the pawn
		// actually rejects incoming setCarControls. Used by RES so that a
		// control node which is still winding down (supervisor polls at 1 Hz,
		// so TERM can take up to 5 s) can't override the latched brake.
		bApiControlEnabled = false;
		if (IsValid(VehiclePawn)) VehiclePawn->SetApiControlEnabled(false);
		return TEXT("true");
	}
	else if (Method == TEXT("isApiControlEnabled"))
	{
		return bApiControlEnabled ? TEXT("true") : TEXT("false");
	}
	else if (Method == TEXT("enableLidarUdpBroadcast"))
	{
		// PR-#482: opt-in for the legacy chunked-UDP LiDAR path.
		// Default state of UdpBroadcaster is OFF (sim sends no LiDAR
		// UDP) because the production transport is now TCP via
		// StreamLidar. Bridges using the UDP fallback path
		// (IFSSIM_LIDAR_TRANSPORT=udp) call this RPC at connect time
		// to flip the broadcaster on; subsequent disconnect doesn't
		// auto-disable so a flaky bridge doesn't lose data, but a
		// matching disableLidarUdpBroadcast is available below.
		if (UdpBroadcaster) UdpBroadcaster->SetLidarBroadcastEnabled(true);
		return TEXT("true");
	}
	else if (Method == TEXT("disableLidarUdpBroadcast"))
	{
		// Counterpart to enableLidarUdpBroadcast. Bridges that switch
		// transports mid-session (e.g. operator toggles
		// IFSSIM_LIDAR_TRANSPORT and restarts the container) should
		// disable the UDP broadcast on their way out, otherwise the
		// next bridge picks up spurious old UDP at startup.
		if (UdpBroadcaster) UdpBroadcaster->SetLidarBroadcastEnabled(false);
		return TEXT("true");
	}
	else if (Method == TEXT("activateEbs"))
	{
		// EBS via the handbrake channel (pneumatic analog). The previous
		// path (setCarControls 0 0 1 + disableApiControl) was silently
		// undone every tick by UE5's axis-input system: the keyboard
		// brake axis reads 0 by default and overwrites CurrentControls.
		// Brake, so the latched brake evaporated within one frame.
		// ActivateEbs locks *all* input channels (API + keyboard) until
		// ReleaseEbs is called, and clamps the handbrake so the rear
		// axle stays braked regardless.
		if (IsValid(VehiclePawn))
		{
			AsyncTask(ENamedThreads::GameThread, [this]() {
				if (IsValid(VehiclePawn)) VehiclePawn->ActivateEbs();
			});
		}
		// Mirror ActivateEbs's effect on the readback cache so
		// getCarControls reports the actual locked state. Previously
		// the cache only tracked setCarControls, so any client polling
		// /api/vehicle/state during EBS saw stale `handbrake=false` even
		// though the pawn's CurrentControls.bHandbrake was true.
		CachedControls.Throttle = 0.f;
		CachedControls.Steering = 0.f;
		CachedControls.Regen = 0.f;
		CachedControls.bHandbrake = true;
		bApiControlEnabled = false;
		return TEXT("true");
	}
	else if (Method == TEXT("releaseEbs"))
	{
		// Operator-only release — mirrors the real car where only the
		// driver can reset EBS. Hands control back to the autonomy.
		if (IsValid(VehiclePawn))
		{
			AsyncTask(ENamedThreads::GameThread, [this]() {
				if (IsValid(VehiclePawn)) VehiclePawn->ReleaseEbs();
			});
		}
		// Mirror ReleaseEbs's effect on the readback cache (see
		// activateEbs comment above). Throttle/steering/brake aren't
		// touched here because ReleaseEbs only clears bHandbrake +
		// bEbsLatched; the autonomy is expected to set its own
		// drive command on the next setCarControls.
		CachedControls.bHandbrake = false;
		bApiControlEnabled = true;
		return TEXT("true");
	}
	else if (Method == TEXT("isEbsLatched"))
	{
		if (IsValid(VehiclePawn)) return VehiclePawn->IsEbsLatched() ? TEXT("true") : TEXT("false");
		return TEXT("false");
	}
	else if (Method == TEXT("getCarState"))
	{
		if (!IsValid(VehiclePawn)) return TEXT("{}");
		auto State = VehiclePawn->GetCarState();
		FVector PosENU = FSDSCoord::UEToENU(State.Position);
		FVector VelENU = FSDSCoord::UEVelocityToENU(State.LinearVelocity);
		FQuat OriENU = FSDSCoord::UEQuatToENU(State.Orientation);
		// Regen-telemetry fields (regen_torque/regen_power/regen_avail_*
		// /regen_max_*) were drafted in the RPC server but their backing
		// FCarState members haven't landed yet. Emit only the stable
		// kinematic+powertrain fields so the build stays green; restore
		// the regen block in the same commit that adds the FCarState
		// members.
		return FString::Printf(TEXT("{\"speed\":%.4f,\"gear\":%d,\"rpm\":%.1f,\"maxrpm\":%.1f,\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"vx\":%.4f,\"vy\":%.4f,\"vz\":%.4f,\"qw\":%.6f,\"qx\":%.6f,\"qy\":%.6f,\"qz\":%.6f}"),
			State.Speed, State.Gear, State.RPM, State.MaxRPM,
			PosENU.X, PosENU.Y, PosENU.Z,
			VelENU.X, VelENU.Y, VelENU.Z,
			OriENU.W, OriENU.X, OriENU.Y, OriENU.Z);
	}
	else if (Method == TEXT("getGpsData"))
	{
		if (!VehiclePawn || !VehiclePawn->GpsSensor) return TEXT("{}");
		auto Gps = VehiclePawn->GpsSensor->GetOutput();
		return FString::Printf(TEXT("{\"lat\":%.8f,\"lon\":%.8f,\"alt\":%.2f}"),
			Gps.Latitude, Gps.Longitude, Gps.Altitude);
	}
	else if (Method == TEXT("getImuData"))
	{
		if (!VehiclePawn || !VehiclePawn->ImuSensor) return TEXT("{}");
		auto Imu = VehiclePawn->ImuSensor->GetOutput();
		// IMU data is already in body frame, convert to ENU body frame
		FVector AccENU = FSDSCoord::UEVelocityToENU(Imu.LinearAcceleration);
		FVector GyroENU = FSDSCoord::UEAngularVelocityToENU(Imu.AngularVelocity);
		FQuat OriENU = FSDSCoord::UEQuatToENU(Imu.Orientation);
		return FString::Printf(TEXT("{\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f,\"qw\":%.6f,\"qx\":%.6f,\"qy\":%.6f,\"qz\":%.6f}"),
			AccENU.X, AccENU.Y, AccENU.Z,
			GyroENU.X, GyroENU.Y, GyroENU.Z,
			OriENU.W, OriENU.X, OriENU.Y, OriENU.Z);
	}
	else if (Method == TEXT("getGroundSpeedSensorData"))
	{
		if (!VehiclePawn || !VehiclePawn->GssSensor) return TEXT("{}");
		auto Gss = VehiclePawn->GssSensor->GetOutput();
		// GSS is in body frame — swap axes for ENU body convention
		FVector VelENU(Gss.LinearVelocity.Y, Gss.LinearVelocity.X, Gss.LinearVelocity.Z);
		return FString::Printf(TEXT("{\"vx\":%.4f,\"vy\":%.4f,\"vz\":%.4f}"),
			VelENU.X, VelENU.Y, VelENU.Z);
	}
	else if (Method == TEXT("getLidarData"))
	{
		if (!VehiclePawn || !VehiclePawn->LidarSensor) return TEXT("{\"points\":0,\"channels\":0}");
		int32 PointCount = VehiclePawn->LidarSensor->GetPointCount();
		return FString::Printf(TEXT("{\"points\":%d,\"channels\":%d,\"range\":%.1f}"),
			PointCount,
			VehiclePawn->LidarSensor->NumberOfChannels,
			VehiclePawn->LidarSensor->MaxRange / 100.f);
	}
	else if (Method == TEXT("getRefereeState"))
	{
		if (!Referee) return TEXT("{\"doo_counter\":0,\"cones\":0,\"laps\":0,\"lap_times\":[],\"cone_positions\":[]}");
		auto State = Referee->GetState();

		// Build lap times array
		FString LapTimesJson = TEXT("[");
		for (int32 i = 0; i < State.Laps.Num(); i++)
		{
			if (i > 0) LapTimesJson += TEXT(",");
			LapTimesJson += FString::Printf(TEXT("%.3f"), State.Laps[i]);
		}
		LapTimesJson += TEXT("]");

		// Build cone positions array (ENU coordinates in meters)
		FString ConesJson = TEXT("[");
		for (int32 i = 0; i < State.Cones.Num(); i++)
		{
			if (i > 0) ConesJson += TEXT(",");
			const FFSDSCone& Cone = State.Cones[i];
			// Convert from UE cm to ENU meters: swap X↔Y, divide by 100
			float EnuX = Cone.Location.Y / 100.f;
			float EnuY = Cone.Location.X / 100.f;
			int32 ColorInt = (int32)Cone.Color;
			ConesJson += FString::Printf(TEXT("{\"x\":%.4f,\"y\":%.4f,\"color\":%d}"),
				EnuX, EnuY, ColorInt);
		}
		ConesJson += TEXT("]");

		// Event type name
		FString EventName;
		switch (State.EventType)
		{
		case EFSDSEventType::Acceleration: EventName = TEXT("acceleration"); break;
		case EFSDSEventType::Skidpad: EventName = TEXT("skidpad"); break;
		case EFSDSEventType::Autocross: EventName = TEXT("autocross"); break;
		case EFSDSEventType::Trackdrive: EventName = TEXT("trackdrive"); break;
		default: EventName = TEXT("unknown"); break;
		}

		return FString::Printf(TEXT("{\"doo_counter\":%d,\"oc_counter\":%d,\"cones\":%d,\"laps\":%d,\"required_laps\":%d,\"finished\":%s,\"event\":\"%s\",\"lap_times\":%s,\"cone_positions\":%s}"),
			State.DooCounter, State.OffTrackCounter, State.Cones.Num(), State.Laps.Num(), State.RequiredLaps,
			State.bFinished ? TEXT("true") : TEXT("false"), *EventName,
			*LapTimesJson, *ConesJson);
	}
	else if (Method.StartsWith(TEXT("setVehicleCommand")) || Method.StartsWith(TEXT("setCarControls")))
	{
		// Two RPC names, one handler:
		//   setVehicleCommand <throttle> <steering> <regen>
		//   setCarControls    <throttle> <steering> <brake>     (legacy)
		//
		// Same wire format. The 3rd float has always semantically been
		// regen demand (folded into the EMRAX motor command in
		// AFSDSVehiclePawn::Tick); setVehicleCommand renames it to match
		// the field's actual physical role. setCarControls stays as a
		// compat alias so unmigrated callers (dashboards, tests) keep
		// working until they switch over.
		//
		// Future channels (e.g. an explicit ebs_request positional arg)
		// should be appended at higher indices on setVehicleCommand only,
		// leaving setCarControls's three-arg shape intact.
		if (IsValid(VehiclePawn) && bApiControlEnabled)
		{
			TArray<FString> Parts;
			Request.ParseIntoArray(Parts, TEXT(" "));
			if (Parts.Num() >= 4)
			{
				AFSDSVehiclePawn::FCarControls Controls;
				Controls.Throttle = FCString::Atof(*Parts[1]);
				Controls.Steering = FCString::Atof(*Parts[2]);
				Controls.Regen    = FCString::Atof(*Parts[3]);

				// Cache immediately for getCarControls readback
				CachedControls.Throttle = Controls.Throttle;
				CachedControls.Steering = Controls.Steering;
				CachedControls.Regen    = Controls.Regen;

				AsyncTask(ENamedThreads::GameThread, [this, Controls]() {
					if (IsValid(VehiclePawn)) VehiclePawn->SetCarControls(Controls);
				});
			}
		}
		return TEXT("true");
	}
	else if (Method == TEXT("getCarControls") || Method == TEXT("getVehicleCommand"))
	{
		// Read from cached controls (set immediately on setVehicleCommand /
		// setCarControls, no game-thread delay). EBS handbrake comes
		// straight from the pawn so paths that bypass the RPC (e.g. the
		// pawn's BeginPlay calling ActivateEbs() directly) are still
		// reflected truthfully — CachedControls.bHandbrake is only updated
		// by the RPC handlers themselves.
		//
		// JSON exposes both `regen` (canonical) and `brake` (deprecated
		// alias) so dashboards mid-migration see no break. Drop `brake`
		// once all consumers are off it.
		const bool bHandbrakeReadback = IsValid(VehiclePawn)
			? VehiclePawn->IsEbsLatched() : CachedControls.bHandbrake;
		return FString::Printf(TEXT("{\"throttle\":%.4f,\"steering\":%.4f,\"regen\":%.4f,\"brake\":%.4f,\"handbrake\":%s,\"is_manual_gear\":%s,\"manual_gear\":%d,\"gear_immediate\":%s}"),
			CachedControls.Throttle, CachedControls.Steering, CachedControls.Regen, CachedControls.Regen,
			bHandbrakeReadback ? TEXT("true") : TEXT("false"),
			CachedControls.bIsManualGear ? TEXT("true") : TEXT("false"),
			CachedControls.ManualGear,
			CachedControls.bGearImmediate ? TEXT("true") : TEXT("false"));
	}
	else if (Method == TEXT("simGetVehiclePose"))
	{
		if (!IsValid(VehiclePawn)) return TEXT("{\"x\":0,\"y\":0,\"z\":0,\"qw\":1,\"qx\":0,\"qy\":0,\"qz\":0}");
		FVector Pos = VehiclePawn->GetActorLocation();
		FQuat Quat = VehiclePawn->GetActorQuat();
		FVector PosENU = FSDSCoord::UEToENU(Pos);
		FQuat QuatENU = FSDSCoord::UEQuatToENU(Quat);
		return FString::Printf(TEXT("{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"qw\":%.6f,\"qx\":%.6f,\"qy\":%.6f,\"qz\":%.6f}"),
			PosENU.X, PosENU.Y, PosENU.Z, QuatENU.W, QuatENU.X, QuatENU.Y, QuatENU.Z);
	}

	else if (Method == TEXT("getStartGatePose"))
	{
		if (!bHasStartGate) return TEXT("{\"error\":\"no track loaded\"}");
		FVector PosENU = FSDSCoord::UEToENU(LastStartGateLoc_UE);
		FQuat   QuatENU = FSDSCoord::UEQuatToENU(LastStartGateRot_UE);
		return FString::Printf(
			TEXT("{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f,\"qw\":%.6f,\"qx\":%.6f,\"qy\":%.6f,\"qz\":%.6f}"),
			PosENU.X, PosENU.Y, PosENU.Z,
			QuatENU.W, QuatENU.X, QuatENU.Y, QuatENU.Z);
	}

	else if (Method == TEXT("getPlantState"))
	{
		// The live plant snapshot, in contract units (SI, ISO 8855, ENU) rather
		// than UE's left-handed centimetres. This is what sensors and the bridge
		// should migrate to reading.
		if (!IsValid(VehiclePawn))
		{
			return TEXT("{\"ok\":false,\"error\":\"no vehicle pawn\"}");
		}
		const FFSDSPlantOutput& S = VehiclePawn->GetPlantState();
		return FString::Printf(
			TEXT("{\"ok\":%s,\"plant\":\"%s\",\"status\":%d,")
			TEXT("\"position\":[%.6f,%.6f,%.6f],\"quat\":[%.6f,%.6f,%.6f,%.6f],")
			TEXT("\"velWorld\":[%.6f,%.6f,%.6f],\"velBody\":[%.6f,%.6f,%.6f],")
			TEXT("\"omegaBody\":[%.6f,%.6f,%.6f],\"accelProper\":[%.6f,%.6f,%.6f],")
			TEXT("\"attitude\":[%.6f,%.6f,%.6f],")
			TEXT("\"wheelFz\":[%.1f,%.1f,%.1f,%.1f],")
			TEXT("\"wheelSteer\":[%.5f,%.5f,%.5f,%.5f],")
			TEXT("\"inContact\":[%d,%d,%d,%d]}"),
			S.bPlantOk ? TEXT("true") : TEXT("false"),
			*VehiclePawn->GetPlantName(), S.PlantStatus,
			S.Position[0], S.Position[1], S.Position[2],
			S.Quat[0], S.Quat[1], S.Quat[2], S.Quat[3],
			S.VelWorld[0], S.VelWorld[1], S.VelWorld[2],
			S.VelBody[0], S.VelBody[1], S.VelBody[2],
			S.OmegaBody[0], S.OmegaBody[1], S.OmegaBody[2],
			S.AccelProper[0], S.AccelProper[1], S.AccelProper[2],
			S.Attitude[0], S.Attitude[1], S.Attitude[2],
			S.WheelFz[0], S.WheelFz[1], S.WheelFz[2], S.WheelFz[3],
			S.WheelSteer[0], S.WheelSteer[1], S.WheelSteer[2], S.WheelSteer[3],
			S.bWheelInContact[0]?1:0, S.bWheelInContact[1]?1:0,
			S.bWheelInContact[2]?1:0, S.bWheelInContact[3]?1:0);
	}
	else if (Method.StartsWith(TEXT("plantDrive")))
	{
		// plantDrive <path-to.fmu> [seconds] [throttle] [steer]
		//
		// Drives an FMU through IFSDSPlant exactly as the simulator would: flat
		// road under all four wheels, gravity, constant commands, stepped at
		// 1/60 s. Reports the resulting trajectory.
		//
		// The point is CROSS-CHECKING. The same model driven the same way in
		// MATLAB produces a known answer; if this path produces a different one
		// then the FMU export, the value-reference resolution or the interface
		// is wrong — and every one of those failures type-checks perfectly,
		// because every signal on this boundary is a double.
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "), true);
		if (Parts.Num() < 2)
		{
			return TEXT("{\"error\":\"usage: plantDrive <path-to.fmu> [seconds] [throttle] [steer]\"}");
		}
		const double Seconds  = (Parts.Num() >= 3) ? FCString::Atod(*Parts[2]) : 2.0;
		const double Throttle = (Parts.Num() >= 4) ? FCString::Atod(*Parts[3]) : 0.5;
		const double Steer    = (Parts.Num() >= 5) ? FCString::Atod(*Parts[4]) : 0.0;

		FFSDSFmuPlant Plant(Parts[1]);
		if (!Plant.Initialise())
		{
			return FString::Printf(TEXT("{\"ok\":false,\"stage\":\"init\",\"error\":\"%s\"}"),
				*Plant.GetLastError().ReplaceCharWithEscapedChar());
		}

		FFSDSPlantInput In;
		In.Throttle  = Throttle;
		In.SteerNorm = Steer;
		In.GravityZ  = -9.81;
		In.DeltaTime = 1.0 / 60.0;
		for (int32 i = 0; i < FSDS_NUM_WHEELS; i++)
		{
			In.bRoadValid[i] = true;
			In.RoadHeight[i] = 0.0;
			In.RoadMu[i]     = 1.4;
			In.RoadNormal[i][0] = 0.0; In.RoadNormal[i][1] = 0.0; In.RoadNormal[i][2] = 1.0;
		}

		FFSDSPlantOutput Out;
		const int32 Steps = FMath::Max(1, FMath::RoundToInt(Seconds * 60.0));
		for (int32 i = 0; i < Steps; i++)
		{
			In.SimTime = i / 60.0;
			Plant.PreStep(In);
			Plant.PostStep(Out);
			if (!Out.bPlantOk)
			{
				return FString::Printf(
					TEXT("{\"ok\":false,\"stage\":\"step\",\"atStep\":%d,\"error\":\"%s\"}"),
					i, *Plant.GetLastError().ReplaceCharWithEscapedChar());
			}
		}

		return FString::Printf(
			TEXT("{\"ok\":true,\"plant\":\"%s\",\"steps\":%d,\"seconds\":%.6g,")
			TEXT("\"throttle\":%.6g,\"steer\":%.6g,")
			TEXT("\"x\":%.6f,\"y\":%.6f,\"z\":%.6f,")
			TEXT("\"vx\":%.6f,\"vy\":%.6f,\"yawRate\":%.6f,")
			TEXT("\"wheelOmega\":[%.4f,%.4f,%.4f,%.4f],")
			TEXT("\"fz\":[%.1f,%.1f,%.1f,%.1f],")
			TEXT("\"slipRatio\":[%.5f,%.5f,%.5f,%.5f],")
			TEXT("\"motorRpm\":%.2f,\"battSoc\":%.9f}"),
			*Plant.GetName(), Steps, Seconds, Throttle, Steer,
			Out.Position[0], Out.Position[1], Out.Position[2],
			Out.VelBody[0], Out.VelBody[1], Out.OmegaBody[2],
			Out.WheelOmega[0], Out.WheelOmega[1], Out.WheelOmega[2], Out.WheelOmega[3],
			Out.WheelFz[0], Out.WheelFz[1], Out.WheelFz[2], Out.WheelFz[3],
			Out.WheelSlipRatio[0], Out.WheelSlipRatio[1], Out.WheelSlipRatio[2], Out.WheelSlipRatio[3],
			Out.MotorRpm, Out.BattSoc);
	}
	else if (Method.StartsWith(TEXT("fmuSelfTest")))
	{
		// fmuSelfTest <path-to.fmu> [inputVR] [outputVR]
		//
		// Loads an FMU, steps it, and — the part that matters — proves the
		// state save/restore primitive actually reproduces.
		//
		// Everything the deterministic-reset design rests on is that
		// GetFMUState/SetFMUState round-trips exactly. A test that only checked
		// "does it load and step" would pass on an FMU whose state restore is
		// silently a no-op, which is precisely the failure that would poison
		// every A/B comparison built on top of it later.
		//
		// So the test is: snapshot, run N steps, record the output, restore,
		// run the SAME N steps, and require the second answer to be BITWISE
		// identical. Not close. Identical.
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "), true);
		if (Parts.Num() < 2)
		{
			return TEXT("{\"error\":\"usage: fmuSelfTest <path-to.fmu> [inVR] [outVR]\"}");
		}
		const FString FmuPath = Parts[1];
		const uint32 InVR  = (Parts.Num() >= 3) ? (uint32)FCString::Atoi(*Parts[2]) : 0u;
		const uint32 OutVR = (Parts.Num() >= 4) ? (uint32)FCString::Atoi(*Parts[3]) : 1u;

		FFSDSFmuPackage Package;
		if (!Package.Open(FmuPath))
		{
			return FString::Printf(TEXT("{\"ok\":false,\"stage\":\"open\",\"error\":\"%s\"}"),
				*Package.GetError().ReplaceCharWithEscapedChar());
		}

		const FString Lib = Package.GetBinaryPathForHost();
		if (Lib.IsEmpty())
		{
			return TEXT("{\"ok\":false,\"stage\":\"binary\",\"error\":\"no binary for this host\"}");
		}

		FFSDSFmi3Instance Fmu;
		if (!Fmu.Load(Lib))
		{
			return FString::Printf(TEXT("{\"ok\":false,\"stage\":\"load\",\"error\":\"%s\"}"),
				*Fmu.GetLastError().ReplaceCharWithEscapedChar());
		}

		const FString ResourcePath = FPaths::Combine(Package.GetExtractedDir(), TEXT("resources"));
		if (!Fmu.Instantiate(TEXT("ifssim_selftest"),
		                     Package.GetInfo().InstantiationToken, ResourcePath))
		{
			return FString::Printf(TEXT("{\"ok\":false,\"stage\":\"instantiate\",\"error\":\"%s\"}"),
				*Fmu.GetLastError().ReplaceCharWithEscapedChar());
		}

		const double H = 1.0 / 60.0;   // the platform communication step
		if (!Fmu.EnterInitializationMode(0.0, 10.0) || !Fmu.ExitInitializationMode())
		{
			return FString::Printf(TEXT("{\"ok\":false,\"stage\":\"init\",\"error\":\"%s\"}"),
				*Fmu.GetLastError().ReplaceCharWithEscapedChar());
		}

		double T = 0.0;
		Fmu.SetFloat64(InVR, 1.0);          // unit input, so the output integrates visibly
		if (!Fmu.DoStep(T, H))
		{
			return FString::Printf(TEXT("{\"ok\":false,\"stage\":\"step\",\"error\":\"%s\"}"),
				*Fmu.GetLastError().ReplaceCharWithEscapedChar());
		}
		T += H;
		double YAfterFirst = 0.0;
		Fmu.GetFloat64(OutVR, YAfterFirst);

		// --- the state round-trip ---
		bool bStateSupported = Package.GetInfo().bCanGetAndSetState;
		bool bStateMatched = false;
		double YRunA = 0.0, YRunB = 0.0;
		void* Snapshot = nullptr;
		FString StateNote;

		if (bStateSupported && Fmu.GetState(Snapshot) && Snapshot)
		{
			const double TSnapshot = T;
			const int32 N = 10;

			for (int32 i = 0; i < N; i++) { Fmu.SetFloat64(InVR, 1.0); Fmu.DoStep(T, H); T += H; }
			Fmu.GetFloat64(OutVR, YRunA);

			// Restore BOTH the FMU state and our own clock. Rewinding one
			// without the other reruns a different interval and the comparison
			// would be meaningless.
			if (Fmu.SetState(Snapshot))
			{
				T = TSnapshot;
				for (int32 i = 0; i < N; i++) { Fmu.SetFloat64(InVR, 1.0); Fmu.DoStep(T, H); T += H; }
				Fmu.GetFloat64(OutVR, YRunB);

				// Bitwise, not near-equal. A tolerance here would accept an
				// FMU that restores approximately, which is not restoring.
				bStateMatched = (FMath::IsNaN(YRunA) == FMath::IsNaN(YRunB)) &&
				                (*reinterpret_cast<const uint64*>(&YRunA) ==
				                 *reinterpret_cast<const uint64*>(&YRunB));
				StateNote = bStateMatched
					? TEXT("bitwise identical across restore")
					: TEXT("DIVERGED after restore — state save/restore does not reproduce");
			}
			else
			{
				StateNote = TEXT("SetFMUState failed");
			}
			Fmu.FreeState(Snapshot);
		}
		else
		{
			StateNote = bStateSupported ? TEXT("GetFMUState failed")
			                            : TEXT("FMU does not advertise state save/restore");
		}

		Fmu.Terminate();
		Fmu.FreeInstance();

		const bool bOk = bStateSupported && bStateMatched;
		return FString::Printf(
			TEXT("{\"ok\":%s,\"fmiVersionReported\":\"%s\",\"library\":\"%s\",")
			TEXT("\"stepSize\":%.9g,\"yAfterFirstStep\":%.17g,")
			TEXT("\"yRunA\":%.17g,\"yRunB\":%.17g,\"stateRoundTrip\":%s,\"stateNote\":\"%s\"}"),
			bOk ? TEXT("true") : TEXT("false"),
			*Fmu.GetVersion(),
			*FPaths::GetCleanFilename(Lib),
			H, YAfterFirst, YRunA, YRunB,
			bStateMatched ? TEXT("true") : TEXT("false"),
			*StateNote);
	}
	else if (Method.StartsWith(TEXT("inspectFmu")))
	{
		// inspectFmu <path-to.fmu>
		//
		// Opens an FMU, extracts it, and checks it against the gates in
		// docs/fmu_plant_migration.md — the same checks tools/fmu/
		// inspect_fmu.py runs offline, so the two must agree. This is the
		// in-engine one, which additionally proves the .fmu can be read by the
		// code that will actually have to load it: the offline tool uses
		// Python's zipfile, and agreeing with it says nothing about whether
		// our own ZIP reader and XML parse work on this file.
		//
		// No FMI runtime is invoked. Nothing is instantiated, nothing steps.
		// This answers "could we run this?", not "does it run?".
		// Parse the whole REQUEST, not Method: the dispatcher above already
		// split Method off as the first word, so Method never contains the
		// argument. Every other multi-argument command here parses Request —
		// this one did not, and reported "usage:" for a perfectly good path.
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "), true);
		if (Parts.Num() < 2)
		{
			return TEXT("{\"error\":\"usage: inspectFmu <path-to.fmu>\"}");
		}

		// Re-join the tail so paths containing spaces survive.
		FString FmuPath;
		for (int32 i = 1; i < Parts.Num(); i++)
		{
			if (i > 1) FmuPath += TEXT(" ");
			FmuPath += Parts[i];
		}

		FFSDSFmuPackage Package;
		if (!Package.Open(FmuPath))
		{
			return FString::Printf(TEXT("{\"ok\":false,\"error\":\"%s\"}"),
				*Package.GetError().ReplaceCharWithEscapedChar());
		}

		// The platform's communication step. Fixed 60 Hz — see
		// Config/DefaultEngine.ini and the determinism posture log.
		const double CommStep = 1.0 / 60.0;
		const bool bPassed = Package.LogGateResults(CommStep);
		const FFSDSFmuInfo& I = Package.GetInfo();

		FString GatesJson;
		for (const FFSDSFmuGate& G : Package.CheckGates(CommStep))
		{
			if (!GatesJson.IsEmpty()) GatesJson += TEXT(",");
			GatesJson += FString::Printf(
				TEXT("{\"name\":\"%s\",\"passed\":%s,\"required\":%s,\"detail\":\"%s\"}"),
				*G.Name, G.bPassed ? TEXT("true") : TEXT("false"),
				G.bRequired ? TEXT("true") : TEXT("false"),
				*G.Detail.ReplaceCharWithEscapedChar());
		}

		return FString::Printf(
			TEXT("{\"ok\":%s,\"fmiVersion\":\"%s\",\"modelName\":\"%s\",")
			TEXT("\"modelIdentifier\":\"%s\",\"tool\":\"%s\",\"token\":\"%s\",")
			TEXT("\"stateAttr\":\"%s\",\"canGetAndSetState\":%s,")
			TEXT("\"fixedInternalStepSize\":%g,\"binaries\":[%s],")
			TEXT("\"hostBinary\":\"%s\",\"sourceCode\":%s,\"resources\":%s,")
			TEXT("\"inputs\":%d,\"outputs\":%d,\"parameters\":%d,\"gates\":[%s]}"),
			bPassed ? TEXT("true") : TEXT("false"),
			I.Version == EFSDSFmiVersion::FMI3 ? TEXT("3.0")
				: I.Version == EFSDSFmiVersion::FMI2 ? TEXT("2.0") : TEXT("unknown"),
			*I.ModelName, *I.ModelIdentifier, *I.GenerationTool, *I.InstantiationToken,
			*I.StateAttributeFound,
			I.bCanGetAndSetState ? TEXT("true") : TEXT("false"),
			I.FixedInternalStepSize,
			*FString::Printf(TEXT("\"%s\""), *FString::Join(I.BinaryPlatforms, TEXT("\",\""))),
			*Package.GetBinaryPathForHost().ReplaceCharWithEscapedChar(),
			I.bHasSourceCode ? TEXT("true") : TEXT("false"),
			I.bHasResources ? TEXT("true") : TEXT("false"),
			I.NumInputs, I.NumOutputs, I.NumParameters, *GatesJson);
	}
	else if (Method.StartsWith(TEXT("getTireLoads")))
	{
		// Parse:  getTireLoads truth
		//         getTireLoads predict ax ay phi theta z
		// "truth"  → reads Chaos's per-tick spring force (the same Fz
		//            the wheel solver is using to compute grip).
		// "predict" with chassis state → runs the closed-form Milliken
		//            decomposition against the cached settings. Used for
		//            offline validation and for forward-planning callers
		//            that need "what-if" Fz at hypothetical accelerations.
		if (!IsValid(VehiclePawn)) return TEXT("{\"error\":\"no pawn\"}");
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		const FString Mode = (Parts.Num() >= 2) ? Parts[1] : TEXT("truth");

		AFSDSVehiclePawn::FTireLoads Loads;
		if (Mode == TEXT("predict"))
		{
			// predict ax ay phi theta z  (any missing args default to 0)
			const float AxIn    = (Parts.Num() >= 3) ? FCString::Atof(*Parts[2]) : 0.f;
			const float AyIn    = (Parts.Num() >= 4) ? FCString::Atof(*Parts[3]) : 0.f;
			const float PhiIn   = (Parts.Num() >= 5) ? FCString::Atof(*Parts[4]) : 0.f;
			const float ThetaIn = (Parts.Num() >= 6) ? FCString::Atof(*Parts[5]) : 0.f;
			const float ZIn     = (Parts.Num() >= 7) ? FCString::Atof(*Parts[6]) : 0.f;
			// The parametric path is pure-state, no async work. Run
			// inline on the TCP thread; settings reads are atomic float
			// reads on the cached shadow.
			Loads = VehiclePawn->ComputeTireLoadsParametric(
				AxIn, AyIn, PhiIn, ThetaIn, ZIn);
		}
		else if (Mode == TEXT("truth"))
		{
			// Truth path needs to bounce to the GameThread to read the
			// current FWheelStatus safely (the array is mutated each
			// physics tick on the game thread). 200 ms timeout matches
			// the rest of the GameThread RPCs.
			Loads = CallOnGameThread<AFSDSVehiclePawn::FTireLoads>(
				[this]() {
					if (!IsValid(VehiclePawn)) return AFSDSVehiclePawn::FTireLoads{};
					return VehiclePawn->GetTireLoadsTruth();
				},
				0.2,
				AFSDSVehiclePawn::FTireLoads{},
				TEXT("getTireLoads truth"));
		}
		else
		{
			return TEXT("{\"error\":\"usage: getTireLoads truth|predict [ax ay phi theta z]\"}");
		}

		return FString::Printf(
			TEXT("{\"mode\":\"%s\",\"FL\":%.4f,\"FR\":%.4f,\"RL\":%.4f,\"RR\":%.4f,\"total\":%.4f}"),
			*Mode, Loads.FL, Loads.FR, Loads.RL, Loads.RR, Loads.Total());
	}

	else if (Method == TEXT("simGetImage")     ||
	         Method == TEXT("simGetImages")    ||
	         Method == TEXT("simGetCameraInfo") ||
	         Method == TEXT("simSetCameraFov")  ||
	         Method == TEXT("simSetCameraOrientation"))
	{
		// Camera sensors were stripped in perf/strip-cameras (the real
		// IFS-08 has no cameras and the autonomy never consumed any
		// /camera/* topics). Return an explicit not-implemented so any
		// stale client surfaces the change instead of silently failing.
		return FString::Printf(TEXT("{\"error\":\"cameras removed; method not implemented: %s\"}"), *Method);
	}
	else if (Method == TEXT("listCameras"))
	{
		// Cameras removed (perf/strip-cameras) — always empty list.
		return TEXT("[]");
	}
	else if (Method == TEXT("getSensorOffset"))
	{
		// Return the ROS-ENU body-frame offset (meters) of a named sensor
		// so clients (most importantly the ROS bridge, for its
		// vehicle→sensor static TFs) don't have to hardcode positions
		// that drift out of sync with settings.json.
		//
		// Supported names:
		//   "lidar"            → the one LiDAR mount
		//
		// Camera sensors were stripped in perf/strip-cameras; "<camera>"
		// names are no longer recognised here. Body frame is REP-103
		// (X=forward, Y=left, Z=up); the plugin stores sensor positions
		// in UE units (X=forward, Y=right, Z=up, centimetres), so we
		// flip Y and convert cm→m.
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		if (Parts.Num() < 2) return TEXT("{\"error\":\"usage: getSensorOffset <name>\"}");
		if (!IsValid(VehiclePawn)) return TEXT("{\"error\":\"no vehicle\"}");

		FString Name = Parts[1];
		FVector UeOffsetCm(0.f, 0.f, 0.f);
		bool bFound = false;

		if (Name.Equals(TEXT("lidar"), ESearchCase::IgnoreCase))
		{
			if (VehiclePawn->LidarSensor)
			{
				UeOffsetCm = VehiclePawn->LidarSensor->SensorOffset;
				bFound = true;
			}
		}

		if (!bFound) return FString::Printf(TEXT("{\"error\":\"no sensor named %s\"}"), *Name);

		// UE (X=fwd, Y=right, Z=up, cm) → ROS body (X=fwd, Y=left, Z=up, m)
		const float X = UeOffsetCm.X / 100.f;
		const float Y = -UeOffsetCm.Y / 100.f;
		const float Z = UeOffsetCm.Z / 100.f;
		return FString::Printf(TEXT("{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f}"), X, Y, Z);
	}
	else if (Method == TEXT("simGetGroundTruthKinematics"))
	{
		if (!IsValid(VehiclePawn)) return TEXT("{}");
		auto State = VehiclePawn->GetCarState();
		FVector PosENU = FSDSCoord::UEToENU(State.Position);
		FVector VelENU = FSDSCoord::UEVelocityToENU(State.LinearVelocity);
		FVector AccENU = FSDSCoord::UEVelocityToENU(State.LinearAcceleration); // Same axis swap
		FVector AngVelENU = FSDSCoord::UEAngularVelocityToENU(State.AngularVelocity);
		FQuat OriENU = FSDSCoord::UEQuatToENU(State.Orientation);
		return FString::Printf(TEXT("{\"px\":%.4f,\"py\":%.4f,\"pz\":%.4f,\"vx\":%.4f,\"vy\":%.4f,\"vz\":%.4f,\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,\"wx\":%.4f,\"wy\":%.4f,\"wz\":%.4f,\"qw\":%.6f,\"qx\":%.6f,\"qy\":%.6f,\"qz\":%.6f}"),
			PosENU.X, PosENU.Y, PosENU.Z,
			VelENU.X, VelENU.Y, VelENU.Z,
			AccENU.X, AccENU.Y, AccENU.Z,
			AngVelENU.X, AngVelENU.Y, AngVelENU.Z,
			OriENU.W, OriENU.X, OriENU.Y, OriENU.Z);
	}

	// === Simulation Control ===

	else if (Method == TEXT("simPause"))
	{
		bSimPaused = true;
		AsyncTask(ENamedThreads::GameThread, [this]() {
			if (World) UGameplayStatics::SetGamePaused(World, true);
		});
		return TEXT("true");
	}
	else if (Method == TEXT("simResume"))
	{
		bSimPaused = false;
		AsyncTask(ENamedThreads::GameThread, [this]() {
			if (World) UGameplayStatics::SetGamePaused(World, false);
		});
		return TEXT("true");
	}
	else if (Method == TEXT("simIsPaused"))
	{
		return bSimPaused ? TEXT("true") : TEXT("false");
	}
	else if (Method == TEXT("simContinueForTime"))
	{
		// Parse: simContinueForTime seconds
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		float Seconds = (Parts.Num() >= 2) ? FCString::Atof(*Parts[1]) : 1.0f;

		AsyncTask(ENamedThreads::GameThread, [this, Seconds]() {
			if (World)
			{
				UGameplayStatics::SetGamePaused(World, false);
				// Schedule re-pause after duration
				FTimerHandle Handle;
				World->GetTimerManager().SetTimer(Handle, [this]() {
					UGameplayStatics::SetGamePaused(World, true);
					bSimPaused = true;
				}, Seconds, false);
			}
		});
		bSimPaused = false;
		return TEXT("true");
	}
	else if (Method == TEXT("reset"))
	{
		// Destructive level reload (OpenLevel) crashed the editor; removed in fix/12.
		// Clients should use resetScenario (soft reset) instead.
		UE_LOG(LogTemp, Warning, TEXT("FSDS RPC: 'reset' is no longer supported; use resetScenario"));
		return TEXT("{\"error\":\"reset removed; use resetScenario\"}");
	}

	else if (Method.StartsWith(TEXT("resetScenario")))
	{
		// Soft scenario reset for repeat runs: restore the sim to the state a
		// fresh run would start from, WITHOUT reloading the level (OpenLevel
		// is what crashed the editor and got 'reset' removed).
		//
		// The point of this call is repeatability. A repeat is only a repeat if
		// every piece of carried-over state is restored — it is easy to reset
		// the visible things (pose, cones) and silently leave the invisible
		// ones (RNG position, IMU bias drift, rotor speed), which produces a
		// run that looks like a repeat and is not one.
		//
		// Optional argument: resetScenario [seed]. Passing a seed makes this
		// the primitive a batch runner sweeps over for N-seed repeats.
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		const bool bHasSeed = (Parts.Num() >= 2);
		const int32 NewSeed = bHasSeed ? FCString::Atoi(*Parts[1]) : 0;

		FString Result;
		FEvent* Done = FPlatformProcess::GetSynchEventFromPool(false);

		AsyncTask(ENamedThreads::GameThread, [this, bHasSeed, NewSeed, &Result, Done]()
		{
			int32 NumCones = 0;

			// 1. Re-seed. Bumps the RNG generation, which makes every cached
			//    sensor stream re-seed on next use and clears IMU bias drift.
			FSDSRandom::SetScenarioSeed(bHasSeed ? NewSeed : FSDSRandom::GetScenarioSeed());

			// 2. Referee counters (DOO / out-of-course / laps / times).
			//
			// ResetForRepeatRun, NOT ResetState. ResetState wipes the cone
			// registry, the cone list and the finish line — fine for
			// loadTrack, which respawns cones immediately afterwards, but
			// resetScenario respawns nothing. Using it here left the referee
			// permanently blind: DOO, off-course and lap detection all dead,
			// so every run after the first silently scored 0 / 0 / 0. Any A/B
			// campaign built on resetScenario was comparing empty scorecards.
			if (Referee) Referee->ResetForRepeatRun();

			// 3. Vehicle: back to the start gate with velocities zeroed, and
			//    powertrain state cleared. EBS stays as-is deliberately — in
			//    the real FS-DV flow the RES Go signal owns that transition
			//    (T 14.8.4), and loadTrack made the same choice.
			if (IsValid(VehiclePawn))
			{
				if (World)
				{
					for (TActorIterator<AFSDSConeSpawner> It(World); It; ++It)
					{
						FVector StartLoc; FQuat StartRot;
						if (It->ComputeStartGatePose(StartLoc, StartRot, 300.f))
						{
							VehiclePawn->SetActorLocationAndRotation(
								StartLoc, StartRot, false, nullptr, ETeleportType::TeleportPhysics);
							NotifyPlantsOfTeleport(VehiclePawn, StartLoc, StartRot);
						}
						NumCones = It->SpawnedCones.Num();
						break;
					}
				}

				// Zero the physics body. Chaos keeps its own velocities on the
				// body instance, so teleporting the actor alone leaves the car
				// carrying the previous run's momentum into the new one.
				if (USkeletalMeshComponent* Mesh = VehiclePawn->GetMesh())
				{
					if (Mesh->IsSimulatingPhysics())
					{
						Mesh->SetPhysicsLinearVelocity(FVector::ZeroVector);
						Mesh->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
					}
				}

				// Rotor speed is integrated state; without this the repeat
				// starts with the previous run's motor spinning.
				if (VehiclePawn->Motor) VehiclePawn->Motor->Reset();
			}

			Result = FString::Printf(
				TEXT("{\"ok\":true,\"seed\":%d,\"generation\":%u,\"cones\":%d}"),
				FSDSRandom::GetScenarioSeed(), FSDSRandom::GetGeneration(), NumCones);

			UE_LOG(LogTemp, Log,
				TEXT("FSDS RPC: resetScenario — seed %d (generation %u), referee cleared, "
					 "vehicle at start gate, velocities and rotor zeroed"),
				FSDSRandom::GetScenarioSeed(), FSDSRandom::GetGeneration());

			Done->Trigger();
		});

		Done->Wait();
		FPlatformProcess::ReturnSynchEventToPool(Done);
		return Result;
	}

	// === Object APIs ===

	else if (Method == TEXT("listSceneObjects"))
	{
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		FString Filter = (Parts.Num() >= 2) ? Parts[1] : TEXT("*");

		return CallOnGameThread<FString>(
			[this, Filter]() -> FString {
				FString Result = TEXT("[");
				bool bFirst = true;
				if (World)
				{
					for (TActorIterator<AActor> It(World); It; ++It)
					{
						FString ActorName = It->GetName();
						if (Filter == TEXT("*") || ActorName.Contains(Filter))
						{
							if (!bFirst) Result += TEXT(",");
							Result += FString::Printf(TEXT("\"%s\""), *ActorName);
							bFirst = false;
						}
					}
				}
				Result += TEXT("]");
				return Result;
			},
			3.0,
			FString(TEXT("{\"error\":\"timeout\"}")),
			TEXT("listSceneObjects"));
	}
	else if (Method == TEXT("getObjectPose"))
	{
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		if (Parts.Num() < 2) return TEXT("{\"error\":\"missing object_name\"}");
		FString ObjName = Parts[1];

		return CallOnGameThread<FString>(
			[this, ObjName]() -> FString {
				if (World)
				{
					for (TActorIterator<AActor> It(World); It; ++It)
					{
						if (It->GetName() == ObjName)
						{
							FVector PosENU = FSDSCoord::UEToENU(It->GetActorLocation());
							FQuat OriENU = FSDSCoord::UEQuatToENU(It->GetActorQuat());
							return FString::Printf(TEXT("{\"px\":%.4f,\"py\":%.4f,\"pz\":%.4f,\"qw\":%.6f,\"qx\":%.6f,\"qy\":%.6f,\"qz\":%.6f}"),
								PosENU.X, PosENU.Y, PosENU.Z, OriENU.W, OriENU.X, OriENU.Y, OriENU.Z);
						}
					}
				}
				return TEXT("{\"error\":\"object not found\"}");
			},
			3.0,
			FString(TEXT("{\"error\":\"timeout\"}")),
			TEXT("getObjectPose"));
	}
	else if (Method == TEXT("setObjectPose"))
	{
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		if (Parts.Num() < 5) return TEXT("{\"error\":\"usage: setObjectPose name x y z\"}");

		FString ObjName = Parts[1];
		FVector PosENU(FCString::Atof(*Parts[2]), FCString::Atof(*Parts[3]), FCString::Atof(*Parts[4]));
		FVector PosUE = FSDSCoord::ENUToUE(PosENU);

		return CallOnGameThread<FString>(
			[this, ObjName, PosUE]() -> FString {
				if (World)
				{
					for (TActorIterator<AActor> It(World); It; ++It)
					{
						if (It->GetName() == ObjName)
						{
							It->SetActorLocation(PosUE, false, nullptr, ETeleportType::TeleportPhysics);
							return TEXT("true");
						}
					}
				}
				return TEXT("{\"error\":\"object not found\"}");
			},
			3.0,
			FString(TEXT("{\"error\":\"timeout\"}")),
			TEXT("setObjectPose"));
	}
	else if (Method == TEXT("simSetVehiclePose"))
	{
		// Parse: simSetVehiclePose x y z [qw qx qy qz]
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		if (Parts.Num() < 4) return TEXT("{\"error\":\"usage: simSetVehiclePose x y z [qw qx qy qz]\"}");

		FVector PosENU(FCString::Atof(*Parts[1]), FCString::Atof(*Parts[2]), FCString::Atof(*Parts[3]));
		FVector PosUE = FSDSCoord::ENUToUE(PosENU);

		bool bHasOrientation = Parts.Num() >= 8;
		FQuat QuatUE = FQuat::Identity;
		if (bHasOrientation)
		{
			float qw = FCString::Atof(*Parts[4]);
			float qx = FCString::Atof(*Parts[5]);
			float qy = FCString::Atof(*Parts[6]);
			float qz = FCString::Atof(*Parts[7]);
			QuatUE = FSDSCoord::ENUQuatToUE(FQuat(qx, qy, qz, qw));
		}

		AsyncTask(ENamedThreads::GameThread, [this, PosUE, QuatUE, bHasOrientation]() {
			if (IsValid(VehiclePawn))
			{
				// Note: this handler used to call ReleaseEbs() here "as
				// a soft reset". Removed because /api/sim/reset
				// activates EBS *immediately before* this teleport so
				// the parked car can't roll, and the auto-release was
				// undoing that same call — the user saw the car drift
				// away and the brake state stay non-zero after every
				// FE-side reset. Callers that need EBS released (e.g.
				// /api/event/start) already do it explicitly via the
				// releaseEbs RPC; everyone else gets to keep the EBS
				// state they had going in.
				// For position-only teleports, save the current heading before
				// ResetVehicleState() so we can restore it after. Chaos's
				// ResetVehicleState/StopMovementImmediately resets the physics
				// body to a canonical (identity) orientation, wiping the
				// track-aligned heading set by loadTrack. This made the car
				// face East instead of North after every teleport-kick, driving
				// it into the cone wall on launch and placing it perpendicular
				// to the start line on reset.
				const FQuat HeadingToRestore = bHasOrientation
					? QuatUE
					: VehiclePawn->GetActorQuat();

				if (bHasOrientation)
					VehiclePawn->SetActorLocationAndRotation(PosUE, QuatUE, false, nullptr, ETeleportType::TeleportPhysics);
				else
					VehiclePawn->SetActorLocation(PosUE, false, nullptr, ETeleportType::TeleportPhysics);

				// Chaos vehicle workaround: SetActorLocationAndRotation with TeleportPhysics
				// does not reliably update the skeletal mesh's physics body rotation — the body
				// keeps its prior orientation and the actor transform snaps back on the next tick.
				// Force the physics body transform directly.
				USkeletalMeshComponent* Mesh = VehiclePawn->GetMesh();
				if (Mesh && Mesh->IsSimulatingPhysics())
				{
					if (bHasOrientation)
					{
						FTransform NewXform(QuatUE, PosUE, Mesh->GetComponentScale());
						Mesh->BodyInstance.SetBodyTransform(NewXform, ETeleportType::TeleportPhysics);
					}
					Mesh->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
					// Also zero linear velocity. Without this, residual
					// momentum (or gravity-on-tilt drift accumulated
					// since the previous tick) survives the teleport,
					// and the chassis arrives at the new pose still
					// moving — broke our "spawn at start gate then
					// arm EBS" sequence with v ≈ 0.4 m/s after every
					// /reset and loadTrack call. A teleport semantically
					// resets pose, which on a vehicle includes its
					// momentum.
					Mesh->SetPhysicsLinearVelocity(FVector::ZeroVector);
				}
				// Clear Chaos vehicle wheel angular velocities + raw inputs +
				// shift to neutral gear. See loadTrack handler for the long
				// explanation; same reasoning applies to any manual teleport.
				if (VehiclePawn->VehicleMovement)
				{
					VehiclePawn->VehicleMovement->ResetVehicleState();

					// ResetVehicleState destroys and recreates the physics
					// state, which re-runs CreateVehicle() and rebuilds every
					// physics wheel from the CLASS DEFAULT OBJECT — silently
					// discarding everything settings.json pushed to the solver.
					// Re-apply and re-verify, or the rest of the session runs a
					// different car than the one that was configured.
					VehiclePawn->ApplyWheelSettingsToSolver();
				}
				// Restore the saved heading — ResetVehicleState wipes it to
				// identity. Apply to both the actor and the physics body so
				// the two stay in sync across the teleport.
				if (Mesh && Mesh->IsSimulatingPhysics())
				{
					FTransform RestoredXform(HeadingToRestore, PosUE, Mesh->GetComponentScale());
					Mesh->BodyInstance.SetBodyTransform(RestoredXform, ETeleportType::TeleportPhysics);
				}
				VehiclePawn->SetActorRotation(HeadingToRestore, ETeleportType::TeleportPhysics);

				NotifyPlantsOfTeleport(VehiclePawn, VehiclePawn->GetActorLocation(), HeadingToRestore);

				// No velocity kick. The previous 5 cm/s body-forward push was
				// a workaround for Chaos pinning at the (v=0, ω=0) degenerate
				// state. It was firing during the SLAM's INIT_CALIBRATING
				// window (3 s stationary requirement) and corrupting the IMU
				// bias estimate, which cascaded into permanent DA-failure
				// rejection of every LiDAR scan. The proper fix lives in
				// the wheel config: rear FrictionForceMultiplier was lowered
				// (1.4 → 1.0) and WheelMass halved (10 → 5 kg) so the EMRAX
				// shaft torque alone can break the rear axle's static-
				// friction lock at standstill. Launch is now closed-loop:
				// the autonomy commands throttle when SLAM is ready and the
				// chassis accelerates from rest under real drive torque.
			}
		});
		return TEXT("true");
	}

	else if (Method == TEXT("validateRoadProbe"))
	{
		// Build deliberately non-flat ground and check the probe against the
		// analytic surface. Every lap so far ran on dead-flat terrain, where a
		// working probe and a stub returning zero produce identical logs — so
		// "4/4 valid on every sample" was never evidence of anything.
		//
		// The crown is the load-bearing case. Its two faces have normals with
		// OPPOSITE y components, so a probe using the axial rule (-x,y,-z)
		// instead of the polar rule (x,-y,z) reports the camber backwards. On
		// flat ground the two rules agree exactly, because y is zero.
		//
		// EVERYTHING HERE RUNS ON THE GAME THREAD. Actor iteration, spawning
		// and line traces all assert IsInGameThread(), and RPC handlers do
		// not run there — calling them directly took the editor down with a
		// SIGSEGV rather than returning an error.
		if (!VehiclePawn) return TEXT("{\"ok\":false,\"error\":\"no vehicle\"}");

		return CallOnGameThread<FString>([this]() -> FString
		{
			UWorld* ProbeWorld = VehiclePawn ? VehiclePawn->GetWorld() : nullptr;
			if (!ProbeWorld) return TEXT("{\"ok\":false,\"error\":\"no world\"}");

			AFSDSTestTerrain* Terrain = nullptr;
			for (TActorIterator<AFSDSTestTerrain> It(ProbeWorld); It; ++It) { Terrain = *It; break; }
			if (!Terrain)
			{
				Terrain = ProbeWorld->SpawnActor<AFSDSTestTerrain>(AFSDSTestTerrain::StaticClass());
				if (!Terrain) return TEXT("{\"ok\":false,\"error\":\"spawn failed\"}");
				Terrain->Build();
			}

			int32 Checked = 0, Missed = 0, BadHeight = 0, BadNormal = 0, BadResidual = 0;
			double WorstHeight = 0.0, WorstNormal = 0.0, WorstPlanarRes = 0.0;
			FString WorstWhere;
			// Per-patch detail. A single pass/fail cannot tell "the probe is
			// wrong" from "the test's geometry is wrong", and I wrote both.
			FString Detail;

			for (const FFSDSTestPatch& P : Terrain->GetPatches())
			{
				// Sample inside the patch, away from the edges — an edge
				// sample would be testing the slab's extent, not the probe.
				for (int32 ix = -1; ix <= 1; ix++)
				for (int32 iy = -1; iy <= 1; iy++)
				{
					const double X = P.CentreX + ix * P.HalfLenX * 0.5;
					const double Y = P.CentreY + iy * P.HalfLenY * 0.5;
					const double TrueZ = P.HeightAt(X, Y);

					// Contract -> UE for the trace start: y negates, m to cm.
					const FVector StartCm(X * 100.0, -Y * 100.0, (TrueZ + 0.3) * 100.0);

					double GotZ = 0.0, GotN[3] = {0,0,1};
					double GotRes = AFSDSVehiclePawn::kRoadResidualNotFitted;
					Checked++;
					if (!VehiclePawn->ProbeRoadPatch(StartCm, GotZ, GotN, GotRes)) { Missed++; continue; }

					// Every patch here is planar, so a working fit must report
					// a residual near zero. A fit that silently failed would
					// return the not-fitted sentinel, which this also catches.
					if (!(GotRes >= 0.0 && GotRes < 0.005)) BadResidual++;
					if (GotRes > WorstPlanarRes) WorstPlanarRes = GotRes;

					const double dZ = FMath::Abs(GotZ - TrueZ);
					const double dN = FMath::Sqrt(
						FMath::Square(GotN[0] - P.Normal[0]) +
						FMath::Square(GotN[1] - P.Normal[1]) +
						FMath::Square(GotN[2] - P.Normal[2]));

					if (dZ > WorstHeight) { WorstHeight = dZ; WorstWhere = P.Name; }
					if (dN > WorstNormal) WorstNormal = dN;
					if (dZ > 0.01) BadHeight++;      // 1 cm
					if (dN > 0.02) BadNormal++;      // ~1.1 deg

					if (ix == 0 && iy == 0)
					{
						Detail += FString::Printf(
							TEXT("%s{\"patch\":\"%s\",\"x\":%.2f,\"y\":%.2f,")
							TEXT("\"trueZ\":%.4f,\"gotZ\":%.4f,")
							TEXT("\"trueN\":[%.3f,%.3f,%.3f],\"gotN\":[%.3f,%.3f,%.3f]}"),
							Detail.IsEmpty() ? TEXT("") : TEXT(","),
							*P.Name, X, Y, TrueZ, GotZ,
							P.Normal[0], P.Normal[1], P.Normal[2],
							GotN[0], GotN[1], GotN[2]);
					}
				}
			}

			// The seam. A residual that is always ~0 would pass every check
			// above while carrying no information at all, so the test has to
			// include ground that no plane describes and confirm the number
			// actually rises.
			double SeamRes = AFSDSVehiclePawn::kRoadResidualNotFitted;
			double SeamZ = 0.0, SeamN[3] = {0,0,1};
			bool bSeamProbed = false;
			{
				const FVector SeamStart(100.0 * 100.0, 0.0, 0.45 * 100.0);
				bSeamProbed = VehiclePawn->ProbeRoadPatch(SeamStart, SeamZ, SeamN, SeamRes);
			}
			// 0.15 m step across an 8 cm span: the fit cannot absorb that, so
			// anything below a centimetre means the residual is not measuring.
			const bool bSeamOk = bSeamProbed && SeamRes > 0.01;

			const bool bOk = (Missed == 0 && BadHeight == 0 && BadNormal == 0
			                  && BadResidual == 0 && bSeamOk);
			return FString::Printf(
				TEXT("{\"ok\":%s,\"checked\":%d,\"missed\":%d,\"badHeight\":%d,")
				TEXT("\"badNormal\":%d,\"worstHeightM\":%.4f,\"worstNormal\":%.4f,")
				TEXT("\"worstPatch\":\"%s\",\"badResidual\":%d,")
				TEXT("\"worstPlanarResidualM\":%.5f,\"seamOk\":%s,\"seamResidualM\":%.5f,")
				TEXT("\"centres\":[%s]}"),
				bOk ? TEXT("true") : TEXT("false"),
				Checked, Missed, BadHeight, BadNormal,
				WorstHeight, WorstNormal, *WorstWhere,
				BadResidual, WorstPlanarRes,
				bSeamOk ? TEXT("true") : TEXT("false"), SeamRes, *Detail);
		}, 20.0, FString(TEXT("{\"ok\":false,\"error\":\"game-thread timeout\"}")),
		   TEXT("validateRoadProbe"));
	}

	// === New Sensors ===

	else if (Method == TEXT("getDistanceSensorData"))
	{
		if (!VehiclePawn || !VehiclePawn->DistanceSensor) return TEXT("{}");
		auto Data = VehiclePawn->DistanceSensor->GetOutput();
		return FString::Printf(TEXT("{\"distance\":%.4f,\"min\":%.2f,\"max\":%.2f}"),
			Data.Distance, Data.MinDistance, Data.MaxDistance);
	}
	else if (Method == TEXT("getBarometerData"))
	{
		if (!VehiclePawn || !VehiclePawn->BarometerSensor) return TEXT("{}");
		auto Data = VehiclePawn->BarometerSensor->GetOutput();
		return FString::Printf(TEXT("{\"altitude\":%.4f,\"pressure\":%.2f,\"temperature\":%.2f}"),
			Data.Altitude, Data.Pressure, Data.Temperature);
	}
	else if (Method == TEXT("getMagnetometerData"))
	{
		if (!VehiclePawn || !VehiclePawn->MagnetometerSensor) return TEXT("{}");
		auto Data = VehiclePawn->MagnetometerSensor->GetOutput();
		return FString::Printf(TEXT("{\"mx\":%.6f,\"my\":%.6f,\"mz\":%.6f}"),
			Data.MagneticField.X, Data.MagneticField.Y, Data.MagneticField.Z);
	}

	// === Camera info/control ===
	// Removed in perf/strip-cameras — the camera RPCs (simGetImage,
	// simGetImages, simGetCameraInfo, simSetCameraFov,
	// simSetCameraOrientation, simGetImageBinary) now return a single
	// "method not implemented" stub handled at the top of the dispatch.

	// === Weather / TimeOfDay — still "true" stubs ===
	// Left as-is because no client cares about the return value today and
	// the team hasn't decided whether it wants real weather/TOD simulation
	// or silent no-ops. Revisit when that decision lands.
	else if (Method == TEXT("simEnableWeather") || Method == TEXT("simSetWeatherParameter") || Method == TEXT("simSetTimeOfDay"))
	{
		return TEXT("true");
	}

	// === Visualization helpers — not implemented ===
	// The AirSim-era simPlot* APIs would draw persistent debug primitives
	// in the world. The driverless pipeline never called them, so nobody
	// missed them when the plugin migrated off AirSim. Previously returned
	// the string "true" as a silent accept, which hides bugs if any new
	// client starts relying on them — report the real state instead.
	else if (Method == TEXT("simPlotPoints") || Method == TEXT("simPlotLineStrip") ||
		Method == TEXT("simPlotLineList") || Method == TEXT("simPlotArrows") ||
		Method == TEXT("simPlotStrings") || Method == TEXT("simPlotTransforms") ||
		Method == TEXT("simPlotTransformsWithNames") || Method == TEXT("simFlushPersistentMarkers"))
	{
		return FString::Printf(TEXT("{\"error\":\"method not implemented: %s\"}"), *Method);
	}

	// === Segmentation — not implemented ===
	// Segmentation camera requires a custom post-process material per
	// object class + per-actor stencil assignment; out of scope today.
	// Returning explicit not-implemented rather than faked success.
	else if (Method == TEXT("simSetSegmentationObjectID") ||
		Method == TEXT("simGetSegmentationObjectID") ||
		Method == TEXT("simSwapTextures"))
	{
		return FString::Printf(TEXT("{\"error\":\"method not implemented: %s\"}"), *Method);
	}

	// === Cone diagnostic: report which cone meshes loaded successfully (game thread) ===
	else if (Method == TEXT("debugCones"))
	{
		return CallOnGameThread<FString>(
			[this]() -> FString {
				static const TCHAR* ConePaths[] = {
					TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_blue.trafficone_mini_blue"),
					TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_yellow.trafficone_mini_yellow"),
					TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_orange.trafficone_mini_orange"),
					TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_big_orange.trafficone_big_orange"),
				};
				// Also try without the .ObjectName suffix (package-only path)
				static const TCHAR* ConePathsShort[] = {
					TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_blue"),
					TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_yellow"),
					TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_mini_orange"),
					TEXT("/Game/RaceCourse/Model/Environment/trafficones_scaled/trafficone_big_orange"),
				};
				static const TCHAR* ConeNames[] = { TEXT("blue"), TEXT("yellow"), TEXT("orange"), TEXT("big_orange") };

				FString Out = TEXT("{");
				for (int32 i = 0; i < 4; i++)
				{
					UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, ConePaths[i]);
					if (!Mesh)
					{
						FSoftObjectPath SoftPath(ConePaths[i]);
						Mesh = Cast<UStaticMesh>(SoftPath.TryLoad());
					}
					if (!Mesh)
					{
						FSoftObjectPath SoftPath2(ConePathsShort[i]);
						Mesh = Cast<UStaticMesh>(SoftPath2.TryLoad());
					}
					Out += FString::Printf(TEXT("\"%s\":%s"), ConeNames[i], Mesh ? TEXT("true") : TEXT("false"));
					if (i < 3) Out += TEXT(",");
				}

				AFSDSConeSpawner* Spawner = nullptr;
				if (World) {
					for (TActorIterator<AFSDSConeSpawner> It(World); It; ++It) { Spawner = *It; break; }
				}
				Out += FString::Printf(TEXT(",\"spawner\":%s,\"world\":%s}"),
					Spawner ? TEXT("true") : TEXT("false"),
					World   ? TEXT("true") : TEXT("false"));
				return Out;
			},
			5.0,
			FString(TEXT("{\"error\":\"timeout\"}")),
			TEXT("debugCones"));
	}

	// === Track loading ===

	else if (Method == TEXT("loadTrack"))
	{
		// Parse: loadTrack path/to/track.csv
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		if (Parts.Num() < 2) return TEXT("{\"error\":\"usage: loadTrack path.csv\"}");

		FString TrackPath = Parts[1];

		// --- Path resolution ---
		// If the caller passes an absolute path that exists, use it directly.
		// Otherwise treat it as a leaf filename and search the same candidate
		// directories that FSDSConeSpawner uses at BeginPlay, so the track
		// generator can write generated CSVs to any of those locations and
		// just pass the bare filename (e.g. "track_20260404_013721.csv").
		if (!FPaths::IsRelative(TrackPath) && FPaths::FileExists(TrackPath))
		{
			// absolute & exists — nothing to do
		}
		else
		{
			// Packaged: sibling tracks/ dir next to the .app
		// ProjectDir = {App}/Contents/UE/IFSSIM/  →  ../../../../ = parent of {App}
		FString SiblingTracksDir = FPaths::ConvertRelativePathToFull(
			FPaths::Combine(FPaths::ProjectDir(), TEXT("../../../../tracks")));
		TArray<FString> SearchDirs = {
				FPaths::Combine(FPaths::ProjectDir(),  TEXT("Content"), TEXT("tracks")), // editor / PIE source tree
				SiblingTracksDir,                                                         // next to .app (distribution layout)
				FPaths::Combine(FPaths::LaunchDir(),   TEXT("Content"), TEXT("tracks")), // LaunchDir fallback
				FPaths::Combine(FPaths::LaunchDir(),   TEXT("tracks")),                  // safety net
				// User-writable fallback for generated tracks
				FPaths::Combine(FPlatformProcess::UserSettingsDir(), TEXT("IFSSIM"), TEXT("tracks")),
			};
			FString Leaf = FPaths::GetCleanFilename(TrackPath);
			bool bResolved = false;
			for (const FString& Dir : SearchDirs)
			{
				FString Candidate = FPaths::Combine(Dir, Leaf);
				if (FPaths::FileExists(Candidate))
				{
					UE_LOG(LogTemp, Log, TEXT("FSDS loadTrack: resolved '%s' → %s"), *TrackPath, *Candidate);
					TrackPath = Candidate;
					bResolved = true;
					break;
				}
			}
			if (!bResolved)
			{
				// Keep the original path — SpawnFromCSV will log the specific error
				UE_LOG(LogTemp, Warning, TEXT("FSDS loadTrack: '%s' not found in any search dir, attempting as-is"), *TrackPath);
			}
		}

		// Find and reload the ConeSpawner on game thread
		return CallOnGameThread<FString>(
			[this, TrackPath]() -> FString {
				if (!World) return TEXT("{\"error\":\"no world\"}");

				AFSDSConeSpawner* Spawner = nullptr;
				for (TActorIterator<AFSDSConeSpawner> It(World); It; ++It)
				{
					Spawner = *It;
					break;
				}
				if (!Spawner) return TEXT("{\"error\":\"no ConeSpawner found\"}");

				// Destroy all previously spawned cone actors (tracked by spawner)
				for (AActor* ConeActor : Spawner->SpawnedCones)
				{
					if (ConeActor && IsValid(ConeActor)) ConeActor->Destroy();
				}
				Spawner->SpawnedCones.Empty();

				if (Referee) Referee->ResetState();

				// Set new CSV path and re-run spawning (ReloadTrack avoids
				// double-calling Super::BeginPlay)
				Spawner->ReloadTrack(TrackPath);

				int32 NumCones = Spawner->SpawnedCones.Num();

				// Auto-orient the car to the track's start gate. Without
				// this, the level's PlayerStart pose (e.g. customMap
				// spawns at (0, 0) yaw=+90°) leaves the autonomy stack
				// fighting a gross misalignment on every fresh track —
				// it has to either teleport the car manually or drive
				// the wrong way through the gate. Spawning behind the
				// orange cones, facing the bulk of the track, makes
				// loadTrack a single self-sufficient operation:
                //   "load this CSV → cones placed → car aligned → ready."
				FVector StartLoc;
				FQuat StartRot;
				bool bAligned = false;
				if (Spawner->ComputeStartGatePose(StartLoc, StartRot, 300.f) && IsValid(VehiclePawn))
				{
					// Mirrors simSetVehiclePose's teleport-with-velocity-reset
					// path. EBS is intentionally NOT released here — the
					// previous "soft reset" pattern was wrong: in the
					// real FS-DV flow the driver releases EBS via the
					// RES Go signal (T 14.8.4), not as a side-effect of
					// loading a track. Track load only spawns the car at
					// the start gate; the autonomous mission state
					// machine owns EBS transitions. The skeletal mesh
					// BodyInstance has to be snapped explicitly because
					// Chaos vehicles otherwise keep their old physics-
					// body rotation and snap the actor transform back on
					// the next tick.
					VehiclePawn->SetActorLocationAndRotation(StartLoc, StartRot, false, nullptr, ETeleportType::TeleportPhysics);
					USkeletalMeshComponent* Mesh = VehiclePawn->GetMesh();
					if (Mesh && Mesh->IsSimulatingPhysics())
					{
						FTransform NewXform(StartRot, StartLoc, Mesh->GetComponentScale());
						Mesh->BodyInstance.SetBodyTransform(NewXform, ETeleportType::TeleportPhysics);
						Mesh->SetPhysicsLinearVelocity(FVector::ZeroVector);
						Mesh->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
					}
					NotifyPlantsOfTeleport(VehiclePawn, StartLoc, StartRot);
					// Chaos vehicles store *wheel* angular velocity in the
					// vehicle simulation core (FWheeledVehicleSimulation), not
					// in the rigid-body's angular velocity. Zeroing the body
					// above doesn't touch it. Without this, after the teleport
					// the wheels keep spinning at their pre-teleport speed and
					// propel the car a few metres in the OLD forward direction
					// before friction stops them — observed in the autocross
					// run validating fix/45: car drifted east-southeast even
					// with throttle=0, the path planner re-anchored to the
					// drifted position, picked the wrong loop direction, and
					// the cascade locked in.
					//
					// `ResetVehicleState()` does StopMovementImmediately
					// (redundant with the SetPhysics*Velocity above, but cheap)
					// + OnDestroyPhysicsState + OnCreatePhysicsState (which
					// rebuilds the wheels with zero angular velocity) +
					// shifts to neutral gear + clears raw throttle/brake/
					// steering inputs. Equivalent to a fresh PIE spawn.
					if (VehiclePawn->VehicleMovement)
					{
						VehiclePawn->VehicleMovement->ResetVehicleState();

					// ResetVehicleState destroys and recreates the physics
					// state, which re-runs CreateVehicle() and rebuilds every
					// physics wheel from the CLASS DEFAULT OBJECT — silently
					// discarding everything settings.json pushed to the solver.
					// Re-apply and re-verify, or the rest of the session runs a
					// different car than the one that was configured.
					VehiclePawn->ApplyWheelSettingsToSolver();
					}
					bAligned = true;
				LastStartGateLoc_UE = StartLoc;
				LastStartGateRot_UE = StartRot;
				bHasStartGate = true;
				}

				return FString::Printf(
					TEXT("{\"loaded\":\"%s\",\"cones\":%d,\"car_aligned\":%s}"),
					*TrackPath, NumCones, bAligned ? TEXT("true") : TEXT("false"));
			},
			5.0,
			FString(TEXT("{\"error\":\"timeout\"}")),
			TEXT("loadTrack"));
	}

	// === UDP Target Registration ===

	else if (Method == TEXT("registerUdpTarget"))
	{
		// Parse: registerUdpTarget <ip>
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		if (Parts.Num() < 2) return TEXT("{\"error\":\"usage: registerUdpTarget ip\"}");

		FString TargetIP = Parts[1];
		if (UdpBroadcaster)
		{
			UdpBroadcaster->SetTargetIP(TargetIP);
			return FString::Printf(TEXT("{\"registered\":\"%s\"}"), *TargetIP);
		}
		return TEXT("{\"error\":\"no UDP broadcaster\"}");
	}

	// === Event Type Control ===

	else if (Method.StartsWith(TEXT("setEventType")))
	{
		// Parse: setEventType trackdrive [10]
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		if (Parts.Num() < 2) return TEXT("{\"error\":\"usage: setEventType acceleration|skidpad|autocross|trackdrive [laps]\"}");

		FString EventName = Parts[1].ToLower();
		int32 NumLaps = (Parts.Num() >= 3) ? FCString::Atoi(*Parts[2]) : 10;

		if (!Referee) return TEXT("{\"error\":\"no referee\"}");

		EFSDSEventType EventType = EFSDSEventType::Trackdrive;
		if (EventName == TEXT("acceleration")) EventType = EFSDSEventType::Acceleration;
		else if (EventName == TEXT("skidpad")) EventType = EFSDSEventType::Skidpad;
		else if (EventName == TEXT("autocross")) EventType = EFSDSEventType::Autocross;
		else if (EventName == TEXT("trackdrive")) EventType = EFSDSEventType::Trackdrive;
		else return FString::Printf(TEXT("{\"error\":\"unknown event: %s\"}"), *EventName);

		AsyncTask(ENamedThreads::GameThread, [this, EventType, NumLaps]() {
			if (Referee) Referee->SetEventType(EventType, NumLaps);
		});

		return FString::Printf(TEXT("{\"event\":\"%s\",\"laps\":%d}"), *EventName, NumLaps);
	}

	// === Sim Status ===

	else if (Method == TEXT("getSimStatus"))
	{
		FString MapName = TEXT("unknown");
		float FPS = 0.f;
		if (World)
		{
			MapName = World->GetMapName();
			MapName.RemoveFromStart(TEXT("UEDPIE_0_"));
		}
		FPS = 1.0f / FApp::GetDeltaTime();

		return FString::Printf(TEXT("{\"map\":\"%s\",\"fps\":%.1f,\"paused\":%s,\"api_control\":%s}"),
			*MapName, FPS,
			bSimPaused ? TEXT("true") : TEXT("false"),
			bApiControlEnabled ? TEXT("true") : TEXT("false"));
	}

	// === Version ===

	else if (Method == TEXT("getServerVersion")) { return TEXT("2"); }
	else if (Method == TEXT("getMinRequiredClientVersion")) { return TEXT("1"); }

	return TEXT("{\"error\":\"unknown method\"}");
}

bool FFSDSRpcServer::ProcessBinaryRequest(const FString& Request, FSocket* ClientSocket)
{
	FString Method, Args;
	Request.Split(TEXT(" "), &Method, &Args);
	if (Method.IsEmpty()) Method = Request;

	if (Method == TEXT("simGetImageBinary"))
	{
		// Camera sensors stripped in perf/strip-cameras. Send a zero-byte
		// IMG:0 header so any in-flight client (e.g. an old packaged
		// bridge) doesn't deadlock waiting for a payload — it'll see
		// length 0 and skip the read. The bridge no longer makes this
		// call from main.
		FString Header = TEXT("IMG:0\n");
		FTCHARToUTF8 HeaderConv(*Header);
		int32 Sent = 0;
		ClientSocket->Send((const uint8*)HeaderConv.Get(), HeaderConv.Length(), Sent);
		return true;
	}
	// `getLidarDataBinary` (PTS:N\n + raw float array) was removed in
	// #322. It was the request/response counterpart to the also-deleted
	// `streamLidar` push, used by no documented client and obsoleted
	// by FSDSUdpBroadcaster::BroadcastLidarFrame which is the single
	// LiDAR transport going forward.

	return false; // Not a binary request
}

void FFSDSRpcServer::BindMethods()
{
	// Methods are handled in ProcessRequest/ProcessBinaryRequest
}

void FFSDSRpcServer::StreamSensors(FSocket* ClientSocket)
{
	UE_LOG(LogTemp, Log, TEXT("FSDS RPC: Sensor streaming started"));

	// Send "OK\n" to confirm streaming mode
	FString Ack = TEXT("OK\n");
	FTCHARToUTF8 AckConv(*Ack);
	int32 Sent = 0;
	ClientSocket->Send((const uint8*)AckConv.Get(), AckConv.Length(), Sent);

	while (bRunning)
	{
		if (!IsValid(VehiclePawn)) { FPlatformProcess::Sleep(0.1f); continue; }

		// Pack sensor frame (reuse the struct from UdpBroadcaster)
		FFSDSSensorFrame Frame;
		Frame.Magic = 0x49465353;
		Frame.FrameID = StreamFrameCounter++;
		// Authoritative capture clock = UE game/sim time (ns), NOT wall clock
		// — this is the primary (TCP) sensor path the bridge consumes, and it
		// must match PackSensorFrame so header.stamp / /clock run on sim time.
		// Reading TimeSeconds off the game thread is a benign plain-float read,
		// same as the sensor GetOutput() calls already made from here.
		{
			UWorld* W = VehiclePawn->GetWorld();
			const double SimSeconds = W ? W->GetTimeSeconds() : 0.0;
			Frame.Timestamp = (uint64)(SimSeconds * 1e9);
		}
		// Wall clock alongside it (latency/health only, never integrated).
		Frame.ExternalTimestamp =
			(uint64)(FPlatformTime::Cycles64() * FPlatformTime::GetSecondsPerCycle64() * 1e9);

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
			Frame.AccelX = Imu.LinearAcceleration.X / 100.f;
			Frame.AccelY = Imu.LinearAcceleration.Y / 100.f;
			Frame.AccelZ = Imu.LinearAcceleration.Z / 100.f;
			Frame.GyroX = Imu.AngularVelocity.X;
			Frame.GyroY = Imu.AngularVelocity.Y;
			Frame.GyroZ = Imu.AngularVelocity.Z;
			FQuat EnuQuat = FSDSCoord::UEQuatToENU(Imu.Orientation);
			Frame.OrientX = EnuQuat.X;
			Frame.OrientY = EnuQuat.Y;
			Frame.OrientZ = EnuQuat.Z;
			Frame.OrientW = EnuQuat.W;
		}

		// GSS — body frame, X=forward (longitudinal), Y=lateral
		if (VehiclePawn->GssSensor)
		{
			auto Gss = VehiclePawn->GssSensor->GetOutput();
			Frame.GssVelX = Gss.LinearVelocity.X;
			Frame.GssVelY = Gss.LinearVelocity.Y;
			Frame.GssVelZ = Gss.LinearVelocity.Z;
		}

		// Pose (ENU meters)
		FVector Pos = VehiclePawn->GetActorLocation();
		FQuat Quat = VehiclePawn->GetActorQuat();
		Frame.PosX = Pos.Y / 100.f;
		Frame.PosY = Pos.X / 100.f;
		Frame.PosZ = Pos.Z / 100.f;
		FQuat EnuQ = FSDSCoord::UEQuatToENU(Quat);
		Frame.PoseOrientX = EnuQ.X;
		Frame.PoseOrientY = EnuQ.Y;
		Frame.PoseOrientZ = EnuQ.Z;
		Frame.PoseOrientW = EnuQ.W;

		// Ground-truth body-frame velocity (#315) — same as the UDP path
		// in FSDSUdpBroadcaster.cpp. Both senders pack the same struct
		// layout; this branch is the TCP `streamSensors` route, which
		// remains the production sensor path. (LiDAR-over-TCP was
		// retired in #322; sensors still ride the TCP push because the
		// ~40 KB/s rate isn't bandwidth-bound.)
		const FVector WorldVel = VehiclePawn->GetVehicleVelocityUe() * 0.01f;
		const FVector BodyVel  = VehiclePawn->GetActorQuat().Inverse().RotateVector(WorldVel);
		Frame.GtVelBodyX =  BodyVel.X;
		Frame.GtVelBodyY = -BodyVel.Y;
		Frame.GtVelBodyZ =  BodyVel.Z;

		// Ground-truth body-frame angular velocity — same pattern as the
		// UDP path. Mirrors FSDSImuSensor::Tick exactly so GT and noisy
		// IMU live in the same body-frame convention, downstream can diff
		// them directly to read off the bias/noise the filter has to bound.
		if (UPrimitiveComponent* RootPrim =
				Cast<UPrimitiveComponent>(VehiclePawn->GetRootComponent());
			RootPrim && VehiclePawn->IsVehicleMotionLive())
		{
			const FVector WorldAngVel = VehiclePawn->GetVehicleAngularVelocityUe();
			const FVector BodyAngVel  = VehiclePawn->GetActorQuat().Inverse().RotateVector(WorldAngVel);
			// UE5 (left-handed, Y-right) → REP-103 (right-handed, Y-left).
			// Angular velocity is an axial vector; under the Y-reflection
			// between the two frames it transforms (X, Y, Z) → (-X, Y, -Z).
			// Matches the bridge's /imu gyro convention so GT and IMU yaw_rate
			// are directly comparable. See sibling fix in FSDSUdpBroadcaster.cpp
			// and issue #466 (historical PR #454 landed on main, not dev).
			Frame.GtAngVelBodyX = -BodyAngVel.X;
			Frame.GtAngVelBodyY =  BodyAngVel.Y;
			Frame.GtAngVelBodyZ = -BodyAngVel.Z;
		}
		else
		{
			Frame.GtAngVelBodyX = 0.f;
			Frame.GtAngVelBodyY = 0.f;
			Frame.GtAngVelBodyZ = 0.f;
		}

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

		// Controls. Frame.Brake is the wire-format name kept for back-compat
		// with downstream sensor-stream consumers; semantically it carries
		// the regen demand (the only retarding channel folded into the
		// motor command — see FCarControls in FSDSVehiclePawn.h).
		auto Controls = VehiclePawn->GetCarControls();
		Frame.Throttle = Controls.Throttle;
		Frame.Steering = Controls.Steering;
		Frame.Brake = Controls.Regen;

		// Send frame — SendAll retries on transient zero-progress (kernel
		// buffer momentarily full) instead of treating it as a disconnect.
		if (!SendAll(ClientSocket, (const uint8*)&Frame, sizeof(Frame)))
		{
			UE_LOG(LogTemp, Log, TEXT("FSDS RPC: Sensor stream client disconnected"));
			return;
		}

		// Pace at ~400Hz — matches BMI088 IMU rate
		FPlatformProcess::Sleep(0.0025f);
	}
}

// =============================================================================
// StreamLidar (PR-#482)
// =============================================================================
//
// TCP push of full LiDAR scans, one frame per scan. Mirrors StreamSensors'
// architectural pattern: hold the per-connection socket open, ACK "OK\n",
// loop sending wire frames until the bridge disconnects.
//
// Why we re-introduced this after #322 deleted the original TCP path:
//
//   The chunked-UDP transport #322 replaced it with hits two unrelated
//   wedges on Docker Desktop:
//     1. Userspace UDP proxy (com.docker.backend) loses its socket
//        binding for the LiDAR port under sustained load — the proxy
//        keeps a stale port reservation but datagrams hit
//        "port unreachable" silently. Originally documented on macOS
//        in #286; same wedge is observable on Windows Docker Desktop
//        too, regardless of whether "Use host networking" is on.
//     2. WSL2 kernel UDP recv buffer caps small (`net.core.rmem_max`
//        is locked, can't be raised from inside the container without
//        privileged + sysctl).  Drops ~28 % of LiDAR chunks under
//        burst load on Windows.
//
//   The `network_mode: host` workaround that fixed #286 on Mac with
//   Docker Desktop ≥4.34 + "Use host networking" enabled doesn't apply
//   on Windows (WSL2 backend) and is a per-machine setup step. The
//   team's release-readiness review rejected per-OS user configuration
//   as a viable solution.
//
//   TCP avoids both wedges: it's a byte stream (no fragmentation
//   concerns; the kernel handles segmentation transparently), and
//   Docker Desktop's TCP proxy / WSL2 grpc-fuse path are reliable.
//   The original throughput cap that motivated #322 (~7 MB/s macOS
//   Docker Desktop TCP loopback) was fixed upstream over the 6+
//   Docker Desktop minor releases between #322 and PR-#482.
//
// Scan production is async (GPU readback ring); we sit in a tight
// short-sleep loop polling GetTimestamp() and emit a frame whenever
// it advances. Loop sleep is 1 ms — well below the 100 ms scan
// interval, gives the kernel plenty of game-thread slack.
void FFSDSRpcServer::StreamLidar(FSocket* ClientSocket)
{
	UE_LOG(LogTemp, Log, TEXT("FSDS RPC: LiDAR streaming started"));

	// Send "OK\n" to confirm streaming mode (same handshake the
	// sensor stream uses; bridge's openStreamSocket reads exactly 3
	// bytes here and then reads payload bytes, so we MUST send these
	// 3 bytes and no more before the first frame).
	{
		FString Ack = TEXT("OK\n");
		FTCHARToUTF8 AckConv(*Ack);
		int32 Sent = 0;
		ClientSocket->Send((const uint8*)AckConv.Get(), AckConv.Length(), Sent);
	}

	// Track which scan we last sent so we don't re-send the same
	// PointCloudBuffer between GPU readbacks. LastTimestamp is the
	// game-tick-stamp the LiDAR sensor records on each successful
	// readback; it advances strictly monotonically.
	uint64 LastSentTimestamp = 0;

	// Reusable wire buffer. Sized for the worst-case scan plus a 64 KB
	// safety margin so single-scan resizes don't churn the allocator
	// every iteration. The actual send length is computed per-scan
	// from header + total_points * 4 floats.
	TArray<uint8> WireBuf;
	WireBuf.Reserve(2 * 1024 * 1024);

	while (bRunning)
	{
		if (!IsValid(VehiclePawn) || !VehiclePawn->LidarSensor)
		{
			// Same idle-wait pattern as StreamSensors when the pawn
			// hasn't spawned yet (sim still loading the level, for
			// example). Aggressive enough to pick up the pawn within
			// one tick of it becoming valid.
			FPlatformProcess::Sleep(0.1f);
			continue;
		}

		const uint64 NowSensorTs = VehiclePawn->LidarSensor->GetTimestamp();
		if (NowSensorTs == 0 || NowSensorTs == LastSentTimestamp)
		{
			// No new scan ready yet. 1 ms poll — finer than the
			// 100 ms scan interval but coarse enough that we're not
			// pegging a core on game-thread polling. timeBeginPeriod(1)
			// from FSDSPlugin::StartupModule ensures this Sleep
			// actually achieves ~1 ms on Windows; on Mac/Linux the
			// default scheduler tick is already finer than 1 ms.
			FPlatformProcess::Sleep(0.001f);
			continue;
		}

		// Snapshot the cloud + timestamp atomically (GetPointCloud
		// takes PointCloudLock; the matching timestamp we just read
		// can race in principle but the LiDAR sensor only updates
		// LastTimestamp AFTER PointCloudBuffer is fully written, so
		// the ordering is safe — if we observed a newer timestamp
		// then the cloud we read here corresponds to that scan).
		TArray<float> Points = VehiclePawn->LidarSensor->GetPointCloud();
		const int32 TotalPoints = Points.Num() / 4;
		if (TotalPoints <= 0)
		{
			// Edge case: timestamp advanced but the buffer is empty
			// (e.g. a scan with zero valid hits — possible if the
			// vehicle drove off the world or all rays hit beyond
			// MaxRange). Don't send a frame with no payload; just
			// roll the timestamp forward so we don't re-poll this
			// same empty scan forever.
			LastSentTimestamp = NowSensorTs;
			continue;
		}

		// Header: capture-to-send lag in ns (#238) — same semantics
		// as the UDP chunked header so all downstream code (bridge
		// onLidarFrame, ROS stamp recovery) is transport-agnostic.
		FFSDSLidarStreamHeader Header;
		Header.Magic = 0x4C494452;
		Header.FrameID = LidarStreamFrameCounter++;
		Header.Channels = VehiclePawn->LidarSensor->NumberOfChannels;
		Header.TotalPoints = TotalPoints;
		// Absolute sim capture time (Option 2) — bridge prefers this over
		// LagNs. NowSensorTs is the scan's wall-clock stamp we polled on; the
		// sim-time twin is read alongside it (same scan, set together).
		Header.SimCaptureNs = (int64)VehiclePawn->LidarSensor->GetTimestampSimNs();
		{
			const uint64 NowCycles = FPlatformTime::Cycles64();
			Header.LagNs = 0;
			if (NowSensorTs > 0 && NowCycles >= NowSensorTs)
			{
				const double LagSeconds =
					(double)(NowCycles - NowSensorTs) *
					FPlatformTime::GetSecondsPerCycle64();
				Header.LagNs = (int64)(LagSeconds * 1e9);
			}
		}

		// Pack header + payload into the wire buffer in a single
		// allocation. We could SendAll(header) then SendAll(payload),
		// but a single contiguous send avoids two syscalls per scan
		// AND avoids the bridge needing two recvs (the bridge's
		// readExact already handles partial-recv internally).
		const int32 PayloadBytes = TotalPoints * 4 * (int32)sizeof(float);
		const int32 WireBytes = (int32)sizeof(FFSDSLidarStreamHeader) + PayloadBytes;
		WireBuf.SetNumUninitialized(WireBytes, EAllowShrinking::No);
		FMemory::Memcpy(WireBuf.GetData(), &Header, sizeof(FFSDSLidarStreamHeader));
		FMemory::Memcpy(
			WireBuf.GetData() + sizeof(FFSDSLidarStreamHeader),
			Points.GetData(),
			PayloadBytes);

		if (!SendAll(ClientSocket, WireBuf.GetData(), WireBytes))
		{
			UE_LOG(LogTemp, Log, TEXT("FSDS RPC: LiDAR stream client disconnected"));
			return;
		}

		LastSentTimestamp = NowSensorTs;
	}
}
