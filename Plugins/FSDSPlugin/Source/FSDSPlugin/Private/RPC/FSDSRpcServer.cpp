#include "RPC/FSDSRpcServer.h"
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
				// Kernel send buffer momentarily full. Sleep briefly and
				// try again; bound the total wait so a genuinely dead
				// peer still returns false within ~MaxIdleMs.
				if (IdleMs >= MaxIdleMs) return false;
				FPlatformProcess::Sleep(0.005f);
				IdleMs += 5;
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
				UE_LOG(LogTemp, Log, TEXT("FSDS RPC: Client connected from %s"), *RemoteAddr->ToString(true));

				// Bump the kernel send buffer well above the default (~64 KB
				// on Linux/Win). LiDAR frames can hit ~150 KB at higher
				// resolutions, and the bridge's downstream consumers (numba
				// cone detection + multiple foxglove subscribers) drain
				// unevenly. With a small buffer, the kernel is full after a
				// single frame and the next Send returns ChunkSent=0 — the
				// failure mode SendAll has to time out on. 1 MB gives ~20
				// LiDAR frames of headroom, smoothing over consumer hiccups.
				int32 ActualSize = 0;
				ClientSocket->SetSendBufferSize(1024 * 1024, ActualSize);

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
					StreamLidar(ClientSocket);
					return;
				}

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

	else if (Method == TEXT("simGetImage"))
	{
		// Parse: simGetImage camera_name image_type
		if (!IsValid(VehiclePawn)) return TEXT("{}");
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		FString CamName = (Parts.Num() >= 2) ? Parts[1] : TEXT("cam1");
		int32 ImgType = (Parts.Num() >= 3) ? FCString::Atoi(*Parts[2]) : 0;

		UFSDSCameraSensor* Cam = VehiclePawn->GetCamera(CamName);
		if (!Cam)
		{
			for (auto& Pair : VehiclePawn->Cameras)
			{
				Cam = Pair.Value;
				break;
			}
		}
		if (!Cam) return TEXT("{\"error\":\"no camera\"}");

		// Image capture MUST run on game thread
		int32 PngSize = CallOnGameThread<int32>(
			[Cam, ImgType]() -> int32 {
				TArray<uint8> PngData = Cam->CaptureImagePNG(static_cast<EFSDSImageType>(ImgType));
				return PngData.Num();
			},
			3.0, 0, TEXT("getImageSize"));

		return FString::Printf(TEXT("{\"size\":%d,\"camera\":\"%s\",\"type\":%d}"),
			PngSize, *CamName, ImgType);
	}
	else if (Method == TEXT("listCameras"))
	{
		if (!IsValid(VehiclePawn)) return TEXT("[]");
		FString Result = TEXT("[");
		bool bFirst = true;
		for (auto& Pair : VehiclePawn->Cameras)
		{
			if (!bFirst) Result += TEXT(",");
			Result += FString::Printf(TEXT("\"%s\""), *Pair.Key);
			bFirst = false;
		}
		Result += TEXT("]");
		return Result;
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
		//   "<camera_name>"    → any camera from settings.json
		//
		// Body frame here is REP-103 (X=forward, Y=left, Z=up); the
		// plugin stores sensor positions in UE units (X=forward,
		// Y=right, Z=up, centimetres), so we flip Y and convert cm→m.
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
		else if (auto* Cam = VehiclePawn->GetCamera(Name))
		{
			UeOffsetCm = Cam->GetRelativeLocation();
			bFound = true;
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
		// Clients should use simSetVehiclePose instead for a soft reset.
		UE_LOG(LogTemp, Warning, TEXT("FSDS RPC: 'reset' is no longer supported; use simSetVehiclePose"));
		return TEXT("{\"error\":\"reset removed; use simSetVehiclePose\"}");
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

	else if (Method == TEXT("simGetCameraInfo"))
	{
		if (!IsValid(VehiclePawn)) return TEXT("{}");
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		FString CamName = (Parts.Num() >= 2) ? Parts[1] : TEXT("cam1");

		UFSDSCameraSensor* Cam = VehiclePawn->GetCamera(CamName);
		if (!Cam) return TEXT("{\"error\":\"camera not found\"}");

		FVector Pos = FSDSCoord::UEToENU(Cam->GetComponentLocation());
		FQuat Ori = FSDSCoord::UEQuatToENU(Cam->GetComponentQuat());

		return FString::Printf(TEXT("{\"camera\":\"%s\",\"fov\":%.1f,\"width\":%d,\"height\":%d,\"px\":%.4f,\"py\":%.4f,\"pz\":%.4f,\"qw\":%.6f,\"qx\":%.6f,\"qy\":%.6f,\"qz\":%.6f}"),
			*CamName, Cam->FOVAngle, Cam->ImageWidth, Cam->ImageHeight,
			Pos.X, Pos.Y, Pos.Z, Ori.W, Ori.X, Ori.Y, Ori.Z);
	}
	else if (Method == TEXT("simSetCameraFov"))
	{
		// Parse: simSetCameraFov camera_name fov_degrees
		if (!IsValid(VehiclePawn)) return TEXT("false");
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		if (Parts.Num() < 3) return TEXT("{\"error\":\"usage: simSetCameraFov cam1 90\"}");

		FString CamName = Parts[1];
		float NewFOV = FCString::Atof(*Parts[2]);

		UFSDSCameraSensor* Cam = VehiclePawn->GetCamera(CamName);
		if (!Cam) return TEXT("{\"error\":\"camera not found\"}");

		Cam->FOVAngle = NewFOV;
		return TEXT("true");
	}
	else if (Method == TEXT("simSetCameraOrientation"))
	{
		// Parse: simSetCameraOrientation camera_name pitch yaw roll
		if (!IsValid(VehiclePawn)) return TEXT("false");
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		if (Parts.Num() < 5) return TEXT("{\"error\":\"usage: simSetCameraOrientation cam1 pitch yaw roll\"}");

		FString CamName = Parts[1];
		float Pitch = FCString::Atof(*Parts[2]);
		float Yaw = FCString::Atof(*Parts[3]);
		float Roll = FCString::Atof(*Parts[4]);

		UFSDSCameraSensor* Cam = VehiclePawn->GetCamera(CamName);
		if (!Cam) return TEXT("{\"error\":\"camera not found\"}");

		Cam->SetRelativeRotation(FRotator(Pitch, Yaw, Roll));
		return TEXT("true");
	}

	// === simGetImages (batch) ===

	else if (Method == TEXT("simGetImages"))
	{
		if (!IsValid(VehiclePawn)) return TEXT("[]");
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		FString Result = TEXT("[");
		bool bFirst = true;
		for (int32 i = 1; i < Parts.Num(); i++)
		{
			FString CamName, TypeStr;
			Parts[i].Split(TEXT(":"), &CamName, &TypeStr);
			int32 ImgType = FCString::Atoi(*TypeStr);
			UFSDSCameraSensor* Cam = VehiclePawn->GetCamera(CamName);
			if (!Cam) continue;
			int32 PngSize = CallOnGameThread<int32>(
				[Cam, ImgType]() -> int32 {
					TArray<uint8> Png = Cam->CaptureImagePNG(static_cast<EFSDSImageType>(ImgType));
					return Png.Num();
				},
				3.0, 0, TEXT("simGetImages[i]"));
			if (!bFirst) Result += TEXT(",");
			Result += FString::Printf(TEXT("{\"camera\":\"%s\",\"type\":%d,\"size\":%d}"), *CamName, ImgType, PngSize);
			bFirst = false;
		}
		Result += TEXT("]");
		return Result;
	}

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
		// Parse: simGetImageBinary camera_name image_type
		if (!IsValid(VehiclePawn)) return false;

		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		FString CamName = (Parts.Num() >= 2) ? Parts[1] : TEXT("cam1");
		int32 ImgType = (Parts.Num() >= 3) ? FCString::Atoi(*Parts[2]) : 0;

		UFSDSCameraSensor* Cam = VehiclePawn->GetCamera(CamName);
		if (!Cam)
		{
			for (auto& Pair : VehiclePawn->Cameras)
			{
				Cam = Pair.Value;
				break;
			}
		}
		if (!Cam) return false;

		// Capture on game thread
		TArray<uint8> PngData = CallOnGameThread<TArray<uint8>>(
			[Cam, ImgType]() -> TArray<uint8> {
				return Cam->CaptureImagePNG(static_cast<EFSDSImageType>(ImgType));
			},
			5.0, TArray<uint8>(), TEXT("simGetImageBinary"));

		// Send header: "IMG:size\n" followed by raw PNG bytes
		FString Header = FString::Printf(TEXT("IMG:%d\n"), PngData.Num());
		FTCHARToUTF8 HeaderConv(*Header);
		int32 Sent = 0;
		ClientSocket->Send((const uint8*)HeaderConv.Get(), HeaderConv.Length(), Sent);

		// Send raw PNG bytes
		if (PngData.Num() > 0)
		{
			int32 TotalSent = 0;
			while (TotalSent < PngData.Num())
			{
				int32 ChunkSent = 0;
				ClientSocket->Send(PngData.GetData() + TotalSent, PngData.Num() - TotalSent, ChunkSent);
				if (ChunkSent <= 0) break;
				TotalSent += ChunkSent;
			}
		}
		return true;
	}
	else if (Method == TEXT("getLidarDataBinary"))
	{
		if (!VehiclePawn || !VehiclePawn->LidarSensor) return false;

		TArray<float> Points = VehiclePawn->LidarSensor->GetPointCloud();
		int32 NumPoints = Points.Num() / 3;

		// Send header: "PTS:num_points\n" followed by raw float data
		FString Header = FString::Printf(TEXT("PTS:%d\n"), NumPoints);
		FTCHARToUTF8 HeaderConv(*Header);
		int32 Sent = 0;
		ClientSocket->Send((const uint8*)HeaderConv.Get(), HeaderConv.Length(), Sent);

		// Send raw float array (x,y,z per point)
		if (Points.Num() > 0)
		{
			int32 ByteSize = Points.Num() * sizeof(float);
			int32 TotalSent = 0;
			const uint8* Data = (const uint8*)Points.GetData();
			while (TotalSent < ByteSize)
			{
				int32 ChunkSent = 0;
				ClientSocket->Send(Data + TotalSent, ByteSize - TotalSent, ChunkSent);
				if (ChunkSent <= 0) break;
				TotalSent += ChunkSent;
			}
		}
		return true;
	}

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

void FFSDSRpcServer::StreamLidar(FSocket* ClientSocket)
{
	UE_LOG(LogTemp, Log, TEXT("FSDS RPC: LiDAR streaming started"));

	FString Ack = TEXT("OK\n");
	FTCHARToUTF8 AckConv(*Ack);
	int32 Sent = 0;
	ClientSocket->Send((const uint8*)AckConv.Get(), AckConv.Length(), Sent);

	while (bRunning)
	{
		if (!VehiclePawn || !VehiclePawn->LidarSensor)
		{
			FPlatformProcess::Sleep(0.1f);
			continue;
		}

		TArray<float> Points = VehiclePawn->LidarSensor->GetPointCloud();
		int32 TotalPoints = Points.Num() / 3;

		if (TotalPoints > 0)
		{
			// Send header — SendAll handles partial-write retry. The
			// previous code only checked `!bOk` for the header, leaving a
			// silent way for a partial header send (BytesSent < sizeof
			// header) to corrupt the bridge's stream framing.
			FFSDSLidarChunkHeader Header;
			Header.Magic = 0x4C494452;
			Header.ChunkIndex = 0;
			Header.TotalChunks = 1; // Single chunk over TCP (no size limit)
			Header.FrameID = StreamFrameCounter;
			Header.PointsInChunk = TotalPoints;
			Header.TotalPoints = TotalPoints;
			Header.Channels = VehiclePawn->LidarSensor->NumberOfChannels;

			if (!SendAll(ClientSocket, (const uint8*)&Header, sizeof(Header)))
			{
				UE_LOG(LogTemp, Log, TEXT("FSDS RPC: LiDAR stream disconnected (header)"));
				return;
			}

			// Send point data — bigger payload (points * 12 bytes), most
			// likely place to hit a momentarily-full send buffer with the
			// 3 cm range jitter introduced in #106 producing larger packet
			// variance per scan.
			const int32 DataSize = TotalPoints * 3 * sizeof(float);
			if (!SendAll(ClientSocket, (const uint8*)Points.GetData(), DataSize))
			{
				UE_LOG(LogTemp, Log, TEXT("FSDS RPC: LiDAR stream disconnected (body, %d bytes)"), DataSize);
				return;
			}
		}

		// 10Hz LiDAR
		FPlatformProcess::Sleep(0.1f);
	}
}
