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

				// Handle each client in its own thread (supports streaming + concurrent clients)
				std::thread ClientThread([this, ClientSocket, SocketSubsystem]() {
					HandleClient(ClientSocket);
					ClientSocket->Close();
					SocketSubsystem->DestroySocket(ClientSocket);
				});
				ClientThread.detach();
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
	else if (Method == TEXT("isApiControlEnabled"))
	{
		return bApiControlEnabled ? TEXT("true") : TEXT("false");
	}
	else if (Method == TEXT("getCarState"))
	{
		if (!IsValid(VehiclePawn)) return TEXT("{}");
		auto State = VehiclePawn->GetCarState();
		FVector PosENU = FSDSCoord::UEToENU(State.Position);
		FVector VelENU = FSDSCoord::UEVelocityToENU(State.LinearVelocity);
		FQuat OriENU = FSDSCoord::UEQuatToENU(State.Orientation);
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
	else if (Method.StartsWith(TEXT("setCarControls")))
	{
		// Parse: setCarControls throttle steering brake
		if (IsValid(VehiclePawn) && bApiControlEnabled)
		{
			TArray<FString> Parts;
			Request.ParseIntoArray(Parts, TEXT(" "));
			if (Parts.Num() >= 4)
			{
				AFSDSVehiclePawn::FCarControls Controls;
				Controls.Throttle = FCString::Atof(*Parts[1]);
				Controls.Steering = FCString::Atof(*Parts[2]);
				Controls.Brake = FCString::Atof(*Parts[3]);

				// Cache immediately for getCarControls readback
				CachedControls.Throttle = Controls.Throttle;
				CachedControls.Steering = Controls.Steering;
				CachedControls.Brake = Controls.Brake;

				AsyncTask(ENamedThreads::GameThread, [this, Controls]() {
					if (IsValid(VehiclePawn)) VehiclePawn->SetCarControls(Controls);
				});
			}
		}
		return TEXT("true");
	}
	else if (Method == TEXT("getCarControls"))
	{
		// Read from cached controls (set immediately on setCarControls, no game-thread delay)
		return FString::Printf(TEXT("{\"throttle\":%.4f,\"steering\":%.4f,\"brake\":%.4f,\"handbrake\":%s,\"is_manual_gear\":%s,\"manual_gear\":%d,\"gear_immediate\":%s}"),
			CachedControls.Throttle, CachedControls.Steering, CachedControls.Brake,
			CachedControls.bHandbrake ? TEXT("true") : TEXT("false"),
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
		int32 PngSize = 0;
		FEvent* DoneEvent = FPlatformProcess::GetSynchEventFromPool(true);

		AsyncTask(ENamedThreads::GameThread, [Cam, ImgType, &PngSize, DoneEvent]() {
			TArray<uint8> PngData = Cam->CaptureImagePNG(static_cast<EFSDSImageType>(ImgType));
			PngSize = PngData.Num();
			DoneEvent->Trigger();
		});

		DoneEvent->Wait(3000); // 3 second timeout
		FPlatformProcess::ReturnSynchEventToPool(DoneEvent);

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

		FString Result;
		FEvent* DoneEvent = FPlatformProcess::GetSynchEventFromPool(true);

		AsyncTask(ENamedThreads::GameThread, [this, Filter, &Result, DoneEvent]() {
			Result = TEXT("[");
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
			DoneEvent->Trigger();
		});

		DoneEvent->Wait(3000);
		FPlatformProcess::ReturnSynchEventToPool(DoneEvent);
		return Result;
	}
	else if (Method == TEXT("getObjectPose"))
	{
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		if (Parts.Num() < 2) return TEXT("{\"error\":\"missing object_name\"}");
		FString ObjName = Parts[1];

		FString Result;
		FEvent* DoneEvent = FPlatformProcess::GetSynchEventFromPool(true);

		AsyncTask(ENamedThreads::GameThread, [this, ObjName, &Result, DoneEvent]() {
			if (World)
			{
				for (TActorIterator<AActor> It(World); It; ++It)
				{
					if (It->GetName() == ObjName)
					{
						FVector PosENU = FSDSCoord::UEToENU(It->GetActorLocation());
						FQuat OriENU = FSDSCoord::UEQuatToENU(It->GetActorQuat());
						Result = FString::Printf(TEXT("{\"px\":%.4f,\"py\":%.4f,\"pz\":%.4f,\"qw\":%.6f,\"qx\":%.6f,\"qy\":%.6f,\"qz\":%.6f}"),
							PosENU.X, PosENU.Y, PosENU.Z, OriENU.W, OriENU.X, OriENU.Y, OriENU.Z);
						DoneEvent->Trigger();
						return;
					}
				}
			}
			Result = TEXT("{\"error\":\"object not found\"}");
			DoneEvent->Trigger();
		});

		DoneEvent->Wait(3000);
		FPlatformProcess::ReturnSynchEventToPool(DoneEvent);
		return Result;
	}
	else if (Method == TEXT("setObjectPose"))
	{
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		if (Parts.Num() < 5) return TEXT("{\"error\":\"usage: setObjectPose name x y z\"}");

		FString ObjName = Parts[1];
		FVector PosENU(FCString::Atof(*Parts[2]), FCString::Atof(*Parts[3]), FCString::Atof(*Parts[4]));
		FVector PosUE = FSDSCoord::ENUToUE(PosENU);

		FString Result;
		FEvent* DoneEvent = FPlatformProcess::GetSynchEventFromPool(true);

		AsyncTask(ENamedThreads::GameThread, [this, ObjName, PosUE, &Result, DoneEvent]() {
			if (World)
			{
				for (TActorIterator<AActor> It(World); It; ++It)
				{
					if (It->GetName() == ObjName)
					{
						It->SetActorLocation(PosUE, false, nullptr, ETeleportType::TeleportPhysics);
						Result = TEXT("true");
						DoneEvent->Trigger();
						return;
					}
				}
			}
			Result = TEXT("{\"error\":\"object not found\"}");
			DoneEvent->Trigger();
		});

		DoneEvent->Wait(3000);
		FPlatformProcess::ReturnSynchEventToPool(DoneEvent);
		return Result;
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
					Mesh->SetPhysicsLinearVelocity(FVector::ZeroVector);
					Mesh->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
				}
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
			int32 PngSize = 0;
			FEvent* Done = FPlatformProcess::GetSynchEventFromPool(true);
			AsyncTask(ENamedThreads::GameThread, [Cam, ImgType, &PngSize, Done]() {
				TArray<uint8> Png = Cam->CaptureImagePNG(static_cast<EFSDSImageType>(ImgType));
				PngSize = Png.Num();
				Done->Trigger();
			});
			Done->Wait(3000);
			FPlatformProcess::ReturnSynchEventToPool(Done);
			if (!bFirst) Result += TEXT(",");
			Result += FString::Printf(TEXT("{\"camera\":\"%s\",\"type\":%d,\"size\":%d}"), *CamName, ImgType, PngSize);
			bFirst = false;
		}
		Result += TEXT("]");
		return Result;
	}

	// === Weather / TimeOfDay stubs ===

	else if (Method == TEXT("simEnableWeather") || Method == TEXT("simSetWeatherParameter") || Method == TEXT("simSetTimeOfDay"))
	{
		return TEXT("true");
	}

	// === Visualization stubs ===

	else if (Method == TEXT("simPlotPoints") || Method == TEXT("simPlotLineStrip") ||
		Method == TEXT("simPlotLineList") || Method == TEXT("simPlotArrows") ||
		Method == TEXT("simPlotStrings") || Method == TEXT("simPlotTransforms") ||
		Method == TEXT("simPlotTransformsWithNames") || Method == TEXT("simFlushPersistentMarkers"))
	{
		return TEXT("true");
	}

	// === Segmentation stubs ===

	else if (Method == TEXT("simSetSegmentationObjectID")) { return TEXT("true"); }
	else if (Method == TEXT("simGetSegmentationObjectID")) { return TEXT("0"); }
	else if (Method == TEXT("simSwapTextures")) { return TEXT("[]"); }

	// === Track loading ===

	else if (Method == TEXT("loadTrack"))
	{
		// Parse: loadTrack path/to/track.csv
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		if (Parts.Num() < 2) return TEXT("{\"error\":\"usage: loadTrack path.csv\"}");

		FString TrackPath = Parts[1];

		// Find and reload the ConeSpawner on game thread
		FString Result;
		FEvent* DoneEvent = FPlatformProcess::GetSynchEventFromPool(true);

		AsyncTask(ENamedThreads::GameThread, [this, TrackPath, &Result, DoneEvent]() {
			if (!World) { Result = TEXT("{\"error\":\"no world\"}"); DoneEvent->Trigger(); return; }

			// Find existing ConeSpawner
			AFSDSConeSpawner* Spawner = nullptr;
			for (TActorIterator<AFSDSConeSpawner> It(World); It; ++It)
			{
				Spawner = *It;
				break;
			}

			if (!Spawner) { Result = TEXT("{\"error\":\"no ConeSpawner found\"}"); DoneEvent->Trigger(); return; }

			// Destroy all previously spawned cone actors (tracked by spawner)
			for (AActor* ConeActor : Spawner->SpawnedCones)
			{
				if (ConeActor && IsValid(ConeActor))
				{
					ConeActor->Destroy();
				}
			}
			Spawner->SpawnedCones.Empty();

			// Reset referee state for new track
			if (Referee)
			{
				Referee->ResetState();
			}

			// Set new CSV path and re-run spawning (ReloadTrack avoids double-calling Super::BeginPlay)
			Spawner->ReloadTrack(TrackPath);

			int32 NumCones = Spawner->SpawnedCones.Num();
			Result = FString::Printf(TEXT("{\"loaded\":\"%s\",\"cones\":%d}"), *TrackPath, NumCones);
			DoneEvent->Trigger();
		});

		DoneEvent->Wait(5000);
		FPlatformProcess::ReturnSynchEventToPool(DoneEvent);
		return Result;
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
		TArray<uint8> PngData;
		FEvent* DoneEvent = FPlatformProcess::GetSynchEventFromPool(true);

		AsyncTask(ENamedThreads::GameThread, [Cam, ImgType, &PngData, DoneEvent]() {
			PngData = Cam->CaptureImagePNG(static_cast<EFSDSImageType>(ImgType));
			DoneEvent->Trigger();
		});

		DoneEvent->Wait(5000);
		FPlatformProcess::ReturnSynchEventToPool(DoneEvent);

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

		// Controls
		auto Controls = VehiclePawn->GetCarControls();
		Frame.Throttle = Controls.Throttle;
		Frame.Steering = Controls.Steering;
		Frame.Brake = Controls.Brake;

		// Send frame
		int32 BytesSent = 0;
		bool bOk = ClientSocket->Send((const uint8*)&Frame, sizeof(Frame), BytesSent);
		if (!bOk || BytesSent != sizeof(Frame))
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
			// Send header
			FFSDSLidarChunkHeader Header;
			Header.Magic = 0x4C494452;
			Header.ChunkIndex = 0;
			Header.TotalChunks = 1; // Single chunk over TCP (no size limit)
			Header.FrameID = StreamFrameCounter;
			Header.PointsInChunk = TotalPoints;
			Header.TotalPoints = TotalPoints;
			Header.Channels = VehiclePawn->LidarSensor->NumberOfChannels;

			int32 BytesSent = 0;
			bool bOk = ClientSocket->Send((const uint8*)&Header, sizeof(Header), BytesSent);
			if (!bOk) { UE_LOG(LogTemp, Log, TEXT("FSDS RPC: LiDAR stream disconnected")); return; }

			// Send point data
			int32 DataSize = TotalPoints * 3 * sizeof(float);
			int32 TotalSent = 0;
			const uint8* Data = (const uint8*)Points.GetData();
			while (TotalSent < DataSize)
			{
				int32 ChunkSent = 0;
				bOk = ClientSocket->Send(Data + TotalSent, DataSize - TotalSent, ChunkSent);
				if (!bOk || ChunkSent <= 0) { UE_LOG(LogTemp, Log, TEXT("FSDS RPC: LiDAR stream disconnected")); return; }
				TotalSent += ChunkSent;
			}
		}

		// 10Hz LiDAR
		FPlatformProcess::Sleep(0.1f);
	}
}
