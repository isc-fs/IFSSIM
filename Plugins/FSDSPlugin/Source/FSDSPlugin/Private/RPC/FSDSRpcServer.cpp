#include "RPC/FSDSRpcServer.h"
#include "FSDSVehiclePawn.h"
#include "FSDSReferee.h"
#include "FSDSCoordinates.h"
#include "Async/Async.h"
#include "Engine/World.h"
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
				HandleClient(ClientSocket);
				ClientSocket->Close();
				SocketSubsystem->DestroySocket(ClientSocket);
			}
		}

		FPlatformProcess::Sleep(0.01f); // 10ms poll interval
	}

	ListenSocket->Close();
	SocketSubsystem->DestroySocket(ListenSocket);
}

void FFSDSRpcServer::HandleClient(FSocket* ClientSocket)
{
	// Simple text-based protocol for now:
	// Client sends: "method_name\n"
	// Server responds: "result_json\n"

	uint8 Buffer[4096];

	while (bRunning && ClientSocket)
	{
		int32 BytesRead = 0;
		ClientSocket->Wait(ESocketWaitConditions::WaitForRead, FTimespan::FromMilliseconds(100));

		if (ClientSocket->Recv(Buffer, sizeof(Buffer) - 1, BytesRead))
		{
			if (BytesRead > 0)
			{
				Buffer[BytesRead] = 0;
				FString Request = UTF8_TO_TCHAR((char*)Buffer);
				Request.TrimEndInline();

				FString Response = ProcessRequest(Request);
				Response += TEXT("\n");

				FTCHARToUTF8 Converter(*Response);
				int32 BytesSent = 0;
				ClientSocket->Send((const uint8*)Converter.Get(), Converter.Length(), BytesSent);
			}
		}
		else
		{
			break; // Client disconnected
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
		return TEXT("true");
	}
	else if (Method == TEXT("isApiControlEnabled"))
	{
		return bApiControlEnabled ? TEXT("true") : TEXT("false");
	}
	else if (Method == TEXT("getCarState"))
	{
		if (!VehiclePawn) return TEXT("{}");
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
		if (!VehiclePawn || !VehiclePawn->LidarSensor) return TEXT("{\"points\":0}");
		auto Points = VehiclePawn->LidarSensor->GetPointCloud();
		return FString::Printf(TEXT("{\"points\":%d}"), Points.Num() / 3);
	}
	else if (Method == TEXT("getRefereeState"))
	{
		if (!Referee) return TEXT("{\"doo_counter\":0,\"cones\":0}");
		auto State = Referee->GetState();
		return FString::Printf(TEXT("{\"doo_counter\":%d,\"cones\":%d,\"laps\":%d}"),
			State.DooCounter, State.Cones.Num(), State.Laps.Num());
	}
	else if (Method.StartsWith(TEXT("setCarControls")))
	{
		// Parse: setCarControls throttle steering brake
		if (VehiclePawn && bApiControlEnabled)
		{
			TArray<FString> Parts;
			Request.ParseIntoArray(Parts, TEXT(" "));
			if (Parts.Num() >= 4)
			{
				AFSDSVehiclePawn::FCarControls Controls;
				Controls.Throttle = FCString::Atof(*Parts[1]);
				Controls.Steering = FCString::Atof(*Parts[2]);
				Controls.Brake = FCString::Atof(*Parts[3]);

				AsyncTask(ENamedThreads::GameThread, [this, Controls]() {
					if (VehiclePawn) VehiclePawn->SetCarControls(Controls);
				});
			}
		}
		return TEXT("true");
	}

	else if (Method == TEXT("simGetImage"))
	{
		// Parse: simGetImage camera_name image_type
		if (!VehiclePawn) return TEXT("{}");
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
		if (!VehiclePawn) return TEXT("[]");
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
		if (!VehiclePawn) return TEXT("{}");
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
		AsyncTask(ENamedThreads::GameThread, [this]() {
			if (World)
			{
				// Restart the current level
				UGameplayStatics::OpenLevel(World, *World->GetMapName(), true);
			}
		});
		return TEXT("true");
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
		// Parse: simSetVehiclePose x y z
		TArray<FString> Parts;
		Request.ParseIntoArray(Parts, TEXT(" "));
		if (Parts.Num() < 4) return TEXT("{\"error\":\"usage: simSetVehiclePose x y z\"}");

		FVector PosENU(FCString::Atof(*Parts[1]), FCString::Atof(*Parts[2]), FCString::Atof(*Parts[3]));
		FVector PosUE = FSDSCoord::ENUToUE(PosENU);

		AsyncTask(ENamedThreads::GameThread, [this, PosUE]() {
			if (VehiclePawn)
				VehiclePawn->SetActorLocation(PosUE, false, nullptr, ETeleportType::TeleportPhysics);
		});
		return TEXT("true");
	}

	return TEXT("{\"error\":\"unknown method\"}");
}

void FFSDSRpcServer::BindMethods()
{
	// Methods are handled in ProcessRequest() — no separate binding needed
}
