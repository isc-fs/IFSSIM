#include "RPC/FSDSRpcServer.h"
#include "FSDSVehiclePawn.h"
#include "FSDSReferee.h"
#include "Async/Async.h"
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
		return FString::Printf(TEXT("{\"speed\":%.4f,\"gear\":%d,\"rpm\":%.1f,\"maxrpm\":%.1f,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}"),
			State.Speed, State.Gear, State.RPM, State.MaxRPM,
			State.Position.X, State.Position.Y, State.Position.Z);
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
		return FString::Printf(TEXT("{\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f,\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f}"),
			Imu.LinearAcceleration.X, Imu.LinearAcceleration.Y, Imu.LinearAcceleration.Z,
			Imu.AngularVelocity.X, Imu.AngularVelocity.Y, Imu.AngularVelocity.Z);
	}
	else if (Method == TEXT("getGroundSpeedSensorData"))
	{
		if (!VehiclePawn || !VehiclePawn->GssSensor) return TEXT("{}");
		auto Gss = VehiclePawn->GssSensor->GetOutput();
		return FString::Printf(TEXT("{\"vx\":%.4f,\"vy\":%.4f,\"vz\":%.4f}"),
			Gss.LinearVelocity.X, Gss.LinearVelocity.Y, Gss.LinearVelocity.Z);
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
		return FString::Printf(TEXT("{\"px\":%.4f,\"py\":%.4f,\"pz\":%.4f,\"vx\":%.4f,\"vy\":%.4f,\"vz\":%.4f,\"ax\":%.4f,\"ay\":%.4f,\"az\":%.4f}"),
			State.Position.X / 100.0, State.Position.Y / 100.0, State.Position.Z / 100.0,
			State.LinearVelocity.X / 100.0, State.LinearVelocity.Y / 100.0, State.LinearVelocity.Z / 100.0,
			State.LinearAcceleration.X / 100.0, State.LinearAcceleration.Y / 100.0, State.LinearAcceleration.Z / 100.0);
	}

	return TEXT("{\"error\":\"unknown method\"}");
}

void FFSDSRpcServer::BindMethods()
{
	// Methods are handled in ProcessRequest() — no separate binding needed
}
