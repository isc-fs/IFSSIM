#pragma once

#include "CoreMinimal.h"
#include <memory>
#include <thread>
#include <atomic>

class FSocket;
class AFSDSVehiclePawn;
class AFSDSReferee;
class UWorld;

/**
 * FSDS RPC Server — TCP server on port 41451.
 * Protocol: "method_name [args]\n" -> "json_response\n"
 *
 * Simulation control: simPause, simResume, simStep, reset
 * Object APIs: listSceneObjects, getObjectPose, setObjectPose
 */
class FSDSPLUGIN_API FFSDSRpcServer
{
public:
	FFSDSRpcServer();
	~FFSDSRpcServer();

	void Start(uint16 Port = 41451);
	void Stop();

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

	/** Streaming modes — hold connection open and push data continuously */
	void StreamSensors(FSocket* ClientSocket);
	void StreamLidar(FSocket* ClientSocket);
	uint32 StreamFrameCounter = 0;

	// Cached binary data for thread-safe transfer
	TArray<uint8> CachedImageData;
	TArray<float> CachedLidarData;
	FCriticalSection BinaryDataLock;

	std::unique_ptr<std::thread> ServerThread;
	std::atomic<bool> bRunning{false};
	std::atomic<bool> bApiControlEnabled{false};
	std::atomic<bool> bSimPaused{false};
	uint16 ServerPort = 41451;

	AFSDSVehiclePawn* VehiclePawn = nullptr;
	AFSDSReferee* Referee = nullptr;
	UWorld* World = nullptr;
	FString SettingsString;
	class FFSDSUdpBroadcaster* UdpBroadcaster = nullptr;

	// Cached car controls for immediate readback (set from TCP thread before game thread applies)
	struct FCachedControls {
		float Throttle = 0.f;
		float Steering = 0.f;
		float Brake = 0.f;
		bool bHandbrake = false;
		bool bIsManualGear = false;
		int32 ManualGear = 0;
		bool bGearImmediate = true;
	};
	FCachedControls CachedControls;
};
