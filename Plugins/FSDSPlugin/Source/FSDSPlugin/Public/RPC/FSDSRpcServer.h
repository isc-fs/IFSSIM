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

	bool IsRunning() const { return bRunning; }

private:
	void BindMethods();
	void ServerThreadFunc();
	void HandleClient(FSocket* ClientSocket);
	FString ProcessRequest(const FString& Request);
	bool ProcessBinaryRequest(const FString& Request, FSocket* ClientSocket);

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
};
