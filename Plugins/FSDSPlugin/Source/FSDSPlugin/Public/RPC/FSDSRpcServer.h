#pragma once

#include "CoreMinimal.h"
#include <memory>
#include <thread>
#include <atomic>

class FSocket;
class AFSDSVehiclePawn;
class AFSDSReferee;

/**
 * FSDS RPC Server — TCP server on port 41451.
 * Accepts text-based commands and returns JSON responses.
 * Protocol: "method_name [args]\n" -> "json_response\n"
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

	bool IsRunning() const { return bRunning; }

private:
	void BindMethods();
	void ServerThreadFunc();
	void HandleClient(FSocket* ClientSocket);
	FString ProcessRequest(const FString& Request);

	std::unique_ptr<std::thread> ServerThread;
	std::atomic<bool> bRunning{false};
	std::atomic<bool> bApiControlEnabled{false};
	uint16 ServerPort = 41451;

	AFSDSVehiclePawn* VehiclePawn = nullptr;
	AFSDSReferee* Referee = nullptr;
	FString SettingsString;
};
