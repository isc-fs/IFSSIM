#pragma once

#include "CoreMinimal.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"

/** One reply from the Mission Control backend. */
struct FSDSPLUGIN_API FFSDSMcReply
{
	/** HTTP status. 0 means no reply: the backend is down or timed out. */
	int32 Code = 0;
	/** Parsed JSON body, null when the body was not JSON. */
	TSharedPtr<FJsonValue> Body;
	/** What went wrong, in words; empty when the call succeeded. */
	FString Error;

	/** 2xx and, when the body carries an "ok" field, ok == true. */
	bool Ok() const { return Error.IsEmpty(); }
	bool Reached() const { return Code != 0; }
	TSharedPtr<FJsonObject> Object() const;
};

/**
 * HTTP client for the Mission Control backend (tools/mission_control/backend).
 *
 * The in-game panel drives a session through the same endpoints as the web
 * UI, so the backend stays the one place the session sequence lives
 * (track load, mission start with its EBS hold and EKF calibration wait,
 * stop, RES). Requests are asynchronous; callbacks run on the game thread.
 */
class FSDSPLUGIN_API FFSDSMissionControlClient
{
public:
	using FCallback = TFunction<void(const FFSDSMcReply&)>;

	FFSDSMissionControlClient(const FString& InBaseUrl, const FString& InApiKey);

	void Get(const FString& Path, FCallback Done, float TimeoutS = 3.f) const;
	void Post(const FString& Path, const TSharedPtr<FJsonObject>& Body, FCallback Done, float TimeoutS = 20.f) const;
	void Delete(const FString& Path, FCallback Done, float TimeoutS = 10.f) const;

	const FString& GetBaseUrl() const { return BaseUrl; }

	/** Percent-encode one path segment (a track name). */
	static FString EncodeSegment(const FString& Segment);

private:
	void Send(const FString& Verb, const FString& Path, const FString& Content, FCallback Done, float TimeoutS) const;

	FString BaseUrl;
	FString ApiKey;
};
