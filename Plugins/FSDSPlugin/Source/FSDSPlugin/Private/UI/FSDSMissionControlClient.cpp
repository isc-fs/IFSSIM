#include "UI/FSDSMissionControlClient.h"

#include "HttpModule.h"
#include "Interfaces/IHttpRequest.h"
#include "Interfaces/IHttpResponse.h"
#include "GenericPlatform/GenericPlatformHttp.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

TSharedPtr<FJsonObject> FFSDSMcReply::Object() const
{
	if (Body.IsValid() && Body->Type == EJson::Object)
	{
		return Body->AsObject();
	}
	return nullptr;
}

FFSDSMissionControlClient::FFSDSMissionControlClient(const FString& InBaseUrl, const FString& InApiKey)
	: BaseUrl(InBaseUrl)
	, ApiKey(InApiKey)
{
	BaseUrl.RemoveFromEnd(TEXT("/"));
}

FString FFSDSMissionControlClient::EncodeSegment(const FString& Segment)
{
	return FGenericPlatformHttp::UrlEncode(Segment);
}

void FFSDSMissionControlClient::Get(const FString& Path, FCallback Done, float TimeoutS) const
{
	Send(TEXT("GET"), Path, FString(), MoveTemp(Done), TimeoutS);
}

void FFSDSMissionControlClient::Post(const FString& Path, const TSharedPtr<FJsonObject>& Body, FCallback Done, float TimeoutS) const
{
	FString Content = TEXT("{}");
	if (Body.IsValid())
	{
		Content.Reset();
		const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Content);
		FJsonSerializer::Serialize(Body.ToSharedRef(), Writer);
	}
	Send(TEXT("POST"), Path, Content, MoveTemp(Done), TimeoutS);
}

namespace
{
	/** The backend answers errors as {"error": ...} (its own) or {"detail": ...} (FastAPI). */
	FString ErrorFromBody(const TSharedPtr<FJsonValue>& Body)
	{
		if (!Body.IsValid() || Body->Type != EJson::Object) return FString();
		const TSharedPtr<FJsonObject> Obj = Body->AsObject();
		FString Msg;
		if (Obj->TryGetStringField(TEXT("error"), Msg) && !Msg.IsEmpty()) return Msg;
		if (Obj->TryGetStringField(TEXT("detail"), Msg) && !Msg.IsEmpty()) return Msg;
		return FString();
	}
}

void FFSDSMissionControlClient::Send(const FString& Verb, const FString& Path, const FString& Content,
	FCallback Done, float TimeoutS) const
{
	const TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Req = FHttpModule::Get().CreateRequest();
	Req->SetURL(BaseUrl + Path);
	Req->SetVerb(Verb);
	Req->SetTimeout(TimeoutS);
	Req->SetHeader(TEXT("Accept"), TEXT("application/json"));
	if (!ApiKey.IsEmpty())
	{
		Req->SetHeader(TEXT("X-API-Key"), ApiKey);
	}
	if (Verb == TEXT("POST"))
	{
		Req->SetHeader(TEXT("Content-Type"), TEXT("application/json"));
		Req->SetContentAsString(Content);
	}

	Req->OnProcessRequestComplete().BindLambda(
		[Done = MoveTemp(Done)](FHttpRequestPtr, FHttpResponsePtr Resp, bool bConnected)
		{
			FFSDSMcReply Reply;
			if (!bConnected || !Resp.IsValid())
			{
				Reply.Error = TEXT("backend not reachable");
				Done(Reply);
				return;
			}
			Reply.Code = Resp->GetResponseCode();

			TSharedPtr<FJsonValue> Parsed;
			const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Resp->GetContentAsString());
			if (FJsonSerializer::Deserialize(Reader, Parsed))
			{
				Reply.Body = Parsed;
			}

			if (Reply.Code == 401)
			{
				Reply.Error = TEXT("the backend wants an API key: set IFSSIM_MC_API_KEY, "
					"or MissionControl.ApiKey in settings.json");
			}
			else if (Reply.Code < 200 || Reply.Code >= 300)
			{
				Reply.Error = ErrorFromBody(Reply.Body);
				if (Reply.Error.IsEmpty()) Reply.Error = FString::Printf(TEXT("HTTP %d"), Reply.Code);
			}
			else if (const TSharedPtr<FJsonObject> Obj = Reply.Object())
			{
				bool bOk = true;
				if (Obj->TryGetBoolField(TEXT("ok"), bOk) && !bOk)
				{
					Reply.Error = ErrorFromBody(Reply.Body);
					if (Reply.Error.IsEmpty()) Reply.Error = TEXT("the backend said no");
				}
			}
			Done(Reply);
		});
	Req->ProcessRequest();
}
