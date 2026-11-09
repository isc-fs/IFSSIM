#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FSDSDaylight.generated.h"

class UDirectionalLightComponent;
class UPostProcessComponent;
class USkyAtmosphereComponent;
class USkyLightComponent;

/**
 * Physically based daylight for the showcase rendering preset
 * (docs/rendering_presets.md): a sky atmosphere, a sky light recaptured in
 * real time from the scene as it is now, and light height fog. The level's
 * sun is aimed where the sun stands at the event; the level's legacy sky
 * (atmospheric fog, sky sphere) and its baked sky light are hidden.
 *
 * The level's own sky light was captured once, from the old flat map, so
 * every face in shade went near black; the real-time one lights shade with
 * the actual sky and, with Lumen, the light bounced off the ground.
 *
 * Exposure is fixed (fsds.Daylight.Exposure): the level's own post-process
 * auto-exposure was tuned for its legacy lighting and blew the scene out by
 * two to three stops, turning 10 % asphalt near white.
 *
 * Destroying it puts the level's lighting back exactly as it was.
 */
UCLASS(NotPlaceable)
class FSDSPLUGIN_API AFSDSDaylight : public AActor
{
	GENERATED_BODY()

public:
	AFSDSDaylight();

	/** Applies fsds.Daylight.Exposure; runs when the variable changes. */
	void ApplyExposure();

	// Formula Student Spain: about 20 August, 14:00 CEST, at Montmeló. Earlier
	// in the day the main grandstand's shadow covers the straight.
	static constexpr double SunElevationDeg = 60.7;
	static constexpr double SunAzimuthDeg = 182.8;   // clockwise from north

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void Tick(float DeltaSeconds) override;

private:
	/** The bearing of the loaded world's +x (0 without a world). */
	double XBearing() const;
	void AimSun();

	UPROPERTY() TObjectPtr<USkyAtmosphereComponent> Atmosphere;
	UPROPERTY() TObjectPtr<USkyLightComponent> Sky;
	UPROPERTY() TObjectPtr<UPostProcessComponent> Exposure;

	struct FSun
	{
		TWeakObjectPtr<UDirectionalLightComponent> Light;
		FRotator Rotation;
		float Intensity = 0.f;
		bool bAtmosphereSunLight = false;
	};
	FSun Sun;
	TArray<TWeakObjectPtr<AActor>> HiddenActors;
	TArray<TWeakObjectPtr<USceneComponent>> HiddenComponents;
	double AimedBearing = -1.0;
	float AppliedExposure = NAN;
};
