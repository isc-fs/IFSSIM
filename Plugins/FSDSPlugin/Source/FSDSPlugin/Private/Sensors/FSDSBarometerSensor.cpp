#include "Sensors/FSDSBarometerSensor.h"

UFSDSBarometerSensor::UFSDSBarometerSensor()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UFSDSBarometerSensor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AActor* Owner = GetOwner();
	if (!Owner) return;

	FBarometerOutput Output;
	Output.Timestamp = FPlatformTime::Cycles64();

	// Altitude from UE position (cm to m) + home offset
	float AltitudeM = Owner->GetActorLocation().Z / 100.f + HomeAltitude;
	Output.Altitude = AltitudeM;

	// Barometric formula: P = P0 * (1 - L*h/T0)^(g*M/(R*L))
	// L = temperature lapse rate = 0.0065 K/m
	// g = 9.80665 m/s², M = 0.0289644 kg/mol, R = 8.31447 J/(mol·K)
	float L = 0.0065f;
	float Exponent = 5.25588f; // g*M/(R*L)
	Output.Pressure = SeaLevelPressure * FMath::Pow(1.f - L * AltitudeM / SeaLevelTemperature, Exponent);
	Output.Temperature = SeaLevelTemperature - L * AltitudeM;

	CachedOutput = Output;
}
