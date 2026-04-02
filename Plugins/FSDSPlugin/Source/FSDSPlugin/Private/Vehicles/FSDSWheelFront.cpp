#include "Vehicles/FSDSWheelFront.h"

UFSDSWheelFront::UFSDSWheelFront()
{
	AxleType = EAxleType::Front;
	bAffectedByHandbrake = false;
	bAffectedBySteering = true;

	WheelRadius = 38.f;
	WheelWidth = 17.f;
	MaxSteerAngle = 50.f;

	SuspensionMaxRaise = 10.f;
	SuspensionMaxDrop = 10.f;
	SuspensionDampingRatio = 1.5f;
}
