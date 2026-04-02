#include "Vehicles/FSDSWheelRear.h"

UFSDSWheelRear::UFSDSWheelRear()
{
	AxleType = EAxleType::Rear;
	bAffectedByHandbrake = true;
	bAffectedBySteering = false;

	WheelRadius = 38.f;
	WheelWidth = 17.f;
	MaxSteerAngle = 0.f;

	SuspensionMaxRaise = 10.f;
	SuspensionMaxDrop = 10.f;
	SuspensionDampingRatio = 1.5f;
}
