#include "Vehicles/FSDSWheelRear.h"

UFSDSWheelRear::UFSDSWheelRear()
{
	AxleType = EAxleType::Rear;
	bAffectedByHandbrake = true;
	bAffectedBySteering = false;

	// IFS-08: Hoosier 16.0x7.5-10 R20
	WheelRadius = 20.f;       // 200mm tire radius
	WheelWidth = 19.f;        // 7.5 inch = 190mm
	MaxSteerAngle = 0.f;

	// IFS-08: 35mm ride height, 300 lbs/in rear springs
	SuspensionMaxRaise = 3.5f;
	SuspensionMaxDrop = 3.5f;
	SuspensionDampingRatio = 1.5f;

	// Hoosier R20 slick friction
	FrictionForceMultiplier = 1.65f;
}
