#include "FSDSLidarDecodeShader.h"

IMPLEMENT_GLOBAL_SHADER(FFSDSLidarDecodeCS,
	"/Plugin/FSDSPlugin/Private/FSDSLidarDecode.usf",
	"FSDSLidarDecodeCS",
	SF_Compute);
