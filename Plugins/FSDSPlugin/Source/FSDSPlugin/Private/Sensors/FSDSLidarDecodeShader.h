#pragma once

#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "RHI.h"
#include "RenderGraphResources.h"
#include "ShaderParameterStruct.h"

// Compute-shader binding for FSDSLidarDecode.usf (#223 Phase 3).
//
// One thread per LiDAR ray; total dispatch is
// ceil(NumChannels × NumHorizontalSteps / 64). Reads scene depth from
// the LiDAR depth RT, applies noise + dropout + far-plane cull, writes
// 3D points (vehicle frame, ROS REP-103) to a structured buffer.
class FFSDSLidarDecodeCS : public FGlobalShader
{
	DECLARE_GLOBAL_SHADER(FFSDSLidarDecodeCS);
	SHADER_USE_PARAMETER_STRUCT(FFSDSLidarDecodeCS, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_RDG_TEXTURE(Texture2D<float>, DepthTexture)
		SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, OutPoints)

		SHADER_PARAMETER(uint32, NumChannels)
		SHADER_PARAMETER(uint32, NumHorizontalSteps)
		SHADER_PARAMETER(uint32, RTWidth)
		SHADER_PARAMETER(uint32, RTHeight)

		SHADER_PARAMETER(float, HorizontalFOVStartRad)
		SHADER_PARAMETER(float, HorizontalFOVEndRad)
		SHADER_PARAMETER(float, VerticalFOVUpperRad)
		SHADER_PARAMETER(float, VerticalFOVLowerRad)
		SHADER_PARAMETER(float, TiltRad)

		SHADER_PARAMETER(float, HHalfPlanar)
		SHADER_PARAMETER(float, VBottomPlanar)
		SHADER_PARAMETER(float, VTopPlanar)

		SHADER_PARAMETER(float, MinRangeCm)
		// Kept as a fallback / sentinel for clipmap rejection inside the
		// shader (depths >= MaxRangeCm * 0.99 = "no hit"). Per-LiDAR
		// channel max range cull is in ChannelMaxRangeCm below.
		SHADER_PARAMETER(float, MaxRangeCm)
		// Per-channel max range, NumberOfChannels entries. Indexed by
		// VIdx in the shader. Always populated (fallback to MaxRangeCm
		// when settings.json doesn't provide overrides — see
		// UFSDSLidarSensor::OnSettingsApplied).
		SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, ChannelMaxRangeCm)

		SHADER_PARAMETER(float, RangeNoiseStdCm)
		SHADER_PARAMETER(float, DropoutRate)

		SHADER_PARAMETER(uint32, RNGSeed)

		SHADER_PARAMETER(float, SensorOffsetXm)
		SHADER_PARAMETER(float, SensorOffsetYm)
		SHADER_PARAMETER(float, SensorOffsetZm)
	END_SHADER_PARAMETER_STRUCT()

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
	}

	static constexpr int32 ThreadGroupSize = 64;
};
