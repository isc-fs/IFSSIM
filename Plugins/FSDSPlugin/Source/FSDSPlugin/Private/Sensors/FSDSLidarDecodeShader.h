#pragma once

#include "CoreMinimal.h"
#include "GlobalShader.h"
#include "RHI.h"
#include "RenderGraphResources.h"
#include "ShaderParameterStruct.h"

// Compute-shader binding for FSDSLidarDecode.usf (#223 Phase 3).
//
// One thread per LiDAR ray; total dispatch is
// ceil(NumChannels × NumHorizontalSteps / 64). Reads scene depth + base
// colour + world-space surface normal from the LiDAR's three render
// targets, applies noise + dropout + far-plane cull, writes 3D points
// AND an intensity scalar (vehicle frame, ROS REP-103) to a structured
// buffer.
//
// Intensity model (#255). Mirrors the Hesai ATX-S01 working principle:
//   intensity = ρ_905 × cos(θ_inc) × (R_ref / range)²
// where ρ_905 is the surface's 905 nm reflectance (placeholder: visible-
// light luminance of the BaseColor capture; real per-material values
// are tuned per cone material — deferred follow-up). cos(θ_inc) is the
// dot product of the inverted ray direction with the world-space
// surface normal. R_ref normalises the inverse-square falloff so a
// perpendicular surface at R_ref returns the unmodified reflectance.
class FFSDSLidarDecodeCS : public FGlobalShader
{
	DECLARE_GLOBAL_SHADER(FFSDSLidarDecodeCS);
	SHADER_USE_PARAMETER_STRUCT(FFSDSLidarDecodeCS, FGlobalShader);

	BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
		SHADER_PARAMETER_RDG_TEXTURE(Texture2D<float>,  DepthTexture)
		SHADER_PARAMETER_RDG_TEXTURE(Texture2D<float4>, ColorTexture)   // SCS_FinalColorLDR — RGB → reflectance proxy via Rec.709 luminance
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

		// Sensor world transform — needed inside the shader to take the
		// vehicle-local ray direction (computed from spherical h/v) into
		// the world frame so it can dot against the world-space normal
		// captured in NormalTexture. Forward-row of the rotation matrix
		// suffices; the C++ side packs the row vectors in column order.
		// FVector3f on the C++ side maps to float3 in the HLSL uniform.
		SHADER_PARAMETER(FVector3f, SensorRotRowX)
		SHADER_PARAMETER(FVector3f, SensorRotRowY)
		SHADER_PARAMETER(FVector3f, SensorRotRowZ)

		// Intensity model (#255). RReferenceM gives the inverse-square
		// reference range; perpendicular surfaces at this range return
		// the unmodified reflectance. Set CPU-side to 5 m to put
		// typical FS-cone returns near full-scale (real Hesai applies
		// receiver-side gain for approximately the same effect).
		SHADER_PARAMETER(float, RReferenceM)
	END_SHADER_PARAMETER_STRUCT()

	static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
	{
		return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
	}

	static constexpr int32 ThreadGroupSize = 64;
};
