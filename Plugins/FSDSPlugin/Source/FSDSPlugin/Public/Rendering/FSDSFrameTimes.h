#pragma once

#include "CoreMinimal.h"

/**
 * Logs where each frame's time goes, every 5 s:
 *
 *   FSDS Frame: 300 frames in 5.0 s (60.0 FPS) | mean ms: frame 16.67, game 6.10,
 *     render 4.20, rhi 2.00, gpu 9.80 | max ms: frame 18.20, game 8.00, render 6.10, gpu 12.40
 *
 * Game, render, RHI and GPU are the numbers `stat unit` shows. The sim is
 * capped at 60 FPS (16.7 ms), so whichever of them passes 16.7 ms is what
 * holds it below real time. Started by the plugin module; logs in game
 * runs only (not in the editor or commandlets).
 */
namespace FSDSFrameTimes
{
	void Start();
	void Stop();
}
