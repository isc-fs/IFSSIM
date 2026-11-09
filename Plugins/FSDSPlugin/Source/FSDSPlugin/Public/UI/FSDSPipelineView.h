#pragma once

#include "CoreMinimal.h"
#include "Containers/Ticker.h"

class AActor;
class AFSDSVehiclePawn;
class ULineBatchComponent;
class UWorld;

/**
 * The pipeline view: what the autonomy pipeline believes, drawn over the
 * real track. The cones it detects this scan, its SLAM cone map, where SLAM
 * puts the car, its planned path and the cone chains it built that from.
 *
 * Fed by the stack's viz_relay node (ros2/src/sim_supervisor), which sends
 * one JSON line ten times a second on a `streamViz` connection to the RPC
 * port. Shown from the box over an autonomous run (Esc); the choice is saved
 * in GameUserSettings [FSDSPipelineView] Shown.
 *
 * Map frame to world: SLAM's origin is the car's pose when it started, with
 * the car still at the start. While the car stands still the anchor follows
 * it (true pose composed with SLAM's inverse, so near the true pose); it
 * locks when the car moves off, and frees again when SLAM's map is empty and
 * the car still (a new session). Drift then shows as the drawn map sliding
 * off the real cones. Detected cones are in the car's frame and are placed
 * on the car's true pose.
 *
 * Drawn only while the pipeline drives (API control on). Drawn with a line batch: no collision (the LiDAR doesn't see it) and
 * hidden from scene captures (nor do cameras).
 */
class FSDSPLUGIN_API FFSDSPipelineView
{
public:
	static FFSDSPipelineView& Get();

	void Init(UWorld* InWorld, AFSDSVehiclePawn* InPawn);
	void Shutdown();

	/** From the RPC thread: one line of viz_relay's stream. */
	void Receive(const FString& JsonLine);

	bool IsShown() const;
	void SetShown(bool bOn);
	/** Whether viz_relay has sent anything in the last two seconds. */
	bool IsReceiving() const;

	/** A bag replays (the Bags page): the car follows the recorded ground
	 *  truth viz_relay forwards, with its physics off, and the view draws
	 *  without the pipeline driving. EndReplay gives the car its physics back. */
	void BeginReplay();
	void EndReplay();
	bool IsReplaying() const { return bReplay; }
	/** Whether a recorded pose came in the last two seconds. */
	bool HasReplayPose() const;

private:
	struct FSnapshot
	{
		bool bHasPose = false;
		FVector2D Pose = FVector2D::ZeroVector;   // map frame, m
		double Yaw = 0.0;                         // map frame, rad
		TArray<FVector2D> Map, Det, Path, Left, Right;
	};

	bool Tick(float Dt);
	void Draw(const FSnapshot& S);
	void Clear();
	/** Map frame (m) to UE world (cm), through the anchor, at ground height. */
	FVector MapToWorld(const FVector2D& P) const;

	TWeakObjectPtr<UWorld> World;
	TWeakObjectPtr<AFSDSVehiclePawn> Pawn;
	TWeakObjectPtr<AActor> Holder;
	TWeakObjectPtr<ULineBatchComponent> Lines;
	FTSTicker::FDelegateHandle TickHandle;

	mutable FCriticalSection Lock;
	FSnapshot Pending;
	bool bPendingNew = false;
	double LastReceived = -1.0;

	// The anchor: map frame origin in ENU (m) and its heading (rad).
	FVector2D AnchorEnu = FVector2D::ZeroVector;
	double AnchorYaw = 0.0;
	bool bAnchored = false;
	bool bAnchorLocked = false;
	double GroundZ = 0.0;   // UE cm
	bool bDrawn = false;

	// The car's speed from its own motion between snapshots: with its physics
	// off in a replay, the car state's speed reads zero.
	FVector2D LastCarEnu = FVector2D::ZeroVector;
	double LastCarAt = -1.0;

	// Replay: the last two recorded poses (ENU, m; yaw rad), and when each came.
	struct FTruth { double At = 0.0; FVector Enu = FVector::ZeroVector; double Yaw = 0.0; };
	void ApplyReplayPose();
	bool bReplay = false;
	bool bPhysicsWas = true;
	FTruth TruthPrev, TruthLast;
	int32 TruthCount = 0;

	mutable int32 ShownCache = -1;
};
