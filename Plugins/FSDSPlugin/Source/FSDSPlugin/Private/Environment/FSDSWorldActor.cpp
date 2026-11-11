#include "Environment/FSDSWorldActor.h"

#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Environment/FSDSEnvironmentBuilder.h"
#include "FSDSCollision.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Engine/StaticMesh.h"
#include "ProceduralMeshComponent.h"

namespace
{
	// Terrain spacing where the detail is not needed: along x outside the
	// world's detail zone, and across y beyond FineHalfWidthM of its axis.
	constexpr double CoarseStepM = 4.0;
	constexpr double FineHalfWidthM = 60.0;
	constexpr double ChunkM = 100.0;

	// Ground surfaces sit this far above the terrain: the step a wheel may
	// meet (docs/collision_matrix.md allows 3 cm). Paint goes just above them.
	constexpr double SurfaceLiftM = 0.03;
	constexpr double BandLiftM = 0.035;
	constexpr double LineLiftM = 0.04;

	constexpr double WallFootM = 0.3;     // walls go this far into the ground
	constexpr double WallStepM = 2.0;     // walls follow the ground in steps this long
	constexpr double PostSpacingM = 2.5;  // fence posts
	constexpr double TierDepthM = 0.85;   // one grandstand row
	constexpr double RoofThicknessM = 0.5;
	constexpr double ColumnSpacingM = 15.0;
	constexpr double ColumnSizeM = 0.6;

	// Prop modules, as modelled (tools/assetgen/blender/build_assets.py).
	constexpr double SeatBlockM = 5.0;    // ten seats
	constexpr int32 BlocksPerBay = 6;     // then an aisle
	constexpr double AisleM = 1.4;
	constexpr double SignM = 3.0, SignGapM = 0.05, SignMaxWallM = 1.5;
	constexpr double GarageM = 6.0, GarageHeightM = 8.25;
	constexpr int32 GarageOpenEvery = 3;  // every third garage stands open (an event day)
	constexpr double CanopyThicknessM = 0.6;
	constexpr double PanelThicknessM = 0.12, PanelLiftM = 0.02;
	// Under a grandstand roof: a tapered rafter at each column, purlins across them.
	constexpr double RafterWidthM = 0.35, RafterDepthBackM = 1.2, RafterDepthFrontM = 0.35;
	constexpr double PurlinSpacingM = 3.0, PurlinWidthM = 0.15, PurlinDepthM = 0.25;
	// The tree models' nominal size (tools/assetgen/blender: trees); each
	// instance is scaled to its measured height and crown radius.
	constexpr double TreeModelHeightM = 10.0, TreeModelRadiusM = 4.0;
	constexpr double TreeCullM = 1500.0;   // the whole world: trees frame every view
	// The ground beyond the survey: out this far, sloping down this much.
	constexpr double SkirtM = 40000.0, SkirtBlendM = 300.0, SkirtDropM = 2.0;
	// Window bands on plain LiDAR blocks: a 1.4 m band per 3.5 m storey, from
	// 1 m above the ground, standing 3 cm proud of the wall.
	constexpr double StoreyM = 3.5, WindowSillM = 1.0, WindowHeightM = 1.4, WindowProudM = 0.03;
	constexpr double RailM = 2.5;
	constexpr float StartLightsEmissive = 3.0f;   // x the lens colour (0.4 red); brighter tonemaps to orange

	const TCHAR* MatDir = TEXT("/Game/RaceCourse/Materials/Environment/");
	const TCHAR* WorldDir = TEXT("/Game/Worlds/");

	/** World frame (m) to UE (cm): the world's origin sits at Origin in the
	 *  track CSV frame, and CSV +y is UE -Y. */
	struct FWorldFrame
	{
		FVector2D Origin;
		FVector Ue(double X, double Y, double Z) const
		{
			return FVector((X + Origin.X) * 100.0, -(Y + Origin.Y) * 100.0, Z * 100.0);
		}
	};

	/** One mesh section being assembled. */
	struct FBuf
	{
		TArray<FVector> V;
		TArray<int32> T;
		TArray<FVector> N;
		TArray<FVector2D> UV;
		TArray<FLinearColor> Col;   // per vertex, or empty (the materials read it only for wear)

		int32 Add(const FVector& P, const FVector& Nrm, const FVector2D& Uv)
		{
			N.Add(Nrm);
			UV.Add(Uv);
			return V.Add(P);
		}

		/** A triangle whose front face looks along Facing. UE draws the face
		 *  where (P1 - P2) x (P0 - P2) points. */
		void Tri(int32 A, int32 B, int32 C, const FVector& Facing)
		{
			const FVector Nrm = FVector::CrossProduct(V[B] - V[C], V[A] - V[C]);
			T.Add(A);
			if (FVector::DotProduct(Nrm, Facing) >= 0.0) { T.Add(B); T.Add(C); }
			else { T.Add(C); T.Add(B); }
		}

		/** A flat quad, corners in order around it, facing Facing; UVs are
		 *  planar in world space, in units of TileM metres (the materials
		 *  set their own repeat, so metres by default). */
		void Quad(const FVector& A, const FVector& B, const FVector& C, const FVector& D,
			const FVector& Facing, double TileM = 1.0)
		{
			FVector Nrm = FVector::CrossProduct(B - A, D - A).GetSafeNormal();
			if (Nrm.IsNearlyZero()) return;
			if (FVector::DotProduct(Nrm, Facing) < 0.0) Nrm = -Nrm;
			const double S = 1.0 / (TileM * 100.0);
			const FVector H = FVector(-Nrm.Y, Nrm.X, 0.0).GetSafeNormal();
			auto Uv = [&](const FVector& P) {
				return FMath::Abs(Nrm.Z) > 0.7 ? FVector2D(P.X, P.Y) * S
				                               : FVector2D(FVector::DotProduct(P, H), -P.Z) * S;
			};
			const int32 I0 = Add(A, Nrm, Uv(A)), I1 = Add(B, Nrm, Uv(B)), I2 = Add(C, Nrm, Uv(C)), I3 = Add(D, Nrm, Uv(D));
			Tri(I0, I1, I2, Nrm);
			Tri(I0, I2, I3, Nrm);
		}

		/** A closed hexahedron: Bottom[0..3] around the base, Top[i] above Bottom[i]. */
		void Box(const FVector Bottom[4], const FVector Top[4], bool bWithBottom = false)
		{
			FVector C = FVector::ZeroVector;
			for (int32 I = 0; I < 4; ++I) C += Bottom[I] + Top[I];
			C /= 8.0;
			auto Face = [&](const FVector& A, const FVector& B, const FVector& Cc, const FVector& D) {
				Quad(A, B, Cc, D, (A + B + Cc + D) * 0.25 - C);
			};
			Face(Top[0], Top[1], Top[2], Top[3]);
			if (bWithBottom) Face(Bottom[0], Bottom[1], Bottom[2], Bottom[3]);
			for (int32 I = 0; I < 4; ++I)
			{
				const int32 J = (I + 1) % 4;
				Face(Bottom[I], Bottom[J], Top[J], Top[I]);
			}
		}

		/** An axis-aligned box (UE cm). */
		void AlignedBox(const FVector& Min, const FVector& Max)
		{
			const FVector Lo[4] = {{Min.X, Min.Y, Min.Z}, {Max.X, Min.Y, Min.Z}, {Max.X, Max.Y, Min.Z}, {Min.X, Max.Y, Min.Z}};
			const FVector Hi[4] = {{Min.X, Min.Y, Max.Z}, {Max.X, Min.Y, Max.Z}, {Max.X, Max.Y, Max.Z}, {Min.X, Max.Y, Max.Z}};
			Box(Lo, Hi);
		}
	};

	/**
	 * The terrain as it is drawn: a rectilinear grid through the world's own
	 * samples, dense where the detail is and sparse elsewhere, each cell split
	 * along the same diagonal. Everything draped on the ground asks this for
	 * its height, so it lies on the triangles the car drives on rather than
	 * on the denser data under them.
	 */
	struct FGround
	{
		const FFSDSWorld& W;
		TArray<double> Xs, Ys;   // node positions, world m
		TArray<double> Hs;       // Xs.Num() x Ys.Num(), index = j * Xs.Num() + i

		explicit FGround(const FFSDSWorld& InW) : W(InW)
		{
			const int32 Step = FMath::Max(1, FMath::RoundToInt(CoarseStepM / W.CellM));
			for (int32 I = 0; I < W.Nx; ++I)
			{
				const double X = W.TerrainX0 + I * W.CellM;
				const bool bFine = !W.bHasDetail || (X >= W.DetailXMin && X <= W.DetailXMax);
				if (bFine || I % Step == 0 || I == W.Nx - 1) Xs.Add(X);
			}
			for (int32 J = 0; J < W.Ny; ++J)
			{
				const double Y = W.TerrainY0 + J * W.CellM;
				if (FMath::Abs(Y) <= FineHalfWidthM || J % Step == 0 || J == W.Ny - 1) Ys.Add(Y);
			}
			Hs.SetNumUninitialized(Xs.Num() * Ys.Num());
			for (int32 J = 0; J < Ys.Num(); ++J)
				for (int32 I = 0; I < Xs.Num(); ++I)
					Hs[J * Xs.Num() + I] = W.HeightAt(Xs[I], Ys[J]);
		}

		static int32 Cell(const TArray<double>& A, double V)
		{
			// The last node at or below V, kept inside [0, Num - 2].
			int32 Lo = 0, Hi = A.Num() - 1;
			if (V <= A[0]) return 0;
			if (V >= A[Hi]) return Hi - 1;
			while (Hi - Lo > 1)
			{
				const int32 Mid = (Lo + Hi) / 2;
				(A[Mid] <= V ? Lo : Hi) = Mid;
			}
			return Lo;
		}

		double Node(int32 I, int32 J) const { return Hs[J * Xs.Num() + I]; }

		double At(double X, double Y) const
		{
			const int32 I = Cell(Xs, X), J = Cell(Ys, Y);
			const double U = FMath::Clamp((X - Xs[I]) / (Xs[I + 1] - Xs[I]), 0.0, 1.0);
			const double V = FMath::Clamp((Y - Ys[J]) / (Ys[J + 1] - Ys[J]), 0.0, 1.0);
			const double H00 = Node(I, J), H11 = Node(I + 1, J + 1);
			// Split along (I, J) - (I + 1, J + 1), as the mesh is.
			return U >= V ? H00 + U * (Node(I + 1, J) - H00) + V * (H11 - Node(I + 1, J))
			              : H00 + V * (Node(I, J + 1) - H00) + U * (H11 - Node(I, J + 1));
		}

		/** Upward normal at node (I, J), UE axes. */
		FVector Normal(int32 I, int32 J) const
		{
			const int32 I0 = FMath::Max(I - 1, 0), I1 = FMath::Min(I + 1, Xs.Num() - 1);
			const int32 J0 = FMath::Max(J - 1, 0), J1 = FMath::Min(J + 1, Ys.Num() - 1);
			const double Dx = (Node(I1, J) - Node(I0, J)) / (Xs[I1] - Xs[I0]);
			const double Dy = (Node(I, J1) - Node(I, J0)) / (Ys[J1] - Ys[J0]);
			// World +y is UE -Y, so the y slope changes sign.
			return FVector(-Dx, Dy, 1.0).GetSafeNormal();
		}

		/** The nodes strictly inside (A, B), with A and B at the ends. */
		static TArray<double> Span(const TArray<double>& Nodes, double A, double B)
		{
			TArray<double> Out{A};
			for (double N : Nodes) if (N > A + 1e-6 && N < B - 1e-6) Out.Add(N);
			Out.Add(B);
			return Out;
		}
	};

	/** Nodes plus every Start + k * Step strictly inside (Lo, Hi), sorted. */
	TArray<double> WithSteps(TArray<double> Nodes, double Lo, double Hi, double Start, double Step)
	{
		for (double V = Start + FMath::CeilToDouble((Lo - Start) / Step) * Step; V < Hi - 1e-6; V += Step)
		{
			if (V > Lo + 1e-6) Nodes.Add(V);
		}
		Nodes.Sort();
		TArray<double> Out;
		for (double V : Nodes) if (Out.Num() == 0 || V - Out.Last() > 1e-4) Out.Add(V);
		return Out;
	}

	/** A grid draped on the ground over Xs x Ys, Lift metres above it. With
	 *  Wear, each vertex carries the wear map's multiplier / 2 in its red
	 *  channel (the asphalt material doubles it back). */
	void Drape(FBuf& Buf, const FGround& G, const FWorldFrame& F, const TArray<double>& Xs, const TArray<double>& Ys,
		double Lift, double TileM, const FFSDSWorld* Wear = nullptr)
	{
		const int32 Base = Buf.V.Num();
		const double S = 1.0 / (TileM * 100.0);
		for (double Y : Ys)
			for (double X : Xs)
			{
				const FVector P = F.Ue(X, Y, G.At(X, Y) + Lift);
				const int32 I = G.Cell(G.Xs, X), J = G.Cell(G.Ys, Y);
				Buf.Add(P, G.Normal(I, J), FVector2D(P.X, P.Y) * S);
				if (Wear)
				{
					const float Half = float(FMath::Clamp(Wear->WearAt(X, Y) * 0.5, 0.0, 1.0));
					Buf.Col.Add(FLinearColor(Half, Half, Half, 1.0f));
				}
			}
		const int32 Nx = Xs.Num();
		for (int32 J = 0; J + 1 < Ys.Num(); ++J)
			for (int32 I = 0; I + 1 < Nx; ++I)
			{
				const int32 A = Base + J * Nx + I, B = A + 1, C = A + Nx + 1, D = A + Nx;
				Buf.Tri(A, B, C, FVector::UpVector);
				Buf.Tri(A, C, D, FVector::UpVector);
			}
	}

	UMaterialInterface* LoadMat(const TCHAR* Name)
	{
		const FString Path = FString::Printf(TEXT("%s%s.%s"), MatDir, Name, Name);
		UMaterialInterface* M = LoadObject<UMaterialInterface>(nullptr, *Path);
		if (!M) UE_LOG(LogTemp, Warning, TEXT("FSDS World: material %s is missing; using a plain colour"), *Path);
		return M;
	}

	/** MI_<Name> from /Game/Worlds (tools/assetgen/materials.json). */
	UMaterialInterface* WorldMat(const TCHAR* Name)
	{
		const FString Path = FString::Printf(TEXT("%sMaterials/MI_%s.MI_%s"), WorldDir, Name, Name);
		UMaterialInterface* M = LoadObject<UMaterialInterface>(nullptr, *Path);
		if (!M) UE_LOG(LogTemp, Warning, TEXT("FSDS World: %s is missing (docs/world_assets.md); using the fallback"), *Path);
		return M;
	}

	/** SM_<Name> from /Game/Worlds; null (and a warning) when it is missing. */
	UStaticMesh* WorldMesh(const FString& Name)
	{
		const FString Path = FString::Printf(TEXT("%sMeshes/SM_%s.SM_%s"), WorldDir, *Name, *Name);
		UStaticMesh* M = LoadObject<UStaticMesh>(nullptr, *Path);
		if (!M) UE_LOG(LogTemp, Warning, TEXT("FSDS World: %s is missing (docs/world_assets.md); not placed"), *Path);
		return M;
	}
}

// FSDS.StartLights 1|0: switch the start lights by hand (the ROS bridge
// does it from /assi/state; see the setStartLights RPC).
static FAutoConsoleCommandWithWorldAndArgs GStartLightsCmd(
	TEXT("FSDS.StartLights"),
	TEXT("FSDS.StartLights 1|0 - light or put out the start lights of the loaded world"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World) {
		if (!World || Args.Num() != 1) return;
		for (TActorIterator<AFSDSWorldActor> It(World); It; ++It) It->SetStartLights(Args[0] == TEXT("1"));
	}));

AFSDSWorldActor::AFSDSWorldActor()
{
	PrimaryActorTick.bCanEverTick = false;
	RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent->SetMobility(EComponentMobility::Static);
}

UProceduralMeshComponent* AFSDSWorldActor::NewMesh(const TCHAR* Profile, bool bCastShadow)
{
	UProceduralMeshComponent* M = NewObject<UProceduralMeshComponent>(this);
	M->bUseAsyncCooking = false;              // props and cones look for the ground right after Build
	M->bUseComplexAsSimpleCollision = true;   // collision equals the drawn surface (collision matrix)
	M->SetCastShadow(bCastShadow);
	M->SetupAttachment(RootComponent);
	M->SetCollisionProfileName(Profile);
	M->RegisterComponent();
	Meshes.Add(M);
	return M;
}

UHierarchicalInstancedStaticMeshComponent* AFSDSWorldActor::AddInstances(UStaticMesh* Mesh, const TArray<FTransform>& Transforms, double CullM, bool bCastShadow)
{
	if (!Mesh || Transforms.Num() == 0) return nullptr;
	UHierarchicalInstancedStaticMeshComponent* H = NewObject<UHierarchicalInstancedStaticMeshComponent>(this);
	H->SetStaticMesh(Mesh);
	H->SetMobility(EComponentMobility::Static);
	H->SetCollisionProfileName(TEXT("NoCollision"));
	H->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	H->SetCastShadow(bCastShadow);
	H->SetCullDistances(int32(CullM * 80.0), int32(CullM * 100.0));   // fade out over the last 20 %
	H->SetupAttachment(RootComponent);
	H->RegisterComponent();
	H->AddInstances(Transforms, /*bShouldReturnIndices=*/false, /*bWorldSpace=*/true);
	Instanced.Add(H);
	NumInstances += Transforms.Num();
	PropTriangles += Transforms.Num() * (Mesh->GetRenderData() && Mesh->GetRenderData()->LODResources.Num()
		? Mesh->GetRenderData()->LODResources[0].GetNumTriangles() : 0);
	return H;
}

void AFSDSWorldActor::SetStartLights(bool bOn)
{
	if (!StartLights || bOn == bStartLightsOn) return;
	bStartLightsOn = bOn;
	// The lens is dark red glass; lit, it glows well above the sky.
	StartLights->SetScalarParameterValue(TEXT("Emissive"), bOn ? StartLightsEmissive : 0.0f);
	UE_LOG(LogTemp, Log, TEXT("FSDS World: start lights %s"), bOn ? TEXT("on") : TEXT("out"));
}

void AFSDSWorldActor::Build(const FFSDSWorld& World, const FVector2D& OriginCsvM)
{
	Clear();
	if (!GetWorld() || World.Nx < 2 || World.Ny < 2) return;
	const double T0 = FPlatformTime::Seconds();

	const FWorldFrame F{OriginCsvM};
	const FGround G(World);

	// Materials: the scanned set in /Game/Worlds; the repo's older
	// environment materials and flat colours if it is missing.
	UMaterialInterface* Basic = LoadObject<UMaterialInterface>(nullptr,
		TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	auto Colour = [&](const FLinearColor& C) -> UMaterialInterface* {
		UMaterialInstanceDynamic* Mid = Basic ? UMaterialInstanceDynamic::Create(Basic, this) : nullptr;
		if (!Mid) return Basic;
		Mid->SetVectorParameterValue(TEXT("Color"), C);
		Mid->SetVectorParameterValue(TEXT("BaseColor"), C);
		return Mid;
	};
	auto Or = [](UMaterialInterface* A, UMaterialInterface* B) { return A ? A : B; };
	UMaterialInterface* Grass = Or(WorldMat(TEXT("Grass")), Colour(FLinearColor(0.16f, 0.17f, 0.09f)));
	UMaterialInterface* Asphalt = Or(WorldMat(TEXT("Asphalt")), Or(LoadMat(TEXT("Road_Inst_no_lines")), Colour(FLinearColor(0.04f, 0.04f, 0.045f))));
	// Lighter, older asphalt (LiDAR intensity), not paint.
	UMaterialInterface* Band = Or(WorldMat(TEXT("AsphaltWorn")), Colour(FLinearColor(0.10f, 0.10f, 0.10f)));
	UMaterialInterface* ConcreteGround = Or(WorldMat(TEXT("ConcreteGround")), Or(LoadMat(TEXT("concrete_ground")), Colour(FLinearColor(0.35f, 0.34f, 0.32f))));
	UMaterialInterface* ConcreteWall = Or(WorldMat(TEXT("ConcreteWall")), Or(LoadMat(TEXT("concrete_wall")), Colour(FLinearColor(0.45f, 0.45f, 0.43f))));
	UMaterialInterface* Roofing = Or(WorldMat(TEXT("SteelWhite")), Or(LoadMat(TEXT("metal")), Colour(FLinearColor(0.5f, 0.5f, 0.52f))));
	UMaterialInterface* Fence = Or(WorldMat(TEXT("ChainLink")), Or(LoadMat(TEXT("Chainlink")), Colour(FLinearColor(0.3f, 0.3f, 0.3f))));
	UMaterialInterface* BuildingWhite = Or(WorldMat(TEXT("ConcreteCream")), Or(LoadMat(TEXT("Buildings_WHITE")), Colour(FLinearColor(0.7f, 0.7f, 0.68f))));
	UMaterialInterface* Glass = Or(WorldMat(TEXT("Glass")), Or(LoadMat(TEXT("windows_Inst")), Colour(FLinearColor(0.1f, 0.12f, 0.15f))));
	// Rubber on the racing line: vertex colours from the wear map (docs/worlds.md).
	UMaterialInterface* AsphaltTrack = Or(WorldMat(TEXT("AsphaltTrack")), Asphalt);
	// Worn paint: the asphalt's grain shows through (M_WorldPaint).
	UMaterialInterface* White = Or(WorldMat(TEXT("PaintWhite")), Or(WorldMat(TEXT("Paint")), Colour(FLinearColor(0.8f, 0.8f, 0.8f))));
	UMaterialInterface* Yellow = Or(WorldMat(TEXT("PaintYellow")), Colour(FLinearColor(0.8f, 0.6f, 0.05f)));
	UMaterialInterface* Red = Or(WorldMat(TEXT("PaintRed")), Colour(FLinearColor(0.5f, 0.03f, 0.02f)));
	UMaterialInterface* Green = Or(WorldMat(TEXT("PaintGreen")), Colour(FLinearColor(0.05f, 0.2f, 0.06f)));
	UMaterialInterface* SteelUnder = Or(WorldMat(TEXT("SteelGalv")), Roofing);
	UMaterialInterface* SolarPanel = Or(WorldMat(TEXT("SolarPanel")), Colour(FLinearColor(0.03f, 0.035f, 0.045f)));
	UMaterialInterface* PanelDark = Or(WorldMat(TEXT("PanelDark")), Colour(FLinearColor(0.14f, 0.11f, 0.09f)));

	// Props in the world frame: yaw is anticlockwise from +x towards +y
	// (UE's yaw turns the other way, since world +y is UE -Y).
	auto Place = [&](double X, double Y, double Z, double YawDeg, const FVector& Scale = FVector::OneVector) {
		return FTransform(FRotator(0.0, -YawDeg, 0.0), F.Ue(X, Y, Z), Scale);
	};
	TArray<FTransform> Posts, Signs, Seats, Rails, Columns, Garages, GaragesOpen;

	auto Emit = [&](UProceduralMeshComponent* M, int32 Section, FBuf& B, UMaterialInterface* Mat, bool bCollision) {
		if (B.T.Num() == 0) return;
		const TArray<FLinearColor> NoColours;
		M->CreateMeshSection_LinearColor(Section, B.V, B.T, B.N, B.UV, B.Col.Num() == B.V.Num() ? B.Col : NoColours, {}, bCollision);
		M->SetMaterial(Section, Mat);
		NumTriangles += B.T.Num() / 3;
	};

	// --- terrain, in chunks so the renderer can cull them ---------------------
	{
		auto Breaks = [](const TArray<double>& Nodes) {
			TArray<int32> Out{0};
			double Next = Nodes[0] + ChunkM;
			for (int32 I = 1; I < Nodes.Num() - 1; ++I)
			{
				if (Nodes[I] >= Next) { Out.Add(I); Next = Nodes[I] + ChunkM; }
			}
			Out.Add(Nodes.Num() - 1);
			return Out;
		};
		const TArray<int32> Bx = Breaks(G.Xs), By = Breaks(G.Ys);
		for (int32 Cy = 0; Cy + 1 < By.Num(); ++Cy)
			for (int32 Cx = 0; Cx + 1 < Bx.Num(); ++Cx)
			{
				FBuf B;
				const int32 I0 = Bx[Cx], I1 = Bx[Cx + 1], J0 = By[Cy], J1 = By[Cy + 1];
				const int32 Nx = I1 - I0 + 1;
				for (int32 J = J0; J <= J1; ++J)
					for (int32 I = I0; I <= I1; ++I)
					{
						const FVector P = F.Ue(G.Xs[I], G.Ys[J], G.Node(I, J));
						B.Add(P, G.Normal(I, J), FVector2D(P.X, P.Y) / 100.0);
					}
				for (int32 J = 0; J < J1 - J0; ++J)
					for (int32 I = 0; I < Nx - 1; ++I)
					{
						// Same diagonal as FGround::At.
						const int32 A = J * Nx + I, Bq = A + 1, C = A + Nx + 1, D = A + Nx;
						B.Tri(A, Bq, C, FVector::UpVector);
						B.Tri(A, C, D, FVector::UpVector);
					}
				Emit(NewMesh(FSDSCollision::TerrainProfile, false), 0, B, Grass, true);
			}
	}

	// --- skirt: the ground carries on past the survey to the horizon ---------
	// Without it the world is an island and the sky shows below the horizon.
	// No collision: nothing that drives gets this far.
	{
		const int32 NX = G.Xs.Num(), NY = G.Ys.Num();
		TArray<FIntPoint> Ring;   // border nodes, once round
		for (int32 I = 0; I < NX; ++I) Ring.Add({I, 0});
		for (int32 J = 1; J < NY; ++J) Ring.Add({NX - 1, J});
		for (int32 I = NX - 2; I >= 0; --I) Ring.Add({I, NY - 1});
		for (int32 J = NY - 2; J >= 1; --J) Ring.Add({0, J});
		// Three rings: the survey edge, a blend ring SkirtBlendM out, and the
		// far ring. Past the blend ring the ground is flat at the edge's mean
		// height, so the long strips all share one plane and one shading.
		double ZFlat = 0.0;
		for (const FIntPoint& N : Ring) ZFlat += G.Node(N.X, N.Y);
		ZFlat = ZFlat / Ring.Num() - SkirtDropM;
		auto Out = [&](const FIntPoint& N, double D, double Z) {
			const double Sx = N.X == 0 ? -1.0 : N.X == NX - 1 ? 1.0 : 0.0;
			const double Sy = N.Y == 0 ? -1.0 : N.Y == NY - 1 ? 1.0 : 0.0;
			return F.Ue(G.Xs[N.X] + Sx * D, G.Ys[N.Y] + Sy * D, Z);
		};
		FBuf B;
		const int32 R = Ring.Num();
		for (const FIntPoint& N : Ring)
			for (const FVector& P : {Out(N, 0.0, G.Node(N.X, N.Y)), Out(N, SkirtBlendM, ZFlat), Out(N, SkirtM, ZFlat)})
				B.Add(P, FVector::UpVector, FVector2D(P.X, P.Y) / 100.0);
		for (int32 K = 0; K < R; ++K)
			for (int32 L = 0; L < 2; ++L)
			{
				const int32 A = 3 * K + L, Bn = 3 * ((K + 1) % R) + L;
				B.Tri(A, Bn, Bn + 1, FVector::UpVector);
				B.Tri(A, Bn + 1, A + 1, FVector::UpVector);
			}
		Emit(NewMesh(TEXT("NoCollision"), false), 0, B, Grass, false);
	}

	// --- surfaces: ground (collision) and painted bands (none) ----------------
	for (const FFSDSWorldSurface& S : World.Surfaces)
	{
		const bool bBand = S.Material == TEXT("asphalt_light");
		// Asphalt inside the wear map: rows at the map's spacing, so the
		// rubber's profile across the track survives in the vertex colours.
		const bool bWorn = S.Material == TEXT("asphalt") && World.WearCovers(S.XMin, S.XMax, S.YMin, S.YMax);
		FBuf B;
		TArray<double> Xs = FGround::Span(G.Xs, S.XMin, S.XMax), Ys = FGround::Span(G.Ys, S.YMin, S.YMax);
		if (bWorn)
		{
			Xs = WithSteps(Xs, S.XMin, S.XMax, World.WearX0, FMath::Max(World.WearDx, CoarseStepM));
			Ys = WithSteps(Ys, S.YMin, S.YMax, World.WearY0, World.WearDy);
		}
		Drape(B, G, F, Xs, Ys, bBand ? BandLiftM : SurfaceLiftM, 1.0, bWorn ? &World : nullptr);
		UMaterialInterface* Mat = bBand ? Band
			: S.Material == TEXT("asphalt") ? (bWorn ? AsphaltTrack : Asphalt)
			: S.Material == TEXT("concrete") ? ConcreteGround : Grass;
		Emit(NewMesh(bBand ? TEXT("NoCollision") : FSDSCollision::TerrainProfile, false),
			0, B, Mat, !bBand);
	}

	// --- lines: strips painted on the ground, tapering if they say so ----------
	{
		FBuf WhiteBuf, YellowBuf, RedBuf, GreenBuf;
		for (const FFSDSWorldLine& L : World.Lines)
		{
			const FVector2D P0(L.X0, L.Y0), P1(L.X1, L.Y1);
			const double Len = (P1 - P0).Size();
			const FVector2D Dir = (P1 - P0) / Len, Side(-Dir.Y, Dir.X);
			const int32 Steps = FMath::Max(1, FMath::CeilToInt(Len));   // follow the ground every metre
			FBuf& B = L.Colour == TEXT("yellow") ? YellowBuf : L.Colour == TEXT("red") ? RedBuf
				: L.Colour == TEXT("green") ? GreenBuf : WhiteBuf;
			const int32 Base = B.V.Num();
			for (int32 K = 0; K <= Steps; ++K)
			{
				const FVector2D C = P0 + Dir * (Len * K / Steps);
				const double Width = FMath::Lerp(L.WidthM, L.EndWidthM, double(K) / Steps);
				for (const double Sgn : {-0.5, 0.5})
				{
					const FVector2D Q = C + Side * (Width * Sgn);
					const FVector P = F.Ue(Q.X, Q.Y, G.At(Q.X, Q.Y) + LineLiftM);
					B.Add(P, FVector::UpVector, FVector2D(P.X, P.Y) / 100.0);
				}
			}
			for (int32 K = 0; K < Steps; ++K)
			{
				const int32 A = Base + 2 * K;
				B.Tri(A, A + 1, A + 3, FVector::UpVector);
				B.Tri(A, A + 3, A + 2, FVector::UpVector);
			}
		}
		UProceduralMeshComponent* M = NewMesh(TEXT("NoCollision"), false);
		Emit(M, 0, WhiteBuf, White, false);
		Emit(M, 1, YellowBuf, Yellow, false);
		Emit(M, 2, RedBuf, Red, false);
		Emit(M, 3, GreenBuf, Green, false);
	}

	// --- walls, each following the ground, with its fence ---------------------
	for (const FFSDSWorldWall& Wl : World.Walls)
	{
		const FVector2D P0(Wl.X0, Wl.Y0), P1(Wl.X1, Wl.Y1);
		const double Len = (P1 - P0).Size();
		if (Len < 0.01) continue;
		const FVector2D Dir = (P1 - P0) / Len, Side = FVector2D(-Dir.Y, Dir.X) * (Wl.ThicknessM * 0.5);
		const int32 Steps = FMath::Max(1, FMath::CeilToInt(Len / WallStepM));
		FBuf WallBuf, FenceBuf;
		auto Pt = [&](double S) { return P0 + Dir * S; };
		for (int32 K = 0; K < Steps; ++K)
		{
			const double S0 = Len * K / Steps, S1 = Len * (K + 1) / Steps;
			const FVector2D A = Pt(S0), Bp = Pt(S1);
			const double Z0 = G.At(A.X, A.Y), Z1 = G.At(Bp.X, Bp.Y);
			const FVector Bot[4] = {F.Ue(A.X - Side.X, A.Y - Side.Y, Z0 - WallFootM), F.Ue(Bp.X - Side.X, Bp.Y - Side.Y, Z1 - WallFootM),
			                        F.Ue(Bp.X + Side.X, Bp.Y + Side.Y, Z1 - WallFootM), F.Ue(A.X + Side.X, A.Y + Side.Y, Z0 - WallFootM)};
			const FVector Top[4] = {F.Ue(A.X - Side.X, A.Y - Side.Y, Z0 + Wl.HeightM), F.Ue(Bp.X - Side.X, Bp.Y - Side.Y, Z1 + Wl.HeightM),
			                        F.Ue(Bp.X + Side.X, Bp.Y + Side.Y, Z1 + Wl.HeightM), F.Ue(A.X + Side.X, A.Y + Side.Y, Z0 + Wl.HeightM)};
			WallBuf.Box(Bot, Top);
			if (Wl.FenceHeightM > Wl.HeightM)
			{
				const FVector Up0(0, 0, (Wl.FenceHeightM - Wl.HeightM) * 100.0);
				const FVector A0 = F.Ue(A.X, A.Y, Z0 + Wl.HeightM), B0 = F.Ue(Bp.X, Bp.Y, Z1 + Wl.HeightM);
				const FVector Facing = F.Ue(Side.X, Side.Y, 0) - F.Ue(0, 0, 0);
				FenceBuf.Quad(A0, B0, B0 + Up0, A0 + Up0, Facing, 1.0);
			}
		}
		if (Wl.FenceHeightM > Wl.HeightM)
		{
			// Posts: a 1 m tube stretched to the fence height.
			const int32 N = FMath::Max(1, FMath::CeilToInt(Len / PostSpacingM));
			for (int32 K = 0; K <= N; ++K)
			{
				const FVector2D C = Pt(Len * K / N);
				Posts.Add(Place(C.X, C.Y, G.At(C.X, C.Y) + Wl.HeightM - 0.05, 0.0,
					FVector(1.0, 1.0, Wl.FenceHeightM - Wl.HeightM + 0.05)));
			}
		}
		if (Wl.HeightM <= SignMaxWallM)
		{
			// Boards along a barrier's track side; a board faces its local -y.
			const FVector2D Normal(-Dir.Y, Dir.X);
			const FVector2D Mid = Pt(Len * 0.5);
			const double ToTrack = FVector2D::DotProduct(Normal, -Mid) >= 0.0 ? 1.0 : -1.0;
			const double Yaw = FMath::RadiansToDegrees(FMath::Atan2(Dir.Y, Dir.X)) + (ToTrack > 0.0 ? 180.0 : 0.0);
			const int32 N = FMath::FloorToInt(Len / (SignM + SignGapM));
			for (int32 K = 0; K < N; ++K)
			{
				const FVector2D C = Pt((SignM + SignGapM) * (K + 0.5)) + Normal * (ToTrack * Wl.ThicknessM * 0.5);
				Signs.Add(Place(C.X, C.Y, G.At(C.X, C.Y) + 0.05, Yaw));
			}
		}
		UProceduralMeshComponent* M = NewMesh(FSDSCollision::PropProfile, true);
		Emit(M, 0, WallBuf, ConcreteWall, true);
		Emit(M, 1, FenceBuf, Fence, false);   // the LiDAR sees through mesh fencing; posts and wall it hits
	}

	// --- buildings: boxes from below the ground to their roof, or raised decks -
	FBuf Blocks, GlassBlocks, Windows, PitPanels;
	for (const FFSDSWorldBuilding& Bd : World.Buildings)
	{
		// Corners and edge midpoints, turned about the centre by the yaw.
		const double Cx = 0.5 * (Bd.XMin + Bd.XMax), Cy = 0.5 * (Bd.YMin + Bd.YMax);
		const double Hl = 0.5 * (Bd.XMax - Bd.XMin), Hw = 0.5 * (Bd.YMax - Bd.YMin);
		const double Cs = FMath::Cos(FMath::DegreesToRadians(Bd.YawDeg)), Sn = FMath::Sin(FMath::DegreesToRadians(Bd.YawDeg));
		auto At = [&](double U, double V) { return FVector2D(Cx + U * Cs - V * Sn, Cy + U * Sn + V * Cs); };
		const FVector2D Corner[4] = {At(-Hl, -Hw), At(Hl, -Hw), At(Hl, Hw), At(-Hl, Hw)};
		double Low = TNumericLimits<double>::Max();
		for (double U : {-Hl, 0.0, Hl})
			for (double V : {-Hw, 0.0, Hw})
			{
				const FVector2D P = At(U, V);
				Low = FMath::Min(Low, G.At(P.X, P.Y));
			}
		Low = Bd.bHasBase ? Bd.BaseZ : Low - WallFootM;
		if (Bd.TopZ <= Low + (Bd.bHasBase ? 0.0 : 1.0))
		{
			UE_LOG(LogTemp, Warning, TEXT("FSDS World: a building's roof (%.2f m) is below its ground (%.2f m); skipped"), Bd.TopZ, Low);
			continue;
		}
		FVector Bot[4], Top[4];
		for (int32 K = 0; K < 4; ++K)
		{
			Bot[K] = F.Ue(Corner[K].X, Corner[K].Y, Low);
			Top[K] = F.Ue(Corner[K].X, Corner[K].Y, Bd.TopZ);
		}
		(Bd.Material == TEXT("glass") ? GlassBlocks : Blocks).Box(Bot, Top, /*bWithBottom=*/Bd.bHasBase);

		// A plain block measured from the LiDAR gets a band of windows on every
		// floor of every face long enough to have them.
		if (!Bd.bHasBase && !Bd.bGarages && Bd.Material == TEXT("concrete"))
		{
			const FVector2D Mid((Cx), (Cy));
			for (int32 K = 0; K < 4; ++K)
			{
				const FVector2D P0 = Corner[K], P1 = Corner[(K + 1) % 4];
				const double Len = FVector2D::Distance(P0, P1);
				if (Len < 4.0) continue;
				const FVector2D Dir = (P1 - P0) / Len;
				FVector2D Out(Dir.Y, -Dir.X);
				if (FVector2D::DotProduct(Out, 0.5 * (P0 + P1) - Mid) < 0.0) Out = -Out;
				const FVector2D A0 = P0 + Dir * 0.8 + Out * WindowProudM, A1 = P1 - Dir * 0.8 + Out * WindowProudM;
				const double Floor0 = G.At(0.5 * (P0.X + P1.X), 0.5 * (P0.Y + P1.Y));
				const FVector N = F.Ue(Out.X, Out.Y, 0.0) - F.Ue(0.0, 0.0, 0.0);
				for (double Z = Floor0 + WindowSillM; Z + WindowHeightM <= Bd.TopZ - 0.6; Z += StoreyM)
				{
					Windows.Quad(F.Ue(A0.X, A0.Y, Z), F.Ue(A1.X, A1.Y, Z), F.Ue(A1.X, A1.Y, Z + WindowHeightM),
						F.Ue(A0.X, A0.Y, Z + WindowHeightM), N);
				}
			}
		}

		// The pit building gets garage fronts on the face towards the track,
		// stretched to its length and roof.
		if (Bd.bGarages && Bd.YawDeg == 0.0)
		{
			const bool bNearIsMax = FMath::Abs(Bd.YMax) < FMath::Abs(Bd.YMin);
			const double FaceY = bNearIsMax ? Bd.YMax : Bd.YMin;
			const int32 N = FMath::Max(1, FMath::RoundToInt((Bd.XMax - Bd.XMin) / GarageM));
			const double Wd = (Bd.XMax - Bd.XMin) / N;
			for (int32 K = 0; K < N; ++K)
			{
				const double X0 = Bd.XMin + Wd * K;
				const double Z = G.At(X0 + Wd * 0.5, FaceY);
				const double H = Bd.TopZ - Z;
				if (H < 4.0 || H > 20.0) continue;
				// The module fronts +y; turned round for a building on the +y side.
				// Some stand open, the same ones every build.
				(K % GarageOpenEvery == 1 ? GaragesOpen : Garages).Add(
					bNearIsMax ? Place(X0, FaceY, Z, 0.0, FVector(Wd / GarageM, 1.0, H / GarageHeightM))
					           : Place(X0 + Wd, FaceY, Z, 180.0, FVector(Wd / GarageM, 1.0, H / GarageHeightM)));
			}
			// A canopy at roof level over the garage fronts (the roof's edge
			// reaches out over the pit lane).
			if (Bd.CanopyM > 0.0)
			{
				const double Out = bNearIsMax ? Bd.CanopyM : -Bd.CanopyM;
				const double Y0 = FaceY - (bNearIsMax ? 0.05 : -0.05), Y1 = FaceY + Out;
				const double Za = Bd.TopZ - CanopyThicknessM, Zb = Bd.TopZ;
				const FVector Lo[4] = {F.Ue(Bd.XMin, Y0, Za), F.Ue(Bd.XMax, Y0, Za), F.Ue(Bd.XMax, Y1, Za), F.Ue(Bd.XMin, Y1, Za)};
				const FVector Hi[4] = {F.Ue(Bd.XMin, Y0, Zb), F.Ue(Bd.XMax, Y0, Zb), F.Ue(Bd.XMax, Y1, Zb), F.Ue(Bd.XMin, Y1, Zb)};
				Blocks.Box(Lo, Hi, /*bWithBottom=*/true);
			}
		}

		// Panels lying on a flat roof.
		for (const FFSDSWorldPanel& P : Bd.RoofPanels)
		{
			const double Za = Bd.TopZ + PanelLiftM, Zb = Za + PanelThicknessM;
			const FVector Lo[4] = {F.Ue(P.XMin, P.YMin, Za), F.Ue(P.XMax, P.YMin, Za), F.Ue(P.XMax, P.YMax, Za), F.Ue(P.XMin, P.YMax, Za)};
			const FVector Hi[4] = {F.Ue(P.XMin, P.YMin, Zb), F.Ue(P.XMax, P.YMin, Zb), F.Ue(P.XMax, P.YMax, Zb), F.Ue(P.XMin, P.YMax, Zb)};
			PitPanels.Box(Lo, Hi);
		}
	}

	{
		UProceduralMeshComponent* M = NewMesh(FSDSCollision::PropProfile, true);
		Emit(M, 0, Blocks, BuildingWhite, true);
		Emit(M, 1, GlassBlocks, Glass, true);
		Emit(M, 2, Windows, Glass, false);
		// Roof panels: out of reach of every sensor, so no collision.
		Emit(NewMesh(TEXT("NoCollision"), false), 0, PitPanels, PanelDark, false);
	}

	// --- grandstands: stepped rows, back wall, roof on rear columns -----------
	for (const FFSDSWorldGrandstand& Gs : World.Grandstands)
	{
		const double Sgn = Gs.FrontY < 0.0 ? -1.0 : 1.0;   // which side of the axis
		const double DF = FMath::Abs(Gs.FrontY), DB = FMath::Abs(Gs.BackY);
		double Base = TNumericLimits<double>::Max();
		for (double X = Gs.XMin; X <= Gs.XMax + 1e-6; X += 10.0)
			for (double D : {DF, DB})
				Base = FMath::Min(Base, G.At(X, Sgn * D));
		Base -= WallFootM;
		// (x, distance from the axis, z) to UE.
		auto P = [&](double X, double D, double Z) { return F.Ue(X, Sgn * D, Z); };
		const FVector Out = P(0, 1, 0) - P(0, 0, 0);   // away from the track

		FBuf Rows, Roof, Under, Steel, Panels;
		const int32 N = FMath::Max(1, FMath::RoundToInt((DB - DF) / TierDepthM));
		double Prev = Base;
		for (int32 I = 0; I < N; ++I)
		{
			const double D0 = DF + (DB - DF) * I / N, D1 = DF + (DB - DF) * (I + 1) / N;
			const double Z = N == 1 ? Gs.BackZ : Gs.FrontZ + (Gs.BackZ - Gs.FrontZ) * I / (N - 1);
			Rows.Quad(P(Gs.XMin, D0, Z), P(Gs.XMax, D0, Z), P(Gs.XMax, D1, Z), P(Gs.XMin, D1, Z), FVector::UpVector);
			Rows.Quad(P(Gs.XMin, D0, Prev), P(Gs.XMax, D0, Prev), P(Gs.XMax, D0, Z), P(Gs.XMin, D0, Z), -Out);
			for (const double X : {Gs.XMin, Gs.XMax})
			{
				const FVector Facing = X == Gs.XMin ? P(-1, 0, 0) - P(0, 0, 0) : P(1, 0, 0) - P(0, 0, 0);
				Rows.Quad(P(X, D0, Base), P(X, D1, Base), P(X, D1, Z), P(X, D0, Z), Facing);
			}
			Prev = Z;
			// Seats on this tread, in bays of six blocks with an aisle between.
			for (int32 K = 0;; ++K)
			{
				const double X = Gs.XMin + 0.6 + K * SeatBlockM + (K / BlocksPerBay) * AisleM;
				if (X + SeatBlockM > Gs.XMax - 0.6) break;
				Seats.Add(Sgn > 0.0 ? Place(X, D0, Z, 0.0) : Place(X + SeatBlockM, -D0, Z, 180.0));
			}
		}
		// A handrail along the front edge.
		for (double X = Gs.XMin; X + RailM <= Gs.XMax + 1e-6; X += RailM)
		{
			Rails.Add(Place(X, Sgn * (DF + 0.08), Gs.FrontZ, 0.0));
		}
		Rows.Quad(P(Gs.XMin, DB, Base), P(Gs.XMax, DB, Base), P(Gs.XMax, DB, Gs.BackZ), P(Gs.XMin, DB, Gs.BackZ), Out);

		// Roof slab, its top on the measured edges; the sheet under it is
		// galvanised steel, carried by a tapered rafter at each column and
		// purlins across the rafters.
		const double RF = FMath::Abs(Gs.RoofFrontY), RB = FMath::Abs(Gs.RoofBackY);
		auto RoofZ = [&](double D) { return FMath::Lerp(Gs.RoofFrontZ, Gs.RoofBackZ, RB != RF ? (D - RF) / (RB - RF) : 0.0); };
		const double DC = FMath::Min(DB + ColumnSizeM, FMath::Max(RB - ColumnSizeM * 0.5, DB));
		const int32 Cols = FMath::Max(1, FMath::RoundToInt((Gs.XMax - Gs.XMin) / ColumnSpacingM));
		auto ColumnX = [&](int32 K) { return FMath::Lerp(Gs.XMin + ColumnSizeM, Gs.XMax - ColumnSizeM, double(K) / Cols); };
		if (Gs.bHasRoof)
		{
			const double T = RoofThicknessM;
			const FVector Bot[4] = {P(Gs.XMin, RF, Gs.RoofFrontZ - T), P(Gs.XMax, RF, Gs.RoofFrontZ - T),
			                        P(Gs.XMax, RB, Gs.RoofBackZ - T), P(Gs.XMin, RB, Gs.RoofBackZ - T)};
			const FVector Top[4] = {P(Gs.XMin, RF, Gs.RoofFrontZ), P(Gs.XMax, RF, Gs.RoofFrontZ),
			                        P(Gs.XMax, RB, Gs.RoofBackZ), P(Gs.XMin, RB, Gs.RoofBackZ)};
			Roof.Box(Bot, Top);
			Under.Quad(Bot[0], Bot[1], Bot[2], Bot[3], -FVector::UpVector, 1.0);

			const double Lo = RF + 0.2, Hi = RB - 0.2;   // rafter ends, inside the slab's edges
			for (int32 K = 0; K <= Cols; ++K)
			{
				const double X = ColumnX(K), Hw = RafterWidthM * 0.5;
				auto Depth = [&](double D) {
					return D >= DC ? RafterDepthBackM
						: FMath::Lerp(RafterDepthFrontM, RafterDepthBackM, (D - Lo) / FMath::Max(DC - Lo, 0.1));
				};
				for (const TPair<double, double>& Seg : {TPair<double, double>(Lo, DC), TPair<double, double>(DC, Hi)})
				{
					if (Seg.Value - Seg.Key < 0.1) continue;
					const double A = Seg.Key, B = Seg.Value, Za = RoofZ(A) - T, Zb = RoofZ(B) - T;
					const FVector SLo[4] = {P(X - Hw, A, Za - Depth(A)), P(X + Hw, A, Za - Depth(A)), P(X + Hw, B, Zb - Depth(B)), P(X - Hw, B, Zb - Depth(B))};
					const FVector SHi[4] = {P(X - Hw, A, Za), P(X + Hw, A, Za), P(X + Hw, B, Zb), P(X - Hw, B, Zb)};
					Steel.Box(SLo, SHi, /*bWithBottom=*/true);
				}
			}
			for (double D = Lo + 1.0; D < Hi - 0.5; D += PurlinSpacingM)
			{
				const double Za = RoofZ(D) - T, Zb = Za - PurlinDepthM, Hw = PurlinWidthM * 0.5;
				const FVector PLo[4] = {P(Gs.XMin + 0.3, D - Hw, Zb), P(Gs.XMax - 0.3, D - Hw, Zb), P(Gs.XMax - 0.3, D + Hw, Zb), P(Gs.XMin + 0.3, D + Hw, Zb)};
				const FVector PHi[4] = {P(Gs.XMin + 0.3, D - Hw, Za), P(Gs.XMax - 0.3, D - Hw, Za), P(Gs.XMax - 0.3, D + Hw, Za), P(Gs.XMin + 0.3, D + Hw, Za)};
				Steel.Box(PLo, PHi, /*bWithBottom=*/true);
			}
			// Solar panels lying on the roof's slope.
			for (const FFSDSWorldPanel& Pn : Gs.RoofPanels)
			{
				const double A = FMath::Min(FMath::Abs(Pn.YMin), FMath::Abs(Pn.YMax));
				const double B = FMath::Max(FMath::Abs(Pn.YMin), FMath::Abs(Pn.YMax));
				const double Za = RoofZ(A) + PanelLiftM, Zb = RoofZ(B) + PanelLiftM, Th = PanelThicknessM;
				const FVector QLo[4] = {P(Pn.XMin, A, Za), P(Pn.XMax, A, Za), P(Pn.XMax, B, Zb), P(Pn.XMin, B, Zb)};
				const FVector QHi[4] = {P(Pn.XMin, A, Za + Th), P(Pn.XMax, A, Za + Th), P(Pn.XMax, B, Zb + Th), P(Pn.XMin, B, Zb + Th)};
				Panels.Box(QLo, QHi);
			}
		}
		// Columns just behind the rows, up to the rafters.
		const double ZRoof = RoofZ(DC) - RoofThicknessM - RafterDepthBackM;
		for (int32 K = 0; Gs.bHasRoof && K <= Cols; ++K)
		{
			Columns.Add(Place(ColumnX(K), Sgn * DC, Base, 0.0, FVector(1.0, 1.0, ZRoof - Base)));
		}
		UProceduralMeshComponent* M = NewMesh(FSDSCollision::PropProfile, true);
		Emit(M, 0, Rows, ConcreteWall, true);
		Emit(M, 1, Roof, Roofing, true);
		Emit(M, 2, Under, SteelUnder, true);
		Emit(M, 3, Steel, Roofing, true);
		Emit(NewMesh(TEXT("NoCollision"), false), 0, Panels, SolarPanel, false);
	}

	// --- props: instanced, with LODs; no collision (see the header) ----------
	AddInstances(WorldMesh(TEXT("FencePost")), Posts, 200.0, true);
	AddInstances(WorldMesh(TEXT("SignBoard")), Signs, 250.0, false);
	{
		// The open variant falls back to the closed one if it isn't imported.
		UStaticMesh* Open = WorldMesh(TEXT("GarageBayOpen"));
		if (!Open) { Garages.Append(GaragesOpen); GaragesOpen.Reset(); }
		AddInstances(WorldMesh(TEXT("GarageBay")), Garages, 600.0, true);
		AddInstances(Open, GaragesOpen, 600.0, true);
	}
	AddInstances(WorldMesh(TEXT("SeatBlock")), Seats, 350.0, false);   // under the roof's shadow
	AddInstances(WorldMesh(TEXT("Handrail")), Rails, 150.0, false);
	AddInstances(WorldMesh(TEXT("RoofColumn")), Columns, 500.0, true);
	{
		TMap<FString, TArray<FTransform>> ByMesh;
		for (const FFSDSWorldProp& Pr : World.Props)
		{
			ByMesh.FindOrAdd(Pr.Mesh).Add(Place(Pr.X, Pr.Y, G.At(Pr.X, Pr.Y), Pr.YawDeg));
		}
		for (const TPair<FString, TArray<FTransform>>& It : ByMesh)
		{
			// The gantry's thin truss throws a ragged shadow across the grid at
			// the sim's shadow quality; it is drawn without one.
			UHierarchicalInstancedStaticMeshComponent* H =
				AddInstances(WorldMesh(It.Key), It.Value, 800.0, It.Key != TEXT("StartGantry"));
			if (H && It.Key == TEXT("StartGantry"))
			{
				const int32 Slot = H->GetMaterialIndex(TEXT("LampRed"));
				if (Slot != INDEX_NONE) StartLights = H->CreateDynamicMaterialInstance(Slot);
			}
		}
	}

	// --- trees: two models per kind, scaled to each measured tree ----------
	{
		TMap<FString, TArray<FTransform>> ByMesh;
		for (const FFSDSWorldTree& T : World.Trees)
		{
			// The variant and turn come from the position, so a rebuild is identical.
			const uint32 Hash = HashCombine(GetTypeHash(FMath::RoundToInt(T.X * 10.0)), GetTypeHash(FMath::RoundToInt(T.Y * 10.0)));
			const FString Kind = T.Kind == TEXT("pine") ? TEXT("Pine") : TEXT("Broadleaf");
			const FString Mesh = FString::Printf(TEXT("Tree%s%s"), *Kind, (Hash & 1) ? TEXT("A") : TEXT("B"));
			const FVector Scale(T.R / TreeModelRadiusM, T.R / TreeModelRadiusM, T.H / TreeModelHeightM);
			ByMesh.FindOrAdd(Mesh).Add(Place(T.X, T.Y, G.At(T.X, T.Y) - 0.1, double((Hash >> 1) % 360), Scale));
		}
		for (const TPair<FString, TArray<FTransform>>& It : ByMesh)
		{
			if (AddInstances(WorldMesh(It.Key), It.Value, TreeCullM, true)) NumTrees += It.Value.Num();
		}
	}

	HideFloors();
	bBuilt = true;
	BuildSeconds = FPlatformTime::Seconds() - T0;
	UE_LOG(LogTemp, Log, TEXT("FSDS World: built %s at (%.2f, %.2f) in %.2f s: %d meshes, %d triangles; %d prop instances, %lld triangles at full detail; floor hidden: %s"),
		*World.Name, OriginCsvM.X, OriginCsvM.Y, BuildSeconds, Meshes.Num(), NumTriangles, NumInstances, PropTriangles,
		HiddenFloorNames.Num() ? *FString::Join(HiddenFloorNames, TEXT(", ")) : TEXT("none found"));
}

void AFSDSWorldActor::HideFloors()
{
	// The level's floor is a big flat WorldStatic slab around the origin; the
	// terrain replaces it. Anything at least 90 m across and at most 5 m thick
	// counts. Props, cones and the car are never WorldStatic slabs that size.
	// The ring of tyre stacks around that floor goes too: it crosses a world's
	// track (on Montmeló, 100 m past the start), and a world brings its own
	// barriers, so the car can run on until it brakes.
	for (TActorIterator<AActor> It(GetWorld()); It; ++It)
	{
		AActor* A = *It;
		if (A == this || A->IsA<AFSDSEnvironmentActor>()) continue;
		const bool bFloorRing = A->GetClass()->GetName().StartsWith(TEXT("spline_tires"));
		TInlineComponentArray<UPrimitiveComponent*> Prims(A);
		for (UPrimitiveComponent* P : Prims)
		{
			if (!P || !P->IsRegistered()) continue;
			if (!bFloorRing)
			{
				if (P->GetCollisionObjectType() != ECC_WorldStatic) continue;
				const FVector Size = P->Bounds.GetBox().GetSize();
				if (Size.X < 9000.0 || Size.Y < 9000.0 || Size.Z > 500.0) continue;
			}
			HiddenFloors.Add({P, P->IsVisible(), P->GetCollisionEnabled()});
			// The ring is hundreds of stacks: one name for the lot.
			if (!bFloorRing) HiddenFloorNames.Add(FString::Printf(TEXT("%s.%s"), *A->GetName(), *P->GetName()));
			else HiddenFloorNames.AddUnique(A->GetName());
			P->SetVisibility(false);
			P->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		}
	}
}

void AFSDSWorldActor::RestoreFloors()
{
	for (const FHiddenFloor& H : HiddenFloors)
	{
		if (UPrimitiveComponent* P = H.Component.Get())
		{
			P->SetVisibility(H.bWasVisible);
			P->SetCollisionEnabled(H.Collision);
		}
	}
	HiddenFloors.Reset();
	HiddenFloorNames.Reset();
}

void AFSDSWorldActor::Clear()
{
	RestoreFloors();
	for (UProceduralMeshComponent* M : Meshes)
	{
		if (M) M->DestroyComponent();
	}
	Meshes.Reset();
	for (UHierarchicalInstancedStaticMeshComponent* H : Instanced)
	{
		if (H) H->DestroyComponent();
	}
	Instanced.Reset();
	StartLights = nullptr;
	bStartLightsOn = false;
	bBuilt = false;
	NumTriangles = 0;
	NumInstances = 0;
	NumTrees = 0;
	PropTriangles = 0;
	BuildSeconds = 0.0;
}
