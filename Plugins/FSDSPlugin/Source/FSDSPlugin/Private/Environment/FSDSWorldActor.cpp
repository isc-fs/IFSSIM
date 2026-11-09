#include "Environment/FSDSWorldActor.h"

#include "Components/PrimitiveComponent.h"
#include "Components/SceneComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Environment/FSDSEnvironmentBuilder.h"
#include "FSDSCollision.h"
#include "HAL/PlatformTime.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
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
	constexpr double PostSizeM = 0.08;
	constexpr double TierDepthM = 0.85;   // one grandstand row
	constexpr double RoofThicknessM = 0.5;
	constexpr double ColumnSpacingM = 15.0;
	constexpr double ColumnSizeM = 0.6;

	const TCHAR* MatDir = TEXT("/Game/RaceCourse/Materials/Environment/");

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
		 *  planar in world space, one repeat every TileM metres. */
		void Quad(const FVector& A, const FVector& B, const FVector& C, const FVector& D,
			const FVector& Facing, double TileM = 4.0)
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

	/** A grid draped on the ground over Xs x Ys, Lift metres above it. */
	void Drape(FBuf& Buf, const FGround& G, const FWorldFrame& F, const TArray<double>& Xs, const TArray<double>& Ys,
		double Lift, double TileM)
	{
		const int32 Base = Buf.V.Num();
		const double S = 1.0 / (TileM * 100.0);
		for (double Y : Ys)
			for (double X : Xs)
			{
				const FVector P = F.Ue(X, Y, G.At(X, Y) + Lift);
				const int32 I = G.Cell(G.Xs, X), J = G.Cell(G.Ys, Y);
				Buf.Add(P, G.Normal(I, J), FVector2D(P.X, P.Y) * S);
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
}

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

void AFSDSWorldActor::Build(const FFSDSWorld& World, const FVector2D& OriginCsvM)
{
	Clear();
	if (!GetWorld() || World.Nx < 2 || World.Ny < 2) return;
	const double T0 = FPlatformTime::Seconds();

	const FWorldFrame F{OriginCsvM};
	const FGround G(World);

	// Materials: the repo's environment set where it fits, flat colours otherwise.
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
	UMaterialInterface* Grass = Colour(FLinearColor(0.16f, 0.17f, 0.09f));
	UMaterialInterface* Asphalt = Or(LoadMat(TEXT("Road_Inst_no_lines")), Colour(FLinearColor(0.04f, 0.04f, 0.045f)));
	UMaterialInterface* Band = Colour(FLinearColor(0.32f, 0.32f, 0.31f));
	UMaterialInterface* ConcreteGround = Or(LoadMat(TEXT("concrete_ground")), Colour(FLinearColor(0.35f, 0.34f, 0.32f)));
	UMaterialInterface* ConcreteWall = Or(LoadMat(TEXT("concrete_wall")), Colour(FLinearColor(0.45f, 0.45f, 0.43f)));
	UMaterialInterface* Metal = Or(LoadMat(TEXT("metal")), Colour(FLinearColor(0.5f, 0.5f, 0.52f)));
	UMaterialInterface* Fence = Or(LoadMat(TEXT("Chainlink")), Colour(FLinearColor(0.3f, 0.3f, 0.3f)));
	UMaterialInterface* BuildingWhite = Or(LoadMat(TEXT("Buildings_WHITE")), Colour(FLinearColor(0.7f, 0.7f, 0.68f)));
	UMaterialInterface* Glass = Or(LoadMat(TEXT("windows_Inst")), Colour(FLinearColor(0.1f, 0.12f, 0.15f)));
	UMaterialInterface* White = Colour(FLinearColor(0.8f, 0.8f, 0.8f));
	UMaterialInterface* Yellow = Colour(FLinearColor(0.8f, 0.6f, 0.05f));

	auto Emit = [&](UProceduralMeshComponent* M, int32 Section, FBuf& B, UMaterialInterface* Mat, bool bCollision) {
		if (B.T.Num() == 0) return;
		M->CreateMeshSection_LinearColor(Section, B.V, B.T, B.N, B.UV, {}, {}, bCollision);
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
						B.Add(P, G.Normal(I, J), FVector2D(P.X, P.Y) / 400.0);
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

	// --- surfaces: ground (collision) and painted bands (none) ----------------
	for (const FFSDSWorldSurface& S : World.Surfaces)
	{
		const bool bBand = S.Material == TEXT("asphalt_light");
		FBuf B;
		Drape(B, G, F, FGround::Span(G.Xs, S.XMin, S.XMax), FGround::Span(G.Ys, S.YMin, S.YMax),
			bBand ? BandLiftM : SurfaceLiftM, 4.0);
		UMaterialInterface* Mat = bBand ? Band
			: S.Material == TEXT("asphalt") ? Asphalt
			: S.Material == TEXT("concrete") ? ConcreteGround : Grass;
		Emit(NewMesh(bBand ? TEXT("NoCollision") : FSDSCollision::TerrainProfile, false),
			0, B, Mat, !bBand);
	}

	// --- lines: thin strips painted on the ground -----------------------------
	{
		FBuf WhiteBuf, YellowBuf;
		for (const FFSDSWorldLine& L : World.Lines)
		{
			const FVector2D P0(L.X0, L.Y0), P1(L.X1, L.Y1);
			const double Len = (P1 - P0).Size();
			const FVector2D Dir = (P1 - P0) / Len, Side(-Dir.Y, Dir.X);
			const int32 Steps = FMath::Max(1, FMath::CeilToInt(Len));   // follow the ground every metre
			FBuf& B = L.Colour == TEXT("yellow") ? YellowBuf : WhiteBuf;
			const int32 Base = B.V.Num();
			for (int32 K = 0; K <= Steps; ++K)
			{
				const FVector2D C = P0 + Dir * (Len * K / Steps);
				for (const double Sgn : {-0.5, 0.5})
				{
					const FVector2D Q = C + Side * (L.WidthM * Sgn);
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
	}

	// --- walls, each following the ground, with its fence ---------------------
	for (const FFSDSWorldWall& Wl : World.Walls)
	{
		const FVector2D P0(Wl.X0, Wl.Y0), P1(Wl.X1, Wl.Y1);
		const double Len = (P1 - P0).Size();
		if (Len < 0.01) continue;
		const FVector2D Dir = (P1 - P0) / Len, Side = FVector2D(-Dir.Y, Dir.X) * (Wl.ThicknessM * 0.5);
		const int32 Steps = FMath::Max(1, FMath::CeilToInt(Len / WallStepM));
		FBuf WallBuf, PostBuf, FenceBuf;
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
			const int32 Posts = FMath::Max(1, FMath::CeilToInt(Len / PostSpacingM));
			const double H = PostSizeM * 50.0;   // half size, cm
			for (int32 K = 0; K <= Posts; ++K)
			{
				const FVector2D C = Pt(Len * K / Posts);
				const double Z = G.At(C.X, C.Y);
				const FVector Bc = F.Ue(C.X, C.Y, Z + Wl.HeightM - 0.05);
				PostBuf.AlignedBox(Bc - FVector(H, H, 0), Bc + FVector(H, H, (Wl.FenceHeightM - Wl.HeightM + 0.05) * 100.0));
			}
		}
		UProceduralMeshComponent* M = NewMesh(FSDSCollision::PropProfile, true);
		Emit(M, 0, WallBuf, ConcreteWall, true);
		Emit(M, 1, PostBuf, Metal, true);
		Emit(M, 2, FenceBuf, Fence, false);   // the LiDAR sees through mesh fencing; posts and wall it hits
	}

	// --- buildings: boxes from below the ground to their roof -----------------
	for (const FFSDSWorldBuilding& Bd : World.Buildings)
	{
		double Low = TNumericLimits<double>::Max();
		for (double X : {Bd.XMin, 0.5 * (Bd.XMin + Bd.XMax), Bd.XMax})
			for (double Y : {Bd.YMin, 0.5 * (Bd.YMin + Bd.YMax), Bd.YMax})
				Low = FMath::Min(Low, G.At(X, Y));
		Low -= WallFootM;
		if (Bd.TopZ <= Low + 1.0)
		{
			UE_LOG(LogTemp, Warning, TEXT("FSDS World: a building's roof (%.2f m) is below its ground (%.2f m); skipped"), Bd.TopZ, Low);
			continue;
		}
		const FVector A = F.Ue(Bd.XMin, Bd.YMin, Low), Bb = F.Ue(Bd.XMax, Bd.YMax, Bd.TopZ);
		FBuf B;
		B.AlignedBox(A.ComponentMin(Bb), A.ComponentMax(Bb));
		Emit(NewMesh(FSDSCollision::PropProfile, true), 0, B,
			Bd.Material == TEXT("glass") ? Glass : BuildingWhite, true);
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

		FBuf Rows, Roof;
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
		}
		Rows.Quad(P(Gs.XMin, DB, Base), P(Gs.XMax, DB, Base), P(Gs.XMax, DB, Gs.BackZ), P(Gs.XMin, DB, Gs.BackZ), Out);

		// Roof slab, its top on the measured edges.
		const double RF = FMath::Abs(Gs.RoofFrontY), RB = FMath::Abs(Gs.RoofBackY);
		{
			const FVector Bot[4] = {P(Gs.XMin, RF, Gs.RoofFrontZ - RoofThicknessM), P(Gs.XMax, RF, Gs.RoofFrontZ - RoofThicknessM),
			                        P(Gs.XMax, RB, Gs.RoofBackZ - RoofThicknessM), P(Gs.XMin, RB, Gs.RoofBackZ - RoofThicknessM)};
			const FVector Top[4] = {P(Gs.XMin, RF, Gs.RoofFrontZ), P(Gs.XMax, RF, Gs.RoofFrontZ),
			                        P(Gs.XMax, RB, Gs.RoofBackZ), P(Gs.XMin, RB, Gs.RoofBackZ)};
			Roof.Box(Bot, Top, /*bWithBottom=*/true);
		}
		// Columns just behind the rows, up to the roof's underside.
		const double DC = FMath::Min(DB + ColumnSizeM, FMath::Max(RB - ColumnSizeM * 0.5, DB));
		const double ZRoof = FMath::Lerp(Gs.RoofFrontZ, Gs.RoofBackZ, RB > RF ? (DC - RF) / (RB - RF) : 1.0) - RoofThicknessM;
		const int32 Cols = FMath::Max(1, FMath::RoundToInt((Gs.XMax - Gs.XMin) / ColumnSpacingM));
		const double Hc = ColumnSizeM * 50.0;
		for (int32 K = 0; K <= Cols; ++K)
		{
			const double X = FMath::Lerp(Gs.XMin + ColumnSizeM, Gs.XMax - ColumnSizeM, double(K) / Cols);
			const FVector C0 = P(X, DC, Base), C1 = P(X, DC, ZRoof);
			Roof.AlignedBox(C0 - FVector(Hc, Hc, 0), C1 + FVector(Hc, Hc, 0));
		}
		UProceduralMeshComponent* M = NewMesh(FSDSCollision::PropProfile, true);
		Emit(M, 0, Rows, ConcreteWall, true);
		Emit(M, 1, Roof, Metal, true);
	}

	HideFloors();
	bBuilt = true;
	BuildSeconds = FPlatformTime::Seconds() - T0;
	UE_LOG(LogTemp, Log, TEXT("FSDS World: built %s at (%.2f, %.2f) in %.2f s: %d meshes, %d triangles; floor hidden: %s"),
		*World.Name, OriginCsvM.X, OriginCsvM.Y, BuildSeconds, Meshes.Num(), NumTriangles,
		HiddenFloorNames.Num() ? *FString::Join(HiddenFloorNames, TEXT(", ")) : TEXT("none found"));
}

void AFSDSWorldActor::HideFloors()
{
	// The level's floor is a big flat WorldStatic slab around the origin; the
	// terrain replaces it. Anything at least 90 m across and at most 5 m thick
	// counts. Props, cones and the car are never WorldStatic slabs that size.
	for (TActorIterator<AActor> It(GetWorld()); It; ++It)
	{
		AActor* A = *It;
		if (A == this || A->IsA<AFSDSEnvironmentActor>()) continue;
		TInlineComponentArray<UPrimitiveComponent*> Prims(A);
		for (UPrimitiveComponent* P : Prims)
		{
			if (!P || !P->IsRegistered() || P->GetCollisionObjectType() != ECC_WorldStatic) continue;
			const FVector Size = P->Bounds.GetBox().GetSize();
			if (Size.X < 9000.0 || Size.Y < 9000.0 || Size.Z > 500.0) continue;
			HiddenFloors.Add({P, P->IsVisible(), P->GetCollisionEnabled()});
			HiddenFloorNames.Add(FString::Printf(TEXT("%s.%s"), *A->GetName(), *P->GetName()));
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
	bBuilt = false;
	NumTriangles = 0;
	BuildSeconds = 0.0;
}
