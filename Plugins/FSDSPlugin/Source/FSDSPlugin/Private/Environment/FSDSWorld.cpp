#include "Environment/FSDSWorld.h"

#include "Dom/JsonObject.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

// Mirrors tools/worldgen/world.py: the two must accept and reject the same files.

namespace
{
	using FObj = FJsonObject;

	struct FReader
	{
		FString& Error;

		bool Fail(const FString& Msg) { Error = Msg; return false; }

		bool Keys(const FObj& O, const FString& Where, std::initializer_list<const TCHAR*> Required,
			std::initializer_list<const TCHAR*> Optional = {})
		{
			for (const auto& Pair : O.Values)
			{
				bool bKnown = false;
				for (const TCHAR* K : Required) bKnown |= (Pair.Key == K);
				for (const TCHAR* K : Optional) bKnown |= (Pair.Key == K);
				if (!bKnown) return Fail(FString::Printf(TEXT("%s: unknown key '%s'"), *Where, *Pair.Key));
			}
			for (const TCHAR* K : Required)
			{
				if (!O.HasField(K)) return Fail(FString::Printf(TEXT("%s: missing '%s'"), *Where, K));
			}
			return true;
		}

		bool Num(const FObj& O, const TCHAR* Key, const FString& Where, double& Out,
			double Lo = -FSDSWorld::MaxCoordM, double Hi = FSDSWorld::MaxCoordM)
		{
			const TSharedPtr<FJsonValue> V = O.TryGetField(Key);
			if (!V.IsValid() || V->Type != EJson::Number)
				return Fail(FString::Printf(TEXT("%s.%s: expected a number"), *Where, Key));
			Out = V->AsNumber();
			if (!FMath::IsFinite(Out) || Out < Lo || Out > Hi)
				return Fail(FString::Printf(TEXT("%s.%s: %g is outside [%g, %g]"), *Where, Key, Out, Lo, Hi));
			return true;
		}

		bool Int(const FObj& O, const TCHAR* Key, const FString& Where, int32& Out, int32 Lo, int32 Hi)
		{
			double D = 0;
			if (!Num(O, Key, Where, D, Lo, Hi)) return false;
			if (D != FMath::FloorToDouble(D)) return Fail(FString::Printf(TEXT("%s.%s: expected an integer"), *Where, Key));
			Out = (int32)D;
			return true;
		}

		bool Str(const FObj& O, const TCHAR* Key, const FString& Where, FString& Out,
			std::initializer_list<const TCHAR*> Choices = {})
		{
			const TSharedPtr<FJsonValue> V = O.TryGetField(Key);
			if (!V.IsValid() || V->Type != EJson::String || V->AsString().IsEmpty())
				return Fail(FString::Printf(TEXT("%s.%s: expected a non-empty string"), *Where, Key));
			Out = V->AsString();
			if (Choices.size() == 0) return true;
			for (const TCHAR* C : Choices) if (Out == C) return true;
			return Fail(FString::Printf(TEXT("%s.%s: '%s' is not allowed"), *Where, Key, *Out));
		}

		bool Obj(const FObj& O, const TCHAR* Key, const FString& Where, const TSharedPtr<FObj>*& Out)
		{
			if (!O.TryGetObjectField(Key, Out)) return Fail(FString::Printf(TEXT("%s.%s: expected an object"), *Where, Key));
			return true;
		}

		/** Each element of an optional array of objects. */
		template <typename F>
		bool Each(const FObj& O, const TCHAR* Key, F&& Fn)
		{
			if (!O.HasField(Key)) return true;
			const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
			if (!O.TryGetArrayField(Key, Arr)) return Fail(FString::Printf(TEXT("world.%s: expected an array"), Key));
			for (int32 i = 0; i < Arr->Num(); ++i)
			{
				const FString Where = FString::Printf(TEXT("world.%s[%d]"), Key, i);
				const TSharedPtr<FObj>* E = nullptr;
				if (!(*Arr)[i].IsValid() || !(*Arr)[i]->TryGetObject(E)) return Fail(Where + TEXT(": expected an object"));
				if (!Fn(**E, Where)) return false;
			}
			return true;
		}

		/** Roof panels: a list of boxes inside [X0, X1] and, in |y|, inside [D0, D1]. */
		bool Panels(const FObj& O, const FString& Where, double X0, double X1, double D0, double D1,
			TArray<FFSDSWorldPanel>& Out)
		{
			const TArray<TSharedPtr<FJsonValue>>* Arr = nullptr;
			const FString W = Where + TEXT(".roof_panels");
			if (!O.TryGetArrayField(TEXT("roof_panels"), Arr) || Arr->Num() == 0 || Arr->Num() > FSDSWorld::MaxRoofPanels)
				return Fail(FString::Printf(TEXT("%s: expected a list of 1 to %d boxes"), *W, FSDSWorld::MaxRoofPanels));
			for (int32 i = 0; i < Arr->Num(); ++i)
			{
				const FString Wi = FString::Printf(TEXT("%s[%d]"), *W, i);
				const TSharedPtr<FObj>* E = nullptr;
				if (!(*Arr)[i].IsValid() || !(*Arr)[i]->TryGetObject(E)) return Fail(Wi + TEXT(": expected an object"));
				FFSDSWorldPanel P;
				if (!Keys(**E, Wi, { TEXT("x_min"), TEXT("x_max"), TEXT("y_min"), TEXT("y_max") }) ||
					!Box(**E, Wi, P.XMin, P.XMax, P.YMin, P.YMax)) return false;
				const double A = FMath::Min(FMath::Abs(P.YMin), FMath::Abs(P.YMax));
				const double B = FMath::Max(FMath::Abs(P.YMin), FMath::Abs(P.YMax));
				if ((P.YMin < 0.0 && 0.0 < P.YMax) || P.XMin < X0 || P.XMax > X1 || A < D0 - 1e-6 || B > D1 + 1e-6)
					return Fail(Wi + TEXT(": outside the roof"));
				Out.Add(P);
			}
			return true;
		}

		bool Box(const FObj& O, const FString& Where, double& XMin, double& XMax, double& YMin, double& YMax)
		{
			if (!Num(O, TEXT("x_min"), Where, XMin) || !Num(O, TEXT("x_max"), Where, XMax) ||
				!Num(O, TEXT("y_min"), Where, YMin) || !Num(O, TEXT("y_max"), Where, YMax)) return false;
			if (XMin >= XMax) return Fail(Where + TEXT(": x_min must be below x_max"));
			if (YMin >= YMax) return Fail(Where + TEXT(": y_min must be below y_max"));
			return true;
		}
	};
}

bool FSDSWorld::IsValidName(const FString& Name)
{
	if (Name.IsEmpty()) return false;
	for (TCHAR C : Name)
	{
		if (!FChar::IsAlnum(C) && C != TEXT('_')) return false;
	}
	return true;
}

FString FSDSWorld::DirFor(const FString& TrackCsvPath, const FString& Name)
{
	return FPaths::Combine(FPaths::GetPath(TrackCsvPath), TEXT("worlds"), Name);
}

double FFSDSWorld::HeightAt(double X, double Y) const
{
	if (Nx < 2 || Ny < 2 || Heights.Num() != Nx * Ny) return 0.0;
	const double Fx = FMath::Clamp((X - TerrainX0) / CellM, 0.0, Nx - 1.0);
	const double Fy = FMath::Clamp((Y - TerrainY0) / CellM, 0.0, Ny - 1.0);
	const int32 I0 = FMath::Min((int32)Fx, Nx - 2);
	const int32 J0 = FMath::Min((int32)Fy, Ny - 2);
	const double Ax = Fx - I0, Ay = Fy - J0;
	auto H = [&](int32 I, int32 J) { return Heights[J * Nx + I] * ZUnitM; };
	return (1 - Ax) * (1 - Ay) * H(I0, J0) + Ax * (1 - Ay) * H(I0 + 1, J0)
		+ (1 - Ax) * Ay * H(I0, J0 + 1) + Ax * Ay * H(I0 + 1, J0 + 1);
}

bool FFSDSWorld::WearCovers(double XMin, double XMax, double YMin, double YMax) const
{
	return bHasWear && Wear.Num() == WearNx * WearNy &&
		XMin >= WearX0 - 1e-6 && XMax <= WearX0 + (WearNx - 1) * WearDx + 1e-6 &&
		YMin >= WearY0 - 1e-6 && YMax <= WearY0 + (WearNy - 1) * WearDy + 1e-6;
}

double FFSDSWorld::WearAt(double X, double Y) const
{
	if (!bHasWear || Wear.Num() != WearNx * WearNy) return 1.0;
	const double Fx = (X - WearX0) / WearDx, Fy = (Y - WearY0) / WearDy;
	if (Fx < -1e-6 || Fy < -1e-6 || Fx > WearNx - 1 + 1e-6 || Fy > WearNy - 1 + 1e-6) return 1.0;
	const double Cx = FMath::Clamp(Fx, 0.0, WearNx - 1.0), Cy = FMath::Clamp(Fy, 0.0, WearNy - 1.0);
	const int32 I0 = FMath::Min((int32)Cx, WearNx - 2), J0 = FMath::Min((int32)Cy, WearNy - 2);
	const double Ax = Cx - I0, Ay = Cy - J0;
	auto V = [&](int32 I, int32 J) { return Wear[J * WearNx + I] / 128.0; };
	return (1 - Ax) * (1 - Ay) * V(I0, J0) + Ax * (1 - Ay) * V(I0 + 1, J0)
		+ (1 - Ax) * Ay * V(I0, J0 + 1) + Ax * Ay * V(I0 + 1, J0 + 1);
}

bool FSDSWorld::Parse(const FString& Json, FFSDSWorld& Out, FString& OutError)
{
	Out = FFSDSWorld();
	FFSDSWorld W;
	FReader R{ OutError };

	TSharedPtr<FObj> Root;
	TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
	if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid()) return R.Fail(TEXT("not a JSON object"));
	const FString WR = TEXT("world");
	if (!R.Keys(*Root, WR, { TEXT("format"), TEXT("name"), TEXT("title"), TEXT("attribution"), TEXT("frame"), TEXT("terrain") },
		{ TEXT("detail"), TEXT("wear"), TEXT("surfaces"), TEXT("lines"), TEXT("walls"), TEXT("buildings"), TEXT("grandstands"), TEXT("props"), TEXT("trees") })) return false;

	FString Format;
	if (!R.Str(*Root, TEXT("format"), WR, Format)) return false;
	if (Format != FormatV1) return R.Fail(FString::Printf(TEXT("world.format: expected \"%s\""), FormatV1));
	if (!R.Str(*Root, TEXT("name"), WR, W.Name) || !R.Str(*Root, TEXT("title"), WR, W.Title)) return false;

	const TArray<TSharedPtr<FJsonValue>>* Attr = nullptr;
	if (!Root->TryGetArrayField(TEXT("attribution"), Attr) || Attr->Num() == 0)
		return R.Fail(TEXT("world.attribution: list every data source and its licence"));
	for (const TSharedPtr<FJsonValue>& A : *Attr)
	{
		if (!A.IsValid() || A->Type != EJson::String || A->AsString().IsEmpty())
			return R.Fail(TEXT("world.attribution: every entry must be a non-empty string"));
		W.Attribution.Add(A->AsString());
	}

	const TSharedPtr<FObj>* Frame = nullptr;
	if (!R.Obj(*Root, TEXT("frame"), WR, Frame)) return false;
	{
		const FString Where = TEXT("world.frame");
		FString Crs; double D = 0;
		if (!R.Keys(**Frame, Where, { TEXT("crs"), TEXT("origin_e"), TEXT("origin_n"), TEXT("origin_z"), TEXT("x_bearing_deg") }) ||
			!R.Str(**Frame, TEXT("crs"), Where, Crs) ||
			!R.Num(**Frame, TEXT("origin_e"), Where, D, -1e7, 1e7) || !R.Num(**Frame, TEXT("origin_n"), Where, D, -1e7, 1e7) ||
			!R.Num(**Frame, TEXT("origin_z"), Where, D, -1e7, 1e7) || !R.Num(**Frame, TEXT("x_bearing_deg"), Where, W.XBearingDeg, 0.0, 360.0))
			return false;
	}

	const TSharedPtr<FObj>* Terrain = nullptr;
	if (!R.Obj(*Root, TEXT("terrain"), WR, Terrain)) return false;
	{
		const FString Where = TEXT("world.terrain");
		FString File;
		if (!R.Keys(**Terrain, Where, { TEXT("file"), TEXT("x0"), TEXT("y0"), TEXT("cell_m"), TEXT("nx"), TEXT("ny"), TEXT("z_unit_m") }) ||
			!R.Str(**Terrain, TEXT("file"), Where, File)) return false;
		if (File != TerrainFile) return R.Fail(FString::Printf(TEXT("world.terrain.file: expected \"%s\""), TerrainFile));
		if (!R.Num(**Terrain, TEXT("x0"), Where, W.TerrainX0) || !R.Num(**Terrain, TEXT("y0"), Where, W.TerrainY0) ||
			!R.Num(**Terrain, TEXT("cell_m"), Where, W.CellM, 0.1, 10.0) ||
			!R.Int(**Terrain, TEXT("nx"), Where, W.Nx, 2, 20000) || !R.Int(**Terrain, TEXT("ny"), Where, W.Ny, 2, 20000) ||
			!R.Num(**Terrain, TEXT("z_unit_m"), Where, W.ZUnitM, 1e-4, 1.0)) return false;
		if ((int64)W.Nx * W.Ny > MaxTerrainCells)
			return R.Fail(FString::Printf(TEXT("world.terrain: %dx%d is over %d cells"), W.Nx, W.Ny, MaxTerrainCells));
	}

	if (Root->HasField(TEXT("detail")))
	{
		const TSharedPtr<FObj>* Detail = nullptr;
		const FString Where = TEXT("world.detail");
		if (!R.Obj(*Root, TEXT("detail"), WR, Detail) || !R.Keys(**Detail, Where, { TEXT("x_min"), TEXT("x_max") }) ||
			!R.Num(**Detail, TEXT("x_min"), Where, W.DetailXMin) || !R.Num(**Detail, TEXT("x_max"), Where, W.DetailXMax)) return false;
		if (W.DetailXMin >= W.DetailXMax) return R.Fail(TEXT("world.detail: x_min must be below x_max"));
		W.bHasDetail = true;
	}

	if (Root->HasField(TEXT("wear")))
	{
		const TSharedPtr<FObj>* Wear = nullptr;
		const FString Where = TEXT("world.wear");
		FString File;
		if (!R.Obj(*Root, TEXT("wear"), WR, Wear) ||
			!R.Keys(**Wear, Where, { TEXT("file"), TEXT("x0"), TEXT("y0"), TEXT("dx_m"), TEXT("dy_m"), TEXT("nx"), TEXT("ny") }) ||
			!R.Str(**Wear, TEXT("file"), Where, File)) return false;
		if (File != WearFile) return R.Fail(FString::Printf(TEXT("world.wear.file: expected \"%s\""), WearFile));
		if (!R.Num(**Wear, TEXT("x0"), Where, W.WearX0) || !R.Num(**Wear, TEXT("y0"), Where, W.WearY0) ||
			!R.Num(**Wear, TEXT("dx_m"), Where, W.WearDx, 0.05, 10.0) || !R.Num(**Wear, TEXT("dy_m"), Where, W.WearDy, 0.05, 10.0) ||
			!R.Int(**Wear, TEXT("nx"), Where, W.WearNx, 2, 20000) || !R.Int(**Wear, TEXT("ny"), Where, W.WearNy, 2, 20000)) return false;
		if ((int64)W.WearNx * W.WearNy > MaxTerrainCells)
			return R.Fail(FString::Printf(TEXT("world.wear: %dx%d is over %d cells"), W.WearNx, W.WearNy, MaxTerrainCells));
		W.bHasWear = true;
	}

	const bool bOk =
		R.Each(*Root, TEXT("surfaces"), [&](const FObj& E, const FString& Where) {
			FFSDSWorldSurface S;
			if (!R.Keys(E, Where, { TEXT("material"), TEXT("x_min"), TEXT("x_max"), TEXT("y_min"), TEXT("y_max") }) ||
				!R.Str(E, TEXT("material"), Where, S.Material, { TEXT("asphalt"), TEXT("asphalt_light"), TEXT("concrete"), TEXT("grass") }) ||
				!R.Box(E, Where, S.XMin, S.XMax, S.YMin, S.YMax)) return false;
			W.Surfaces.Add(S);
			return true;
		}) &&
		R.Each(*Root, TEXT("lines"), [&](const FObj& E, const FString& Where) {
			FFSDSWorldLine L;
			if (!R.Keys(E, Where, { TEXT("x0"), TEXT("y0"), TEXT("x1"), TEXT("y1"), TEXT("width_m"), TEXT("colour") }, { TEXT("end_width_m") }) ||
				!R.Num(E, TEXT("x0"), Where, L.X0) || !R.Num(E, TEXT("y0"), Where, L.Y0) ||
				!R.Num(E, TEXT("x1"), Where, L.X1) || !R.Num(E, TEXT("y1"), Where, L.Y1) ||
				!R.Num(E, TEXT("width_m"), Where, L.WidthM, 0.02, MaxLineWidthM) ||
				!R.Str(E, TEXT("colour"), Where, L.Colour, { TEXT("white"), TEXT("yellow"), TEXT("red"), TEXT("green") })) return false;
			L.EndWidthM = L.WidthM;
			if (E.HasField(TEXT("end_width_m")) && !R.Num(E, TEXT("end_width_m"), Where, L.EndWidthM, 0.02, MaxLineWidthM)) return false;
			if (L.X0 == L.X1 && L.Y0 == L.Y1) return R.Fail(Where + TEXT(": the two ends are the same point"));
			W.Lines.Add(L);
			return true;
		}) &&
		R.Each(*Root, TEXT("walls"), [&](const FObj& E, const FString& Where) {
			FFSDSWorldWall L;
			if (!R.Keys(E, Where, { TEXT("material"), TEXT("x0"), TEXT("y0"), TEXT("x1"), TEXT("y1"), TEXT("height_m"), TEXT("thickness_m") },
					{ TEXT("fence_height_m") }) ||
				!R.Str(E, TEXT("material"), Where, L.Material, { TEXT("concrete") }) ||
				!R.Num(E, TEXT("x0"), Where, L.X0) || !R.Num(E, TEXT("y0"), Where, L.Y0) ||
				!R.Num(E, TEXT("x1"), Where, L.X1) || !R.Num(E, TEXT("y1"), Where, L.Y1) ||
				!R.Num(E, TEXT("height_m"), Where, L.HeightM, 0.1, 20.0) ||
				!R.Num(E, TEXT("thickness_m"), Where, L.ThicknessM, 0.05, 5.0)) return false;
			if (E.HasField(TEXT("fence_height_m")))
			{
				if (!R.Num(E, TEXT("fence_height_m"), Where, L.FenceHeightM, 0.0, 20.0)) return false;
				if (L.FenceHeightM <= L.HeightM) return R.Fail(Where + TEXT(": fence_height_m must be above height_m"));
			}
			W.Walls.Add(L);
			return true;
		}) &&
		R.Each(*Root, TEXT("buildings"), [&](const FObj& E, const FString& Where) {
			FFSDSWorldBuilding B;
			if (!R.Keys(E, Where, { TEXT("material"), TEXT("x_min"), TEXT("x_max"), TEXT("y_min"), TEXT("y_max"), TEXT("top_z") },
					{ TEXT("yaw_deg"), TEXT("base_z"), TEXT("garages"), TEXT("canopy_m"), TEXT("roof_panels") }) ||
				!R.Str(E, TEXT("material"), Where, B.Material, { TEXT("concrete"), TEXT("glass") }) ||
				!R.Box(E, Where, B.XMin, B.XMax, B.YMin, B.YMax) ||
				!R.Num(E, TEXT("top_z"), Where, B.TopZ, -50.0, 200.0)) return false;
			if (E.HasField(TEXT("yaw_deg")) && !R.Num(E, TEXT("yaw_deg"), Where, B.YawDeg, -360.0, 360.0)) return false;
			if (E.HasField(TEXT("base_z")))
			{
				if (!R.Num(E, TEXT("base_z"), Where, B.BaseZ, -50.0, 200.0)) return false;
				if (B.BaseZ >= B.TopZ) return R.Fail(Where + TEXT(": base_z must be below top_z"));
				B.bHasBase = true;
			}
			if (E.HasField(TEXT("garages")))
			{
				const TSharedPtr<FJsonValue> V = E.TryGetField(TEXT("garages"));
				if (V->Type != EJson::Boolean) return R.Fail(Where + TEXT(".garages: expected true or false"));
				B.bGarages = V->AsBool();
			}
			if (E.HasField(TEXT("canopy_m")))
			{
				if (!R.Num(E, TEXT("canopy_m"), Where, B.CanopyM, 0.5, 10.0)) return false;
				if (!B.bGarages || B.YawDeg != 0.0) return R.Fail(Where + TEXT(": a canopy shades garages, on a building with yaw_deg 0"));
			}
			if (E.HasField(TEXT("roof_panels")))
			{
				if (B.YawDeg != 0.0 || B.bHasBase) return R.Fail(Where + TEXT(": roof panels need a building with yaw_deg 0 and no base_z"));
				if (B.YMin < 0.0 && 0.0 < B.YMax) return R.Fail(Where + TEXT(": roof panels need a building on one side of the axis"));
				const double Near = FMath::Min(FMath::Abs(B.YMin), FMath::Abs(B.YMax)), Far = FMath::Max(FMath::Abs(B.YMin), FMath::Abs(B.YMax));
				if (!R.Panels(E, Where, B.XMin, B.XMax, Near, Far, B.RoofPanels)) return false;
			}
			W.Buildings.Add(B);
			return true;
		}) &&
		R.Each(*Root, TEXT("grandstands"), [&](const FObj& E, const FString& Where) {
			FFSDSWorldGrandstand G;
			static const TCHAR* RoofKeys[] = { TEXT("roof_front_y"), TEXT("roof_back_y"), TEXT("roof_front_z"), TEXT("roof_back_z") };
			if (!R.Keys(E, Where, { TEXT("x_min"), TEXT("x_max"), TEXT("front_y"), TEXT("back_y"), TEXT("front_z"), TEXT("back_z") },
					{ RoofKeys[0], RoofKeys[1], RoofKeys[2], RoofKeys[3], TEXT("roof_panels") }) ||
				!R.Num(E, TEXT("x_min"), Where, G.XMin) || !R.Num(E, TEXT("x_max"), Where, G.XMax) ||
				!R.Num(E, TEXT("front_y"), Where, G.FrontY) || !R.Num(E, TEXT("back_y"), Where, G.BackY) ||
				!R.Num(E, TEXT("front_z"), Where, G.FrontZ, -50.0, 200.0) || !R.Num(E, TEXT("back_z"), Where, G.BackZ, -50.0, 200.0)) return false;
			int32 NRoof = 0;
			for (const TCHAR* K : RoofKeys) NRoof += E.HasField(K) ? 1 : 0;
			if (NRoof != 0 && NRoof != 4) return R.Fail(Where + TEXT(": a roof needs all of roof_front_y, roof_back_y, roof_front_z, roof_back_z, or none of them"));
			G.bHasRoof = NRoof == 4;
			if (G.bHasRoof &&
				(!R.Num(E, TEXT("roof_front_y"), Where, G.RoofFrontY) || !R.Num(E, TEXT("roof_back_y"), Where, G.RoofBackY) ||
				 !R.Num(E, TEXT("roof_front_z"), Where, G.RoofFrontZ, -50.0, 200.0) ||
				 !R.Num(E, TEXT("roof_back_z"), Where, G.RoofBackZ, -50.0, 200.0))) return false;
			if (G.XMin >= G.XMax) return R.Fail(Where + TEXT(": x_min must be below x_max"));
			if (FMath::Abs(G.BackY) <= FMath::Abs(G.FrontY)) return R.Fail(Where + TEXT(": back_y must be further from the track (|y|) than front_y"));
			if (G.BackZ <= G.FrontZ) return R.Fail(Where + TEXT(": the seating must rise from front to back"));
			if (E.HasField(TEXT("roof_panels")))
			{
				if (!G.bHasRoof) return R.Fail(Where + TEXT(": roof panels need a roof"));
				if (G.RoofFrontY * G.RoofBackY <= 0.0) return R.Fail(Where + TEXT(": roof panels need a roof on one side of the axis"));
				const double Near = FMath::Min(FMath::Abs(G.RoofFrontY), FMath::Abs(G.RoofBackY));
				const double Far = FMath::Max(FMath::Abs(G.RoofFrontY), FMath::Abs(G.RoofBackY));
				if (!R.Panels(E, Where, G.XMin, G.XMax, Near, Far, G.RoofPanels)) return false;
			}
			W.Grandstands.Add(G);
			return true;
		}) &&
		R.Each(*Root, TEXT("props"), [&](const FObj& E, const FString& Where) {
			FFSDSWorldProp P;
			if (!R.Keys(E, Where, { TEXT("mesh"), TEXT("x"), TEXT("y") }, { TEXT("yaw_deg") }) ||
				!R.Str(E, TEXT("mesh"), Where, P.Mesh) ||
				!R.Num(E, TEXT("x"), Where, P.X) || !R.Num(E, TEXT("y"), Where, P.Y)) return false;
			bool bKnown = false;
			for (const TCHAR* M : PropMeshes) bKnown |= P.Mesh == M;
			if (!bKnown) return R.Fail(FString::Printf(TEXT("%s.mesh: '%s' is not allowed"), *Where, *P.Mesh));
			if (E.HasField(TEXT("yaw_deg")) && !R.Num(E, TEXT("yaw_deg"), Where, P.YawDeg, -360.0, 360.0)) return false;
			W.Props.Add(P);
			return true;
		}) &&
		R.Each(*Root, TEXT("trees"), [&](const FObj& E, const FString& Where) {
			if (W.Trees.Num() >= MaxTrees) return R.Fail(FString::Printf(TEXT("world.trees: more than %d"), MaxTrees));
			FFSDSWorldTree T;
			if (!R.Keys(E, Where, { TEXT("x"), TEXT("y"), TEXT("h"), TEXT("r"), TEXT("kind") }) ||
				!R.Num(E, TEXT("x"), Where, T.X) || !R.Num(E, TEXT("y"), Where, T.Y) ||
				!R.Num(E, TEXT("h"), Where, T.H, 1.0, 60.0) || !R.Num(E, TEXT("r"), Where, T.R, 0.3, 20.0) ||
				!R.Str(E, TEXT("kind"), Where, T.Kind, { TreeKinds[0], TreeKinds[1] })) return false;
			W.Trees.Add(T);
			return true;
		});
	if (!bOk) return false;

	Out = MoveTemp(W);
	return true;
}

bool FSDSWorld::Load(const FString& Dir, FFSDSWorld& Out, FString& OutError)
{
	Out = FFSDSWorld();
	FString Text;
	if (!FFileHelper::LoadFileToString(Text, *FPaths::Combine(Dir, TEXT("world.json"))))
	{
		OutError = FString::Printf(TEXT("cannot read %s/world.json"), *Dir);
		return false;
	}
	FFSDSWorld W;
	if (!Parse(Text, W, OutError)) return false;

	TArray<uint8> Raw;
	const FString TerrainPath = FPaths::Combine(Dir, TerrainFile);
	if (!FFileHelper::LoadFileToArray(Raw, *TerrainPath))
	{
		OutError = FString::Printf(TEXT("cannot read %s"), *TerrainPath);
		return false;
	}
	const int64 N = (int64)W.Nx * W.Ny;
	if (Raw.Num() != 2 * N)
	{
		OutError = FString::Printf(TEXT("%s: %d bytes, expected %lld for %dx%d int16"), TerrainFile, Raw.Num(), 2 * N, W.Nx, W.Ny);
		return false;
	}
	W.Heights.SetNumUninitialized(N);
	for (int64 i = 0; i < N; ++i)
	{
		// Little-endian on disk, whatever the host.
		W.Heights[i] = (int16)(uint16(Raw[2 * i]) | (uint16(Raw[2 * i + 1]) << 8));
	}
	if (W.bHasWear)
	{
		const FString WearPath = FPaths::Combine(Dir, WearFile);
		if (!FFileHelper::LoadFileToArray(W.Wear, *WearPath))
		{
			OutError = FString::Printf(TEXT("cannot read %s"), *WearPath);
			return false;
		}
		if (W.Wear.Num() != W.WearNx * W.WearNy)
		{
			OutError = FString::Printf(TEXT("%s: %d bytes, expected %d for %dx%d uint8"), WearFile, W.Wear.Num(),
				W.WearNx * W.WearNy, W.WearNx, W.WearNy);
			return false;
		}
	}
	W.Dir = Dir;
	Out = MoveTemp(W);
	return true;
}
