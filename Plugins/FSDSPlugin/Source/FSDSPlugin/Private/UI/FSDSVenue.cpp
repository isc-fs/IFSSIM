#include "UI/FSDSVenue.h"

#include "UI/SFSDSTrackMap.h"

#include "Dom/JsonObject.h"
#include "HAL/FileManager.h"
#include "Layout/Geometry.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Rendering/DrawElements.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

// Qualified everywhere (VenueDraw::...): other UI files share the unity blob.
namespace VenueDraw
{
	const FLinearColor LotEdge = FLinearColor::FromSRGBColor(FColor(120, 120, 128)).CopyWithNewOpacity(0.7f);
	const FLinearColor Sidewalk = FLinearColor::FromSRGBColor(FColor(90, 90, 96));
	const FLinearColor Tree = FLinearColor::FromSRGBColor(FColor(34, 120, 60)).CopyWithNewOpacity(0.85f);
	const FLinearColor Car = FLinearColor::FromSRGBColor(FColor(150, 160, 175));
	const FLinearColor Lamp = FLinearColor::FromSRGBColor(FColor(230, 220, 150));
	const FLinearColor Bench = FLinearColor::FromSRGBColor(FColor(160, 110, 60));
	const FLinearColor Berm = FLinearColor::FromSRGBColor(FColor(140, 105, 60));
	const FLinearColor Small = FLinearColor::FromSRGBColor(FColor(170, 170, 180));

	/** The footprints in tools/envgen/catalogue.py: a box, or a circle. */
	struct FShape { float LengthM = 0.f, WidthM = 0.f, RadiusM = 0.f; };

	bool ShapeOf(const FString& Class, FShape& Out)
	{
		struct FEntry { const TCHAR* Name; FShape Shape; };
		static const FEntry Table[] = {
			{ TEXT("bollard"),    { 0.f, 0.f, 0.10f } },
			{ TEXT("bin"),        { 0.f, 0.f, 0.25f } },
			{ TEXT("tripod"),     { 0.f, 0.f, 0.50f } },
			{ TEXT("tyre_wall"),  { 2.4f, 0.6f, 0.f } },
			{ TEXT("cone_stack"), { 0.f, 0.f, 0.12f } },
			{ TEXT("tree"),       { 0.f, 0.f, 1.50f } },
			{ TEXT("lamp_post"),  { 0.f, 0.f, 0.15f } },   // the pole; its arm is drawn as a line
			{ TEXT("parked_car"), { 4.4f, 1.8f, 0.f } },
			{ TEXT("bench"),      { 1.8f, 0.5f, 0.f } },
			{ TEXT("sidewalk"),   { 2.0f, 2.0f, 0.f } },
			{ TEXT("berm"),       { 10.0f, 5.9f, 0.f } },
		};
		for (const FEntry& E : Table)
		{
			if (Class == E.Name) { Out = E.Shape; return true; }
		}
		return false;
	}

	FLinearColor ColorOf(const FString& Class)
	{
		if (Class == TEXT("sidewalk")) return Sidewalk;
		if (Class == TEXT("tree")) return Tree;
		if (Class == TEXT("parked_car")) return Car;
		if (Class == TEXT("lamp_post")) return Lamp;
		if (Class == TEXT("bench")) return Bench;
		if (Class == TEXT("berm")) return Berm;
		return Small;
	}

	bool ReadXY(const TSharedPtr<FJsonValue>& V, FVector2D& Out)
	{
		const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
		if (!V.IsValid() || !V->TryGetArray(A) || A->Num() != 2) return false;
		double X = 0, Y = 0;
		if (!(*A)[0]->TryGetNumber(X) || !(*A)[1]->TryGetNumber(Y)) return false;
		Out = FVector2D(X, Y);
		return true;
	}
}

namespace FSDSVenue
{
	FBox2D FVenue::Bounds() const
	{
		FBox2D B(ForceInit);
		for (const FVector2D& P : Lot) B += P;
		for (const FProp& P : Props)
		{
			VenueDraw::FShape S;
			const float R = VenueDraw::ShapeOf(P.Class, S) ? FMath::Max(S.RadiusM, 0.5f * FMath::Max(S.LengthM, S.WidthM)) : 0.5f;
			B += P.Pos - FVector2D(R, R);
			B += P.Pos + FVector2D(R, R);
		}
		return B;
	}

	bool LoadFile(const FString& Path, FVenue& Out)
	{
		FString Text;
		TSharedPtr<FJsonObject> Obj;
		if (!FFileHelper::LoadFileToString(Text, *Path)
			|| !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Obj) || !Obj.IsValid())
		{
			return false;
		}
		FString Format;
		if (!Obj->TryGetStringField(TEXT("format"), Format) || Format != TEXT("ifssim-venue/1")) return false;

		Out = FVenue();
		if (!Obj->TryGetStringField(TEXT("name"), Out.Name) || Out.Name.IsEmpty() || Out.Name.Contains(Sep)) return false;
		if (!Obj->TryGetStringField(TEXT("title"), Out.Title)) Out.Title = Out.Name;
		double Margin = 2.0;
		if (Obj->TryGetNumberField(TEXT("margin_m"), Margin)) Out.MarginM = (float)Margin;

		const TArray<TSharedPtr<FJsonValue>>* Lot = nullptr;
		if (!Obj->TryGetArrayField(TEXT("lot"), Lot)) return false;
		for (const TSharedPtr<FJsonValue>& V : *Lot)
		{
			FVector2D P;
			if (!VenueDraw::ReadXY(V, P)) return false;
			Out.Lot.Add(P);
		}
		if (Out.Lot.Num() < 3) return false;

		const TArray<TSharedPtr<FJsonValue>>* Props = nullptr;
		if (Obj->TryGetArrayField(TEXT("props"), Props))
		{
			for (const TSharedPtr<FJsonValue>& V : *Props)
			{
				const TSharedPtr<FJsonObject>* P = nullptr;
				if (!V->TryGetObject(P)) continue;
				FProp Prop;
				double X = 0, Y = 0, Yaw = 0;
				(*P)->TryGetStringField(TEXT("class"), Prop.Class);
				(*P)->TryGetNumberField(TEXT("x"), X);
				(*P)->TryGetNumberField(TEXT("y"), Y);
				(*P)->TryGetNumberField(TEXT("yaw_deg"), Yaw);
				Prop.Pos = FVector2D(X, Y);
				Prop.YawDeg = (float)Yaw;
				Out.Props.Add(Prop);
			}
		}

		// Rows, as envgen expands them: `from` to `to`, `spacing` apart, facing
		// along the row unless yaw_deg is given. Props first, then rows, so
		// the order matches envgen's (TrackShift relies on it).
		const TArray<TSharedPtr<FJsonValue>>* Rows = nullptr;
		if (Obj->TryGetArrayField(TEXT("rows"), Rows))
		{
			for (const TSharedPtr<FJsonValue>& V : *Rows)
			{
				const TSharedPtr<FJsonObject>* R = nullptr;
				if (!V->TryGetObject(R)) continue;
				FString Class;
				FVector2D From, To;
				double Spacing = 0;
				(*R)->TryGetStringField(TEXT("class"), Class);
				if (!VenueDraw::ReadXY((*R)->TryGetField(TEXT("from")), From)
					|| !VenueDraw::ReadXY((*R)->TryGetField(TEXT("to")), To)
					|| !(*R)->TryGetNumberField(TEXT("spacing"), Spacing) || Spacing <= 0.0)
				{
					continue;
				}
				const double Len = FVector2D::Distance(From, To);
				double Yaw = FMath::RadiansToDegrees(FMath::Atan2(To.Y - From.Y, To.X - From.X));
				(*R)->TryGetNumberField(TEXT("yaw_deg"), Yaw);
				const int32 N = FMath::FloorToInt(Len / Spacing + 1e-9) + 1;
				for (int32 K = 0; K < N; ++K)
				{
					const double T = (N == 1 || Len <= 0.0) ? 0.0 : K * Spacing / Len;
					FProp Prop;
					Prop.Class = Class;
					Prop.Pos = From + (To - From) * T;
					Prop.YawDeg = (float)Yaw;
					Out.Props.Add(Prop);
				}
			}
		}
		return true;
	}

	TArray<FVenueRef> Scan(const TArray<FString>& TrackDirs)
	{
		TArray<FVenueRef> Found;
		TSet<FString> Seen;
		for (const FString& Dir : TrackDirs)
		{
			const FString VenueDir = FPaths::Combine(Dir, TEXT("venues"));
			TArray<FString> Files;
			IFileManager::Get().FindFiles(Files, *FPaths::Combine(VenueDir, TEXT("*.venue.json")), true, false);
			for (const FString& File : Files)
			{
				TSharedRef<FVenue> V = MakeShared<FVenue>();
				if (!LoadFile(FPaths::Combine(VenueDir, File), *V))
				{
					UE_LOG(LogTemp, Warning, TEXT("FSDS Venue: %s isn't a venue file"), *FPaths::Combine(VenueDir, File));
					continue;
				}
				if (Seen.Contains(V->Name)) continue;
				Seen.Add(V->Name);
				Found.Add(V);
			}
		}
		Found.Sort([](const FVenueRef& A, const FVenueRef& B) { return A->Title < B->Title; });
		return Found;
	}

	FString VenueOfTrack(const FString& TrackName)
	{
		FString Left, Right;
		return TrackName.Split(Sep, &Left, &Right) ? Left : FString();
	}

	const FVenue* Find(const TArray<FVenueRef>& Venues, const FString& Name)
	{
		for (const FVenueRef& V : Venues)
		{
			if (V->Name == Name) return &V.Get();
		}
		return nullptr;
	}

	bool TrackShift(const FVenue& V, const FString& TrackCsvPath, FVector2D& Out)
	{
		const FString Side = FPaths::Combine(FPaths::GetPath(TrackCsvPath), FPaths::GetBaseFilename(TrackCsvPath) + TEXT(".env.json"));
		FString Text;
		TSharedPtr<FJsonObject> Obj;
		const TArray<TSharedPtr<FJsonValue>>* Props = nullptr;
		if (V.Props.Num() == 0 || !FFileHelper::LoadFileToString(Text, *Side)
			|| !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Obj) || !Obj.IsValid()
			|| !Obj->TryGetArrayField(TEXT("props"), Props) || Props->Num() != V.Props.Num())
		{
			return false;
		}
		auto PosOf = [&](int32 I, FVector2D& P)
		{
			const TSharedPtr<FJsonObject>* O = nullptr;
			double X = 0, Y = 0;
			if (!(*Props)[I]->TryGetObject(O) || !(*O)->TryGetNumberField(TEXT("x"), X) || !(*O)->TryGetNumberField(TEXT("y"), Y)) return false;
			P = FVector2D(X, Y);
			return true;
		};
		// The first prop gives the shift; the last confirms it is this venue's sidecar.
		FVector2D First, Last;
		if (!PosOf(0, First) || !PosOf(Props->Num() - 1, Last)) return false;
		const FVector2D Shift = First - V.Props[0].Pos;
		if (!(Last - V.Props.Last().Pos - Shift).IsNearlyZero(0.01)) return false;
		Out = Shift;
		return true;
	}

	void Paint(const FVenue& V, TFunctionRef<FVector2f(const FVector2D&)> ToScreen, float PxPerM,
		const FGeometry& Geo, FSlateWindowElementList& Out, int32 LayerId)
	{
		auto Lines = [&](int32 Layer, const TArray<FVector2f>& Pts, const FLinearColor& C, float Thick)
		{
			FSlateDrawElement::MakeLines(Out, Layer, Geo.ToPaintGeometry(), Pts, ESlateDrawEffect::None, C, true, Thick);
		};
		auto Dot = [&](int32 Layer, const FVector2f& At, float D, const FLinearColor& C)
		{
			D = (float)FMath::Max(2, FMath::RoundToInt(D));
			FSlateDrawElement::MakeBox(Out, Layer,
				Geo.ToPaintGeometry(FVector2f(D, D), FSlateLayoutTransform(At - FVector2f(D, D) * 0.5f)),
				SFSDSTrackMap::DotBrush(D), ESlateDrawEffect::None, C);
		};

		// The lot's edge.
		{
			TArray<FVector2f> Edge;
			for (const FVector2D& P : V.Lot) Edge.Add(ToScreen(P));
			const FVector2f First = Edge[0];
			Edge.Add(First);
			Lines(LayerId, Edge, VenueDraw::LotEdge, 2.f);
		}

		// Flat things first, so the rest draws over them.
		for (int32 Pass = 0; Pass < 2; ++Pass)
		{
			for (const FProp& P : V.Props)
			{
				const bool bFlat = P.Class == TEXT("sidewalk");
				if (bFlat != (Pass == 0)) continue;
				VenueDraw::FShape S;
				if (!VenueDraw::ShapeOf(P.Class, S)) continue;
				const FLinearColor C = VenueDraw::ColorOf(P.Class);
				const FVector2f At = ToScreen(P.Pos);
				const float Yaw = FMath::DegreesToRadians(P.YawDeg);
				const FVector2D Along(FMath::Cos(Yaw), FMath::Sin(Yaw)), Across(-Along.Y, Along.X);

				if (S.RadiusM > 0.f)
				{
					Dot(LayerId + 1, At, FMath::Min(2.f * S.RadiusM * PxPerM, 32.f), C);
					if (P.Class == TEXT("lamp_post"))
					{
						Lines(LayerId + 1, { At, ToScreen(P.Pos + Along * 1.3) }, C, 1.f);
					}
					continue;
				}
				const FVector2D L = Along * (S.LengthM * 0.5), W = Across * (S.WidthM * 0.5);
				const FVector2f A = ToScreen(P.Pos - L - W), B = ToScreen(P.Pos + L - W);
				const FVector2f Cc = ToScreen(P.Pos + L + W), D = ToScreen(P.Pos - L + W);
				Lines(LayerId + 1, { A, B, Cc, D, A }, C, bFlat ? 1.f : 1.5f);
				if (P.Class == TEXT("berm"))
				{
					Lines(LayerId + 1, { ToScreen(P.Pos - L), ToScreen(P.Pos + L) }, C, 1.f);   // the ridge
				}
			}
		}
	}
}
