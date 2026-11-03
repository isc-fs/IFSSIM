#include "UI/FSDSTrackSpec.h"

#include "Dom/JsonValue.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

namespace FSDSTrackSpec
{
	static const TCHAR* Format = TEXT("ifssim-trackspec/1");

	static double Round1(double V) { return FMath::RoundToDouble(V * 10.0) / 10.0; }

	static TSharedRef<FJsonValue> Pair(const FVector2D& P)
	{
		return MakeShared<FJsonValueArray>(TArray<TSharedPtr<FJsonValue>>{
			MakeShared<FJsonValueNumber>(Round1(P.X)), MakeShared<FJsonValueNumber>(Round1(P.Y)) });
	}

	static bool ReadPair(const TSharedPtr<FJsonValue>& V, FVector2D& Out)
	{
		const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
		if (!V.IsValid() || !V->TryGetArray(A) || A->Num() != 2) return false;
		double X = 0, Y = 0;
		if (!(*A)[0]->TryGetNumber(X) || !(*A)[1]->TryGetNumber(Y)) return false;
		Out = FVector2D(X, Y);
		return true;
	}

	static void ReadPairs(const TSharedPtr<FJsonObject>& Obj, const TCHAR* Field, TArray<FVector2D>& Out)
	{
		Out.Reset();
		const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
		if (!Obj->TryGetArrayField(Field, A)) return;
		for (const TSharedPtr<FJsonValue>& V : *A)
		{
			FVector2D P;
			if (ReadPair(V, P)) Out.Add(P);
		}
	}

	bool IsProtectedName(const FString& Name)
	{
		const FString N = FPaths::GetBaseFilename(Name).ToLower();
		return N == TEXT("acceleration") || N == TEXT("skidpad") || N.StartsWith(TEXT("stress_"));
	}

	FString CopyName(const FString& Name)
	{
		FString Base = FPaths::GetBaseFilename(Name);
		if (Base.StartsWith(TEXT("stress_"))) Base.RightChopInline(7);
		return Base + TEXT("_copy");
	}

	int32 FPreview::NumErrors() const
	{
		int32 N = 0;
		for (const FIssue& I : Issues) N += I.bError ? 1 : 0;
		return N;
	}

	TSharedRef<FJsonObject> ToJson(const FSpec& Spec)
	{
		TSharedRef<FJsonObject> O = MakeShared<FJsonObject>();
		O->SetStringField(TEXT("format"), Format);
		O->SetBoolField(TEXT("closed"), Spec.bClosed);
		TArray<TSharedPtr<FJsonValue>> Pts;
		for (const FVector2D& P : Spec.Points) Pts.Add(Pair(P));
		O->SetArrayField(TEXT("points"), Pts);
		O->SetNumberField(TEXT("width"), Spec.Width);
		TSharedRef<FJsonObject> Spacing = MakeShared<FJsonObject>();
		Spacing->SetNumberField(TEXT("straight"), Spec.SpacingStraight);
		Spacing->SetNumberField(TEXT("corner_min"), Spec.SpacingCornerMin);
		O->SetObjectField(TEXT("spacing"), Spacing);
		O->SetNumberField(TEXT("seed"), Spec.Seed);
		if (Spec.Sections.Num() > 0)
		{
			TArray<TSharedPtr<FJsonValue>> Secs;
			for (const FSection& S : Spec.Sections)
			{
				TSharedRef<FJsonObject> SO = MakeShared<FJsonObject>();
				SO->SetNumberField(TEXT("from"), S.From);
				SO->SetNumberField(TEXT("to"), S.To);
				if (S.Width.IsSet()) SO->SetNumberField(TEXT("width"), *S.Width);
				if (S.Spacing.IsSet()) SO->SetNumberField(TEXT("spacing"), *S.Spacing);
				if (S.Drop > 0.f) SO->SetNumberField(TEXT("drop"), S.Drop);
				if (S.Side != TEXT("both")) SO->SetStringField(TEXT("side"), S.Side);
				if (!S.Recolor.IsEmpty()) SO->SetStringField(TEXT("recolor"), S.Recolor);
				if (S.Jitter > 0.f) SO->SetNumberField(TEXT("jitter"), S.Jitter);
				Secs.Add(MakeShared<FJsonValueObject>(SO));
			}
			O->SetArrayField(TEXT("sections"), Secs);
		}
		return O;
	}

	bool FromJson(const TSharedPtr<FJsonObject>& Obj, FSpec& Out)
	{
		if (!Obj.IsValid()) return false;
		FString F;
		if (!Obj->TryGetStringField(TEXT("format"), F) || F != Format) return false;
		Out = FSpec();
		Obj->TryGetBoolField(TEXT("closed"), Out.bClosed);
		ReadPairs(Obj, TEXT("points"), Out.Points);
		double D = 0;
		if (Obj->TryGetNumberField(TEXT("width"), D)) Out.Width = (float)D;
		const TSharedPtr<FJsonObject>* Sp = nullptr;
		if (Obj->TryGetObjectField(TEXT("spacing"), Sp))
		{
			if ((*Sp)->TryGetNumberField(TEXT("straight"), D)) Out.SpacingStraight = (float)D;
			if ((*Sp)->TryGetNumberField(TEXT("corner_min"), D)) Out.SpacingCornerMin = (float)D;
		}
		if (Obj->TryGetNumberField(TEXT("seed"), D)) Out.Seed = (int32)D;
		const TArray<TSharedPtr<FJsonValue>>* Secs = nullptr;
		if (Obj->TryGetArrayField(TEXT("sections"), Secs))
		{
			for (const TSharedPtr<FJsonValue>& V : *Secs)
			{
				const TSharedPtr<FJsonObject>* SO = nullptr;
				if (!V->TryGetObject(SO)) continue;
				FSection S;
				if ((*SO)->TryGetNumberField(TEXT("from"), D)) S.From = (int32)D;
				if ((*SO)->TryGetNumberField(TEXT("to"), D)) S.To = (int32)D;
				if ((*SO)->TryGetNumberField(TEXT("width"), D)) S.Width = (float)D;
				if ((*SO)->TryGetNumberField(TEXT("spacing"), D)) S.Spacing = (float)D;
				if ((*SO)->TryGetNumberField(TEXT("drop"), D)) S.Drop = (float)D;
				if ((*SO)->TryGetNumberField(TEXT("jitter"), D)) S.Jitter = (float)D;
				(*SO)->TryGetStringField(TEXT("side"), S.Side);
				(*SO)->TryGetStringField(TEXT("recolor"), S.Recolor);
				Out.Sections.Add(S);
			}
		}
		return true;
	}

	bool LoadFile(const FString& Path, FSpec& Out)
	{
		FString Text;
		TSharedPtr<FJsonObject> Obj;
		return FFileHelper::LoadFileToString(Text, *Path)
			&& FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Obj)
			&& FromJson(Obj, Out);
	}

	bool ParsePreview(const TSharedPtr<FJsonObject>& Obj, FPreview& Out)
	{
		if (!Obj.IsValid() || Obj->HasField(TEXT("error"))) return false;
		Out = FPreview();
		Obj->TryGetBoolField(TEXT("ok"), Out.bOk);
		double D = 0;
		if (Obj->TryGetNumberField(TEXT("length_m"), D)) Out.LengthM = (float)D;

		const TArray<TSharedPtr<FJsonValue>>* Cones = nullptr;
		if (Obj->TryGetArrayField(TEXT("cones"), Cones))
		{
			for (const TSharedPtr<FJsonValue>& V : *Cones)
			{
				const TSharedPtr<FJsonObject>* C = nullptr;
				if (!V->TryGetObject(C)) continue;
				FCone Cone;
				double X = 0, Y = 0;
				(*C)->TryGetStringField(TEXT("type"), Cone.Type);
				(*C)->TryGetNumberField(TEXT("x"), X);
				(*C)->TryGetNumberField(TEXT("y"), Y);
				Cone.Pos = FVector2D(X, Y);
				if (Cone.Type == TEXT("blue")) ++Out.Blue;
				else if (Cone.Type == TEXT("yellow")) ++Out.Yellow;
				else if (Cone.Type.Contains(TEXT("orange"))) ++Out.Orange;
				Out.Cones.Add(Cone);
			}
		}
		const TArray<TSharedPtr<FJsonValue>>* Off = nullptr;
		if (Obj->TryGetArrayField(TEXT("offset"), Off) && Off->Num() == 2)
		{
			Out.Offset = FVector2D((*Off)[0]->AsNumber(), (*Off)[1]->AsNumber());
		}
		ReadPairs(Obj, TEXT("left"), Out.Left);
		ReadPairs(Obj, TEXT("right"), Out.Right);

		const TSharedPtr<FJsonObject>* Start = nullptr;
		if (Obj->TryGetObjectField(TEXT("start"), Start))
		{
			const TArray<TSharedPtr<FJsonValue>>* A = nullptr;
			if ((*Start)->TryGetArrayField(TEXT("spawn"), A) && A->Num() >= 2)
			{
				Out.Spawn = FVector2D((*A)[0]->AsNumber(), (*A)[1]->AsNumber());
				Out.bHasStart = true;
			}
			if ((*Start)->TryGetArrayField(TEXT("forward"), A) && A->Num() >= 2)
			{
				Out.Forward = FVector2D((*A)[0]->AsNumber(), (*A)[1]->AsNumber());
			}
		}

		const TArray<TSharedPtr<FJsonValue>>* Issues = nullptr;
		if (Obj->TryGetArrayField(TEXT("issues"), Issues))
		{
			for (const TSharedPtr<FJsonValue>& V : *Issues)
			{
				const TSharedPtr<FJsonObject>* I = nullptr;
				if (!V->TryGetObject(I)) continue;
				FIssue Issue;
				FString Sev;
				(*I)->TryGetStringField(TEXT("code"), Issue.Code);
				(*I)->TryGetStringField(TEXT("severity"), Sev);
				(*I)->TryGetStringField(TEXT("message"), Issue.Message);
				Issue.bError = Sev == TEXT("error");
				if ((*I)->TryGetNumberField(TEXT("s"), D)) Issue.S = (float)D;
				const TArray<TSharedPtr<FJsonValue>>* XY = nullptr;
				if ((*I)->TryGetArrayField(TEXT("xy"), XY) && XY->Num() == 2)
				{
					Issue.XY = FVector2D((*XY)[0]->AsNumber(), (*XY)[1]->AsNumber());
				}
				Out.Issues.Add(Issue);
			}
		}
		return true;
	}

	TArray<FVector2D> Simplify(const TArray<FVector2D>& Pts, double Tol)
	{
		if (Pts.Num() < 3) return Pts;
		const FVector2D A = Pts[0], B = Pts.Last();
		const double Dx = B.X - A.X, Dy = B.Y - A.Y;
		const double Len = FMath::Max(FMath::Sqrt(Dx * Dx + Dy * Dy), 1e-9);
		double Worst = 0;
		int32 Idx = 0;
		for (int32 i = 1; i < Pts.Num() - 1; ++i)
		{
			const double D = FMath::Abs(Dy * Pts[i].X - Dx * Pts[i].Y + B.X * A.Y - B.Y * A.X) / Len;
			if (D > Worst) { Worst = D; Idx = i; }
		}
		if (Worst <= Tol) return { A, B };
		TArray<FVector2D> L = Simplify(TArray<FVector2D>(Pts.GetData(), Idx + 1), Tol);
		TArray<FVector2D> R = Simplify(TArray<FVector2D>(Pts.GetData() + Idx, Pts.Num() - Idx), Tol);
		L.Pop();
		L.Append(R);
		return L;
	}

	TArray<FVector2D> StrokeToPoints(const TArray<FVector2D>& Stroke, bool bClosed)
	{
		// Two control points 2 m apart make a kink the spline turns into a fold.
		TArray<FVector2D> Out;
		for (const FVector2D& P : Simplify(Stroke, 1.5))
		{
			if (Out.Num() == 0 || FVector2D::Distance(Out.Last(), P) >= 4.0)
			{
				Out.Add(FVector2D(Round1(P.X), Round1(P.Y)));
			}
		}
		// A loop is drawn back to where it began, usually a little past it.
		while (bClosed && Out.Num() > 3 && FVector2D::Distance(Out[0], Out.Last()) < 15.0)
		{
			Out.Pop();
		}
		return Out;
	}

	void ShiftSections(TArray<FSection>& Sections, int32 At, int32 Delta)
	{
		for (FSection& S : Sections)
		{
			if (S.From > At || (Delta > 0 && S.From == At)) S.From += Delta;
			if (S.To > At || (Delta > 0 && S.To == At)) S.To += Delta;
		}
		Sections.RemoveAll([](const FSection& S) { return S.From == S.To; });
	}
}
