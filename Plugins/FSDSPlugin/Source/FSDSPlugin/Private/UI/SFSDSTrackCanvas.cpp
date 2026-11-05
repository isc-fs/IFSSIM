#include "UI/SFSDSTrackCanvas.h"

#include "UI/SFSDSTrackMap.h"

#include "Fonts/FontMeasure.h"
#include "Framework/Application/SlateApplication.h"
#include "Rendering/DrawElements.h"
#include "Styling/CoreStyle.h"

// Qualified everywhere (Canvas::...): other UI files share the unity blob.
namespace FSDSTrackCanvasStyle
{
	// The web editor's colours.
	const FLinearColor Backdrop = FLinearColor::FromSRGBColor(FColor(13, 13, 13));
	const FLinearColor GridLine = FLinearColor::FromSRGBColor(FColor(28, 28, 28));
	const FLinearColor GridZero = FLinearColor::FromSRGBColor(FColor(58, 58, 58));
	const FLinearColor Safe = FLinearColor::FromSRGBColor(FColor(47, 79, 47));
	const FLinearColor LeftLine = FLinearColor::FromSRGBColor(FColor(30, 58, 138));
	const FLinearColor RightLine = FLinearColor::FromSRGBColor(FColor(113, 63, 18));
	const FLinearColor Blue = FLinearColor::FromSRGBColor(FColor(59, 130, 246));
	const FLinearColor Yellow = FLinearColor::FromSRGBColor(FColor(250, 204, 21));
	const FLinearColor BigOrange = FLinearColor::FromSRGBColor(FColor(249, 115, 22));
	// Deep orange, not the web editor's peach: that reads as yellow next to a yellow cone.
	const FLinearColor SmallOrange = FLinearColor::FromSRGBColor(FColor(220, 70, 10));
	const FLinearColor StartArrow = FLinearColor::FromSRGBColor(FColor(239, 68, 68));
	const FLinearColor IssueError = FLinearColor::FromSRGBColor(FColor(239, 68, 68));
	const FLinearColor IssueWarn = FLinearColor::FromSRGBColor(FColor(245, 158, 11));
	const FLinearColor Path = FLinearColor::FromSRGBColor(FColor(85, 85, 85));
	const FLinearColor StrokeCol = FLinearColor::FromSRGBColor(FColor(255, 184, 28));
	const FLinearColor PointCol = FLinearColor::FromSRGBColor(FColor(229, 229, 229));
	const FLinearColor PointStart = FLinearColor::FromSRGBColor(FColor(34, 197, 94));
	const FLinearColor PointSel = FLinearColor::FromSRGBColor(FColor(255, 184, 28));
	const FLinearColor Label = FLinearColor::FromSRGBColor(FColor(156, 163, 175));

	constexpr float PointPx = 11.f;
	constexpr float HitPx = 9.f;
	constexpr float DragThresholdPx = 4.f;
	constexpr double MinHalf = 10.0, MaxHalf = 400.0;

	/** customMap's floor is about 200 m x 250 m: the square a track should fit in. */
	constexpr double SafeHalfM = 100.0;
}

void SFSDSTrackCanvas::Construct(const FArguments& InArgs)
{
	Spec = InArgs._Spec;
	OnChanged = InArgs._OnChanged;
	FitView();
}

void SFSDSTrackCanvas::FitView()
{
	if (!Spec.IsValid() || Spec->Points.Num() == 0)
	{
		ViewCentre = FVector2D::ZeroVector;
		ViewHalf = 70.0;
		return;
	}
	FBox2D B(ForceInit);
	for (const FVector2D& P : Spec->Points) B += P;
	ViewCentre = B.GetCenter();
	ViewHalf = FMath::Clamp(B.GetSize().GetMax() * 0.6 + 10.0, FSDSTrackCanvasStyle::MinHalf, FSDSTrackCanvasStyle::MaxHalf);
	Invalidate(EInvalidateWidgetReason::Paint);
}

float SFSDSTrackCanvas::PxPerM(const FVector2f& Size) const
{
	return float(FMath::Min(Size.X, Size.Y) * 0.5 / ViewHalf);
}

FVector2f SFSDSTrackCanvas::ToScreen(const FVector2D& P, const FVector2f& Size) const
{
	const float K = PxPerM(Size);
	return FVector2f(Size.X * 0.5f + float(P.X - ViewCentre.X) * K, Size.Y * 0.5f - float(P.Y - ViewCentre.Y) * K);
}

FVector2D SFSDSTrackCanvas::ToWorld(const FVector2f& Local, const FVector2f& Size) const
{
	const double K = PxPerM(Size);
	const double X = ViewCentre.X + (Local.X - Size.X * 0.5) / K;
	const double Y = ViewCentre.Y - (Local.Y - Size.Y * 0.5) / K;
	return FVector2D(FMath::RoundToDouble(X * 10.0) / 10.0, FMath::RoundToDouble(Y * 10.0) / 10.0);
}

int32 SFSDSTrackCanvas::HitPoint(const FVector2f& Local, const FVector2f& Size) const
{
	if (!Spec.IsValid()) return INDEX_NONE;
	int32 Best = INDEX_NONE;
	float BestD = FSDSTrackCanvasStyle::HitPx;
	for (int32 i = 0; i < Spec->Points.Num(); ++i)
	{
		const float D = FVector2f::Distance(ToScreen(Spec->Points[i], Size), Local);
		if (D <= BestD) { BestD = D; Best = i; }
	}
	return Best;
}

void SFSDSTrackCanvas::Changed()
{
	Invalidate(EInvalidateWidgetReason::Paint);
	OnChanged.ExecuteIfBound();
}

// ---------------------------------------------------------------------------
// Input

FReply SFSDSTrackCanvas::OnMouseButtonDown(const FGeometry& Geo, const FPointerEvent& E)
{
	const FVector2f Size = FVector2f(Geo.GetLocalSize());
	const FVector2f Local = FVector2f(Geo.AbsoluteToLocal(E.GetScreenSpacePosition()));
	FReply Reply = FReply::Handled().CaptureMouse(SharedThis(this)).SetUserFocus(SharedThis(this), EFocusCause::Mouse);

	if (E.GetEffectingButton() == EKeys::MiddleMouseButton || E.GetEffectingButton() == EKeys::RightMouseButton)
	{
		Drag = EDrag::Pan;
		DragStartPx = Local;
		DragStartCentre = ViewCentre;
		return Reply;
	}
	if (E.GetEffectingButton() != EKeys::LeftMouseButton || !Spec.IsValid()) return FReply::Unhandled();

	const int32 Hit = HitPoint(Local, Size);
	if (Hit != INDEX_NONE)
	{
		Selected = Hit;
		Drag = EDrag::Point;
		DragPoint = Hit;
		Invalidate(EInvalidateWidgetReason::Paint);
		return Reply;
	}
	if (bFreehand)
	{
		Drag = EDrag::Stroke;
		Stroke = { ToWorld(Local, Size) };
		return Reply;
	}
	Drag = EDrag::Pending;
	DragStartPx = Local;
	DragStartCentre = ViewCentre;
	return Reply;
}

FReply SFSDSTrackCanvas::OnMouseMove(const FGeometry& Geo, const FPointerEvent& E)
{
	if (Drag == EDrag::None || !Spec.IsValid()) return FReply::Unhandled();
	const FVector2f Size = FVector2f(Geo.GetLocalSize());
	const FVector2f Local = FVector2f(Geo.AbsoluteToLocal(E.GetScreenSpacePosition()));

	switch (Drag)
	{
	case EDrag::Stroke:
	{
		const FVector2D W = ToWorld(Local, Size);
		if (FVector2D::Distance(W, Stroke.Last()) > 0.5) Stroke.Add(W);
		Invalidate(EInvalidateWidgetReason::Paint);
		break;
	}
	case EDrag::Point:
		if (Spec->Points.IsValidIndex(DragPoint))
		{
			Spec->Points[DragPoint] = ToWorld(Local, Size);
			Changed();
		}
		break;
	case EDrag::Pending:
		if (FVector2f::Distance(Local, DragStartPx) < FSDSTrackCanvasStyle::DragThresholdPx) break;
		Drag = EDrag::Pan;
		[[fallthrough]];
	case EDrag::Pan:
	{
		const double K = PxPerM(Size);
		const FVector2f D = Local - DragStartPx;
		ViewCentre = DragStartCentre - FVector2D(D.X / K, -D.Y / K);
		Invalidate(EInvalidateWidgetReason::Paint);
		break;
	}
	default:
		break;
	}
	return FReply::Handled();
}

FReply SFSDSTrackCanvas::OnMouseButtonUp(const FGeometry& Geo, const FPointerEvent& E)
{
	if (Drag == EDrag::None) return FReply::Unhandled();
	const FVector2f Size = FVector2f(Geo.GetLocalSize());
	const FVector2f Local = FVector2f(Geo.AbsoluteToLocal(E.GetScreenSpacePosition()));
	const EDrag Was = Drag;
	Drag = EDrag::None;
	DragPoint = INDEX_NONE;

	if (Was == EDrag::Stroke && Spec.IsValid())
	{
		TArray<FVector2D> Pts = FSDSTrackSpec::StrokeToPoints(Stroke, Spec->bClosed);
		Stroke.Reset();
		if (Pts.Num() >= 2)
		{
			// A new stroke is a new path: sections by point number no longer apply.
			Spec->Points = MoveTemp(Pts);
			Spec->Sections.Reset();
			Selected = INDEX_NONE;
			Changed();
		}
		Invalidate(EInvalidateWidgetReason::Paint);
	}
	else if (Was == EDrag::Pending && Spec.IsValid())
	{
		// A click: a new point after the selected one, or at the end.
		const int32 At = Selected == INDEX_NONE ? Spec->Points.Num() : Selected + 1;
		Spec->Points.Insert(ToWorld(Local, Size), At);
		FSDSTrackSpec::ShiftSections(Spec->Sections, At, +1);
		Selected = At;
		Changed();
	}
	return FReply::Handled().ReleaseMouseCapture();
}

FReply SFSDSTrackCanvas::OnMouseWheel(const FGeometry& Geo, const FPointerEvent& E)
{
	// Zoom about the cursor: the point under it stays put.
	const FVector2f Size = FVector2f(Geo.GetLocalSize());
	const FVector2f Local = FVector2f(Geo.AbsoluteToLocal(E.GetScreenSpacePosition()));
	const double K0 = PxPerM(Size);
	const FVector2D Under(ViewCentre.X + (Local.X - Size.X * 0.5) / K0, ViewCentre.Y - (Local.Y - Size.Y * 0.5) / K0);
	ViewHalf = FMath::Clamp(ViewHalf * FMath::Exp(-0.15 * E.GetWheelDelta()), FSDSTrackCanvasStyle::MinHalf, FSDSTrackCanvasStyle::MaxHalf);
	const double K1 = PxPerM(Size);
	ViewCentre = FVector2D(Under.X - (Local.X - Size.X * 0.5) / K1, Under.Y + (Local.Y - Size.Y * 0.5) / K1);
	Invalidate(EInvalidateWidgetReason::Paint);
	return FReply::Handled();
}

FReply SFSDSTrackCanvas::OnKeyDown(const FGeometry&, const FKeyEvent& E)
{
	if ((E.GetKey() == EKeys::Delete || E.GetKey() == EKeys::BackSpace) && Spec.IsValid() && Spec->Points.IsValidIndex(Selected))
	{
		Spec->Points.RemoveAt(Selected);
		FSDSTrackSpec::ShiftSections(Spec->Sections, Selected, -1);
		Selected = INDEX_NONE;
		Changed();
		return FReply::Handled();
	}
	if (E.GetKey() == EKeys::Escape && Selected != INDEX_NONE)
	{
		Selected = INDEX_NONE;
		Invalidate(EInvalidateWidgetReason::Paint);
		return FReply::Handled();
	}
	return FReply::Unhandled();
}

FCursorReply SFSDSTrackCanvas::OnCursorQuery(const FGeometry&, const FPointerEvent&) const
{
	if (Drag == EDrag::Pan) return FCursorReply::Cursor(EMouseCursor::GrabHandClosed);
	return FCursorReply::Cursor(bFreehand ? EMouseCursor::Crosshairs : EMouseCursor::Default);
}

// ---------------------------------------------------------------------------
// Drawing

int32 SFSDSTrackCanvas::OnPaint(const FPaintArgs&, const FGeometry& Geo, const FSlateRect& Cull,
	FSlateWindowElementList& Out, int32 LayerId, const FWidgetStyle&, bool) const
{
	namespace Canvas = FSDSTrackCanvasStyle;
	const FSlateBrush* White = FCoreStyle::Get().GetBrush("WhiteBrush");
	const FVector2f Size = FVector2f(Geo.GetLocalSize());
	const float K = PxPerM(Size);
	const FSlateFontInfo SmallFont = FCoreStyle::GetDefaultFontStyle("Regular", 9);

	auto Line = [&](int32 Layer, const TArray<FVector2f>& Pts, const FLinearColor& C, float Thick)
	{
		if (Pts.Num() >= 2)
		{
			FSlateDrawElement::MakeLines(Out, Layer, Geo.ToPaintGeometry(), Pts, ESlateDrawEffect::None, C, true, Thick);
		}
	};
	auto Dot = [&](int32 Layer, const FVector2f& At, float D, const FLinearColor& C)
	{
		FSlateDrawElement::MakeBox(Out, Layer,
			Geo.ToPaintGeometry(FVector2f(D, D), FSlateLayoutTransform(At - FVector2f(D, D) * 0.5f)),
			SFSDSTrackMap::DotBrush(D), ESlateDrawEffect::None, C);
	};
	auto Text = [&](int32 Layer, const FString& S, const FVector2f& At, const FLinearColor& C)
	{
		FSlateDrawElement::MakeText(Out, Layer, Geo.ToPaintGeometry(FVector2f(200.f, 16.f), FSlateLayoutTransform(At)),
			S, SmallFont, ESlateDrawEffect::None, C);
	};

	// Everything is clipped to the canvas.
	Out.PushClip(FSlateClippingZone(Geo));
	FSlateDrawElement::MakeBox(Out, LayerId, Geo.ToPaintGeometry(), White, ESlateDrawEffect::None, Canvas::Backdrop);

	// Grid: 10 m, or 50 m zoomed out; the axes a little brighter.
	const double Step = ViewHalf > 150.0 ? 50.0 : 10.0;
	const double HalfW = Size.X * 0.5 / K, HalfH = Size.Y * 0.5 / K;
	for (double X = FMath::FloorToDouble((ViewCentre.X - HalfW) / Step) * Step; X <= ViewCentre.X + HalfW; X += Step)
	{
		const float Sx = ToScreen(FVector2D(X, 0), Size).X;
		Line(LayerId + 1, { FVector2f(Sx, 0.f), FVector2f(Sx, Size.Y) }, FMath::IsNearlyZero(X) ? Canvas::GridZero : Canvas::GridLine, 1.f);
	}
	for (double Y = FMath::FloorToDouble((ViewCentre.Y - HalfH) / Step) * Step; Y <= ViewCentre.Y + HalfH; Y += Step)
	{
		const float Sy = ToScreen(FVector2D(0, Y), Size).Y;
		Line(LayerId + 1, { FVector2f(0.f, Sy), FVector2f(Size.X, Sy) }, FMath::IsNearlyZero(Y) ? Canvas::GridZero : Canvas::GridLine, 1.f);
	}
	Text(LayerId + 1, FString::Printf(TEXT("grid %.0f m"), Step), FVector2f(8.f, Size.Y - 20.f), Canvas::Label);

	// The backend recentres the cones on the origin: shift them back so they
	// sit under the drawn points.
	const FSDSTrackSpec::FPreview* P = Preview.Get();
	const FVector2D Off = P ? P->Offset : FVector2D::ZeroVector;
	auto Un = [&](const FVector2D& V) { return ToScreen(V - Off, Size); };

	// The floor's safe square, centred where the track will be.
	{
		const FVector2f A = Un(FVector2D(-Canvas::SafeHalfM, -Canvas::SafeHalfM));
		const FVector2f B = Un(FVector2D(Canvas::SafeHalfM, Canvas::SafeHalfM));
		Line(LayerId + 1, { A, FVector2f(B.X, A.Y), B, FVector2f(A.X, B.Y), A }, Canvas::Safe, 1.f);
	}

	const bool bClosed = Spec.IsValid() && Spec->bClosed;
	if (P)
	{
		auto Poly = [&](const TArray<FVector2D>& Pts)
		{
			TArray<FVector2f> S;
			for (const FVector2D& V : Pts) S.Add(Un(V));
			if (bClosed && S.Num() > 0) { const FVector2f First = S[0]; S.Add(First); }   // not S.Add(S[0]): it may reallocate under the reference
			return S;
		};
		Line(LayerId + 2, Poly(P->Left), Canvas::LeftLine, 1.f);
		Line(LayerId + 2, Poly(P->Right), Canvas::RightLine, 1.f);

		for (const FSDSTrackSpec::FCone& C : P->Cones)
		{
			FLinearColor Col = Canvas::Label;
			float Rm = 0.4f;
			if (C.Type == TEXT("blue")) Col = Canvas::Blue;
			else if (C.Type == TEXT("yellow")) Col = Canvas::Yellow;
			else if (C.Type == TEXT("big_orange")) { Col = Canvas::BigOrange; Rm = 0.55f; }
			else if (C.Type == TEXT("small_orange")) Col = Canvas::SmallOrange;
			Dot(LayerId + 3, Un(C.Pos), FMath::Clamp(2.f * Rm * K, 3.f, 12.f), Col);
		}

		// Where the sim will put the car, facing the way it will drive.
		if (P->bHasStart)
		{
			const FVector2f S = Un(P->Spawn);
			const FVector2f E = Un(P->Spawn + P->Forward * 6.0);
			Line(LayerId + 4, { S, E }, Canvas::StartArrow, 3.f);
			const FVector2f Dir = (E - S).GetSafeNormal();
			const FVector2f Side(-Dir.Y, Dir.X);
			Line(LayerId + 4, { E - Dir * 9.f + Side * 6.f, E, E - Dir * 9.f - Side * 6.f }, Canvas::StartArrow, 3.f);
		}

		// Rule issues, ringed where they are (4 m across).
		for (const FSDSTrackSpec::FIssue& I : P->Issues)
		{
			if (!I.XY.IsSet()) continue;
			const FVector2f C = Un(*I.XY);
			const float R = FMath::Max(4.f * K, 8.f);
			TArray<FVector2f> Ring;
			for (int32 k = 0; k <= 24; ++k)
			{
				const float A = 2.f * PI * k / 24.f;
				Ring.Add(C + FVector2f(FMath::Cos(A), FMath::Sin(A)) * R);
			}
			Line(LayerId + 4, Ring, I.bError ? Canvas::IssueError : Canvas::IssueWarn, 2.f);
		}
	}

	if (Spec.IsValid())
	{
		// The drawn path, then its control points with their numbers.
		TArray<FVector2f> PathPts;
		for (const FVector2D& V : Spec->Points) PathPts.Add(ToScreen(V, Size));
		if (bClosed && PathPts.Num() > 2) { const FVector2f First = PathPts[0]; PathPts.Add(First); }
		Line(LayerId + 5, PathPts, Canvas::Path, 1.f);

		for (int32 i = 0; i < Spec->Points.Num(); ++i)
		{
			const FVector2f At = ToScreen(Spec->Points[i], Size);
			const FLinearColor C = i == Selected ? Canvas::PointSel : i == 0 ? Canvas::PointStart : Canvas::PointCol;
			Dot(LayerId + 6, At, Canvas::PointPx + 2.f, FLinearColor::Black);
			Dot(LayerId + 7, At, Canvas::PointPx, C);
			Text(LayerId + 7, FString::FromInt(i), At + FVector2f(8.f, -18.f), Canvas::Label);
		}
	}

	// A freehand stroke in progress.
	if (Stroke.Num() > 1)
	{
		TArray<FVector2f> S;
		for (const FVector2D& V : Stroke) S.Add(ToScreen(V, Size));
		Line(LayerId + 8, S, Canvas::StrokeCol, 2.f);
	}

	Out.PopClip();
	return LayerId + 8;
}
