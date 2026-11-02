#pragma once

#include "CoreMinimal.h"
#include "UI/FSDSTrackSpec.h"
#include "Widgets/SLeafWidget.h"

/**
 * The track editor's drawing area, the web editor's canvas in Slate.
 *
 * Click mode: a click adds a point (after the selected one, if any), a drag on
 * a point moves it, a drag on empty space pans. Freehand mode: a drag draws
 * the whole path, simplified to control points. The wheel zooms. Delete
 * removes the selected point; Escape deselects.
 *
 * It edits the spec it is given and calls OnChanged; the preview (cones, cone
 * lines, start pose, rule issues) is drawn from what the editor hands it.
 */
class SFSDSTrackCanvas : public SLeafWidget
{
public:
	SLATE_BEGIN_ARGS(SFSDSTrackCanvas) {}
		SLATE_ARGUMENT(TSharedPtr<FSDSTrackSpec::FSpec>, Spec)
		SLATE_EVENT(FSimpleDelegate, OnChanged)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	void SetPreview(const TSharedPtr<const FSDSTrackSpec::FPreview>& InPreview) { Preview = InPreview; }
	void SetFreehand(bool bOn) { bFreehand = bOn; }
	bool IsFreehand() const { return bFreehand; }
	int32 GetSelected() const { return Selected; }
	void ClearSelection() { Selected = INDEX_NONE; }
	/** Centre the view on the drawn points. */
	void FitView();

	virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
		FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle,
		bool bParentEnabled) const override;
	virtual FVector2D ComputeDesiredSize(float) const override { return FVector2D(500.f, 500.f); }
	virtual bool SupportsKeyboardFocus() const override { return true; }

	virtual FReply OnMouseButtonDown(const FGeometry& Geo, const FPointerEvent& E) override;
	virtual FReply OnMouseMove(const FGeometry& Geo, const FPointerEvent& E) override;
	virtual FReply OnMouseButtonUp(const FGeometry& Geo, const FPointerEvent& E) override;
	virtual FReply OnMouseWheel(const FGeometry& Geo, const FPointerEvent& E) override;
	virtual FReply OnKeyDown(const FGeometry& Geo, const FKeyEvent& E) override;
	virtual FCursorReply OnCursorQuery(const FGeometry& Geo, const FPointerEvent& E) const override;

private:
	enum class EDrag : uint8 { None, Pending, Pan, Point, Stroke };

	float PxPerM(const FVector2f& Size) const;
	FVector2f ToScreen(const FVector2D& P, const FVector2f& Size) const;
	FVector2D ToWorld(const FVector2f& Local, const FVector2f& Size) const;
	int32 HitPoint(const FVector2f& Local, const FVector2f& Size) const;
	void Changed();

	TSharedPtr<FSDSTrackSpec::FSpec> Spec;
	TSharedPtr<const FSDSTrackSpec::FPreview> Preview;
	FSimpleDelegate OnChanged;

	bool bFreehand = false;
	int32 Selected = INDEX_NONE;

	// The view: centre and half the shorter side, in metres.
	FVector2D ViewCentre = FVector2D::ZeroVector;
	double ViewHalf = 70.0;

	EDrag Drag = EDrag::None;
	int32 DragPoint = INDEX_NONE;
	FVector2f DragStartPx = FVector2f::ZeroVector;
	FVector2D DragStartCentre = FVector2D::ZeroVector;
	TArray<FVector2D> Stroke;
};
