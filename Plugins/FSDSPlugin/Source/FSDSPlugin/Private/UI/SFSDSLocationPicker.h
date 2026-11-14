#pragma once

#include "CoreMinimal.h"
#include "UI/FSDSVenue.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/Input/SComboBox.h"

DECLARE_DELEGATE_OneParam(FOnFSDSLocationPicked, const FString& /*VenueName, empty for none*/);

/**
 * Where: a dropdown of the venues (docs/venues.md), with one entry first for
 * no venue ("Anywhere" on Drive, "Open floor" in the track editor). One row
 * however many venues there are; Drive and the track editor share it.
 */
class SFSDSLocationPicker : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SFSDSLocationPicker) {}
		/** The first entry, for no venue. */
		SLATE_ARGUMENT(FText, NoneLabel)
		SLATE_ARGUMENT(TArray<FSDSVenue::FVenueRef>, Venues)
		/** The venue's name, or empty. */
		SLATE_ATTRIBUTE(FString, Selected)
		/** Optional: how many tracks an entry holds, shown after its name. */
		SLATE_ARGUMENT(TFunction<int32(const FString&)>, CountOf)
		SLATE_EVENT(FOnFSDSLocationPicked, OnPicked)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);

	/** The venue files changed. */
	void SetVenues(const TArray<FSDSVenue::FVenueRef>& InVenues);

private:
	using FItem = TSharedPtr<FString>;

	FText LabelOf(const FString& Name) const;

	FText NoneLabel;
	TArray<FSDSVenue::FVenueRef> Venues;
	TAttribute<FString> Selected;
	TFunction<int32(const FString&)> CountOf;
	FOnFSDSLocationPicked OnPicked;

	TArray<FItem> Items;
	TSharedPtr<SComboBox<FItem>> Combo;
};
