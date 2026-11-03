#include "UI/SFSDSLocationPicker.h"

#include "Styling/CoreStyle.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "FSDSLocationPicker"

// Always qualified: other UI files in the same unity blob bring their own.
namespace LocationUi
{
	const FLinearColor Ink = FLinearColor::FromSRGBColor(FColor(232, 232, 236));

	FSlateFontInfo PickFont(int32 Size, const TCHAR* Weight = TEXT("Regular"))
	{
		return FCoreStyle::GetDefaultFontStyle(Weight, Size);
	}
}

void SFSDSLocationPicker::Construct(const FArguments& InArgs)
{
	NoneLabel = InArgs._NoneLabel;
	Selected = InArgs._Selected;
	CountOf = InArgs._CountOf;
	OnPicked = InArgs._OnPicked;

	ChildSlot
	[
		SAssignNew(Combo, SComboBox<FItem>)
		.IsFocusable(false)
		.OptionsSource(&Items)
		.OnGenerateWidget_Lambda([this](FItem Item)
		{
			return SNew(STextBlock).Text(LabelOf(*Item)).Font(LocationUi::PickFont(11));
		})
		// The choice can change behind it (opening a venue's track): match it
		// before the list shows, so every entry reports when picked.
		.OnComboBoxOpening_Lambda([this]()
		{
			const FString Now = Selected.Get();
			for (const FItem& I : Items)
			{
				if (*I == Now) { Combo->SetSelectedItem(I); break; }
			}
		})
		.OnSelectionChanged_Lambda([this](FItem Item, ESelectInfo::Type)
		{
			if (Item.IsValid() && *Item != Selected.Get()) OnPicked.ExecuteIfBound(*Item);
		})
		[
			SNew(STextBlock)
			.Text_Lambda([this]() { return LabelOf(Selected.Get()); })
			.Font(LocationUi::PickFont(11, TEXT("Bold")))
			.ColorAndOpacity(LocationUi::Ink)
		]
	];
	SetVenues(InArgs._Venues);
}

void SFSDSLocationPicker::SetVenues(const TArray<FSDSVenue::FVenueRef>& InVenues)
{
	Venues = InVenues;
	Items.Reset();
	Items.Add(MakeShared<FString>());
	for (const FSDSVenue::FVenueRef& V : Venues)
	{
		Items.Add(MakeShared<FString>(V->Name));
	}
	if (Combo.IsValid()) Combo->RefreshOptions();
}

FText SFSDSLocationPicker::LabelOf(const FString& Name) const
{
	FText Label = NoneLabel;
	if (!Name.IsEmpty())
	{
		const FSDSVenue::FVenue* V = FSDSVenue::Find(Venues, Name);
		Label = FText::FromString(V ? V->Title : Name);
	}
	if (!CountOf) return Label;
	return FText::Format(LOCTEXT("WithCount", "{0}  ({1})"), Label, FText::AsNumber(CountOf(Name)));
}

#undef LOCTEXT_NAMESPACE
