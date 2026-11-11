#pragma once

#include "CoreMinimal.h"
#include "Widgets/SCompoundWidget.h"

class FFSDSStackStarter;
class SMultiLineEditableTextBox;

/**
 * The menu's Logs page: the pipeline's and the backend's output, so nobody
 * needs `docker compose logs` in a terminal. The last lines of one service,
 * a filter, "problems only" (errors, warnings, tracebacks), live follow
 * (every 3 s while the page is up) and Save to a file
 * (Documents\IFSSIM\Logs). The text can be selected and copied.
 */
class SFSDSLogsPage : public SCompoundWidget
{
public:
	SLATE_BEGIN_ARGS(SFSDSLogsPage) {}
		/** Whether the page is on screen: it only asks docker then. */
		SLATE_ATTRIBUTE(bool, IsActive)
	SLATE_END_ARGS()

	void Construct(const FArguments& InArgs);
	virtual ~SFSDSLogsPage() override;

private:
	enum class ESource : int32 { Pipeline = 0, Backend = 1 };

	void Fetch();
	void ApplyFilter();
	void Save();
	EActiveTimerReturnType Poll(double Now, float Dt);
	static bool IsProblem(const FString& Line);
	static const TCHAR* ServiceOf(ESource S);

	TAttribute<bool> IsActive;
	TSharedPtr<FFSDSStackStarter> Runner;
	TSharedPtr<SMultiLineEditableTextBox> TextBox;

	ESource Source = ESource::Pipeline;
	FString Filter;
	bool bProblemsOnly = false;
	bool bFollow = true;
	TArray<FString> Lines;
	FString Status;
	bool bStatusError = false;
	double LastFetch = -1000.0;
	bool bWasActive = false;
	FString Shown;   // what the box holds, so an unchanged refresh doesn't move it
};
