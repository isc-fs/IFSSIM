#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "FSDSCustomMapLoader.generated.h"

/**
 * Blueprint function library for loading track definitions from CSV files.
 * Used by custom maps to define cone positions procedurally.
 */
UCLASS()
class FSDSPLUGIN_API UFSDSCustomMapLoader : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, Category = "FSDS Map Loader")
	static bool FileLoadString(FString FileName, FString& OutText);

	UFUNCTION(BlueprintCallable, Category = "FSDS Map Loader")
	static TArray<FString> ProcessFile(FString Data, TArray<FTransform>& BlueCones, TArray<FTransform>& YellowCones, TArray<FTransform>& BigOrangeCones);

	UFUNCTION(BlueprintCallable, Category = "FSDS Map Loader")
	static FTransform GetFinishTransform(TArray<FTransform> BigOrangeCones);

	UFUNCTION(BlueprintCallable, Category = "FSDS Map Loader")
	static void SetCustomMapPath(FString Path);

	UFUNCTION(BlueprintCallable, Category = "FSDS Map Loader")
	static FString GetCustomMapPath();
};
