// GncBytesLibrary: text <-> bytes for Blueprints, for the rover's TCP link.
//
// The TCP-Unreal component receives and emits byte arrays; the rover Blueprint
// turns the bytes from component_rover_leo_raph into the JSON text it parses
// (wheel commands, headlights) and its telemetry JSON into bytes to emit:
//   On Received Bytes -> GNC Bytes To String -> Load Json from String ...
//   Format Text (telemetry) -> GNC String To Bytes -> Emit
// Same conversions as the SocketIO plugin's CoreUtility nodes it replaces, so
// that plugin is not needed.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "GncBytesLibrary.generated.h"

UCLASS()
class UNREALSIMCLOCK_API UGncBytesLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:
	// Bytes as text: UTF-8 (or UTF-16 with a byte-order mark).
	UFUNCTION(BlueprintPure, Category = "GNC|Bytes", meta = (DisplayName = "GNC Bytes To String"))
	static FString BytesToString(const TArray<uint8>& Bytes);

	// Text as UTF-8 bytes (no terminating zero).
	UFUNCTION(BlueprintPure, Category = "GNC|Bytes", meta = (DisplayName = "GNC String To Bytes"))
	static TArray<uint8> StringToBytes(const FString& Text);
};
