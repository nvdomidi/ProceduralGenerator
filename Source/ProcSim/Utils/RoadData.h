#pragma once

//#include "CoreMinimal.h"
//#include "Kismet/BlueprintFunctionLibrary.h"
#include "RoadData.generated.h"
/* Used to store information about each road segment */
USTRUCT(BlueprintType)
struct FMetaRoadData {

	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite, Category = "RoadData")
	bool isHighway;
	UPROPERTY(BlueprintReadWrite, Category = "RoadData")
	float roadWidth;
	// more to come here
};

/* Used to determine how straight the road is */
UENUM(BlueprintType)
enum class ESTRAIGHTNESS : uint8 {
	SE_CURVED       UMETA(DisplayName = "Curved"),
	SE_STRAIGHT        UMETA(DisplayName = "Straight"),
	SE_VERYSTRAIGHT        UMETA(DisplayName = "VeryStraight"),
};