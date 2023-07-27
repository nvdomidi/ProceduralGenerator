#pragma once

// This enum is used to determine the straightness of the roads generated

UENUM(BlueprintType)
enum class ESTRAIGHTNESS : uint8 {
    SE_CURVED       UMETA(DisplayName="Curved"),
    SE_STRAIGHT        UMETA(DisplayName="Straight"),
    SE_VERYSTRAIGHT        UMETA(DisplayName="VeryStraight"),
};