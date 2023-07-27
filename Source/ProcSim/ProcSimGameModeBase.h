// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once


#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "ProcSimGameModeBase.generated.h"

/**
 * 
 */
UCLASS()
class PROCSIM_API AProcSimGameModeBase : public AGameModeBase
{
	GENERATED_BODY()

		virtual void BeginPlay();
};
