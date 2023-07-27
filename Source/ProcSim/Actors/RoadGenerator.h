// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "ProcSim/Utils/Straightness.h"
#include "ProcSim/MapGen/Quadtree.h"
#include "ProcSim/MapGen/Math.h"
#include "ProcSim/MapGen/MapGen.h"
#include "ProcSim/MapGen/SimplexNoise.h"
#include "ProcSim/Utils/ImageHandler.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine/StaticMeshActor.h"
#include "RoadGenerator.generated.h"

UCLASS()
class PROCSIM_API ARoadGenerator : public AActor
{
	GENERATED_BODY()
	
public:
	
	/* Choose Image from Directory and load onto plane */
	UFUNCTION(BlueprintCallable, Category = "RoadGenerator")
	void ChooseImageAndApplyToPlane(UProceduralMeshComponent* PlaneReference);

	/* Create random heatmap and apply to plane*/
	UFUNCTION(BlueprintCallable, Category = "RoadGenerator")
	void CreateRandomHeatmapAndApplyToPlane(UProceduralMeshComponent* PlaneReference, bool completelyRandom = false);

	/* Create Roads after heatmap is created */
	UFUNCTION(BlueprintCallable, Category = "RoadGenerator")
	bool CreateRoads(FVector regionStartPoint, FVector regionEndPoint, ESTRAIGHTNESS straightness);

	/* Segments to splines */
	UFUNCTION(BlueprintCallable, Category = "RoadGenerator")
	void RoadSegmentsToStartAndEndPoints(TArray<FVector>& startPoints, TArray<FVector>& endPoints, float z = 40.0f);

	UPROPERTY()
	TArray<uint8> pixels;

	Heatmap* heatmap = nullptr;
	std::vector<Segment*> segments;

	// Sets default values for this actor's properties
	ARoadGenerator();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	
};
