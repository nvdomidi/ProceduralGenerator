// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "ProcSim/Utils/RoadData.h"
#include "ProcSim/MapGen/Quadtree.h"
#include "ProcSim/MapGen/Math.h"
#include "ProcSim/MapGen/MapGen.h"
#include "ProcSim/MapGen/SimplexNoise.h"
#include "ProcSim/Utils/ImageHandler.h"
#include "ProcSim/Actors/ProceduralMeshMaker.h"
#include "ProcSim/Actors/CityBlocksMaker.h"


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
	bool CreateRoads(FVector regionStartPoint, FVector regionEndPoint, ESTRAIGHTNESS straightness, int numSegments);

	/* Divide City into Blocks */
	UFUNCTION(BlueprintCallable, Category = "RoadGenerator")
	bool CreateBlocks();
	
	/* set actor for intersection showing */
	UFUNCTION(BlueprintCallable, Category = "RoadGenerator")
	void SetIntersectionBlueprints(TSubclassOf<AActor> TwoWay, TSubclassOf<AActor> ThreeWay,
		TSubclassOf<AActor> FourWay, TSubclassOf<AActor> MoreThanFourWay, TSubclassOf<AActor> Intersection);

	/* Creates the procedural mesh maker and generates the mesh */
	void CreateProceduralMeshForRoads(TArray<FVector> startPoints, TArray<FVector> endPoints, TArray<FMetaRoadData> roadData);

	/* Transform coordinates from algorithm to unreal engine coordinates */
	void TransformToUECoordinates(FVector midPoint);

	/* Remove segments and intersections outside of the region */
	void RemoveOutsideOfRegion(FVector regionStartPoint, FVector regionEndPoint);

	/* Spawn something on intersections*/
	void CreateIntersections(FVector midPoint);

	/* Segments to splines */
	UFUNCTION(BlueprintCallable, Category = "RoadGenerator")
	void RoadSegmentsToStartAndEndPoints(TArray<FVector>& startPoints, TArray<FVector>& endPoints,
	TArray<FMetaRoadData>& roadData, float z = 40.0f);

	/* pass object from proceduralmeshmaker and generate mesh for the intersections */
	UFUNCTION(BlueprintCallable, Category = "RoadGenerator")
	void GenerateMeshIntersections(AProceduralMeshMaker* ProcMeshMaker);

	//UPROPERTY(EditAnywhere, Category = "RoadGenerator")
	TSubclassOf<AActor> RoadBlueprint;

	/* These blueprints are going to be markers for the intersections*/
	/* for test purposes */
	TSubclassOf<AActor> TwoWayBlueprint;
	TSubclassOf<AActor> ThreeWayBlueprint;
	TSubclassOf<AActor> FourWayBlueprint;
	TSubclassOf<AActor> MoreThanFourWayBlueprint;
	/* This blueprint is going to be actually spawned at the intersection location */
	TSubclassOf<AActor> IntersectionBlueprint;


	UPROPERTY()
	TArray<uint8> pixels;

	Heatmap* heatmap = nullptr;
	std::vector<Segment*> segments;
	std::vector<Intersection*> intersections;
	AProceduralMeshMaker* ProceduralMeshMaker = nullptr;
	ACityBlocksMaker* CityBlocksMaker = nullptr;

	// Sets default values for this actor's properties
	ARoadGenerator();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	
};
