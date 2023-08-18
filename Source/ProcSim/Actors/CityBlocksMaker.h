// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "ProcSim/MapGen/MapGen.h"
#include "ProcSim/BlocksGen/Graph.h"
#include "ProceduralMeshComponent.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "CityBlocksMaker.generated.h"

UCLASS()
class PROCSIM_API ACityBlocksMaker : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ACityBlocksMaker();

	UProceduralMeshComponent* ProceduralMesh;

	/* Turns intersections and segments into graph */
	Graph<Intersection*> MakeGraph(std::vector<Intersection*> intersections, std::vector<Segment*> segments);

	/* Make cycles from the graph created */
	TArray<TArray<int>> MakeCycles(Graph<Intersection*> graph);
};
