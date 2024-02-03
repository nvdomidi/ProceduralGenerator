// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "ProcSim/MapGen/MapGen.h"
#include "ProcSim/BlocksGen/Graph.h"
#include "ProcSim/BlocksGen/GraphVertex.h"
#include "ProcSim/BlocksGen/Parcel.h"
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

	UPROPERTY()
	UProceduralMeshComponent* ProceduralMesh;

	/* Turns intersections and segments into graph */
	Graph<GraphVertex*>* MakeGraph(std::vector<Intersection*> intersections, std::vector<Segment*> segments);

	/* Find faces from the graph created */
	TArray<TArray<int>> FindFaces(Graph<GraphVertex*>* graph);

	/* Parcel the found faces into smaller blocks */
	TArray<Block*> ParcelBlocks(Graph<GraphVertex*>* graph, FVector midpoint);

	/* Creates procedural mesh component for the input parcel */
	void ParcelToMesh(const Parcel* p, const FVector midPoint, int section);

	/* Turn a face into a parcel */
	Parcel* faceToParcel(Graph<GraphVertex*>* graph, std::vector<int> face);

	/* set actor for intersection showing */
	void SetBlueprints(TSubclassOf<AActor> beforeBP, TSubclassOf<AActor> afterBP);

	std::vector<Intersection*> in11;
	TArray<FVector> cyclepositions{};

	TSubclassOf<AActor> befBP;
	TSubclassOf<AActor> aftBP;
};
