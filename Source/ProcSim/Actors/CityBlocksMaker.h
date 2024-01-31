// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "ProcSim/MapGen/MapGen.h"
#include "ProcSim/BlocksGen/Graph.h"
#include "ProcSim/BlocksGen/MinimalCycleBasis.h"
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
	Graph<Intersection> MakeGraph(std::vector<Intersection*> intersections, std::vector<Segment*> segments);

	/* Make cycles from the graph created */
	TArray<TArray<int>> MakeCycles(Graph<Intersection*> graph);

	/* Find faces from the graph created */
	TArray<TArray<int>> FindFaces(Graph<Intersection> graph);

	/* Remove the outer cycles */
	TArray<TArray<int>> RemoveOuterCycles(TArray<TArray<int>> cycles, Graph<Intersection*> graph);

	/* recursive function */
	void DFSCycles(std::shared_ptr<gte::MinimalCycleBasis<double>::Tree> tree, int& num);

	/* Minimum Cycle Basis Extraction from the graph */
	TArray<TArray<int>> MinimumCycleBasis(std::vector<Intersection*> intersections, std::vector<Segment*> segments);

	/* Parcel the found faces into smaller blocks */
	void ParcelBlocks(Graph<Intersection> graph, FVector midpoint);

	/* set actor for intersection showing */
	void SetBlueprints(TSubclassOf<AActor> beforeBP, TSubclassOf<AActor> afterBP);



	/*
	int getMostCCW(int v, TArray<int> candidates, Point prevEdge, Graph<Intersection*> graph);
	bool isCCW(TArray<int> face, Graph<Intersection*> graph);
	int getBestFaceCandidate(int nextVert, TArray<int> candidates, Point prevEdge, Graph<Intersection*> graph);
	*/

	std::vector<Intersection*> in11;
	TArray<FVector> cyclepositions{};

	TSubclassOf<AActor> befBP;
	TSubclassOf<AActor> aftBP;
};
