// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <vector>

#include "ProcSim/MapGen/MapGen.h"
#include "ProcSim/Utils/RoadData.h"
#include "ProceduralMeshComponent.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshMaker.generated.h"



UCLASS()
class PROCSIM_API AProceduralMeshMaker : public AActor
{
	GENERATED_BODY()
	
public:

	AProceduralMeshMaker();

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "ProceduralMeshMaker")
	UProceduralMeshComponent* ProceduralRoadMesh;

	
	bool generated = false;

	/* for test purposes. used inside blueprint constructor to test things */
	UFUNCTION(BlueprintCallable, Category = "ProceduralMeshMaker")
	void CPPConstruction();

	/* takes in FVector start and end points and roadmetadata and creates the procedural mesh */
	void GenerateMesh(TArray<FVector> startPoints, TArray<FVector> endPoints, TArray<FMetaRoadData> roadData);

	/* this function can be used to make the mesh connecting the intersections */
	void GenerateMeshIntersections(std::vector<Intersection*> intersections, float height = 40.0f);

	/* helper functions to generate vertices, triangles, UVs for the procedural mesh */
	TArray<FVector> CalculateVerticesForProceduralMesh(TArray<FVector> startPoints, TArray<FVector> endPoints, TArray<FMetaRoadData> roadData);
	TArray<int> CalculateTrianglesForProceduralMesh(TArray<FVector> vertices);
	TArray<FVector2D> CalculateUVsForProceduralMesh(TArray<FVector> vertices);
};
