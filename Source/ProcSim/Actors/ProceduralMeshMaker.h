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

	// This function is used to create the mesh in the editor for test reasons
	//virtual void OnConstruction(const FTransform& Transform) override;
	bool generated = false;

	UFUNCTION(BlueprintCallable, Category = "ProceduralMeshMaker")
	void CPPConstruction();

	UFUNCTION(BlueprintCallable, Category = "ProceduralMeshMaker")
	void GenerateMesh(TArray<FVector> startPoints, TArray<FVector> endPoints, TArray<FMetaRoadData> roadData);

	void GenerateProceduralRoads(std::vector<Segment*> segments, std::vector<Intersection*> intersections, float height = 40.0f);

	//UFUNCTION(BlueprintCallable, Category = )
	void GenerateMeshIntersections(std::vector<Intersection*> intersections, float height = 40.0f);

	TArray<FVector> CalculateVerticesForProceduralMesh(TArray<FVector> startPoints, TArray<FVector> endPoints, TArray<FMetaRoadData> roadData);
	TArray<int> CalculateTrianglesForProceduralMesh(TArray<FVector> vertices);
	TArray<FVector2D> CalculateUVsForProceduralMesh(TArray<FVector> vertices);
};
