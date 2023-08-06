// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "ProcSim/Utils/RoadData.h"

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HierarchicalISMMaker.generated.h"

UCLASS()
class PROCSIM_API AHierarchicalISMMaker : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AHierarchicalISMMaker();

	// Roads will be added to this
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "HierarchicalMeshMaker")
	UHierarchicalInstancedStaticMeshComponent* HierarchicalRoadMesh;

	// Mesh for the roads
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HierarchicalMeshMaker")
	UStaticMesh* RoadMesh;

	// test
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HierarchicalMeshMaker")
	bool testBool;

	// for test purposes on blueprint construction script
	bool generated = false;

	UFUNCTION(BlueprintCallable, Category = "HierarchicalMeshMaker")
	void CPPConstruction();

	UFUNCTION(BlueprintCallable, Category = "ProceduralMeshMaker")
	void GenerateMesh(TArray<FVector> startPoints, TArray<FVector> endPoints, TArray<FMetaRoadData> roadData);

	virtual void BeginPlay() override;


	
};
