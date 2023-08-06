// Fill out your copyright notice in the Description page of Project Settings.


#include "HierarchicalISMMaker.h"

// Sets default values
AHierarchicalISMMaker::AHierarchicalISMMaker()
{
	PrimaryActorTick.bCanEverTick = true;
	HierarchicalRoadMesh = CreateDefaultSubobject<UHierarchicalInstancedStaticMeshComponent>(TEXT("HierarchicalRoadMesh"));
	RootComponent = HierarchicalRoadMesh;
}

void AHierarchicalISMMaker::CPPConstruction()
{
	UE_LOG(LogTemp, Warning, TEXT("OnConstruction called"));
	
	UE_LOG(LogTemp, Warning, TEXT("the test bool is: %f"), testBool);

	if (true) {

		if (RoadMesh == nullptr) {
			UE_LOG(LogTemp, Error, TEXT("The road mesh is null!"));
			return;
		}
		UE_LOG(LogTemp, Warning, TEXT("The road mesh is not null!"));

		HierarchicalRoadMesh = NewObject<UHierarchicalInstancedStaticMeshComponent>(this, TEXT("HierarchicalRoadMesh"));
		RootComponent = HierarchicalRoadMesh;
		HierarchicalRoadMesh->SetStaticMesh(RoadMesh);

		FTransform transform = GetTransform();
		// generate stuff here
		for (int i = 0; i < 4; i++) {
			
			transform.SetLocation(transform.GetLocation() + i * FVector{0.0f, 40.0f, 0.0f});

			HierarchicalRoadMesh->AddInstance(transform);
		}
	}
}

void AHierarchicalISMMaker::GenerateMesh(TArray<FVector> startPoints, TArray<FVector> endPoints, TArray<FMetaRoadData> roadData)
{
}

void AHierarchicalISMMaker::BeginPlay()
{
	Super::BeginPlay();
	UE_LOG(LogTemp, Warning, TEXT("Navid is anything working!"));
	UE_LOG(LogTemp, Warning, TEXT("beginplay bool is: %d"), testBool);
	CPPConstruction();
}
