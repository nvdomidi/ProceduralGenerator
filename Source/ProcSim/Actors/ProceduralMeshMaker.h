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

/* namespace is used for mathematical functions to calculate things for roads*/
namespace roadMath {

	// Takes start and end point of a segment and calculates the 4 corner coordinates
	inline TArray<FVector> FindFourCornersFromStartAndEndPoint(FVector startPoint, FVector endPoint, float width) {
		FVector2D direction{ endPoint.X - startPoint.X, endPoint.Y - startPoint.Y };
		FVector2D perpendicularDirection{ -direction.Y, direction.X };
		perpendicularDirection.Normalize();
		FVector direction3D{ perpendicularDirection.X, perpendicularDirection.Y, 0.0 };

		return TArray<FVector>{ startPoint + (direction3D * width / 2), startPoint - (direction3D * width / 2),
			endPoint + (direction3D * width / 2), endPoint - (direction3D * width / 2)};
	}

	// Takes in all roads, subdivides each road into length specified
	inline void SubdivideRoadsByLength(TArray<FVector>& startPoints, TArray<FVector>& endPoints,
		TArray<FMetaRoadData>& roadData, float length) {

		TArray<FVector> startRoadParts{};
		TArray<FVector> endRoadParts{};
		TArray<FMetaRoadData> roadPartsData{};

		if (startPoints.Num() != endPoints.Num()) {
			UE_LOG(LogTemp, Error, TEXT("Number of starting points and ending points are not the same!"));
		}
		for (int i = 0; i < startPoints.Num(); i++) {
			float roadLength = FVector::Distance(startPoints[i], endPoints[i]);
			int numRoadParts = floor(roadLength / length);

			FVector direction = endPoints[i] - startPoints[i];
			direction.Normalize();

			/* divide into default roadpart length, then make a smaller segment for the remaining*/
			if (numRoadParts == 0) {
				startRoadParts.Add(startPoints[i]);
				endRoadParts.Add(endPoints[i]);
				roadPartsData.Add(roadData[i]);
				continue;
			}
			for (int j = 0; j < numRoadParts; j++) {
				startRoadParts.Add(startPoints[i] + j * direction * length);
				endRoadParts.Add(startPoints[i] + (j + 1) * direction * length);
				roadPartsData.Add(roadData[i]);
			}
			if (FMath::Fmod(roadLength, length) > 1e-1) {
				startRoadParts.Add(startPoints[i] + (numRoadParts)*direction * length);
				endRoadParts.Add(endPoints[i]);
				roadPartsData.Add(roadData[i]);
			}
		}

		startPoints = startRoadParts;
		endPoints = endRoadParts;
		roadData = roadPartsData;
	}
}