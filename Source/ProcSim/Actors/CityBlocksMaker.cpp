// Fill out your copyright notice in the Description page of Project Settings.


#include "ProcSim/Actors/CityBlocksMaker.h"
#include "ProcSim/Actors/ProceduralMeshMaker.h"
#include "ProcSim/MapGen/Config.h"
#include "ProcSim/MapGen/Math.h"

#include "ProceduralMeshComponent.h"

#include <algorithm>
#include <array>
#include <unordered_map>
#include <set>


// Sets default values
ACityBlocksMaker::ACityBlocksMaker()
{
	ProceduralMesh = CreateDefaultSubobject<UProceduralMeshComponent>(FName("CityBlocksMakerProcMesh"));
	RootComponent = ProceduralMesh;
}

/* This function turns segments and intersections into a graph */
Graph<GraphVertex*>* ACityBlocksMaker::MakeGraph(std::vector<Intersection*> intersections, std::vector<Segment*> segments)
{

	Graph<GraphVertex*>* graph = new Graph<GraphVertex*>{};
	
	// Node ID: intersection ID, Node MetaData: GraphVertex pointer
	for (auto intersection : intersections) {
		GraphVertex* node = new GraphVertex(intersection->position, intersection->ID);
		graph->AddNode(node->ID, node);
	}

	for (auto segment : segments) {
		graph->AddEdge(segment->startIntersectionID, segment->endIntersectionID);
	}
	return graph;
}

TArray<TArray<int>> ACityBlocksMaker::FindFaces(Graph<GraphVertex*>* graph)
{
	TArray<TArray<int>> faces{};
	graph->FindFaces();
	std::vector<std::vector<int>> fs = graph->faces;

	for (auto f : fs) {
		TArray<int> face{};
		for (auto node : f) {
			face.Add(node);
		}
		faces.Add(face);
	}

	return faces;
}

// This function takes graph and loop of integers as input and returns array of graphVertices
Parcel* ACityBlocksMaker::faceToParcel(Graph<GraphVertex*>* graph, std::vector<int> face)
{
	std::vector<GraphVertex*> graphNodes{};

	for (int i : face) {
		GraphVertex* graphNode = graph->vertices[i]->data;
		graphNodes.push_back(graphNode);
	}

	return new Parcel(graphNodes);
}

TArray<Block*> ACityBlocksMaker::ParcelBlocks(Graph<GraphVertex*>* graph, FVector midpoint)
{
	// Step 1: Find faces of the city graph
	graph->FindFaces();

	// Step 2: Turn each city graph face into one parcel
	std::vector<std::vector<int>> cityFaces = graph->faces;
	std::vector<Parcel*> cityFacesParcels{};

	for (std::vector<int> cityFace : cityFaces) {
		Parcel* faceParcel = faceToParcel(graph, cityFace);
		cityFacesParcels.push_back(faceParcel);
	}

	// Spawn something at each node of each parcel (visualization)
	//for (Parcel* cityFaceParcel : cityFacesParcels) {
	//	for (GraphVertex* node : cityFaceParcel->face) {
	//		GetWorld()->SpawnActor<AActor>(this->befBP, FVector{ static_cast<float>(node->position.x * 100 + midpoint.X),
	//			static_cast<float>(node->position.y * 100 + midpoint.Y), 60.0f }, FRotator{}, FActorSpawnParameters{});
	//	}
	//}

	// Step 3: Inset every parcel by a constant perpendiculat amount
	for (Parcel* cityFaceParcel : cityFacesParcels) {
		cityFaceParcel->insetParcelUniformXY(20, 5);
	}

	// Spawn something at each node of each parcel (visualization)
	for (Parcel* cityFaceParcel : cityFacesParcels) {
		for (GraphVertex* node : cityFaceParcel->face) {
			GetWorld()->SpawnActor<AActor>(this->aftBP, FVector{ static_cast<float>(node->position.x * 100 + midpoint.X),
				static_cast<float>(node->position.y * 100 + midpoint.Y), 60.0f }, FRotator{}, FActorSpawnParameters{});
		}
	}

	// Subdivide parcels
	/*
	Parcel* selected = cityFacesParcels[0];

	FString selectedString = "parcel: ";

	for (GraphVertex* node : selected->face) {
		selectedString += (FString::Printf(TEXT("ID:%d(%f,%f)"), node->ID, node->position.x, node->position.y));
	}

	UE_LOG(LogTemp, Warning, TEXT("SelectedParcel: %s"), *selectedString);

	Block* lotBlock = new Block();
	lotBlock->parcels = std::vector<Parcel*>{selected};


	UE_LOG(LogTemp, Warning, TEXT("subdivided--> length BEFORE: %d"), lotBlock->parcels.size());

	lotBlock->subdivideParcels();

	UE_LOG(LogTemp, Warning, TEXT("subdivided--> length AFTER: %d"), lotBlock->parcels.size());
	*/
	TArray<Block*> allBlocks{};

	for (Parcel* cityFaceParcel : cityFacesParcels) {
		Block* lotBlock = new Block();
		lotBlock->parcels = std::vector<Parcel*>{ cityFaceParcel };
		lotBlock->subdivideParcels(20.0f, -0.2f, 0.2f, false, 3);
		
		// for visualization of the center
		for (Parcel* lotParcel : lotBlock->parcels) {
			
			FVector2D pos{ 0.0f, 0.0f };
			for (GraphVertex* node : lotParcel->face) {
				pos += FVector2D{float(node->position.x), float(node->position.y)};
			}
			pos /= lotParcel->face.size();

			GetWorld()->SpawnActor<AActor>(this->befBP, FVector{ static_cast<float>(pos.X * 100 + midpoint.X),
				static_cast<float>(pos.Y * 100 + midpoint.Y), 60.0f }, FRotator{}, FActorSpawnParameters{});
		}

		allBlocks.Add(lotBlock);

	}

	int section = 0;

	for (Block* b : allBlocks) {
		for (Parcel* p : b->parcels) {
			ParcelToMesh(p, midpoint, section);
			section++;
			//parcelMesh->AttachToComponent(this->GetOwner()->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
		}
	}

	return allBlocks;
}

void ACityBlocksMaker::ParcelToMesh(const Parcel* p, const FVector midPoint, int section)
{
	FString name = FString::Printf(TEXT("p%d-%d"), p->face.front()->ID, p->face.back()->ID);
	//auto proc = CreateDefaultSubobject<UProceduralMeshComponent>(FName(name));

	//auto proc = NewObject<UProceduralMeshComponent>(this, UProceduralMeshComponent::StaticClass(), FName(name));
	
	TArray<FVector> allVerts{};
	TArray<int> allTris{};
	
	for (int i = 0; i < p->face.size(); i++) {
		auto currNode = p->face[i];
		auto nextNode = p->face[(i + 1) % p->face.size()];
		FVector currPos = FVector{ float(currNode->position.x), float(currNode->position.y), 0.0f};
		FVector nextPos = FVector{ float(nextNode->position.x), float(nextNode->position.y), 0.0f};

		// to UE coords
		currPos = currPos * 100 + midPoint;
		nextPos = nextPos * 100 + midPoint;

		TArray<FVector> segVerts = roadMath::FindFourCornersFromStartAndEndPoint(currPos, nextPos, 250.0f);

		TArray<int> segTris = TArray<int>{ i * 4, i * 4 + 2, i * 4 + 1,
			/* two triangles */  i * 4 + 2, i * 4 + 3, i * 4 + 1 };

		allVerts.Append(segVerts);
		allTris.Append(segTris);
	}

	this->ProceduralMesh->CreateMeshSection(section, allVerts, allTris, TArray<FVector>(),
		TArray<FVector2D>{}, TArray<FColor>(), TArray<FProcMeshTangent>(), true);
	
	
}



void ACityBlocksMaker::SetBlueprints(TSubclassOf<AActor> beforeBP, TSubclassOf<AActor> afterBP)
{
	this->befBP = beforeBP;
	this->aftBP = afterBP;
}

