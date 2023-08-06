// Fill out your copyright notice in the Description page of Project Settings.
#include "RoadGenerator.h"
#include "ProcSim/Actors/ProceduralMeshMaker.h"

#include <memory>
#include "ProcSim/Utils/ImageHandler.h"


// Sets default values
ARoadGenerator::ARoadGenerator()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ARoadGenerator::BeginPlay()
{
	Super::BeginPlay();
}


// Choose image from disk and apply it to proceduralmeshcomponent
void ARoadGenerator::ChooseImageAndApplyToPlane(UProceduralMeshComponent* PlaneReference)
{
	FString filePath = ImageHandler::ChooseImageFromFileDialog();

	if (filePath.IsEmpty())
		return;

	int outWidth;
	int outHeight;

	bool loaded = ImageHandler::LoadImageFromFile(filePath, this->pixels, outWidth, outHeight);

	if (loaded) {

		auto Texture = ImageHandler::PixelsToTexture(this->pixels, outWidth, outHeight);

		if (Texture == nullptr) {
			UE_LOG(LogTemp, Error, TEXT("Couldn't open Heatmap"));
			return;
		}

		ImageHandler::ApplyTextureToProceduralMeshComponent(PlaneReference, Texture, "/Game/Materials/HeatmapMaterial");
	}

	/* image is stored as shared pointer to unsigned character array */
	std::unique_ptr<unsigned char[]> loadedImage(new unsigned char[outWidth * outHeight]);

	for (int y = 0; y < outHeight; y++) {
		for (int x = 0; x < outWidth; x++) {
			loadedImage[y * outWidth + x] = pixels[y * outWidth + x];
		}
	}

	/* create heatmap from the loaded image */
	this->heatmap = new Heatmap(loadedImage.release(), outWidth, outHeight);

}

// If completely random, the heatmap will be made of simplex noise
void ARoadGenerator::CreateRandomHeatmapAndApplyToPlane(UProceduralMeshComponent* PlaneReference, bool completelyRandom)
{
	/* parameters for the heatmap */
	Config::COMPLETELYRANDOM = completelyRandom;
	int width = 800; int height = 800;
	Heatmap *heat = new Heatmap(width, height);
	
	/* pixels TArray is populated with the data from the generated heatmap image*/
	this->pixels.Empty();

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			this->pixels.Add(static_cast<uint8>(heat->image[y * width + x]));
		}
	}

	/* create texture and apply it to the plane*/
	auto Texture = ImageHandler::PixelsToTexture(this->pixels, width, height);

	if (Texture == nullptr) {
		UE_LOG(LogTemp, Warning, TEXT("Couldnt make random heatmap"));
		return;
	}

	ImageHandler::ApplyTextureToProceduralMeshComponent(PlaneReference, Texture, FString("/Game/Materials/HeatmapMaterial"));

	this->heatmap = heat;
}

// Main algorithm for creating the 3D roads
bool ARoadGenerator::CreateSegmentsAndIntersections(FVector regionStartPoint, FVector regionEndPoint, ESTRAIGHTNESS straightness, int numSegments)
{
	if (this->heatmap == nullptr)
		return false;
	
	/* The boundary for generation is calculated from the selected region. divide by 100 to transform units*/
	Config::minx = -abs((regionStartPoint.X/100) - (regionEndPoint.X/100)) / 2;
	Config::maxx = +abs((regionStartPoint.X/100) - (regionEndPoint.X/100)) / 2;

	Config::miny = -abs((regionStartPoint.Y/100) - (regionEndPoint.Y/100)) / 2;
	Config::maxy = +abs((regionStartPoint.Y/100) - (regionEndPoint.Y/100)) / 2;

	Config::STRAIGHTNESS = straightness;
	Config::SEGMENT_COUNT_LIMIT = numSegments;

	/* generation algorithm starts here*/
	PriorityQueue<Segment*> priorityQ([](const Segment* s) { return s->t; });
	DebugData debugData;
	std::vector<Segment*> initialSegments = makeInitialSegments();

	for (auto initialSegment : initialSegments) {
		priorityQ.enqueue(initialSegment);
	}

	Quadtree<Segment*> qTree(*(new Bounds{ -2e4,
			-2e4,
			4e4,
			4e4 }), Config::QUADTREE_MAX_OBJECTS, Config::QUADTREE_MAX_LEVELS);

	while (!priorityQ.empty() && segments.size() < Config::SEGMENT_COUNT_LIMIT) {
		generationStep(priorityQ, segments, qTree, debugData, intersections, *(this->heatmap));
	}
	/* generation algorithm ends here*/

	UE_LOG(LogTemp, Warning, TEXT("Number of segments created: %d"), segments.size());
	UE_LOG(LogTemp, Warning, TEXT("Number of intersections: %d, intersectionsradius: %d, snaps: %d"),
		debugData.intersections.size(), debugData.intersectionsRadius.size(), debugData.snaps.size());

	FVector midPoint = (regionStartPoint + regionEndPoint) / 2;

	// segments to unreal engine coordinates and remove segments outside of range specified
	bringSegmentsIntoUnrealEngineCoordinatesAndRemoveOutsideOfRegion(segments, regionStartPoint, regionEndPoint);
	// 
	// intersections might be spawned with equal positions
	removeDuplicateIntersections(intersections);

	// merge intersections close to eachother
	mergeCloseIntersections(intersections);

	// bring intersections to unreal engine coordinates ( to cm + offest )
	bringIntersectionsIntoUnrealEngineCoordinates(intersections, midPoint);

	// cut the roads leading into the intersection
	//cutRoadsLeadingIntoIntersections(segments, intersections);

	// make procuderal intersections for twoway, threeway and fourway


	/* use this to visualize the intersections*/
	CreateIntersections(midPoint);

	/* fix ordering of roads in Z value*/
	findOrderOfRoads(segments);

	/* generate intersections procedural mesh */
	TArray<FVector> startPoints, endPoints;
	TArray<FMetaRoadData> roadData;
	RoadSegmentsToStartAndEndPoints(startPoints, endPoints, roadData, 40.0f);

	CreateProceduralMeshForRoads(startPoints, endPoints, roadData);

	return true;
}

void ARoadGenerator::CreateProceduralMeshForRoads(TArray<FVector> startPoints, TArray<FVector> endPoints, TArray<FMetaRoadData> roadData)
{


	this->ProceduralMeshMaker = Cast<AProceduralMeshMaker>(GetWorld()->SpawnActor<AProceduralMeshMaker>(FActorSpawnParameters{}));
	this->ProceduralMeshMaker->GenerateMesh(startPoints, endPoints, roadData);
}

// you have to calls this first before CreateIntersections
void ARoadGenerator::SetIntersectionBlueprints(TSubclassOf<AActor> TwoWay, TSubclassOf<AActor> ThreeWay,
	TSubclassOf<AActor> FourWay, TSubclassOf<AActor> MoreThanFourWay)
{
	this->TwoWayBlueprint = TwoWay;
	this->ThreeWayBlueprint = ThreeWay;
	this->FourWayBlueprint = FourWay;
	this->MoreThanFourWayBlueprint = MoreThanFourWay;
}

// create intersections and spawn something on them
void ARoadGenerator::CreateIntersections(FVector midPoint)
{
	// first of all, lets create different markers for the different intersections

	for (auto intersection : intersections) {
		
		/* two way*/
		//if (intersection->branches.size() == 2) {
			//GetWorld()->SpawnActor<AActor>(this->TwoWayBlueprint, FVector{ static_cast<float>(intersection->position.x),
			//	static_cast<float>(intersection->position.y), 45.0f }, FRotator{}, FActorSpawnParameters{});
		//}
		//else if (intersection->branches.size() == 3) {
			//GetWorld()->SpawnActor<AActor>(this->ThreeWayBlueprint, FVector{ static_cast<float>(intersection->position.x),
			//	static_cast<float>(intersection->position.y), 45.0f }, FRotator{}, FActorSpawnParameters{});
		//}
		if (intersection->branches.size() >= 2) {
			float height{};
			int maxOrder{ -999999 };
			for (auto branch : intersection->branches) {
				if (intersection->ID == branch->startIntersectionID) {
					if (branch->startOrder > maxOrder)
						maxOrder = branch->startOrder;
				}
				else {
					if (branch->endOrder > maxOrder)
						maxOrder = branch->endOrder;
				}
			}
			UE_LOG(LogTemp, Warning, TEXT("maxOrder: %d"), maxOrder);
			GetWorld()->SpawnActor<AActor>(this->FourWayBlueprint, FVector{ static_cast<float>(intersection->position.x),
				static_cast<float>(intersection->position.y), 90.0f }, FRotator{}, FActorSpawnParameters{});
		}
		else if (intersection->branches.size() > 4) {
			//GetWorld()->SpawnActor<AActor>(this->MoreThanFourWayBlueprint, FVector{ static_cast<float>(intersection->position.x),
			//	static_cast<float>(intersection->position.y), 45.0f }, FRotator{}, FActorSpawnParameters{});
		}

		/*GetWorld()->SpawnActor<AActor>(this->RoadBlueprint, FVector{ static_cast<float>(intersection->position.x),
			static_cast<float>(intersection->position.y), 40.0f }, FRotator{}, FActorSpawnParameters{});*/
	}

}

// Creates lists of starting and ending points from the segments generated
void ARoadGenerator::RoadSegmentsToStartAndEndPoints(TArray<FVector>& startPoints, TArray<FVector>& endPoints,
	TArray<FMetaRoadData>& roadData, float z)
{
	startPoints.Empty();
	endPoints.Empty();
	roadData.Empty();

	for (auto segment : this->segments) {
		startPoints.Add(*(new FVector(segment->start.x,  segment->start.y, z + segment->startOrder)));
		endPoints.Add(*(new FVector(segment->end.x, segment->end.y, z + segment->endOrder)));
		roadData.Add(FMetaRoadData{ segment->q.highway, static_cast<float>(segment->width)  * 100});

	}
}

void ARoadGenerator::GenerateMeshIntersections(AProceduralMeshMaker* ProcMeshMaker)
{
	ProcMeshMaker->GenerateMeshIntersections(intersections);
}
