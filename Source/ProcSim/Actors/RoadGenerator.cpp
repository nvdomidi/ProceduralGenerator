// Fill out your copyright notice in the Description page of Project Settings.
#include <memory>

#include "RoadGenerator.h"
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

void ARoadGenerator::ChooseImageAndApplyToPlane(UProceduralMeshComponent* PlaneReference)
{
	FString filePath = ImageHandler::ChooseImageFromFileDialog();

	int outWidth;
	int outHeight;

	bool loaded = ImageHandler::LoadImageFromFile(filePath, this->pixels, outWidth, outHeight);

	if (loaded) {

		auto Texture = ImageHandler::PixelsToTexture(this->pixels, outWidth, outHeight);

		if (Texture == nullptr) {
			UE_LOG(LogTemp, Error, TEXT("Couldn't open Heatmap"));
			return;
		}

		ImageHandler::ApplyTextureToProceduralMeshComponent(PlaneReference, Texture);
	}

	std::unique_ptr<unsigned char[]> loadedImage(new unsigned char[outWidth * outHeight]);

	for (int y = 0; y < outHeight; y++) {
		for (int x = 0; x < outWidth; x++) {
			loadedImage[y * outWidth + x] = pixels[y * outWidth + x];
		}
	}

	this->heatmap = new Heatmap(loadedImage.release(), outWidth, outHeight);

}

void ARoadGenerator::CreateRandomHeatmapAndApplyToPlane(UProceduralMeshComponent* PlaneReference, bool completelyRandom)
{
	Config::COMPLETELYRANDOM = completelyRandom;
	int width = 800; int height = 800;
	Heatmap *heat = new Heatmap(width, height);
	
	this->pixels.Empty();

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			this->pixels.Add(static_cast<uint8>(heat->image[y * width + x]));
		}
	}

	auto Texture = ImageHandler::PixelsToTexture(this->pixels, width, height);

	if (Texture == nullptr) {
		UE_LOG(LogTemp, Warning, TEXT("Couldnt make random heatmap"));
		return;
	}

	ImageHandler::ApplyTextureToProceduralMeshComponent(PlaneReference, Texture);

	this->heatmap = heat;
}

bool ARoadGenerator::CreateRoads(FVector regionStartPoint, FVector regionEndPoint)
{
	if (this->heatmap == nullptr)
		return false;
	
	Config::minx = -abs((regionStartPoint.X/100) - (regionEndPoint.X/100)) / 2;
	Config::maxx = +abs((regionStartPoint.X/100) - (regionEndPoint.X/100)) / 2;

	Config::miny = -abs((regionStartPoint.Y/100) - (regionEndPoint.Y/100)) / 2;
	Config::maxy = +abs((regionStartPoint.Y/100) - (regionEndPoint.Y/100)) / 2;


	PriorityQueue<Segment*> priorityQ([](const Segment* s) { return s->t; });
	std::vector<Segment*> initialSegments = makeInitialSegments();

	for (auto initialSegment : initialSegments) {
		priorityQ.enqueue(initialSegment);
	}

	Quadtree<Segment*> qTree(*(new Bounds{ -2e4,
			-2e4,
			4e4,
			4e4 }), Config::QUADTREE_MAX_OBJECTS, Config::QUADTREE_MAX_LEVELS);

	while (!priorityQ.empty() && segments.size() < Config::SEGMENT_COUNT_LIMIT) {
		generationStep(priorityQ, segments, qTree, *(this->heatmap));
	}

	FVector midPoint = (regionStartPoint + regionEndPoint) / 2;
	
	UE_LOG(LogTemp, Warning, TEXT("segments craeted %d"), segments.size());

	for (auto& segment : segments) {
		
		segment->start = segment->start * 100 + Point(midPoint.X, midPoint.Y);
		segment->end = segment->end * 100 + Point(midPoint.X, midPoint.Y);

	}

	return true;
}

void ARoadGenerator::RoadSegmentsToStartAndEndPoints(TArray<FVector>& startPoints, TArray<FVector>& endPoints, float z)
{
	startPoints.Empty();
	endPoints.Empty();

	for (auto segment : this->segments) {
		startPoints.Add(*(new FVector(segment->start.x,  segment->start.y, z)));
		endPoints.Add(*(new FVector(segment->end.x, segment->end.y, z)));
	}

}
