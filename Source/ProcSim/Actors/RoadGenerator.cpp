// Fill out your copyright notice in the Description page of Project Settings.
#include "RoadGenerator.h"
#include "Engine/Classes/Components/TextRenderComponent.h"

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

void ARoadGenerator::VisualizeSegmentLinks() {
	for (auto segment : segments) {
		Point dir = segment->end - segment->start;
		dir = dir / dir.length();

		Point pos = segment->start + dir * segment->length() / 2;
		FVector pos3d{ static_cast<float>(pos.x), static_cast<float>(pos.y), 55.0f };

		auto bp = GetWorld()->SpawnActor<AActor>(this->IntersectionBlueprint, pos3d, FRotator{}, FActorSpawnParameters{});
		auto textlinks = Cast<UTextRenderComponent>(bp->GetComponentByClass(UTextRenderComponent::StaticClass()));
		textlinks->SetText(FString::Printf(TEXT("b: %d, f: %d"), segment->links_b.size(), segment->links_f.size()));
	}
}

// Iterate over segments, find close ones in qTree and check if there is intersection (there shouldn't be)
void ARoadGenerator::PrintConflictingSegments()
{
	int num = 0;
	for (auto segment : segments) {
		for (auto other : qTree->retrieve(segment->limits())) {
			auto in = other->intersectWith(segment);
			if (in != nullptr) {
				UE_LOG(LogTemp, Warning, TEXT("CONFLICT: segment: start(%f,%f) end(%f,%f) other: start(%f,%f) end(%f,%f)"),
					segment->start.x, segment->start.y, segment->end.x, segment->end.y,
					other->start.x, other->start.y, other->end.x, other->end.y)
					num++;
			}
		}
	}
	UE_LOG(LogTemp, Warning, TEXT("Num mistakes: %d"), num / 2);
	UE_LOG(LogTemp, Warning, TEXT("Hey is this functions getting claled?"));
}


// Main algorithm for creating the 3D roads
bool ARoadGenerator::CreateRoads(FVector regionStart, FVector regionEnd, ESTRAIGHTNESS straightness, int numSegments)
{
	regionStartPoint = regionStart;
	regionEndPoint = regionEnd;

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
	priorityQ = new PriorityQueue<Segment*>([](const Segment* s) { return s->t; });
	std::vector<Segment*> initialSegments = makeInitialSegments();

	for (auto initialSegment : initialSegments) {
		priorityQ->enqueue(initialSegment);
	}

	qTree = new Quadtree<Segment*>(*(new Bounds{ Config::minx,
			Config::miny,
			Config::maxx - Config::minx,
			Config::maxy - Config::miny }), Config::QUADTREE_MAX_OBJECTS, Config::QUADTREE_MAX_LEVELS);

	while (!priorityQ->empty() && segments.size() < Config::SEGMENT_COUNT_LIMIT) {
		generationStep(*priorityQ, segments, *qTree, debugData, intersections, *(this->heatmap));
	}
	/* generation algorithm ends here*/

	UE_LOG(LogTemp, Warning, TEXT("Number of segments created: %d"), segments.size());

	// remove conflicting segments
	removeConflictingSegments(segments, *qTree);
	// check if properly removed
	PrintConflictingSegments();

	populateSegmentLinks(segments);
	/* find intersections here */
	GenerateIntersections(segments, intersections);

	UE_LOG(LogTemp, Warning, TEXT("intersections size: %d"), intersections.size());

	return true;
}

void ARoadGenerator::ShowRoads()
{
	FVector midPoint = (regionStartPoint + regionEndPoint) / 2;

	this->TransformToUECoordinates(midPoint);

	// TEST: put this above the before
	this->RemoveOutsideOfRegion();

	/* fix ordering of roads in Z value*/

	findOrderOfRoads(segments);


	/* generate intersections procedural mesh */
	TArray<FVector> startPoints, endPoints;
	TArray<FMetaRoadData> roadData;
	RoadSegmentsToStartAndEndPoints(startPoints, endPoints, roadData, 40.0f);

	CreateProceduralMeshForRoads(startPoints, endPoints, roadData);

	/* use this to visualize the intersections*/
	CreateIntersections(midPoint);

	// show cycle centers
	for (auto face : this->faces) {
		Point centerPos{};

		for (int v : face) {
			centerPos = centerPos + graph.vertices[v]->data->position;
		}

		centerPos = centerPos / face.Num();
		FVector centerPosVec = FVector{ static_cast<float>(centerPos.x), static_cast<float>(centerPos.y), 70.0f };
		GetWorld()->SpawnActor<AActor>(this->IntersectionBlueprint, centerPosVec, FRotator{}, FActorSpawnParameters{});
	}



	UE_LOG(LogTemp, Warning, TEXT("Conflicting segments AFTER transforming to UE Coordinates: "));
	PrintConflictingSegments();
}

namespace roadMath {
	float cross_product(Point A, Point B, Point C) {
		return (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
	}

	bool isConvex(const TArray<Point>& points) {
		bool got_positive = false;
		bool got_negative = false;

		int n = points.Num();

		for (int i = 0; i < n; i++) {
			Point A = points[i];
			Point B = points[(i + 1) % n];
			Point C = points[(i + 2) % n];

			double cross = cross_product(A, B, C);

			if (cross < 0) {
				got_negative = true;
			}
			else if (cross > 0) {
				got_positive = true;
			}

			if (got_positive && got_negative) {
				return false;  // It's concave
			}
		}

		return true;  // It's convex
	}
}

/* Makes graph and divides into cycles*/
bool ARoadGenerator::CreateBlocks()
{
	this->CityBlocksMaker = Cast<ACityBlocksMaker>(GetWorld()->SpawnActor<ACityBlocksMaker>(FActorSpawnParameters{}));
	graph = this->CityBlocksMaker->MakeGraph(this->intersections, this->segments);
	
	int vertices = graph.vertices.size();

	int edges = segments.size();


	UE_LOG(LogTemp, Warning, TEXT("EULERS FORMULA: FACES = 2 - Vertices(%d) + Edges(%d) = %d"), vertices, edges, 2 - vertices + edges);
	UE_LOG(LogTemp, Warning, TEXT("Num intersections: %d"), intersections.size());

	this->faces = this->CityBlocksMaker->FindFaces(graph);

	int numFaces = 0;

	for (auto face : this->faces) {

		numFaces++;

		FString toPrint = FString::Printf(TEXT("Face_%d: "), numFaces);
		for (int v : face) {
			toPrint += (FString::FromInt(v) + ",");
		}

		UE_LOG(LogTemp, Warning, TEXT("%s"), *toPrint);


	}

	UE_LOG(LogTemp, Warning, TEXT("numFaces: %d"), numFaces);


	/*
	this->CityBlocksMaker->MinimumCycleBasis(intersections, segments);
	
	for (auto pos : this->CityBlocksMaker->cyclepositions) {
		GetWorld()->SpawnActor<AActor>(this->IntersectionBlueprint, pos, FRotator{}, FActorSpawnParameters{});
	}

	// spawning something at the mid position

	TArray<TArray<int>> cycles = {};

	for (TArray<int> cycle : cycles) {
		Point pos{};
		FString cyclestring = "";
		for (int i : cycle) {
			cyclestring += FString::Printf(TEXT("%d-"), i);
			pos = pos + graph.vertices[i]->data->position;
		}
		pos = pos / cycle.Num();

		FVector pos3d{ static_cast<float>(pos.x), static_cast<float>(pos.y), 45.0f };
		auto cyclebp = GetWorld()->SpawnActor<AActor>(this->IntersectionBlueprint, pos3d, FRotator{}, FActorSpawnParameters{});
		//auto textcycle = Cast<UTextRenderComponent>(cyclebp->GetComponentByClass(UTextRenderComponent::StaticClass()));
		//textcycle->SetText(cyclestring);
	}

	*/

	return true;
}

/* Creates the procedural mesh maker and generates the mesh */
void ARoadGenerator::CreateProceduralMeshForRoads(TArray<FVector> startPoints, TArray<FVector> endPoints, TArray<FMetaRoadData> roadData)
{
	this->ProceduralMeshMaker = Cast<AProceduralMeshMaker>(GetWorld()->SpawnActor<AProceduralMeshMaker>(FActorSpawnParameters{}));
	this->ProceduralMeshMaker->GenerateMesh(startPoints, endPoints, roadData);
}

/* Transform coordinates from algorithm to unreal engine coordinates and clean outside of region */
void ARoadGenerator::TransformToUECoordinates(FVector midPoint)
{
	for (auto& segment : segments) {

		segment->start = segment->start * 100 + Point(midPoint.X, midPoint.Y);
		segment->end = segment->end * 100 + Point(midPoint.X, midPoint.Y);
	}

	for (auto intersection : intersections) {
		intersection->position = intersection->position * 100 + Point{midPoint.X, midPoint.Y};
	}

}

/* Remove segments and intersections outside of the region */
void ARoadGenerator::RemoveOutsideOfRegion()
{
	// find the min and max points
	float minx, maxx, miny, maxy;
	if (regionStartPoint.X < regionEndPoint.X) {
		minx = regionStartPoint.X;
		maxx = regionEndPoint.X;
	}
	else {
		maxx = regionStartPoint.X;
		minx = regionEndPoint.X;
	}
	if (regionStartPoint.Y < regionEndPoint.Y) {
		miny = regionStartPoint.Y;
		maxy = regionEndPoint.Y;
	}
	else {
		maxy = regionStartPoint.Y;
		miny = regionEndPoint.Y;
	}

	auto isOutside = [minx, maxx, miny, maxy](float x, float y) {
		return (x > maxx || x < minx || y > maxy || y < miny);
	};

	// iterate over all segments, select the ones outside of the region
	// remove it from the branches vector from each of the intersections containing it
	// then remove the segment itself
	for (auto it = segments.begin(); it != segments.end();) {
		auto segment = *it;
		bool startOutside = isOutside(segment->start.x, segment->start.y);
		bool endOutside = isOutside(segment->end.x, segment->end.y);

		if (startOutside || endOutside) {
			// two lambdas to compare start intersectionID and endID to intersectionID //
			auto cmpStart = [segment](Intersection* intersection) {return segment->startIntersectionID == intersection->ID; };
			auto cmpEnd = [segment](Intersection* intersection) {return segment->endIntersectionID == intersection->ID; };

			// iterator to find intersection corresponding to segment's start and end
			std::vector<Intersection*>::iterator it2;
			
			// compare the startintersectionID to each intersectionID
			it2 = std::find_if(intersections.begin(), intersections.end(), cmpStart);

			// erase segment from the branches vector in that intersection
			if (it2 != intersections.end()) {
				auto it3 = std::find((*it2)->branches.begin(), (*it2)->branches.end(), segment);
				if (it3 != (*it2)->branches.end()) {
					(*it2)->branches.erase(it3);
				}
			}
			
			// compare the endintersectionID to each intersectionID
			it2 = std::find_if(intersections.begin(), intersections.end(), cmpEnd);

			// erase segment from the branches vector in that intersection
			if (it2 != intersections.end()) {
				auto it3 = std::find((*it2)->branches.begin(), (*it2)->branches.end(), segment);
				if (it3 != (*it2)->branches.end()) {
					(*it2)->branches.erase(it3);
				}
			}
		
			it = segments.erase(it);
		}
		else {
			++it;
		}
	}

	// iterate over all intersections and remove the ones outside of the region specified
	for (auto it = intersections.begin(); it != intersections.end();) {
		Intersection* intersection = *it;
		if (isOutside(intersection->position.x, intersection->position.y)) {
			/*for (auto branch : intersection->branches) {
				if (branch->startIntersectionID == intersection->ID) {
					UE_LOG(LogTemp, Warning, TEXT("doing somthing here to start"));
					branch->startIntersectionID = -1;
				}
				if (branch->endIntersectionID == intersection->ID) {
					UE_LOG(LogTemp, Warning, TEXT("doing somthing here to end"));
					branch->endIntersectionID = -1;
				}
			}*/

			for (auto segment : segments) {
				if (segment->startIntersectionID == intersection->ID) {
					segment->startIntersectionID = -1;
				}
				if (segment->endIntersectionID == intersection->ID) {
					segment->endIntersectionID = -1;
				}
			}
			//UE_LOG(LogTemp, Warning, TEXT("OUTSIDE OF REGION: %d"), intersection->ID);
			it = intersections.erase(it);
		}
		else {
			++it;
		}
	}

}


// you have to calls this first before CreateIntersections
void ARoadGenerator::SetIntersectionBlueprints(TSubclassOf<AActor> TwoWay, TSubclassOf<AActor> ThreeWay,
	TSubclassOf<AActor> FourWay, TSubclassOf<AActor> MoreThanFourWay, TSubclassOf<AActor> Intersection, TSubclassOf<AActor> Check)
{
	this->TwoWayBlueprint = TwoWay;
	this->ThreeWayBlueprint = ThreeWay;
	this->FourWayBlueprint = FourWay;
	this->MoreThanFourWayBlueprint = MoreThanFourWay;
	this->IntersectionBlueprint = Intersection;
	this->CheckBlueprint = Check;
}

// create intersections and spawn something on them
void ARoadGenerator::CreateIntersections(FVector midPoint)
{
	// first of all, lets create different markers for the different intersections

	for (auto intersection : intersections) {
		
		//FOR TEST
		// two way
		if (intersection->branches.size() == 2) {
			GetWorld()->SpawnActor<AActor>(this->TwoWayBlueprint, FVector{ static_cast<float>(intersection->position.x),
				static_cast<float>(intersection->position.y), 45.0f }, FRotator{}, FActorSpawnParameters{});
		}
		// three way
		else if (intersection->branches.size() == 3) {
			GetWorld()->SpawnActor<AActor>(this->ThreeWayBlueprint, FVector{ static_cast<float>(intersection->position.x),
				static_cast<float>(intersection->position.y), 45.0f }, FRotator{}, FActorSpawnParameters{});
		}
		// four way
		else if (intersection->branches.size() == 4) {
			GetWorld()->SpawnActor<AActor>(this->FourWayBlueprint, FVector{ static_cast<float>(intersection->position.x),
				static_cast<float>(intersection->position.y), 45.0f }, FRotator{}, FActorSpawnParameters{});
		}
		else if (intersection->branches.size() > 4) {
			GetWorld()->SpawnActor<AActor>(this->MoreThanFourWayBlueprint, FVector{ static_cast<float>(intersection->position.x),
				static_cast<float>(intersection->position.y), 45.0f }, FRotator{}, FActorSpawnParameters{});
		}
		

		
		// intersection blueprint
		
		if (intersection->branches.size() >= 1) {
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
			auto inter = GetWorld()->SpawnActor<AActor>(this->IntersectionBlueprint, FVector{ static_cast<float>(intersection->position.x),
				static_cast<float>(intersection->position.y), 40.0f + (maxOrder+1)* 5}, FRotator{}, FActorSpawnParameters{});
			

			auto textrender = Cast<UTextRenderComponent>(inter->GetComponentByClass(UTextRenderComponent::StaticClass()));
			
			textrender->SetText(FString::Printf(TEXT("%d"), intersection->ID));
		}
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
		startPoints.Add(*(new FVector(segment->start.x,  segment->start.y, z + segment->startOrder * 5))); // add 5cm for each order
		endPoints.Add(*(new FVector(segment->end.x, segment->end.y, z + segment->endOrder * 5))); // add 5cm for each order
		roadData.Add(FMetaRoadData{ segment->q.highway, static_cast<float>(segment->width)  * 100});

	}
}

void ARoadGenerator::GenerateMeshIntersections(AProceduralMeshMaker* ProcMeshMaker)
{
	ProcMeshMaker->GenerateMeshIntersections(intersections);
}
