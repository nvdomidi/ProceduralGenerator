// Fill out your copyright notice in the Description page of Project Settings.


#include "ProceduralMeshMaker.h"

#include "ProcSim/Actors/CityBlocksMaker.h"
#include "ProcSim/Utils/ImageHandler.h"
#include "ProcSim/MapGen/Config.h"
#include <algorithm>
#include <map>


/* Constructor: creates procedrual mesh component */
AProceduralMeshMaker::AProceduralMeshMaker()
{
	ProceduralRoadMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralRoadMesh"));
	RootComponent = ProceduralRoadMesh;
}

/* some roads are specified, their mesh and  and the mesh between them is created */
void AProceduralMeshMaker::CPPConstruction()
{
	UE_LOG(LogTemp, Warning, TEXT("OnConstruction called"));
	ProceduralRoadMesh = NewObject<UProceduralMeshComponent>(this, TEXT("ProceduralRoadMesh"));
	RootComponent = ProceduralRoadMesh;


	if (!generated) {
		UE_LOG(LogTemp, Warning, TEXT("Generating!!!"));

		TArray<FVector> startPoints{
			FVector{ 0, 0, 1 },
				FVector{ 400,0,1 },
				FVector{ 0,0,1 },
				FVector{ 0,-400,1 },
				FVector{ 800,-800,1 },
				FVector{ 400,-800,1 },
				FVector{ 800,0,1 },
				FVector{ 800,-400,1 },
				FVector{ 0,-1600,1 },
				FVector{ 0,-1200,1 },
				FVector{ 800,0,1 },
				FVector{ 1200,0,1 },
				FVector{ 1000,-400,1 },
				FVector{ 1200,0,1 }
		};

		TArray<FVector> endPoints{
			FVector{ 400, 0, 1 },
				FVector{ 800,0,1 },
				FVector{ 0,-400,1 },
				FVector{ 0,-800,1 },
				FVector{ 400,-800,1 },
				FVector{ 0,-800,1 },
				FVector{ 800,-400,1 },
				FVector{ 800,-800,1 },
				FVector{ 0,-1200,1 },
				FVector{ 0,-800,1 },
				FVector{ 1200,0,1 },
				FVector{ 1000,-400,1 },
				FVector{ 800,-800,1 },
				FVector{ 1600,0,1 }
		};

		TArray<FMetaRoadData> roadData{};
		for (FVector point : startPoints) {
			roadData.Add(FMetaRoadData{ false, (Config::DEFAULT_SEGMENT_WIDTH) });
		}

		Segment* segment1 = new Segment({ 0,0 }, { 400,0 });
		Segment* segment2 = new Segment({ 400,0 }, { 800,0 });
		Segment* segment3 = new Segment({ 0,0 }, { 0,-400 });
		Segment* segment4 = new Segment({ 0,-400 }, { 0,-800 });
		Segment* segment5 = new Segment({ 800,-800 }, { 400,-800 });
		Segment* segment6 = new Segment({ 400,-800 }, { 0,-800 });
		Segment* segment7 = new Segment({ 800,0 }, { 800,-400 });
		Segment* segment8 = new Segment({ 800,-400 }, { 800,-800 });
		Segment* segment9 = new Segment({ 0,-1600 }, { 0,-1200 });
		Segment* segment10 = new Segment({ 0,-1200 }, { 0,-800 });
		Segment* segment11 = new Segment({ 800,0 }, { 1200,0 });
		Segment* segment12 = new Segment({ 1200,0 }, { 1000,-400 });
		Segment* segment13 = new Segment({ 1000,-400 }, { 800,-800 });
		Segment* segment14 = new Segment({ 1200,0 }, { 1600,0 });

		segment1->links_b = { segment3 };
		segment1->links_f = { segment2 };
		segment2->links_b = { segment1 };
		segment2->links_f = { segment11, segment7 };
		segment3->links_b = { segment1 };
		segment3->links_f = { segment4 };
		segment4->links_b = { segment3 };
		segment4->links_f = { segment6, segment10 };
		segment5->links_b = { segment8,segment13 };
		segment5->links_f = { segment6 };
		segment6->links_b = { segment5 };
		segment6->links_f = { segment4, segment10 };
		segment7->links_b = { segment2, segment11 };
		segment7->links_f = { segment8 };
		segment8->links_b = { segment7 };
		segment8->links_f = { segment13, segment5 };
		segment9->links_b = {  };
		segment9->links_f = { segment10 };
		segment10->links_b = { segment9 };
		segment10->links_f = { segment6, segment4 };
		segment11->links_b = { segment2, segment7 };
		segment11->links_f = { segment12, segment14 };
		segment12->links_b = { segment14, segment11 };
		segment12->links_f = { segment13 };
		segment13->links_b = { segment12 };
		segment13->links_f = { segment8, segment5 };
		segment14->links_b = { segment11, segment12 };
		segment14->links_f = { };

		std::vector<Segment*> int1segments = { segment1, segment3 };
		std::vector<Segment*> int2segments = { segment2, segment11, segment7 };
		std::vector<Segment*> int3segments = { segment11, segment12, segment14 };
		std::vector<Segment*> int4segments = { segment4, segment6, segment10 };
		std::vector<Segment*> int5segments = { segment5, segment8, segment13 };

		Point int1pos = {0,0};
		Point int2pos = { 800,0 };
		Point int3pos = { 1200,0 };
		Point int4pos = { 0,-800 };
		Point int5pos = { 800,-800 };

		Intersection* inter1 = new Intersection(int1segments, int1pos);
		Intersection* inter2 = new Intersection(int2segments, int2pos);
		Intersection* inter3 = new Intersection(int3segments, int3pos);
		Intersection* inter4 = new Intersection(int4segments, int4pos);
		Intersection* inter5 = new Intersection(int5segments, int5pos);

		segment1->startIntersectionID = inter1->ID;
		segment2->endIntersectionID = inter2->ID;
		segment3->startIntersectionID = inter1->ID;
		segment4->endIntersectionID = inter4->ID;
		segment5->startIntersectionID = inter5->ID;
		segment6->endIntersectionID = inter4->ID;
		segment7->startIntersectionID = inter2->ID;
		segment8->endIntersectionID = inter5->ID;
		segment10->endIntersectionID = inter4->ID;
		segment11->startIntersectionID = inter2->ID;
		segment11->endIntersectionID = inter3->ID;
		segment12->startIntersectionID = inter3->ID;
		segment13->endIntersectionID = inter5->ID;
		segment14->startIntersectionID = inter3->ID;

		std::vector<Intersection*> intersections = { inter1, inter2, inter3, inter4, inter5 };
		std::vector<Segment*> segments = { segment1, segment2, segment3, segment4, segment5, segment6,
		segment7, segment8, segment9, segment10, segment11, segment12, segment13, segment14 };

		//TArray<FVector> startPoints{FVector(-400, 0, 1), FVector(400, 0, 1), FVector(0, -400, 1), FVector(0, 400, 1)};
		//TArray<FVector> endPoints{FVector(-10, 0, 1), FVector(7, 0, 1), FVector(0, -6, 1), FVector(0, 9, 1)};
		/*TArray<FMetaRoadData> roadData{FMetaRoadData{false, (Config::DEFAULT_SEGMENT_WIDTH)},
			FMetaRoadData{ false, (Config::DEFAULT_SEGMENT_WIDTH) },
			FMetaRoadData{ false, (Config::DEFAULT_SEGMENT_WIDTH) },
			FMetaRoadData{ false, (Config::DEFAULT_SEGMENT_WIDTH) }};*/
		/*Segment* segment1 = new Segment({-400,0}, {-10,0});
		Segment* segment2 = new Segment({ 400,0 }, { 7,0 });
		Segment* segment3 = new Segment({ 0,-400 }, { 0,-6 });
		Segment* segment4 = new Segment({ 0,400 }, { 0,9 });
		std::vector<Segment*> segments = { segment1, segment2, segment3, segment4 };
		Intersection* intersection = new Intersection(segments, { 0.0, 0.0 });
		std::vector<Intersection*> intersections = { intersection };*/


		GenerateMesh(startPoints, endPoints, roadData);
		auto blocksmaker = Cast<ACityBlocksMaker>(GetWorld()->SpawnActor<ACityBlocksMaker>(ACityBlocksMaker::StaticClass(), FActorSpawnParameters{}));
		blocksmaker->MakeGraph(intersections, segments);
		//GenerateMeshIntersections(intersections);

		generated = true;
	}
}

/* created the procedural mesh based on the vertices, triangles and UVs it calculates */
void AProceduralMeshMaker::GenerateMesh(TArray<FVector> startPoints, TArray<FVector> endPoints, TArray<FMetaRoadData> roadData)
{
	//Everything is in one mesh
	//calculate the vertices
	TArray<FVector> vertices = CalculateVerticesForProceduralMesh(startPoints, endPoints, roadData);
	//calculate the triangles
	TArray<int> triangles = CalculateTrianglesForProceduralMesh(vertices);
	//calculate the UVs
	TArray<FVector2D> uvs = CalculateUVsForProceduralMesh(vertices);
	//add mesh section to ProceduralMeshComponent
	if (ProceduralRoadMesh == nullptr) {
		UE_LOG(LogTemp, Error, TEXT("ProceduralRoadMesh is null!!"));
		return;
	}
	ProceduralRoadMesh->CreateMeshSection(0, vertices, triangles, TArray<FVector>(),
		uvs, TArray<FColor>(), TArray<FProcMeshTangent>(), true);

	// load texture from path and apply it to the proceduralmeshcomponent
	UTexture2D* RoadTexture = Cast<UTexture2D>(StaticLoadObject(UTexture2D::StaticClass(), NULL, *FString("/Game/Textures/road1")));
	if (RoadTexture == nullptr) {
		UE_LOG(LogTemp, Error, TEXT("Roadtexture is null!"));
		return;
	}
	ImageHandler::ApplyTextureToProceduralMeshComponent(ProceduralRoadMesh, RoadTexture, FString("/Game/Materials/RoadMaterial"));
}

// this function is used to create procedural mesh for all the intersections
void AProceduralMeshMaker::GenerateMeshIntersections(std::vector<Intersection*> intersections, float height)
{
	UE_LOG(LogTemp, Warning, TEXT("Generating mesh intersections"));
	TArray<FVector> vertices{};
	TArray<int> triangles{};

	for (auto intersection : intersections) {
		// Sort the middlepoints clockwise
		// first we find the middlePoint
		Point middlePoint{};
		for (auto branch : intersection->branches) {
			bool isStart = (branch->start - intersection->position).length() < (branch->end - intersection->position).length();
			middlePoint = middlePoint + (isStart ? branch->start : branch->end);
		}
		middlePoint = middlePoint / intersection->branches.size();

		// we pair segment ID to angle of the middlepoints of segment end close to intersection, to X-axis
		std::vector<std::pair<int, float>> pointsPairVector;
		pointsPairVector.reserve(intersection->branches.size());
		for (auto branch : intersection->branches) {
			// the middlepoint of the segment is transformed to coordinate system with center of all points as origin
			bool isStart = (branch->start - intersection->position).length() < (branch->end - intersection->position).length();
			Point transformedMiddlePoint = (isStart ? branch->start : branch->end) - middlePoint;
			float angle = atan2(transformedMiddlePoint.y, transformedMiddlePoint.x);
			pointsPairVector.emplace_back(std::pair<int, float>{branch->ID, angle});
		}

		// we want array of sorted segments
		auto cmp = [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
			return (a.second < b.second);
		};

		std::sort(pointsPairVector.begin(), pointsPairVector.end(), cmp);


		// now we can get the sorted segments
		std::vector<Segment*> sortedSegments{};
		sortedSegments.reserve(intersection->branches.size());

		for (size_t i = 0; i < pointsPairVector.size(); i++) {
			auto it = std::find_if(intersection->branches.begin(), intersection->branches.end(),
				[pointsPairVector,i](const Segment* segment) {return segment->ID == pointsPairVector[i].first; });
			if (it != intersection->branches.end()) {
				sortedSegments.emplace_back(*it);
			}
		}



		/* iterate over clockwise sorted segments */
		TArray<FVector> corners{};
		for (auto branch : sortedSegments) {
			
			FVector startVector = FVector{ static_cast<float>(branch->start.x), static_cast<float>(branch->start.y), height };
			FVector endVector = FVector{ static_cast<float>(branch->end.x), static_cast<float>(branch->end.y), height };

			// Find four edges from start and end point of that branch
			TArray<FVector> allEnds = roadMath::FindFourCornersFromStartAndEndPoint(
				startVector,
				endVector,
				branch->q.highway ? Config::HIGHWAY_SEGMENT_WIDTH*100 : Config::DEFAULT_SEGMENT_WIDTH*100);


			// find the end that is at the intersection
			bool isStart = (branch->start - intersection->position).length() < (branch->end - intersection->position).length();
			

			if (isStart) {
				allEnds.RemoveAt(3);
				allEnds.RemoveAt(2);
			}
			else {
				allEnds.RemoveAt(1);
				allEnds.RemoveAt(0);
			}

			corners.Append(allEnds);
		}

		// we now have the corners sorted clockwise
		
		for (int i = 0; i < corners.Num() - 2; i++) {
			triangles.Append({ vertices.Num() , vertices.Num() + i + 2, vertices.Num() + i + 1});
		}
		vertices.Append(corners);
		/*
		if (intersection->branches.size() == 2) {
			int len = vertices.Num();
			TArray<int> triangle1 = {0+len,2+len,1+len};
			TArray<int> triangle2 = {0+len,3+len,2+len};
			TArray<int> triangle3 = { 0 + len,1 + len, 2 + len };
			TArray<int> triangle4 = { 0 + len,2 + len,3 + len };
			triangles.Append(triangle1);
			triangles.Append(triangle2);
			triangles.Append(triangle3);
			triangles.Append(triangle4);
			vertices.Append(corners);
		}
		*/
		
	}

	if (ProceduralRoadMesh == nullptr) {
		UE_LOG(LogTemp, Error, TEXT("ProceduralRoadMesh is null!!"));
		return;
	}
	ProceduralRoadMesh->CreateMeshSection(1, vertices, triangles, TArray<FVector>(),
		TArray<FVector2D>(), TArray<FColor>(), TArray<FProcMeshTangent>(), true);

}

// Iterates over all road segments, divides them into a certain length and calculates their vertices.
TArray<FVector> AProceduralMeshMaker::CalculateVerticesForProceduralMesh(TArray<FVector> startPoints, TArray<FVector> endPoints, TArray<FMetaRoadData> roadData)
{
	// check if they are all the same size
	if (startPoints.Num() != roadData.Num() || startPoints.Num() != endPoints.Num()) {
		UE_LOG(LogTemp, Error, TEXT("ERROR: The startPoints, endPoints and roadData arrays are not the same size"));
		UE_LOG(LogTemp, Error, TEXT("startPoints: %d, endPoints: %d, roadData: %d"), startPoints.Num(),
			endPoints.Num(), roadData.Num());

		return {};
	}

	// first we must subdivide the segments into road parts, each road part gets the whole texture assigned to it
	//roadMath::SubdivideRoadsByLength(startPoints, endPoints, roadData,
	//	(Config::DEFAULT_ROADPART_LENGTH * 100));

	TArray<FVector> vertices{};
	TArray<FVector> arrowVertices{};


	for (int i = 0; i < startPoints.Num(); i++) {
		/* randomize the Z value a bit */
		//float z = FMath::RandRange(startPoints[i].Z - startPoints[i].Z / 100, startPoints[i].Z + startPoints[i].Z / 100);
		//startPoints[i].Z = startPoints[i].Z + z;
		//endPoints[i].Z = endPoints[i].Z + z;

		TArray<FVector> fourCorners = roadMath::FindFourCornersFromStartAndEndPoint(startPoints[i], endPoints[i], roadData[i].roadWidth);
		vertices.Append(fourCorners);


		/* TODO: remove this */
		
		FVector dir = endPoints[i] - startPoints[i];
		FVector p{ -dir.Y, dir.X, 0.0 };
		p.Normalize();
		FVector p1 = endPoints[i] - (roadData[i].roadWidth) * p;
		FVector p2 = endPoints[i] + (roadData[i].roadWidth) * p;
		dir.Normalize();
		FVector p3 = endPoints[i] + (roadData[i].roadWidth * dir);
		arrowVertices.Append(TArray<FVector>{p1, p2, p3});
		

	}

	vertices.Append(arrowVertices);

	return vertices;
}

// iterates over vertices and adds triangles (3 ints) for each of them
// returns an array of ints that has a factor of 3 number of elements
TArray<int> AProceduralMeshMaker::CalculateTrianglesForProceduralMesh(TArray<FVector> vertices)
{
	// checks to see if number of vertices is factor of 4
	if (vertices.Num() % 4 != 0) {
		//UE_LOG(LogTemp, Error, TEXT("ERROR: Number of vertices is not a factor of 4."));
		//return {};
	}

	TArray<int> triangles{};

	//for (int i = 0; i < vertices.Num() / 4; i++) {
	//	triangles.Append(TArray<int>{ i * 4 + 0, i * 4 + 2, i * 4 + 1,
			/* two triangles */  //i * 4 + 2, i * 4 + 3, i * 4 + 1 });
	//}

	for (int i = 0; i < vertices.Num() * 4 / 7; i = i+4) {
		triangles.Append(TArray<int>{ i, i + 2, i + 1,
			/* two triangles */  i + 2, i + 3, i + 1 });
	}

	for (int i = vertices.Num() * 4 / 7; i < vertices.Num(); i=i+3) {
		triangles.Append(TArray<int>{i, i + 1, i + 2});
	}


	return triangles;
}

TArray<FVector2D> AProceduralMeshMaker::CalculateUVsForProceduralMesh(TArray<FVector> vertices)
{
	TArray<FVector2D> UVs{};

	if (vertices.Num() % 4 != 0) {
		//UE_LOG(LogTemp, Error, TEXT("Invalid number of vertices. Not a factor of 4!"));
		//return UVs;
	}

	for (int i = 0; i < vertices.Num() / 4; i++) {
		// check the length
		float length = FVector::Dist(vertices[i * 4], vertices[i * 4 + 2]);
		float roadLength = (Config::DEFAULT_ROADPART_LENGTH * 100);
		// if more than one percent difference
		if (FMath::Abs(roadLength - length) > roadLength/100) {
			UVs.Append(TArray<FVector2D> {FVector2D{0.0f, 0.0f}, FVector2D{ 1.0f, 0.0f },
				FVector2D{ 0.0f, length/roadLength},
				FVector2D{ 1.0f, length/roadLength}});
		}
		else {
			UVs.Append(TArray<FVector2D> {FVector2D{0.0f, 0.0f}, FVector2D{ 1.0f, 0.0f },
				FVector2D{ 0.0f, 1.0f }, FVector2D{ 1.0f, 1.0f }});
		}
	}
	
	return UVs;
}

