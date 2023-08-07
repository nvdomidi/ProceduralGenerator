// Fill out your copyright notice in the Description page of Project Settings.


#include "ProceduralMeshMaker.h"
#include "ProcSim/Utils/ImageHandler.h"
#include "ProcSim/MapGen/Config.h"
#include <algorithm>
#include <map>


/* namespace is used for mathematical functions to calculate things for roads*/
namespace roadMath {

	// Takes start and end point of a segment and calculates the 4 corner coordinates
	TArray<FVector> FindFourCornersFromStartAndEndPoint(FVector startPoint, FVector endPoint, float width) {
		FVector2D direction{ endPoint.X - startPoint.X, endPoint.Y - startPoint.Y };
		FVector2D perpendicularDirection { -direction.Y, direction.X };
		perpendicularDirection.Normalize();
		FVector direction3D{ perpendicularDirection.X, perpendicularDirection.Y, 0.0 };

		return TArray<FVector>{ startPoint + (direction3D * width / 2), startPoint - (direction3D * width / 2),
								endPoint + (direction3D * width / 2), endPoint - (direction3D * width / 2)};		
	}

	// Takes in all roads, subdivides each road into length specified
	void SubdivideRoadsByLength(TArray<FVector>& startPoints, TArray<FVector>& endPoints,
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
				endRoadParts.Add(startPoints[i] + (j+1) * direction * length);
				roadPartsData.Add(roadData[i]);
			}
			if ( FMath::Fmod(roadLength, length) > 1e-1) {
				startRoadParts.Add(startPoints[i] + (numRoadParts) * direction * length);
				endRoadParts.Add(endPoints[i]);
				roadPartsData.Add(roadData[i]);
			}
		}

		startPoints = startRoadParts;
		endPoints = endRoadParts;
		roadData = roadPartsData;
	}
}

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

		TArray<FVector> startPoints{FVector(-400, 0, 1), FVector(400, 0, 1), FVector(0, -400, 1), FVector(0, 400, 1)};
		TArray<FVector> endPoints{FVector(-10, 0, 1), FVector(7, 0, 1), FVector(0, -6, 1), FVector(0, 9, 1)};
		TArray<FMetaRoadData> roadData{FMetaRoadData{false, static_cast<float>(Config::DEFAULT_SEGMENT_WIDTH)},
			FMetaRoadData{ false, static_cast<float>(Config::DEFAULT_SEGMENT_WIDTH) },
			FMetaRoadData{ false, static_cast<float>(Config::DEFAULT_SEGMENT_WIDTH) },
			FMetaRoadData{ false, static_cast<float>(Config::DEFAULT_SEGMENT_WIDTH) }};
		Segment* segment1 = new Segment({-400,0}, {-10,0});
		Segment* segment2 = new Segment({ 400,0 }, { 7,0 });
		Segment* segment3 = new Segment({ 0,-400 }, { 0,-6 });
		Segment* segment4 = new Segment({ 0,400 }, { 0,9 });
		std::vector<Segment*> segments = { segment1, segment2, segment3, segment4 };
		Intersection* intersection = new Intersection(segments, { 0.0, 0.0 });
		std::vector<Intersection*> intersections = { intersection };


		GenerateMesh(startPoints, endPoints, roadData);
		GenerateMeshIntersections(intersections);

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
		std::vector<std::pair<int, double>> pointsPairVector;
		pointsPairVector.reserve(intersection->branches.size());
		for (auto branch : intersection->branches) {
			// the middlepoint of the segment is transformed to coordinate system with center of all points as origin
			bool isStart = (branch->start - intersection->position).length() < (branch->end - intersection->position).length();
			Point transformedMiddlePoint = (isStart ? branch->start : branch->end) - middlePoint;
			double angle = atan2(transformedMiddlePoint.y, transformedMiddlePoint.x);
			pointsPairVector.emplace_back(std::pair<int, double>{branch->ID, angle});
		}

		// we want array of sorted segments
		auto cmp = [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
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
	roadMath::SubdivideRoadsByLength(startPoints, endPoints, roadData,
		static_cast<float>(Config::DEFAULT_ROADPART_LENGTH * 100));

	TArray<FVector> vertices{};

	for (int i = 0; i < startPoints.Num(); i++) {
		/* randomize the Z value a bit */
		//float z = FMath::RandRange(startPoints[i].Z - startPoints[i].Z / 100, startPoints[i].Z + startPoints[i].Z / 100);
		//startPoints[i].Z = startPoints[i].Z + z;
		//endPoints[i].Z = endPoints[i].Z + z;

		TArray<FVector> fourCorners = roadMath::FindFourCornersFromStartAndEndPoint(startPoints[i], endPoints[i], roadData[i].roadWidth);
		vertices.Append(fourCorners);
	}

	return vertices;
}

// iterates over vertices and adds triangles (3 ints) for each of them
// returns an array of ints that has a factor of 3 number of elements
TArray<int> AProceduralMeshMaker::CalculateTrianglesForProceduralMesh(TArray<FVector> vertices)
{
	// checks to see if number of vertices is factor of 4
	if (vertices.Num() % 4 != 0) {
		UE_LOG(LogTemp, Error, TEXT("ERROR: Number of vertices is not a factor of 4."));
		return {};
	}

	TArray<int> triangles{};

	for (int i = 0; i < vertices.Num() / 4; i++) {
		triangles.Append(TArray<int>{ i * 4 + 0, i * 4 + 2, i * 4 + 1,
			/* two triangles */  i * 4 + 2, i * 4 + 3, i * 4 + 1 });
	}

	return triangles;
}

TArray<FVector2D> AProceduralMeshMaker::CalculateUVsForProceduralMesh(TArray<FVector> vertices)
{
	TArray<FVector2D> UVs{};

	if (vertices.Num() % 4 != 0) {
		UE_LOG(LogTemp, Error, TEXT("Invalid number of vertices. Not a factor of 4!"));
		return UVs;
	}

	for (int i = 0; i < vertices.Num() / 4; i++) {
		// check the length
		float length = FVector::Dist(vertices[i * 4], vertices[i * 4 + 2]);
		float roadLength = static_cast<float>(Config::DEFAULT_ROADPART_LENGTH * 100);
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

