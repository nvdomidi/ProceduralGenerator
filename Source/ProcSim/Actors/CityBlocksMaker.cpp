// Fill out your copyright notice in the Description page of Project Settings.


#include "ProcSim/Actors/CityBlocksMaker.h"
#include "ProceduralMeshComponent.h"

#include <algorithm>
#include <unordered_map>
#include <set>


// Sets default values
ACityBlocksMaker::ACityBlocksMaker()
{
	//ProceduralMesh = CreateDefaultSubobject<UProceduralMeshComponent>(FName("Proceduralmesh"));
	//RootComponent = ProceduralMesh;
}


/* This function turns segments and intersections into a graph */
Graph<Intersection*> ACityBlocksMaker::MakeGraph(std::vector<Intersection*> intersections, std::vector<Segment*> segments)
{
	std::vector<int> intersectionIDs{};

	for (auto intersection : intersections) {
		if (std::find(intersectionIDs.begin(), intersectionIDs.end(), intersection->ID) == intersectionIDs.end()) {
			intersectionIDs.push_back(intersection->ID);
		}
		else {
			UE_LOG(LogTemp, Warning, TEXT("DUPLICATE ID: %d"), intersection->ID);
		}
	}

	for (auto segment : segments) {
		if (std::find(intersectionIDs.begin(), intersectionIDs.end(), segment->startIntersectionID) == intersectionIDs.end() && segment->startIntersectionID != -1) {
			UE_LOG(LogTemp, Warning, TEXT("startIntersectionID NOT FOUND: %d"), segment->startIntersectionID);
		}
		if (std::find(intersectionIDs.begin(), intersectionIDs.end(), segment->endIntersectionID) == intersectionIDs.end() && segment->endIntersectionID != -1) {
			UE_LOG(LogTemp, Warning, TEXT("endIntersectionID NOT FOUND: %d"), segment->endIntersectionID);
		}
	}



	Graph<Intersection*> graph;
	

	
	// Node ID: intersection ID, Node MetaData: Intersection pointer
	for (auto intersection : intersections) {
		UE_LOG(LogTemp, Warning, TEXT("Adding Node#%d : (%f,%f)"), intersection->ID, intersection->position.x, intersection->position.y);
		graph.AddNode(intersection->ID, intersection);
	}

	/*UE_LOG(LogTemp, Warning, TEXT("whats giong on segment size: %d"), segments.size());

	for (auto segment : segments) {
		UE_LOG(LogTemp, Warning, TEXT("%d to %d"), segment->startIntersectionID, segment->endIntersectionID);
		graph.AddEdge(segment->startIntersectionID, segment->endIntersectionID);
	}

	for (auto s : graph.vertices) {
		
		for (auto a : s.second->adj) {
			UE_LOG(LogTemp, Warning, TEXT("HAS ADJ: %d"), a);
		}
	}*/

	
	int e = 0;
	std::set<std::pair<int, int>> edges;

	for (auto intersection : intersections) {
		for (auto segment : intersection->branches) {

			Point currPos = intersection->position;
			Segment* currSegment = segment;
			int nextID = -1;
			int numSegments = 0;
			while (true) {
				numSegments++;
				bool isStart = (currSegment->start - currPos).length() < (currSegment->end - currPos).length();
				std::vector<Segment*> relevantLinks = isStart ? currSegment->links_f : currSegment->links_b;
				int relevantIntersectionID = isStart ? currSegment->endIntersectionID : currSegment->startIntersectionID;
				Point relevantEndPoint = isStart ? currSegment->end : currSegment->start;

				if (relevantIntersectionID != -1) {
					UE_LOG(LogTemp, Warning, TEXT("found edge"));
					e++;
					nextID = relevantIntersectionID;
					break;
				}
				else if (relevantLinks.empty()) {
					UE_LOG(LogTemp, Warning, TEXT("found dead end"));
					nextID = -1;
					break;
				}
				else if (relevantLinks.size() == 1) {
					UE_LOG(LogTemp, Warning, TEXT("traversing"));
					currPos = relevantEndPoint;
					currSegment = relevantLinks[0];
				}
				else {
					UE_LOG(LogTemp, Warning, TEXT("smth weird happening"));
					if (numSegments > 10) {
						break;
					}
				}

			}

			if (nextID != -1)
			{
				UE_LOG(LogTemp, Warning, TEXT("Adding edge: %d :: %d"), intersection->ID, nextID);
				if (graph.AddEdge(intersection->ID, nextID)) {
					int first = (intersection->ID < nextID) ? intersection->ID : nextID;
					int second = (intersection->ID < nextID) ? nextID : intersection->ID;
					edges.insert(std::pair<int, int>{first, second});
				}
			}
		}
	}

	//// block of code that shows the intersections
	TArray<FVector> vertices{};
	TArray<int> triangles{};

	for (auto edge : edges) {
		Point pos1 = graph.vertices[edge.first]->data->position;
		Point pos2 = graph.vertices[edge.second]->data->position;
		UE_LOG(LogTemp, Warning, TEXT("#%d pos1: (%f,%f)"), edge.first, pos1.x, pos1.y);
		UE_LOG(LogTemp, Warning, TEXT("#%d pos2: (%f,%f)"), edge.second, pos2.x, pos2.y);

		Point dir = pos2 - pos1;
		Point perdir{ dir.y, -dir.x };
		perdir = perdir / perdir.length();

		float width = 50;

		Point botleft = pos1 + perdir * width;
		Point botright = pos1 - perdir * width;
		Point topleft = pos2 + perdir * width;
		Point topright = pos2 - perdir * width;

		FVector bl{ static_cast<float>(botleft.x), static_cast<float>(botleft.y), 60 };
		FVector br{ static_cast<float>(botright.x), static_cast<float>(botright.y), 60 };
		FVector tl{ static_cast<float>(topleft.x), static_cast<float>(topleft.y), 60 };
		FVector tr{ static_cast<float>(topright.x), static_cast<float>(topright.y), 60 };

		vertices.Add(bl);
		vertices.Add(br);
		vertices.Add(tl);
		vertices.Add(tr);
	}

	UE_LOG(LogTemp, Warning, TEXT("num vertices: %d"), vertices.Num());

	
	for (int i = 0; i < vertices.Num() / 4 ; i++) {
		TArray<int> triangle = { i * 4 + 0, i * 4 + 1, i * 4 + 2,
							     i * 4 + 1, i * 4 + 3, i * 4 + 2 };
		triangles.Append(triangle);
	}

	ProceduralMesh = NewObject<UProceduralMeshComponent>(this);
	ProceduralMesh->SetupAttachment(this->RootComponent);
	ProceduralMesh->RegisterComponent();
	

	if (ProceduralMesh == nullptr) {
		UE_LOG(LogTemp, Warning, TEXT("NULLPTR"));
		return graph;
	}
	//ProceduralMesh->CreateMeshSection(0, vertices, triangles, TArray<FVector>(),
	//	TArray<FVector2D>(), TArray<FColor>(), TArray<FProcMeshTangent>(), true);




	UE_LOG(LogTemp, Warning, TEXT("vertices: %d"), graph.vertices.size());
	UE_LOG(LogTemp, Warning, TEXT("edges: %d"), e);
	UE_LOG(LogTemp, Warning, TEXT("no duplicate edges: %d"), edges.size());


	return graph;
}

/* Uses the graph.h header to find all the cycles in the graph */
/* then converts it into TArray of TArrays of integers         */
TArray<TArray<int>> ACityBlocksMaker::MakeCycles(Graph<Intersection*> graph)
{
	TArray<TArray<int>> Cycles{};
	
	std::list<std::list<int>> C = ChordlessCycles(graph);
	
	for (auto it = C.begin(); it != C.end(); ++it) {
		TArray<int> cycle{};
		for (auto it2 = (*it).begin(); it2 != (*it).end(); ++it2) {
			cycle.Add(*it2);
		}
		Cycles.Add(cycle);
	}

	return Cycles;
}

