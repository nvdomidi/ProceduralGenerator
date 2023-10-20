// Fill out your copyright notice in the Description page of Project Settings.


#include "ProcSim/Actors/CityBlocksMaker.h"
#include "ProcSim/MapGen/Config.h"
#include "ProceduralMeshComponent.h"

#include <algorithm>
#include <array>
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

	UE_LOG(LogTemp, Warning, TEXT("whats giong on segment size: %d"), segments.size());

	for (auto segment : segments) {
		UE_LOG(LogTemp, Warning, TEXT("%d to %d"), segment->startIntersectionID, segment->endIntersectionID);
		graph.AddEdge(segment->startIntersectionID, segment->endIntersectionID);
	}

	for (auto s : graph.vertices) {
		
		for (auto a : s.second->adj) {
			UE_LOG(LogTemp, Warning, TEXT("HAS ADJ: %d"), a);
		}
	}

	
	/*int e = 0;
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
	}*/

	//// block of code that shows the intersections
	/*TArray<FVector> vertices{};
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
	}*/
	//ProceduralMesh->CreateMeshSection(0, vertices, triangles, TArray<FVector>(),
	//	TArray<FVector2D>(), TArray<FColor>(), TArray<FProcMeshTangent>(), true);




	/*UE_LOG(LogTemp, Warning, TEXT("vertices: %d"), graph.vertices.size());
	UE_LOG(LogTemp, Warning, TEXT("edges: %d"), e);
	UE_LOG(LogTemp, Warning, TEXT("no duplicate edges: %d"), edges.size());*/


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

namespace blocksMath {
	// Bounds that goes from lowest x,y to highest x,y
	Bounds CycleToBounds(TArray<int> cycle, Graph<Intersection*> graph) {
		
		float minx = FLT_MAX;
		float miny = FLT_MAX;
		float maxx = FLT_MIN;
		float maxy = FLT_MIN;

		for (int i : cycle) {
			Point p = graph.vertices[i]->data->position;
			if (p.x < minx)
				minx = p.x;
			if (p.y < miny)
				miny = p.y;
			if (p.x > maxx)
				maxx = p.x;
			if (p.y > maxy)
				maxy = p.y;
		}

		return Bounds { minx, miny, maxx - minx, maxy - miny };
	}

	// check if bounds collide
	bool DoBoundingBoxesCollide(Bounds b1, Bounds b2) {
		bool xApart = (b1.x > b2.x + b2.width) || (b2.x > b1.x + b1.width);
		bool yApart = (b1.y > b2.y + b2.height) || (b2.y > b1.y + b1.height);
		bool apart = xApart || yApart;
		return !apart;
	}

	// check if point is in cycle polygon
	bool PointInPolygon(Point p, TArray<int> cycle, Graph<Intersection*> graph) {
		bool inside = false;
		// iterate over edges
		for (size_t i = 0; i < cycle.Num(); i++) {
			int idx1 = cycle[i % cycle.Num()];
			int idx2 = cycle[(i + 1) % cycle.Num()];
			// start and end point of edge
			Point a = graph.vertices[idx1]->data->position;
			Point b = graph.vertices[idx2]->data->position;
			if ((a.y > p.y) != (b.y > p.y) && p.x < (p.y - a.y) * (b.x - a.x) / (b.y - a.y) + a.x)
				inside = !inside;
		}

		return inside;
	}	
}

TArray<TArray<int>> ACityBlocksMaker::RemoveOuterCycles(TArray<TArray<int>> cycles, Graph<Intersection*> graph)
{
	Quadtree<TArray<int>> cyclesQtree{Bounds{Config::minx, Config::maxx, Config::maxx - Config::minx, Config::maxy - Config::miny},
		Config::QUADTREE_MAX_OBJECTS, Config::QUADTREE_MAX_LEVELS};

	for (auto cycle : cycles) {
		Bounds b = blocksMath::CycleToBounds(cycle, graph);
		UE_LOG(LogTemp, Warning, TEXT("inserting cycle into graph"));
		cyclesQtree.insert(b, cycle);
	}

	for (int i = 0; i < cycles.Num(); i++) {
		TArray<int> cycle1 = cycles[i];
		UE_LOG(LogTemp, Warning, TEXT("number of colliding cycles found for c1: %d"), cyclesQtree.retrieve(blocksMath::CycleToBounds(cycle1, graph)).size());
		for (auto cycle2 : cyclesQtree.retrieve(blocksMath::CycleToBounds(cycle1, graph))) {
			//UE_LOG(LogTemp, Warning, TEXT(""));
		}
	}

	return {};


	//TSet<int> cyclesToRemove {};

	//for (int i = 0; i < cycles.Num(); i++) {
	//	for (int j = 0; j < cycles.Num(); j++) {
	//		UE_LOG(LogTemp, Warning, TEXT("checking cycle#%d against %d out of: %d"), j, i, cycles.Num());
	//		if (i!=j) {
	//			Bounds b1 = blocksMath::CycleToBounds(cycles[i], graph);
	//			Bounds b2 = blocksMath::CycleToBounds(cycles[j], graph);
	//			// If their bounding box collision does not collide, don't even continue
	//			if (blocksMath::DoBoundingBoxesCollide(b1, b2)) {
	//				TArray<int> cycle1 = cycles[i];
	//				TArray<int> cycle2 = cycles[j];
	//				// check all points of cycle1 inside cycle2
	//				for (int k : cycle1) {
	//					Point p = graph.vertices[k]->data->position;
	//					if (blocksMath::PointInPolygon(p, cycle2, graph)) {
	//						// point is inside
	//						cyclesToRemove.Add(j);
	//					}
	//				}
	//				for (int k : cycle2) {
	//					Point p = graph.vertices[k]->data->position;
	//					if (blocksMath::PointInPolygon(p, cycle1, graph)) {
	//						// point is inside
	//						cyclesToRemove.Add(i);
	//					}
	//				}


	//			}
	//		}
	//	}
	//}

	//TArray<TArray<int>> cyclesToKeep {};
	//for (int i = 0; i < cycles.Num(); i++) {
	//	if (!cyclesToRemove.Contains(i)) {
	//		cyclesToKeep.Add(cycles[i]);
	//	}
	//}

	//return cyclesToKeep;
}

void ACityBlocksMaker::DFSCycles(std::shared_ptr<gte::MinimalCycleBasis<double>::Tree> tree, int& num) {
	if (tree->cycle.size() != 0) {
		num += 1;

		Point pos{};
		for (int v = 0; v < tree->cycle.size() - 1; v++) {
			int id = tree->cycle[v];
			pos = pos+ in11[id]->position;
		}
		pos = pos / (tree->cycle.size() - 1);

		cyclepositions.Add(FVector{ static_cast<float>(pos.x), static_cast<float>(pos.y), 70.0f });

	}
	for (auto child : tree->children) {
		DFSCycles(child, num);
	}
}

TArray<TArray<int>> ACityBlocksMaker::MinimumCycleBasis(std::vector<Intersection*> intersections, std::vector<Segment*> segments)
{
	in11 = intersections;
	UE_LOG(LogTemp, Warning, TEXT("Number of intersectoins: %d"), in11.size());
	std::vector<std::array<double, 2>> positions{};
	for (auto intersection : intersections) {
		Point p = intersection->position;
		positions.push_back({ p.x, p.y });
	}

	std::vector<std::array<int32_t, 2>> edges{};
	for (auto segment : segments) {
		edges.push_back({ segment->startIntersectionID, segment->endIntersectionID });
	}


	std::vector<std::shared_ptr<gte::MinimalCycleBasis<double>::Tree>> forest;
	gte::MinimalCycleBasis<double> mcb(positions, edges, forest);

	int numCycles = 0;

	DFSCycles(forest[0], numCycles);
	UE_LOG(LogTemp, Warning, TEXT("numcycles: %d"), numCycles);


	for (const auto& tree : forest) {
		std::cout << "Main Cycle: ";
		for (int32_t v : tree->cycle) {
			std::cout << v << " ";
		}
		std::cout << std::endl;

		for (const auto& child : tree->children) {
			std::cout << "  Sub-Cycle or Filament: ";
			for (int32_t v : child->cycle) {
				std::cout << v << " ";
			}
			std::cout << std::endl;
		}
	}

	return TArray<TArray<int>>();
}

