// Fill out your copyright notice in the Description page of Project Settings.


#include "ProcSim/Actors/CityBlocksMaker.h"
#include "ProcSim/MapGen/Config.h"
#include "ProcSim/MapGen/Math.h"

#include "ProcSim/BlocksGen/Parcel.h"

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
Graph<Intersection> ACityBlocksMaker::MakeGraph(std::vector<Intersection*> intersections, std::vector<Segment*> segments)
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

	Graph<Intersection> graph;
	
	// Node ID: intersection ID, Node MetaData: Intersection pointer
	for (auto intersection : intersections) {
		
		graph.AddNode(intersection->ID, *intersection);
	}

	for (auto segment : segments) {
		graph.AddEdge(segment->startIntersectionID, segment->endIntersectionID);
	}
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

TArray<TArray<int>> ACityBlocksMaker::FindFaces(Graph<Intersection> graph)
{
	TArray<TArray<int>> faces{};
	graph.FindFaces();
	std::vector<std::vector<int>> fs = graph.faces;

	for (auto f : fs) {
		TArray<int> face{};
		for (auto node : f) {
			face.Add(node);
		}
		faces.Add(face);
	}

	return faces;
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
	UE_LOG(LogTemp, Warning, TEXT("Number of intersections: %d"), in11.size());
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

	UE_LOG(LogTemp, Warning, TEXT("MinimalCycleBasis was performed !"));
	/*
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
	*/
	return TArray<TArray<int>>();
}

void ACityBlocksMaker::ParcelBlocks(Graph<Intersection> graph, FVector midpoint)
{
	// we must change the type to GraphVertex instead of Intersection
	Graph<GraphVertex> cityGraph{};
	
	// adding the nodes
	for (auto v : graph.vertices) {
		GraphVertex vert(v.second->data.position);
		vert.ID = v.second->data.ID;
		
		cityGraph.AddNode(vert.ID, vert);
	}

	// adding the edges
	for (auto v : graph.vertices) {
		for (int neighbor : v.second->adj) {
			cityGraph.AddEdge(v.second->id, neighbor);
		}
	}

	// finding the faces and printing to confirm
	cityGraph.FindFaces();

	int numFaces = 0;

	for (auto face : cityGraph.faces) {

		numFaces++;

		FString toPrint = FString::Printf(TEXT("Face_%d: "), numFaces);
		for (int v : face) {
			toPrint += (FString::FromInt(v) + ",");
		}

		UE_LOG(LogTemp, Warning, TEXT("%s"), *toPrint);


	}
	
	UE_LOG(LogTemp, Warning, TEXT("Number of cityGraph faces: %d"), cityGraph.faces.size());



	for (int i = 0; i < cityGraph.faces.size(); i++) {
		std::vector<int> blockFace = cityGraph.faces[i];

		std::vector<GraphVertex> blockNodes{};

		for (int node : blockFace) {
			blockNodes.push_back(cityGraph.vertices[node]->data);
		}

		Parcel blockParcel(blockNodes);

		FString parcel_string_before = "";
		FString parcel_nodes_string_before = "";

		for (GraphVertex node : blockParcel.face) {
			parcel_string_before += (FString::FromInt(node.ID) + ",");

			parcel_nodes_string_before += ("(" + FString::SanitizeFloat(float(node.position.x)) + "," +
				FString::SanitizeFloat(float(node.position.y)) + ")");

			GetWorld()->SpawnActor<AActor>(this->befBP, FVector{ static_cast<float>(midpoint.X + 100*node.position.x),
				static_cast<float>(midpoint.Y + 100 * node.position.y), 80.0f }, FRotator{}, FActorSpawnParameters{});
		}

		
		//blockParcel.squareAcuteAngles(40, 0.1);
		//blockParcel.insetParcelUniformXY(0.08);

		
		Block* lotBlock = new Block();
		lotBlock->parcels = std::vector<Parcel>{ blockParcel };

		lotBlock->subdivideParcels();




		FString parcel_string_after = "";
		FString parcel_nodes_string_after = "";

		for (GraphVertex node : blockParcel.face) {
			parcel_string_after += (FString::FromInt(node.ID) + ",");

			parcel_nodes_string_after += ("(" + FString::SanitizeFloat(float(node.position.x)) + "," +
				FString::SanitizeFloat(float(node.position.y)) + ")");

			GetWorld()->SpawnActor<AActor>(this->aftBP, FVector{ static_cast<float>(midpoint.X + 100 * node.position.x),
				static_cast<float>(midpoint.Y + 100 * node.position.y), 80.0f }, FRotator{}, FActorSpawnParameters{});
		}

		/*
		if (parcel_string_before != parcel_string_after || true) {
			UE_LOG(LogTemp, Warning, TEXT("Parcel Before: %s"), *parcel_string_before);
			UE_LOG(LogTemp, Warning, TEXT("Parcel Before: %s"), *parcel_nodes_string_before);
			UE_LOG(LogTemp, Warning, TEXT("Parcel After: %s"), *parcel_string_after);
			UE_LOG(LogTemp, Warning, TEXT("Parcel After: %s"), *parcel_nodes_string_after);
		}
		*/

		UE_LOG(LogTemp, Warning, TEXT("Parcels before: %s"), *parcel_string_before);

		FString lotBlockString = "";

		for (Parcel lotBlockParcel : lotBlock->parcels) {
			lotBlockString += FString::Printf(TEXT("Block#%d Parcel:"), i);
			
			for (GraphVertex node : lotBlockParcel.face) {
				lotBlockString += ("(" + FString::SanitizeFloat(float(node.position.x)) + "," +
					FString::SanitizeFloat(float(node.position.y)) + ")");
			}
		}

		UE_LOG(LogTemp, Warning, TEXT("Block after: %s"), *lotBlockString);

	}

	

	
	

}

void ACityBlocksMaker::SetBlueprints(TSubclassOf<AActor> beforeBP, TSubclassOf<AActor> afterBP)
{
	this->befBP = beforeBP;
	this->aftBP = afterBP;
}

