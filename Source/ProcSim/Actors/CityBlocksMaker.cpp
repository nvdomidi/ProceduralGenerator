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

	Graph<Intersection*> graph;
	
	// Node ID: intersection ID, Node MetaData: Intersection pointer
	for (auto intersection : intersections) {
		graph.AddNode(intersection->ID, intersection);
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

FString convertFaceToString(TArray<int> face) {
	FString res = "";
	int minId = std::numeric_limits<int>::max();
	int minIdInd = 0;

	//start traversing the face at the minimum id, since same faces may have different starting indices
	for (int i = 0; i < face.Num(); i++) {
		if (face[i] < minId) {
			minId = face[i];
			minIdInd = i;
		}
	}

	for (int i = minIdInd; i < minIdInd + face.Num(); i++) {
		int toAdd = face[i % face.Num()];
		res += (FString::FromInt(toAdd) + ",");
	}

	return res;
}

// calculates angles and finds the most counter clockwise edge to prevEdge
int ACityBlocksMaker::getMostCCW(int v, TArray<int> candidates, Point prevEdge, Graph<Intersection*> graph) {
	Point v_p = graph.vertices[v]->data->position;
	int mostCC = -1;
	float minA = std::numeric_limits<float>::max();

	int colinear = -1;

	int leastClockwise = -1;
	float maxA = -std::numeric_limits<float>::max();

	for (int i = 0; i < candidates.Num(); i++) {
		Point v_cand = graph.vertices[candidates[i]]->data->position;
		Point nextEdge = v_cand - v_p;
		int orient = FMath::Sign(Math::crossProduct(prevEdge, nextEdge));
		prevEdge = prevEdge / prevEdge.length();
		nextEdge = nextEdge / nextEdge.length();

		//angle represents angle we have to rotate prevedge to get to nextedge, 
		//represents clockwise or ccw respective to orientation
		//so if orientation is -1, want the most counter clockwise rotated
		float a = Math::angleBetween(prevEdge, nextEdge);

		if (orient == -1 && maxA < a) {
			mostCC = candidates[i];
			maxA = a;
		}
		else if (orient == 0) {
			colinear = candidates[i];
		}
		else if (orient == +1 && minA > a) {
			leastClockwise = candidates[i];
			minA = a;
		}
	}

	if (mostCC != -1) {
		return mostCC;
	}
	else if (colinear != -1) {
		return colinear;
	}
	else {
		return leastClockwise;
	}
}

// return true if face is in CCW order, false o.w
bool ACityBlocksMaker::isCCW(TArray<int> face, Graph<Intersection*> graph) {
	bool ccw = false;
	if (face.Num() > 2) {
		float orient = 0;
		for (int i = 0; i < face.Num(); i++) {
			Point p1 = graph.vertices[face[i]]->data->position;
			Point p2 = graph.vertices[face[(i+1)%face.Num()]]->data->position;
			orient += (p2.x - p1.x) * (p2.y + p1.y);
		}
		if (orient > 0)
			ccw = true;
	}

	return ccw;
}

// Gets the most counterclockwise candidate, if none exist return -1
int ACityBlocksMaker::getBestFaceCandidate(int nextVert, TArray<int> candidates, Point prevEdge, Graph<Intersection*> graph)
{
	if (candidates.Num() > 1) {
		int mostCC = getMostCCW(nextVert, candidates, prevEdge, graph);
		return mostCC;
	}
	else if (candidates.Num() == 1) {
		return candidates[0];
	}
	else {
		return -1;
	}

}

// Extracts the faces of the planar graph
TArray<TArray<int>> ACityBlocksMaker::FindFaces(Graph<Intersection*> graph)
{
	TArray<TArray<int>> faces;
	TSet<int> finishedVerts;
	TSet<FString> foundFaces;

	// iterate over each vertex
	for (auto vert : graph.vertices) {
		int v = vert.first;
		auto neighbors = vert.second->adj;
		
		// iterate over all neighbors
		for (int vadj : neighbors) {
			// keep track of visited nodes (from each neighbor node)
			TArray<int> visit;
			// for traversing the graph
			int prevVert = v;
			int nextVert = vadj;
			visit.Add(nextVert);

			bool foundV = false;
			bool forceStop = false;

			while (!foundV && visit.Num() < 50 && !forceStop) {

				// stop if finished vertex is visited
				if (finishedVerts.Contains(nextVert))
					forceStop = true;

				Point v_p = graph.vertices[prevVert]->data->position;
				Point vadj_p = graph.vertices[nextVert]->data->position;
				Point prevEdge = vadj_p - v_p;

				TArray<int> candidates{};

				// each neighbor of neighbor is a potential candidate
				for (int cand : graph.vertices[nextVert]->adj) {
					if (cand != prevVert) {
						candidates.Add(cand);
					}
				}

				// go to next vertex (next BEST vertex)
				if (candidates.Num() > 0) {
					prevVert = nextVert;
					nextVert = getBestFaceCandidate(nextVert, candidates, prevEdge, graph);
				}
				else {
					forceStop = true;
				}

				// found initial vertex = face found
				if (nextVert == v) {
					foundV = true;
				}

				visit.Add(nextVert);
			}

			bool ccw = isCCW(visit, graph);

			if (foundV && ccw) {
				FString faceString = convertFaceToString(visit);
				if (!foundFaces.Contains(faceString)) {
					faces.Add(visit);
					foundFaces.Add(faceString);
				}
			}

		}

		finishedVerts.Add(v);

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

void ACityBlocksMaker::ParcelBlocks(TArray<TArray<int>> faces, Graph<Intersection*> graph)
{
}

