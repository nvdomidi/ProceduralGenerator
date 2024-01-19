#pragma once

#include <list>
#include <map>
#include <string>
#include <unordered_set>

enum Color { WHITE, GRAY, BLACK };

template<typename MetaData>
class Node {
public:
	// each node has ID, color, degree, label and a blocked value
	int id;
	Color color;
	int degree;
	int label;
	int blocked;

	std::unordered_set<int> adj; // ID of neighbours (adjacency list)
	MetaData data; // allows the node to hold anything

	Node(int id, const MetaData data) : id(id), data(data) {};
};

template<typename MetaData>
class Graph {

public:
	std::map<int, Node<MetaData>*> vertices;
	std::vector<std::vector<int>> faces;

	// adds node with id and data specified
	void AddNode(const int id, const MetaData data) {
		vertices[id] = new Node<MetaData>(id, data);
	}
	// adds edges in both directions
	bool AddEdge(const int id1, const int id2) {
		if (id1 == id2 || vertices.find(id1) == vertices.end() || vertices.find(id2) == vertices.end()) {
			UE_LOG(LogTemp, Error, TEXT("ERROR ADDING EDGE"));
			return false;
		}

		vertices[id1]->adj.insert(id2);
		vertices[id2]->adj.insert(id1);
		return true;
	}

	// removes edge
	bool RemoveEdge(const int id1, const int id2) {
		if (id1 == id2 || vertices.find(id1) == vertices.end() || vertices.find(id2) == vertices.end()) {
			UE_LOG(LogTemp, Error, TEXT("ERROR REMOVING EDGE"));
			return false;
		}

		vertices[id1]->adj.erase(vertices[id1]->adj.find(id2));
		vertices[id2]->adj.erase(vertices[id2]->adj.find(id1));
		return true;
	}

	// constructs graph from the input face
	void FromFace(const std::vector<MetaData> datas) {

		vertices.clear();

		// adding the nodes
		for (int i = 0; i < datas.size(); i++) {
			this->AddNode(datas[i].ID, datas[i]);
		}
		// adding the edges of the cycle
		for (int i = 0; i < datas.size(); i++) {
			this->AddEdge(datas[i].ID, datas[(i + 1) % datas.size()].ID);
		}
	}

	std::string convertFaceToString(std::vector<int> face) {
		std::string res = "";
		int minId = std::numeric_limits<int>::max();
		int minIdInd = 0;

		//start traversing the face at the minimum id, since same faces may have different starting indices
		for (int i = 0; i < face.size(); i++) {
			if (face[i] < minId) {
				minId = face[i];
				minIdInd = i;
			}
		}

		for (int i = minIdInd; i < minIdInd + face.size(); i++) {
			int toAdd = face[i % face.size()];
			res += (std::to_string(toAdd) + ",");
		}

		return res;
	}

	// calculates angles and finds the most counter clockwise edge to prevEdge
	int getMostCCW(int v, std::vector<int> candidates, Point prevEdge) {
		Point v_p = this->vertices[v]->data.position;
		int mostCC = -1;
		float minA = std::numeric_limits<float>::max();

		int colinear = -1;

		int leastClockwise = -1;
		float maxA = -std::numeric_limits<float>::max();

		for (int i = 0; i < candidates.size(); i++) {
			Point v_cand = this->vertices[candidates[i]]->data.position;
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
	bool isCCW(std::vector<int> face) {
		bool ccw = false;
		if (face.size() > 2) {
			float orient = 0;
			for (int i = 0; i < face.size(); i++) {
				Point p1 = this->vertices[face[i]]->data.position;
				Point p2 = this->vertices[face[(i + 1) % face.size()]]->data.position;
				orient += (p2.x - p1.x) * (p2.y + p1.y);
			}
			if (orient > 0)
				ccw = true;
		}

		return ccw;
	}


	// Gets the most counterclockwise candidate, if none exist return -1
	int getBestFaceCandidate(int nextVert, std::vector<int> candidates, Point prevEdge) {
		
		if (candidates.size() > 1) {
			int mostCC = this->getMostCCW(nextVert, candidates, prevEdge);
			return mostCC;
		}
		else if (candidates.size() == 1) {
			return candidates[0];
		}
		else {
			return -1;
		}
	}

	// Extracts the faces of a planar graph
	void FindFaces() {
		std::vector<std::vector<int>> fs;
		std::set<int> finishedVerts;
		std::set<std::string> foundFaces;

		// iterate over each vertex
		for (auto vert : vertices) {
			
			int v = vert.first;
			auto neighbors = vert.second->adj;

			// iterate over all neighbors
			for (int vadj : neighbors) {

				// keep track of visited nodes (from each neighbor node)
				std::vector<int> visit;
				// for traversing the graph
				int prevVert = v;
				int nextVert = vadj;
				
				visit.push_back(nextVert);

				bool foundV = false;
				bool forceStop = false;

				while (!foundV && visit.size() < 50 && !forceStop) {

					// stop if finished vertex is visited
					if (finishedVerts.count(nextVert))
						forceStop = true;

					Point v_p = this->vertices[prevVert]->data.position;
					Point vadj_p = this->vertices[nextVert]->data.position;
					Point prevEdge = vadj_p - v_p;

					std::vector<int> candidates{};

					// each neighbor of neighbor is a potential candidate
					for (int cand : this->vertices[nextVert]->adj) {
						if (cand != prevVert) {
							candidates.push_back(cand);
						}
					}

					// go to next vertex (next BEST vertex)
					if (candidates.size() > 0) {
						prevVert = nextVert;
						nextVert = this->getBestFaceCandidate(nextVert, candidates, prevEdge);
					}
					else {
						forceStop = true;
					}

					// found initial vertex = face found
					if (nextVert == v) {
						foundV = true;
					}

					visit.push_back(nextVert);
				}

				bool ccw = this->isCCW(visit);

				if (foundV && ccw) {
					std::string faceString = convertFaceToString(visit);
					if (!foundFaces.count(faceString)) {
						fs.push_back(visit);
						foundFaces.insert(faceString);
					}
				}
			}
			finishedVerts.insert(v);
		}

		this->faces = fs;

	}
};

/* This function gives labels to each vertex of the graph based on their degree */
template<typename MetaData>
void DegreeLabeling(Graph<MetaData>& graph)
{
	// for each vertex in graph.
	// set color to white.
	// set degree to number of adjacent vertices
	for (auto it = graph.vertices.begin(); it != graph.vertices.end(); ++it) {
		Node<MetaData>* vertex = (*it).second;
		vertex->degree = 0;
		vertex->color = WHITE;
		for (int adjIdx : vertex->adj) {
			vertex->degree++;
		}
	}

	// do this n times = graph vertices cardinality
	int n = graph.vertices.size();
	Node<MetaData>* vertex{};
	for (int i = 0; i < n; i++) {
		// find vertex with minimum degree
		int min_degree = n;
		for (auto& pair : graph.vertices) {
			if (pair.second->color == WHITE && pair.second->degree < min_degree) {
				vertex = pair.second;
				min_degree = pair.second->degree;
			}
		}
		// set its label to i and color to black
		vertex->label = i;
		vertex->color = BLACK;

		// decrease degree of neighbours as if it was removed from the graph
		for (int u : vertex->adj) {
			if (graph.vertices[u]->color == WHITE) {
				graph.vertices[u]->degree--;
			}
		}
	}
}

/* This function finds the initial triplets that can form chordless paths and cycles of 3 */
template<typename MetaData>
void Triplets(Graph<MetaData>& graph, std::list<std::list<int>>& T, std::list<std::list<int>>& C)
{
	T = std::list<std::list<int>>{};
	C = std::list<std::list<int>>{};

	// iterate over all vertices
	for (auto it = graph.vertices.begin(); it != graph.vertices.end(); ++it) {
		Node<MetaData>* vertex = (*it).second;
		// all permutations of two neighbours for vertices that have more than one neighbour
		for (auto first = vertex->adj.begin(); first != vertex->adj.end(); ++first) {
			for (auto second = vertex->adj.begin(); second != vertex->adj.end(); ++second) {
				// dont compare something with itself
				if (*first == *second)
					continue;
				// then two neighbours
				Node<MetaData>* x = graph.vertices[*first];
				Node<MetaData>* y = graph.vertices[*second];
				// check if labels are valid unique triplet
				if (vertex->label < x->label && x->label < y->label) {
					// if there is no edge = triplet
					if (std::find(x->adj.begin(), x->adj.end(), y->id) == x->adj.end()) {
						T.push_back(std::list<int>{x->id, vertex->id, y->id});
					}
					// if there is edge = cycle of 3
					else {
						C.push_back(std::list<int>{x->id, vertex->id, y->id});
					}
				}

			}
		}
	}
}

/* This function is used to block the neighbors of the input vertex while expanding the path */
template<typename MetaData>
void BlockNeighbors(Node<MetaData>* v, Graph<MetaData>& graph)
{
	// increase the blocked value of each neighbor
	for (int adjID : v->adj) {
		graph.vertices[adjID]->blocked++;
	}
}

/* This function is used to unblock the neighbors of the input vertex */
template<typename MetaData>
void UnblockNeighbors(Node<MetaData>* v, Graph<MetaData>& graph)
{
	// decrease the blocked value of each neighbor
	for (int adjID : v->adj) {
		if (graph.vertices[adjID]->blocked > 0)
			graph.vertices[adjID]->blocked--;
	}
}

/* This function takes a chordless path as input and expands it to find cycles */
/*
template<typename MetaData>
void CC_Visit(std::list<int>& p, std::list<std::list<int>>& C, int key, Graph<MetaData>& graph)
{
	// the chordless path shouldnt be empty
	if (p.empty())
		throw std::runtime_error("The path is empty");

	// block the neighbors of the last vertex in the path
	Node<MetaData>* ut = graph.vertices[p.back()];
	BlockNeighbors(ut, graph);

	for (auto it = ut->adj.begin(); it != ut->adj.end(); ++it) {
		// now we are iterating over all neighbors of the last node in the path
		Node<MetaData>* v = graph.vertices[*it];
		// if the neighbor's key is bigger than the least one in cycle and has low val of blocked
		if (v->label > key && v->blocked == 1) {
			// create new path adding this neighbor
			std::list<int> pprime = p;
			pprime.push_back(v->id);
			// if there is a back edge to the first one => cycle
			Node<MetaData>* u1 = graph.vertices[pprime.front()];
			if (std::find(v->adj.begin(), v->adj.end(), u1->id) != v->adj.end()) {
				C.push_back(pprime);
			}
			else {
				CC_Visit(pprime, C, key, graph);
			}
		}
	}

	UnblockNeighbors(ut, graph);
}*/

template<typename MetaData>
void CC_Visit(std::list<int>& p, std::list<std::list<int>>& C, int key, Graph<MetaData>& graph, int depth = 0)
{
	if (depth > 15) {
		return; // Stop further recursion if depth exceeds 
	}

	// the chordless path shouldn't be empty
	if (p.empty())
		throw std::runtime_error("The path is empty");

	// block the neighbors of the last vertex in the path
	Node<MetaData>* ut = graph.vertices[p.back()];
	BlockNeighbors(ut, graph);

	for (auto it = ut->adj.begin(); it != ut->adj.end(); ++it) {
		// now we are iterating over all neighbors of the last node in the path
		Node<MetaData>* v = graph.vertices[*it];
		// if the neighbor's key is bigger than the least one in cycle and has low val of blocked
		if (v->label > key && v->blocked == 1) {
			// create a new path adding this neighbor
			std::list<int> pprime = p;
			pprime.push_back(v->id);
			// if there is a back edge to the first one => cycle
			Node<MetaData>* u1 = graph.vertices[pprime.front()];
			if (std::find(v->adj.begin(), v->adj.end(), u1->id) != v->adj.end()) {
				C.push_back(pprime);
			}
			else {
				CC_Visit(pprime, C, key, graph, depth + 1);
			}
		}
	}

	UnblockNeighbors(ut, graph);
}

/* This function is the main algorithm to find chordless cycles in a graph */
template<typename MetaData>
std::list<std::list<int>> ChordlessCycles(Graph<MetaData>& graph)
{
	DegreeLabeling(graph);
	std::list<std::list<int>> T;
	std::list<std::list<int>> C;
	Triplets(graph, T, C);

	UE_LOG(LogTemp, Warning, TEXT("number of triplets: %d"), T.size());

	for (auto pair : graph.vertices) {
		pair.second->blocked = 0;
	}

	while (T.size() != 0) {
		std::list<int> p = T.front(); // p is a chordless path
		T.pop_front();

		auto it = std::next(p.begin());
		Node<MetaData>* u = graph.vertices[*it];
		BlockNeighbors(u, graph);
		CC_Visit(p, C, u->label, graph);
		UnblockNeighbors(u, graph);
	}

	return C;
}