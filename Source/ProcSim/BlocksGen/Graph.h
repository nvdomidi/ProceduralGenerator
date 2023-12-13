#pragma once

#include <list>
#include <map>
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