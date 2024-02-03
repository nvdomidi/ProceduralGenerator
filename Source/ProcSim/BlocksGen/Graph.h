#pragma once

#include <list>
#include <map>
#include <string>
#include <set>
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
		if (vertices.find(id) == vertices.end()) {
			vertices[id] = new Node<MetaData>(id, data);
		}
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
			this->AddNode(datas[i]->ID, datas[i]);
		}
		// adding the edges of the cycle
		for (int i = 0; i < datas.size(); i++) {
			this->AddEdge(datas[i]->ID, datas[(i + 1) % datas.size()]->ID);
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
		Point v_p = this->vertices[v]->data->position;
		int mostCC = -1;
		float minA = std::numeric_limits<float>::max();

		int colinear = -1;

		int leastClockwise = -1;
		float maxA = -std::numeric_limits<float>::max();

		for (int i = 0; i < candidates.size(); i++) {
			Point v_cand = this->vertices[candidates[i]]->data->position;
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
				Point p1 = this->vertices[face[i]]->data->position;
				Point p2 = this->vertices[face[(i + 1) % face.size()]]->data->position;
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

					Point v_p = this->vertices[prevVert]->data->position;
					Point vadj_p = this->vertices[nextVert]->data->position;
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