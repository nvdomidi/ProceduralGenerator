#pragma once

#include "ProcSim/BlocksGen/Graph.h"
#include "ProcSim/MapGen/MapGen.h"

#include <limits>

class BoundingBox2D {

private:
	FVector2D minCorner, maxCorner;

public:

	BoundingBox2D(float minX, float maxX, float minY, float maxY) {
		this->minCorner = FVector2D{ minX, minY };
		this->maxCorner = FVector2D{ maxX, maxY };
	}

	FVector2D getSize() {
		return FVector2D{ FMath::Abs(this->maxCorner[0] - this->minCorner[0]), FMath::Abs(this->maxCorner[1] - this->minCorner[1]) };
	}

	float getArea() {
		return FMath::Abs(this->maxCorner[0] - this->minCorner[0]) * FMath::Abs(this->maxCorner[1] - this->minCorner[1]);
	}

	FVector2D getCenter() {
		float x = this->minCorner[0] + FMath::Abs(this->maxCorner[0] - this->minCorner[0]) * 0.5;
		float y = this->minCorner[1] + FMath::Abs(this->maxCorner[1] - this->minCorner[1]) * 0.5;

		return FVector2D{ x,y };
	}

	void fromFVectorList(TArray<FVector> face) {
		float minX = std::numeric_limits<float>::max();
		float maxX = -std::numeric_limits<float>::max();
		float minY = std::numeric_limits<float>::max();
		float maxY = -std::numeric_limits<float>::max();

		for (int i = 0; i < face.Num(); i++) {
			minX = FMath::Min(face[i].X, minX);
			maxX = FMath::Max(face[i].X, maxX);
			minY = FMath::Min(face[i].Y, minY);
			maxY = FMath::Max(face[i].Y, maxY);
		}

		this->minCorner = FVector2D{ minX, minY };
		this->maxCorner = FVector2D{ maxX, maxY };
	}

};

class OrientedBoundingBox2D {
	
public:
	FVector2D pos; // center point
	float rot_angle;
	FVector2D extents;
	FVector2D long_axis;
	FVector2D short_axis;
	TArray<FVector2D> corners;

	OrientedBoundingBox2D() {
		this->pos = FVector2D{};
		this->rot_angle = 0.0f;
		this->extents = FVector2D{};
		this->long_axis = FVector2D{};
		this->short_axis = FVector2D{};
		this->corners = TArray<FVector2D>{};
	}

	float getArea() {
		return this->extents.X * this->extents.Y;
	}

	// this function transforms corners on unit box to OBB box
	// e.g: input: {-0.5,-0.5} --> output: bottom left corner on OBB
	FVector2D transformPoint(FVector2D p) {
		float deltaX = this->pos.X;
		float deltaY = this->pos.Y;
		float theta = this->rot_angle; // TODO: check -this->rot_angle; 
		float Sx = this->extents.X;
		float Sy = this->extents.Y;

		float xx = Sx * FMath::Cos(theta) * p.X - Sy * FMath::Sin(theta) * p.Y + deltaX;
		float yy = Sx * FMath::Sin(theta) * p.X + Sy * FMath::Cos(theta) * p.Y + deltaY;

		return FVector2D{ xx,yy };
	}

	void getCorners() {
		this->corners = TArray<FVector2D>{};

		// clockwise from bottom left
		FVector2D p1{ -0.5f, -0.5f };
		FVector2D p2{ -0.5f, 0.5f };
		FVector2D p3{ 0.5f, 0.5f };
		FVector2D p4{ 0.5f, -0.5f };
	
		p1 = transformPoint(p1);
		p2 = transformPoint(p2);
		p3 = transformPoint(p3);
		p4 = transformPoint(p4);

		this->corners.Add(p1);
		this->corners.Add(p2);
		this->corners.Add(p3);
		this->corners.Add(p4);

	}

	void getAxes() {
		FVector2D p1 = this->corners[0];
		FVector2D p2 = this->corners[1];
		FVector2D p3 = this->corners[2];

		float p1p2 = FVector2D::Distance(p1, p2);
		float p2p3 = FVector2D::Distance(p2, p3);

		FVector2D dir{};
		FVector2D ortho{};

		if (p1p2 < p2p3) {
			dir = p2 - p1;
			ortho = p3 - p2;
		}
		else {
			dir = p3 - p2;
			ortho = p2 - p1;
		}

		dir.Normalize();
		ortho.Normalize();

		this->short_axis = dir;
		this->long_axis = ortho;
	}

	// takes a face as input e.g: {1,10,20,30,1} and finds its obb
	void getMinimumFromFace(TArray<FVector> face) {
		float minArea = std::numeric_limits<float>::max();
		BoundingBox2D bb(0, 0, 0, 0);
		bb.fromFVectorList(face);
		FVector2D center = bb.getCenter();
		this->pos = center;
		for (int i = 0; i < face.Num(); i++) {
			FVector2D p1{ face[i].X, face[i].Y };
			FVector2D p2{ face[(i+1)%face.Num()].X, face[(i + 1) % face.Num()].Y};
			FVector2D e1 = p2 - p1;

			UE_LOG(LogTemp, Warning, TEXT("p1: {%f, %f}, p2: {%f, %f}"), p1.X, p1.Y, p2.X, p2.Y);

			e1.Normalize();

			FVector2D dir = e1;

			FVector2D xAxis{ 1,0 };

			
			int o = FMath::Sign(-dir.Y); // TODO: check  FMath::Sign(-dir.Y);
			float d = FVector2D::DotProduct(dir, xAxis);

			float angle = FMath::Acos(d);

			if (o == -1)
				angle *= -1;

			// rotate the face to be in the space of edge
			TArray<FVector> rotated{};

			for (int j = 0; j < face.Num(); j++) {
				FVector c1{ face[j].X, face[j].Y, 0.0f };
				// rotate
				float c1x = c1.X - center.X;
				float c1y = c1.Y - center.Y;
				float sinC = FMath::Sin(angle);
				float cosC = FMath::Cos(angle);

				c1.X = c1x * cosC - c1y * sinC + center.X;
				c1.Y = c1x * sinC + c1y * cosC + center.Y;
				// rotated
				rotated.Add(c1);
			}

			BoundingBox2D curBB(0, 0, 0, 0);
			curBB.fromFVectorList(rotated);
			float currArea = curBB.getArea();
			if (currArea < minArea) {
				this->extents = curBB.getSize();
				this->rot_angle = -angle;
				minArea = currArea;
			}

		}

		this->getCorners();
		this->getAxes();
	}
	
};

class GraphVertex {
public:
	static int currentID;
	int ID;
	Point position;

	GraphVertex(Point pos) {
		this->position = pos;
		this->ID = currentID;
		currentID++;
	}

	bool operator==(const GraphVertex& other) const {
		return this->ID == other.ID;
	}
};

int GraphVertex::currentID = 1;

namespace std {
	template <>
	struct hash<GraphVertex> {
		size_t operator()(const GraphVertex& v) const {
			// Combine the hash of the ID and the position
			size_t h1 = hash<int>()(v.ID);
			size_t h2 = hash<float>()(v.position.x) ^ (hash<float>()(v.position.y) << 1);
			return h1 ^ (h2 << 1); // or use boost::hash_combine
		}
	};
}

// This function gets all the edges overlapping with the input segment (s1 to s2)

inline std::vector<std::tuple<Point, GraphVertex, GraphVertex, float>> getAllEdgeOverlaps(GraphVertex s1, GraphVertex s2, Graph<GraphVertex>* graph) {
	std::vector<std::tuple<Point, GraphVertex, GraphVertex, float>> res{}; // intersections
	std::unordered_set<GraphVertex> visited{};

	for (auto vert : graph->vertices) {
		GraphVertex v1 = vert.second->data;
		std::unordered_set<GraphVertex> neighbors{};
		for (auto id : vert.second->adj) {
			neighbors.insert(graph->vertices[id]->data);
		}
		
		if (v1.position.x != s1.position.x || v1.position.y != s1.position.y) { // prevent self-intersection if edges share vertex
			for (auto v2 : neighbors) {
				if (v2.position.x != s1.position.x || v2.position.y != s1.position.y) {
					if (!visited.count(v2)) {
						Segment* seg1 = new Segment(s1.position, s2.position);
						Segment* seg2 = new Segment(v1.position, v2.position);

						auto intersct = seg1->intersectWith(seg2);
						if (intersct != nullptr) {
							Point resPoint{ intersct->x, intersct->y };
							float dist = sqrt((resPoint.x - s1.position.x)*(resPoint.x - s1.position.x) +
								(resPoint.y - s1.position.y) * (resPoint.y - s1.position.y));

							res.push_back(std::tuple<Point, GraphVertex, GraphVertex, float>{resPoint, v1, v2, dist});
						}
					}
				}
			}
		}

		visited.insert(v1);

	}


		return res;
}


inline void splitEdge(GraphVertex v1, GraphVertex v2, GraphVertex midpt, Graph<GraphVertex>* graph) {
	graph->AddNode(midpt.ID, midpt);
	graph->AddEdge(v1.ID, midpt.ID);
	graph->AddEdge(midpt.ID, v2.ID);

	graph->RemoveEdge(v1.ID, v2.ID);
}

class Parcel {
public:
	Graph<GraphVertex> *graph;
	std::vector<GraphVertex> face; // 
	OrientedBoundingBox2D obb;
	bool street_access = false;
	bool flag = false;
	bool has_street_vert = false;

	// construct parcel from nodes
	Parcel(std::vector<GraphVertex> nodes) {
		this->graph = new Graph<GraphVertex>;
		this->graph->FromFace(nodes);
		this->face = nodes;
		//this->getOBB();
	}

	// copy constructor
	Parcel(const Parcel& other) {
		this->graph = new Graph<GraphVertex>();
		this->graph->FromFace(other.face);
		this->face = other.face;
		this->flag = other.flag;
		this->obb = other.obb;
	}

	// Checks if there is any acute angle in the parcel
	// It separates the acute angles and extracts the square in the middle of the parcel
	void squareAcuteAngles(float angle_delta, float length_delta) {
		
		// to store the parcel 
		std::vector<GraphVertex> squared_arr{};

		// iterate over faces and separate acute angles
		for (int i = 0; i < this->face.size(); i++) {
			int prev = (i == 0) ? this->face.size() - 1 : (i - 1);
			int next = (i + 1) % this->face.size();

			// Vector from this node to previous one 
			Point prevEdge = this->face[prev].position - this->face[i].position;
			float l1 = float(prevEdge.length());
			prevEdge = prevEdge / l1;

			// Vector from this node to next one
			Point nextEdge = this->face[next].position - this->face[i].position;
			float l2 = float(nextEdge.length());
			nextEdge = nextEdge / l2;

			// if its acute we want to inset by half the amount of the smaller edge
			float d = std::min(l1 * 0.5, l2 * 0.5);
			float dot = float(prevEdge.dot(nextEdge));
			float angle = acos(dot);

			// If angle is acute and d is small, omit that part
			if (angle * 57.2958 < angle_delta) {// radians to degrees
				if (!(d < length_delta)) {

					// create two new nodes, insetting current node by d 
					Point s1 = this->face[i].position + prevEdge * d;
					Point s2 = this->face[i].position + nextEdge * d;

					GraphVertex i1(s1);
					GraphVertex i2(s2);

					squared_arr.push_back(i1);
					squared_arr.push_back(i2);
				}
			}
			else {
				squared_arr.push_back(this->face[i]);
			}
		}

		this->face = squared_arr;
		this->graph->FromFace(this->face);
	}

	// splitscale
	void splitScale(Point scale) {
		std::vector<GraphVertex> original_face = this->face;

		this->scaleParcelUniformXY(scale);
		this->addOriginalFace(original_face);
	}

	// splitinset
	void splitInset(float inset) {
		std::vector<GraphVertex> original_face = this->face;
		this->insetParcelUniformXY(inset);
		this->addOriginalFace(original_face);
	}

	// creates the face and assigns neighbors to corresponding verts in this face
	void addOriginalFace(std::vector<GraphVertex> original_face) {
		for (int i = 0; i < this->face.size(); i++) {
			GraphVertex node1 = original_face[i];
			GraphVertex node2 = this->face[i];
			GraphVertex node3 = original_face[(i + 1) % this->face.size()];

			this->graph->AddNode(node1.ID, node1);
			this->graph->AddNode(node3.ID, node3);

			this->graph->AddEdge(node1.ID, node2.ID);
			this->graph->AddEdge(node1.ID, node3.ID);

		}
	}

	// insetParcelUniformXY
	void insetParcelUniformXY(float inset, float limit = 0.06) {
		FVector2D center = this->obb.pos;
		std::vector<GraphVertex> inset_arr{};

		for (int i = 0; i < this->face.size(); i++) {
			float clamp_inset = inset;

			float dist = FVector2D::Distance(FVector2D{ float(this->face[i].position.x),
				float(this->face[i].position.y) }, center);

			clamp_inset = std::min(inset, dist - limit); // TODO: units?

			int prev = i == 0 ? this->face.size() - 1 : (i - 1);
			int next = (i + 1) % this->face.size();

			FVector zAxis{ 0.0,0.0,1.0 };

			// get the perpendicular directions for inset
			Point prevEdgePoint = this->face[i].position - this->face[prev].position;
			FVector prevEdge{ float(prevEdgePoint.x), float(prevEdgePoint.y), 0.0 };
			prevEdge.Normalize();

			FVector prevPerp = FVector::CrossProduct(prevEdge, zAxis);
			prevPerp.Normalize();

			//
			Point nextEdgePoint = this->face[next].position - this->face[i].position;
			FVector nextEdge{ float(nextEdgePoint.x), float(nextEdgePoint.y), 0.0 };
			nextEdge.Normalize();

			FVector nextPerp = FVector::CrossProduct(nextEdge, zAxis);
			nextPerp.Normalize();

			//
			float tol = 0.0;

			Point p1 = this->face[prev].position;
			p1 = p1 + Point{prevPerp.X, prevPerp.Y} *(-clamp_inset);
			p1 = p1 + Point{prevEdge.X, prevEdge.Y} *(-tol);


			Point p2 = this->face[i].position;
			p2 = p2 + Point{prevPerp.X, prevPerp.Y} *(-clamp_inset);
			p2 = p2 + Point{prevEdge.X, prevEdge.Y} *(tol);


			Point p3 = this->face[i].position;
			p3 = p3 + Point{nextPerp.X, nextPerp.Y} *(-clamp_inset);
			p3 = p3 + Point{nextEdge.X, nextEdge.Y} *(-tol);


			Point p4 = this->face[next].position;
			p4 = p4 + Point{nextPerp.X, nextPerp.Y} *(-clamp_inset);
			p4 = p4 + Point{nextEdge.X, nextEdge.Y} *(+tol);


			bool colinear = areFourPointsCollinear(p1, p2, p3, p4);

			Segment* seg1 = new Segment(p1, p2);
			Segment* seg2 = new Segment(p3, p4);

			auto intersct = seg1->intersectWith(seg2);
			Point isect{};
			if (intersct != nullptr) {
				isect.x = intersct->x;
				isect.y = intersct->y;
			}

			if (colinear) {
				isect = this->face[i].position + Point{prevPerp.X, prevPerp.Y} *(-clamp_inset);
			}
			else {
				if (intersct == nullptr) {
					isect = this->face[i].position + Point{prevPerp.X, prevPerp.Y} *(-clamp_inset);
				}
			}

			FVector dir = FMath::Lerp(prevEdge, nextEdge, 0.5);
			dir.Normalize();

			Point sc = this->face[i].position + Point{dir.X, dir.Y}*inset;

			if (intersct != nullptr) {
				inset_arr.push_back(*(new GraphVertex(isect)));
			}
		}

		this->face = inset_arr;
		this->graph->FromFace(this->face);

	}

	// rotateParcelZ
	void rotateParcelZ(float angle) {
		FVector2D center = this->obb.pos;
		std::vector<GraphVertex> rotated{};
		for (int i = 0; i < this->face.size(); i++) {
			Point sc = this->face[i].position;
			float scx = sc.x - center.X;
			float scy = sc.y - center.Y;

			float newcx = scx * cos(angle) - scy * sin(angle) + center.X;
			float newcy = scx * sin(angle) + scy * cos(angle) + center.Y;

			rotated.push_back(*(new GraphVertex(Point{ newcx, newcy })));
		}

		this->face = rotated;
		this->graph->FromFace(this->face);
		this->obb.rot_angle += angle;
	}

	// scaleParcelUniformXY
	void scaleParcelUniformXY(Point scale) {
		FVector2D center = this->obb.pos;
		std::vector<GraphVertex> scaled{};
		for (int i = 0; i < face.size(); i++) {
			Point sc = this->face[i].position;
			sc = Point{ (sc.x - center.X) * scale.x + center.X, (sc.y - center.Y) * scale.y + center.Y };
			scaled.push_back(*(new GraphVertex(sc)));
		}

		this->face = scaled;
		this->graph->FromFace(this->face);
	}

	// getOBB
	void getOBB() {
		this->obb = *(new OrientedBoundingBox2D());
		if (this->face.size() > 0) {
			TArray<FVector> faceVectors{};
			for (int i = 0; i < this->face.size(); i++) {
				faceVectors.Add(FVector{float(this->face[i].position.x), float(this->face[i].position.y), 0.0 });
			}
			this->obb.getMinimumFromFace(faceVectors);
		}
	}

	// w: pivot offset from midpoint at which to split; fraction of long axis length
	void splitOBB(float w = 0.0) {
		w *= std::max(this->obb.extents[0], this->obb.extents[1]);

		FVector2D midpt{ this->obb.pos[0], this->obb.pos[1] };
		FVector2D dir3{ this->obb.short_axis[0], this->obb.short_axis[1] };
		FVector2D ortho3{ this->obb.long_axis[0], this->obb.long_axis[1] };

		midpt = midpt + ortho3 * w;
		this->splitAtPointAlong(midpt, dir3);

	}

	// splitOBBSym
	void splitOBBSym(float w = 0.0) {
		w *= std::max(this->obb.extents[0], this->obb.extents[1]);
		FVector2D midpt{ this->obb.pos[0], this->obb.pos[1] };
		FVector2D dir3{ this->obb.short_axis[0], this->obb.short_axis[1] };
		FVector2D ortho3{ this->obb.long_axis[0], this->obb.long_axis[1] };

		FVector2D m1 = midpt + ortho3 * w;
		this->splitAtPointAlong(m1, dir3);

		FVector2D m2 = midpt + ortho3 * (-w);
		this->splitAtPointAlong(m2, dir3);
	}

	// splitAtPointAlong
	void splitAtPointAlong(FVector2D midpt, FVector2D dir) {
		FVector2D m1 = midpt + dir * 100.0;
		FVector2D m2 = midpt + dir * -100.0;

		GraphVertex s1 = *(new GraphVertex(Point{ m1.X, m1.Y }));
		GraphVertex s2 = *(new GraphVertex(Point{ m2.X, m2.Y }));

		this->splitAlong(s1, s2);
	}

	// p1 and p2 are endpoints of segment, w is offset from midpoint on which to split
	void splitPerpendicularTo(FVector2D p1, FVector2D p2, float w = 0.0) {
		FVector e1 = FVector{p1.X, p1.Y, 0.0} - FVector{p2.X, p2.Y, 0.0};

		FVector dir = FVector::CrossProduct(e1, FVector{ 0.0,0.0,1.0 });

		FVector midpt = FMath::Lerp(FVector{ p1.X, p1.Y, 0.0 }, FVector{ p2.X, p2.Y, 0.0 }, 0.5);

		this->splitAtPointAlong(FVector2D{ midpt.X, midpt.Y }, FVector2D{ dir.X, dir.Y });
		
	}


	// splitInHalf
	void splitInHalf() {
		FVector e1{ float(this->face[1].position.x - this->face[0].position.x),
			float(this->face[1].position.y - this->face[0].position.y), 0.0 };
		FVector dir = FVector::CrossProduct(e1, FVector{ 0.0,0.0,1.0 });
		FVector midpt = FMath::Lerp(FVector{ float(this->face[1].position.x), float(this->face[1].position.y), 0.0 },
			FVector{ float(this->face[0].position.x), float(this->face[0].position.y), 0.0 }, 0.5);
		midpt = midpt + dir * 0.1;

		FVector m2 = dir * -100;

		GraphVertex s1 = *(new GraphVertex(Point{ midpt.X, midpt.Y }));
		GraphVertex s2 = *(new GraphVertex(Point{ m2.X, m2.Y }));

		this->splitAlong(s1, s2);
	}

	// splitAlong
	void splitAlong(GraphVertex s1, GraphVertex s2) {
		std::vector<std::tuple<Point, GraphVertex, GraphVertex, float>> isects = getAllEdgeOverlaps(s1, s2, this->graph);
		std::sort(isects.begin(), isects.end(),
			[](std::tuple<Point, GraphVertex, GraphVertex, float> isect1,
				std::tuple<Point, GraphVertex, GraphVertex, float> isect2) {return std::get<float>(isect1) < std::get<float>(isect2); });

		if (isects.size() % 2 != 0) {
			this->flag = true;
			return;
		}

		for (int i = 0; i < isects.size() - 1; i += 2) {
			std::tuple<Point, GraphVertex, GraphVertex, float> i1 = isects[i];
			std::tuple<Point, GraphVertex, GraphVertex, float> i2 = isects[i+1];
			
			GraphVertex v1 = *(new GraphVertex(Point{std::get<Point>(i1)}));
			splitEdge(std::get<1>(i1), std::get<2>(i1), v1, this->graph);

			GraphVertex v2 = *(new GraphVertex(Point{ std::get<Point>(i2) }));
			splitEdge(std::get<1>(i2), std::get<2>(i2), v2, this->graph);

			this->graph->AddNode(v1.ID, v1);
			this->graph->AddNode(v2.ID, v2);

			this->graph->AddEdge(v1.ID, v2.ID);
		}
	}

	// getAllEdgeOverlaps
	std::vector<std::tuple<Point, GraphVertex, GraphVertex, float>> getAllEdgeOverlapsParcel (GraphVertex s1, GraphVertex s2) {
		std::vector<std::tuple<Point, GraphVertex, GraphVertex, float>> res{};

		for (int i = 0; i < this->face.size(); i++) {
			GraphVertex v1 = this->face[i];
			GraphVertex v2 = this->face[(i + 1) % this->face.size()];

			Segment* seg1 = new Segment(s1.position, s2.position);
			Segment* seg2 = new Segment(v1.position, v2.position);
			auto intersct = seg1->intersectWith(seg2);

			if (intersct != nullptr) {
				float dist = sqrt(pow(s1.position.x - intersct->x, 2) + pow(s1.position.y - intersct->y, 2));
				res.push_back(std::tuple<Point, GraphVertex, GraphVertex, float>(Point{ intersct->x, intersct->y },
					v1, v2, dist));
			}
		}

		return res;
	}

	// hasStreetAccess
	void hasStreetAccess(std::vector<GraphVertex> streets) {
		for (int j = 0; j < this->face.size(); j++) {
			std::vector<GraphVertex> f = this->face;
			Segment* s1 = new Segment(f[j].position, f[(j + 1) % f.size()].position);

			for (int ff = 0; ff < streets.size(); ff++) {
				Segment* s2 = new Segment(streets[ff].position, streets[(ff + 1) % streets.size()].position);

				if (areFourPointsCollinear(s1->start, s1->end, s2->start, s2->end)) {
					this->street_access = true;
				}
				
				float dist = sqrt(pow(streets[ff].position.x - f[j].position.x, 2) + pow(streets[ff].position.y - f[j].position.y, 2));
				if (dist < 1e-5) {
					this->has_street_vert = true;
				}

			}
		}
	}

};

class Block {

public:
	std::vector<Parcel> parcels;

	Block() {
		this->parcels = std::vector<Parcel>{};
	}

	void insetParcels(float inset) {
		std::vector<GraphVertex> original_face = this->parcels[0].face;
		std::vector<Parcel> next_parcels{};

		for (int i = 0; i < this->parcels.size(); i++) {
			Parcel p = this->parcels[i];
			p.splitInset(inset);
			p.graph->FindFaces();
			for (int j = 0; j < p.graph->faces.size(); j++) {

				std::vector<GraphVertex> next_parc_verts{};

				for (auto node : p.graph->faces[j]) {
					next_parc_verts.push_back(p.graph->vertices[node]->data);
				}

				Parcel next_parc(next_parc_verts);
				next_parc.flag = p.flag;
				next_parcels.push_back(next_parc);
			}
		}

		this->parcels = next_parcels;

		for (int i = 0; i < this->parcels.size(); i++) {
			this->parcels[i].hasStreetAccess(original_face);
		}
	}

	void scaleParcel(Point scale) {
		std::vector<GraphVertex> original_face = this->parcels[0].face;

		std::vector<Parcel> next_parcels{};

		for (int i = 0; i < this->parcels.size(); i++) {
			Parcel p = this->parcels[i];
			p.splitScale(scale);
			p.graph->FindFaces();
			for (int j = 0; j < p.graph->faces.size(); j++) {
				
				std::vector<GraphVertex> next_parc_verts{};
				for (auto node : p.graph->faces[j]) {
					next_parc_verts.push_back(p.graph->vertices[node]->data);
				}
				
				Parcel next_parc(next_parc_verts);
				next_parc.flag = p.flag;
				next_parcels.push_back(next_parc);
			}
		}

		this->parcels = next_parcels;

		for (int i = 0; i < this->parcels.size(); i++) {
			this->parcels[i].hasStreetAccess(original_face);
		}
	}

	void subdivideParcels(float minArea = 0.14, float w_min = -0.2, float w_max = 0.2, bool sym = false, int iterations = 6) {
		std::vector<GraphVertex> original_face = this->parcels[0].face;

		int n = 0;
		bool below_area = false;
		while (n < iterations) {
			n++;
			std::vector<Parcel> next_parcels;
			for (int i = 0; i < this->parcels.size(); i++) {
				Parcel p = this->parcels[i];

				// if larger than area limit, split again, otherwise keep
				if (p.obb.getArea() > minArea && !p.flag) {
					
					std::random_device rd;
					std::mt19937 gen(rd());

					std::uniform_real_distribution<float> dis(w_min, w_max);
					float w = dis(gen);

					if (sym) {
						p.splitOBBSym(w);
					}
					else {
						p.splitOBB(w);
					}
					
					p.graph->FindFaces();
					for (int j = 0; j < p.graph->faces.size(); j++) {

						std::vector<GraphVertex> next_parc_verts{};
						for (auto node : p.graph->faces[j]) {
							next_parc_verts.push_back(p.graph->vertices[node]->data);
						}

						Parcel next_parc(next_parc_verts);
						next_parc.flag = p.flag;
						next_parcels.push_back(next_parc);

					}
				}
				else {
					below_area = true;
					if (!p.flag) {
						next_parcels.push_back(p);
					}
				}
			}

			this->parcels = next_parcels;
		}

		for (int i = 0; i < this->parcels.size(); i++) {
			this->parcels[i].hasStreetAccess(original_face);
		}
	}

};