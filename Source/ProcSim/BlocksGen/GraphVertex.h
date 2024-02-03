#pragma once
#include "ProcSim/MapGen/Math.h"

class GraphVertex {
public:
	int ID;
	Point position;

	GraphVertex(Point pos, int num) {
		this->position = pos;
		this->ID = num;
	}

	bool operator==(const GraphVertex& other) const {
		return this->ID == other.ID;
	}
};
