#include "MapGen.h"


bool localConstraints(Segment* segment, std::vector<Segment*>& segments, Quadtree<Segment*>& qTree,
	DebugData& debugData, std::vector<Intersection*>& intersections) {

	if (Config::IGNORE_CONFLICTS) return true;

	Action action = { 0, nullptr, 0.0 };

	for (auto other : qTree.retrieve(segment->limits())) {

		// Intersection check
		if (action.priority <= 4) {

			LineSegmentIntersection* intersection = segment->intersectWith(other);


			if (intersection != nullptr) {
				if (action.func == nullptr || intersection->t < action.t) {
					action.t = intersection->t;
					action.priority = 4;

					action.func = [other, segment, &segments, &qTree, intersection, &debugData, &intersections]() -> bool {



						// If intersecting lines are too similar, don't continue
						if (Math::minDegreeDifference(other->dir(), segment->dir()) < Config::MINIMUM_INTERSECTION_DEVIATION) {
							return false;
						}
						Point intersectionPoint{ intersection->x, intersection->y };
						other->split(intersectionPoint, segment, segments, qTree, intersections);
						segment->end = Point{ intersection->x, intersection->y };
						segment->q.severed = true;
						(debugData).intersections.push_back(*intersection);

						return true;
					};
				}
			}
		}

		// Snap to crossing within radius check
		if (action.priority <= 3) {
			if (Math::length(segment->end, other->end) <= Config::ROAD_SNAP_DISTANCE) {
				Point point = other->end;
				action.priority = 3;

				action.func = [point, other, segment, &segments, &qTree, &debugData, &intersections]() {
					segment->end = point;
					segment->q.severed = true;
					auto links = other->startIsBackwards() ? other->links_f : other->links_b;

					for (auto ll : links) {
						if ((Math::equalV(ll->start, segment->end) && Math::equalV(ll->end, segment->start)) ||
							(Math::equalV(ll->start, segment->start) && Math::equalV(ll->end, segment->end))) {
						}
					}

					if (std::any_of(links.begin(), links.end(), [segment](const auto link) {
						return (Math::equalV(link->start, segment->end) && Math::equalV(link->end, segment->start)) ||
							(Math::equalV(link->start, segment->start) && Math::equalV(link->end, segment->end));
					})) {
						return false;
					}

					for (auto link : links) {
						bool front = false;
						auto containing = link->linksForEndContaining(other, front);

						if (!front)
						{
							link->links_b.push_back(segment);
						}
						else {
							link->links_f.push_back(segment);
						}

						segment->links_f.push_back(link);
					}


					links.push_back(segment);
					segment->links_f.push_back(other);
					(debugData).snaps.push_back({ point.x, point.y });

					// TODO: fix
					/* adding the newly created segment to the previously created intersection*/
					//for (auto intersection : intersections) {
						
						//if (intersection->IsAtQueriedPosition(point)) {
						//	intersection->branches.push_back(segment);
						//	bool isStart = ((segment->start - intersection->position).length() < (segment->end - intersection->position).length());
						//	if (isStart)
						//		segment->startIntersectionID = intersection->ID;
						//	else
						//		segment->endIntersectionID = intersection->ID;
						//	break;
						//}
					//}


					return true;
				};
			}
		}

		//  intersection within radius check
		if (action.priority <= 2) {
			Math::DistanceToLineResult distanceToLineResult = Math::distanceToLine(segment->end, other->start, other->end);
			if (distanceToLineResult.distance2 < Config::ROAD_SNAP_DISTANCE * Config::ROAD_SNAP_DISTANCE &&
				distanceToLineResult.lineProj2 >= 0 &&
				distanceToLineResult.lineProj2 <= distanceToLineResult.length2) {

				Point point = distanceToLineResult.pointOnLine;
				action.priority = 2;

				action.func = [segment, point, other, &segments, &qTree, &debugData, &intersections]() {
					segment->end = point;
					segment->q.severed = true;
					// if intersecting lines are too closely aligned don't continue
					if (Math::minDegreeDifference(other->dir(), segment->dir()) < Config::MINIMUM_INTERSECTION_DEVIATION) {
						return false;
					}

					other->split(point, segment, segments, qTree, intersections);
					(debugData).intersectionsRadius.push_back(Point{ point.x, point.y });
					return true;
				};


			}
		}
	}

	if (action.func != nullptr)
	{
		return action.func();
	}
	return true;
}

std::vector<Segment*> globalGoalsGenerate(Segment* previousSegment, Heatmap heatmap) {
	std::vector<Segment*> newBranches;

	if (!previousSegment->q.severed) {
		auto templateFunc = [=](
			double direction, double length, double t, MetaInfo* q = nullptr) {
			return Segment::usingDirection(previousSegment->end, t, *q, previousSegment->dir() + direction, length);
		};

		auto templateContinue = [=](double direction) {
			return templateFunc(direction, previousSegment->length(), 0.0, &previousSegment->q);
		};

		auto templateBranch = [=](double direction) {
			return templateFunc(direction, Config::DEFAULT_SEGMENT_LENGTH,
				previousSegment->q.highway ? Config::NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY : 0,
				new MetaInfo{ 0, previousSegment->q.color, previousSegment->q.severed });
		};

		Segment* continueStraight = templateContinue(0);

		double straightPop = heatmap.popOnRoad(*continueStraight);

		if (previousSegment->q.highway) {
			double maxPop = straightPop;
			Segment* bestSegment = continueStraight;

			for (int i = 0; i < Config::HIGHWAY_POPULATION_SAMPLE_SIZE; i++) {
				Segment* curSegment = templateContinue(Config::RANDOM_STRAIGHT_ANGLE());
				double curPop = heatmap.popOnRoad(*curSegment);

				if (curPop > maxPop) {
					maxPop = curPop;
					bestSegment = curSegment;
				}
			}

			newBranches.push_back(bestSegment);

			if (maxPop > Config::HIGHWAY_BRANCH_POPULATION_THRESHOLD) {
				if (std::rand() / static_cast<double>(RAND_MAX) < Config::HIGHWAY_BRANCH_PROBABILITY) {
					newBranches.push_back(templateContinue(-90 + Config::RANDOM_BRANCH_ANGLE()));
				}
				else {
					if (std::rand() / static_cast<double>(RAND_MAX) < Config::HIGHWAY_BRANCH_PROBABILITY) {
						newBranches.push_back(templateContinue(+90 + Config::RANDOM_BRANCH_ANGLE()));
						
					}
				}
			}
		}
		else if (straightPop > Config::NORMAL_BRANCH_POPULATION_THRESHOLD) {
			newBranches.push_back(continueStraight);
		}

		if (!Config::ONLY_HIGHWAYS && straightPop > Config::NORMAL_BRANCH_POPULATION_THRESHOLD) {
			if (std::rand() / static_cast<double>(RAND_MAX) < Config::DEFAULT_BRANCH_PROBABILITY) {
				newBranches.push_back(templateBranch(-90 + Config::RANDOM_BRANCH_ANGLE()));
			}
			else {
				if (std::rand() / static_cast<double>(RAND_MAX) < Config::DEFAULT_BRANCH_PROBABILITY) {
					newBranches.push_back(templateBranch(+90 + Config::RANDOM_BRANCH_ANGLE()));
				}
			}
		}
	}

	for (auto branch : newBranches) {

		// setup links between each current branch and each existing branch stemming from the previous segment

		branch->setupBranchLinks = [branchPtr = std::shared_ptr<Segment>(branch), previousSegmentPtr = std::shared_ptr<Segment>(previousSegment)]() {
			Segment* currentBranch = branchPtr.get();
			Segment* prevSegment = previousSegmentPtr.get();

			for (auto link : prevSegment->links_f) {
				currentBranch->links_b.push_back(link);

				bool front = false;
				auto containing = link->linksForEndContaining(prevSegment, front);

				if (containing == std::vector<Segment*> {}) throw std::runtime_error("Impossible");

				if (!front)
					link->links_b.push_back(currentBranch);
				else
					link->links_f.push_back(currentBranch);

			}

			prevSegment->links_f.push_back(currentBranch);
			currentBranch->links_b.push_back(prevSegment);




		};

	}

	return newBranches;
}



std::vector<Segment*> makeInitialSegments() {
	std::vector<Segment*> segments;

	// Setup first segments in queue

	MetaInfo q{};
	q.highway = !Config::START_WITH_NORMAL_STREETS;
	Segment* rootSegment = new Segment(*(new Point{ 0,0 }), *(new Point{ Config::HIGHWAY_SEGMENT_LENGTH, 0 }), 0.0, q);

	if (!Config::TWO_SEGMENTS_INITIALLY) {
		segments.push_back(rootSegment);
		return segments;
	}

	Segment* oppositeDirection = new Segment(rootSegment->start,
		{ rootSegment->start.x - Config::HIGHWAY_SEGMENT_LENGTH, rootSegment->end.y },
		0.0,
		q);


	oppositeDirection->links_b.push_back(rootSegment);
	rootSegment->links_b.push_back(oppositeDirection);

	segments.push_back(rootSegment);
	segments.push_back(oppositeDirection);

	return segments;
}


void generationStep(
	PriorityQueue<Segment*>& priorityQ,
	std::vector<Segment*>& segments,
	Quadtree<Segment*>& qTree,
	DebugData& debugData,
	std::vector<Intersection*>& intersections,
	Heatmap heatmap
) {
	Segment* minSegment = priorityQ.dequeue();

	if (minSegment == nullptr) throw std::runtime_error("no segment remaining");
	bool accepted = localConstraints(minSegment, segments, qTree, debugData, intersections);

	if (accepted) {

		if (minSegment->setupBranchLinks != nullptr) {
			minSegment->setupBranchLinks();
		};

		segments.push_back(minSegment);
		Bounds minSegmentLimits = minSegment->limits();
		qTree.insert(minSegmentLimits, minSegment);
		std::vector<Segment*> newSegments = globalGoalsGenerate(minSegment, heatmap);
		for (auto newSegment : newSegments) {
			//smth about inersections here
			newSegment->t = minSegment->t + 1 + newSegment->t;
			priorityQ.enqueue(newSegment);
		}
	}
}

void findOrderAtEnd(Segment* segment, bool isStart)
{
	std::vector<Segment*> links = (isStart ? segment->links_b : segment->links_f);
	
	// if there is no link do nothing
	if (links.size() < 1) {}
	// if there is one link set it to 1 + others order
	else if (links.size() == 1) {
		if (isStart) {
			if (std::find(links[0]->links_b.begin(), links[0]->links_b.end(), segment) != links[0]->links_b.end()) {
				segment->startOrder = links[0]->startOrder + 1;
			}
			else {
				segment->startOrder = links[0]->endOrder + 1;
			}
		}
		else {
			if (std::find(links[0]->links_b.begin(), links[0]->links_b.end(), segment) != links[0]->links_b.end()) {
				segment->endOrder = links[0]->startOrder + 1;
			}
			else {
				segment->endOrder = links[0]->endOrder + 1;
			}
		}
	}

	else if (links.size() == 2) {
		if (isStart) {
			// check the angle it has with the other two
			if (abs(fmod(links[0]->dir(), 180) - fmod(links[1]->dir(), 180)) < 20) {
				// if its in the backwards links of the other
				if (std::find(links[0]->links_b.begin(), links[0]->links_b.end(), segment) != links[0]->links_b.end()) {
					links[0]->startOrder = segment->startOrder + 1;
				}
				// if its in the front links of the other
				else {
					links[0]->endOrder = segment->startOrder + 1;
				}
				// if its in the backwards links of the other
				if (std::find(links[1]->links_b.begin(), links[1]->links_b.end(), segment) != links[1]->links_b.end()) {
					links[1]->startOrder = segment->startOrder + 1;
				}
				// if its in the front links of the other
				else {
					links[1]->endOrder = segment->startOrder + 1;
				}
			}
				
		}
		else {
			// check the angle it has with the other two
			if (abs(fmod(links[0]->dir(), 180) - fmod(links[1]->dir(), 180)) < 20) {
				// if its in the backwards links of the other
				if (std::find(links[0]->links_b.begin(), links[0]->links_b.end(), segment) != links[0]->links_b.end()) {
					links[0]->startOrder = segment->endOrder + 1;
				}
				// if its in the front links of the other
				else {
					links[0]->endOrder = segment->endOrder + 1;
				}
				// if its in the backwards links of the other
				if (std::find(links[1]->links_b.begin(), links[1]->links_b.end(), segment) != links[1]->links_b.end()) {
					links[1]->startOrder = segment->endOrder + 1;
				}
				// if its in the front links of the other
				else {
					links[1]->endOrder = segment->endOrder + 1;
				}
			}
		}
	}

	else if (links.size() >= 3) {
		std::vector<std::pair<Segment*, double>> segmentAngles;
		for (auto link : links) {
			double angle = abs(fmod(link->dir(), 180) - fmod(segment->dir(), 180));
			segmentAngles.push_back({ link, angle });
		}

		auto cmp = [](const std::pair<Segment*, double>& lhs, const std::pair<Segment*, double>& rhs) { return (lhs.second < rhs.second);  };
		std::sort(segmentAngles.begin(), segmentAngles.end(), cmp);

		int order = 0;
		if (std::find(segmentAngles[1].first->links_b.begin(), segmentAngles[1].first->links_b.end(), segment) != segmentAngles[1].first->links_b.end()) {
			order = segmentAngles[1].first->startOrder + 1;
		}
		else {
			order = segmentAngles[1].first->endOrder + 1;
		}

		if (isStart) {
			segment->startOrder = order;
		}
		else {
			segment->endOrder = order;
		}
		if (std::find(segmentAngles[0].first->links_b.begin(), segmentAngles[0].first->links_b.end(), segment) != segmentAngles[0].first->links_b.end()) {
			segmentAngles[0].first->startOrder = order;
		}
		else {
			segmentAngles[0].first->endOrder = order;
		}
		
	}
}

/* it checks roads and gives them a Z ordering based on forwards and backwards links */
void findOrderOfRoads(std::vector<Segment*>& segments)
{
	for (auto segment : segments) {
		findOrderAtEnd(segment, true);
		findOrderAtEnd(segment, false);
	}
}

/* remove intersections with the same position */
void removeDuplicateIntersections(std::vector<Intersection*>& intersections) {
	std::vector<Point> seenPositions;

	for (size_t i = 0; i < intersections.size(); ++i) {
		bool isDuplicate = false;
		for (size_t j = 0; j < seenPositions.size(); ++j) {
			if (intersections[i]->position == seenPositions[j]) {
				isDuplicate = true;
				break;
			}
		}

		if (!isDuplicate) {
			seenPositions.push_back(intersections[i]->position);
		}
		else {
			delete intersections[i]; // Don't forget to deallocate memory if necessary
			intersections.erase(intersections.begin() + i);
			--i; // Decrement to adjust for removed element
		}
	}
}

/* merge intersections that are closer than some value */
void mergeCloseIntersections(std::vector<Intersection*>& intersections) {
	
	// lambda function for comparing two intersections based on their distance to origin
	auto compareIntersectionsBasedOnDistanceToOrigin = [](const Intersection* inter1, const Intersection* inter2) {
		return (inter1->position.length() < inter2->position.length());
	};

	// sort the intersections based on their distance to origin
	std::sort(intersections.begin(), intersections.end(), compareIntersectionsBasedOnDistanceToOrigin);

	// iterate through the intersections
	for (auto it = intersections.begin(); it != intersections.end() - 1;) {
		auto thisIntersection = *it;
		auto nextIntersection = *(it + 1);
		/* if adjacent intersections are close than some distance */
		if ((thisIntersection->position - nextIntersection->position).length() < Config::DEFAULT_ROADPART_LENGTH) {
			/* their position shall be averaged */
			Point newPosition = (thisIntersection->position + nextIntersection->position) / 2;
			/* dont include duplicate segments when merging two intersections */
			std::unordered_set<int> uniqueIDs;
			std::vector<Segment*> uniqueSegments;
			/* for the first intersection just include all the branches */
			for (auto branch : thisIntersection->branches) {
				uniqueIDs.insert(branch->ID);
				uniqueSegments.push_back(branch);
			}
			/* for the second one, check with the ones already added, if not present then add */
			for (auto branch : nextIntersection->branches) {
				if (uniqueIDs.find(branch->ID) != uniqueIDs.end()) {
					uniqueIDs.insert(branch->ID);
					uniqueSegments.push_back(branch);
				}
			}

			Intersection* newIntersection = new Intersection(uniqueSegments, newPosition);
			*it = newIntersection;
			
			// now fix the startIntersectionID and endIntersectionID for all the segments inside
			for (auto segment : newIntersection->branches) {
				bool isStart = ((segment->start - newIntersection->position).length() < (segment->end - newIntersection->position).length());
				if (isStart)
					segment->startIntersectionID = newIntersection->ID;
				else
					segment->endIntersectionID = newIntersection->ID;
			}

			UE_LOG(LogTemp, Warning, TEXT("deleted something"));
			it = intersections.erase(it + 1);
		}
		else {
			++it;
		}
	}

}

/*  > this functions is used to cut a road off by speficic amount at specified end. 
	> used for intersctions														*/
void cutRoadFromSpecifiedEndBySpecifiedAmount(Segment* segment, bool isStart, double amount) {
	// check which vertex it is
	if (isStart) {
		// I'm just using the point class, its actually a vector
		// calculate direction to cut by
		Point direction = segment->end - segment->start;
		// normalize
		direction = { direction.x / sqrt(direction.x * direction.x + direction.y * direction.y),
					direction.y / sqrt(direction.x * direction.x + direction.y * direction.y) };
		// set the point
		segment->start = segment->start + direction * amount;
	}
	else {
		// calculate direction to cut by
		Point direction = segment->start - segment->end;
		// normalize
		direction = { direction.x / sqrt(direction.x * direction.x + direction.y * direction.y),
					direction.y / sqrt(direction.x * direction.x + direction.y * direction.y) };
		// multiply by amount
		Point directionMultipliedByAmount = direction * amount;
		// set the point
		segment->end = segment->end + direction * amount;
	}
	// (this is all in 2D)
}

void cutRoadsLeadingIntoIntersections(std::vector<Segment*>& segments, std::vector<Intersection*> intersections)
{
	// iterate over intersections
	for (auto intersection : intersections) {
		// iterate over segments inside the intersections
		for (auto intersectionSegment : intersection->branches) {
			// find the segment from the list of all segments
			int currentSegmentID = intersectionSegment->ID;
			for (auto it = segments.begin(); it != segments.end(); ++it) {
				if ((*it)->ID == currentSegmentID) {
					auto currentSegment = *it;
					// don't cut if smaller than specified amount
					bool canCut = currentSegment->length() > 1000;
					// we have to cut off currentSegment, NOT intersectionSegment. throw that away
					bool isStart = (currentSegment->start - intersection->position).length() < (currentSegment->end - intersection->position).length();
					if (isStart && canCut) {
						// prune start point by a specific amount
						// TODO: calculate this amount based on road width and angles
						cutRoadFromSpecifiedEndBySpecifiedAmount(currentSegment, true, 600);
					}
					else if (!isStart && canCut) {
						// prune end point by a specific amount
						// TODO: calculate this amount based on road width and angles
						cutRoadFromSpecifiedEndBySpecifiedAmount(currentSegment, false, 600);
					}
				}
			}
		}
	}
}




bool arePerpendicular(Segment* s1, Segment* s2)
{
	// Represent segments as vectors
	/*Point v1 = s1->end - s1->start;
	Point v2 = s2->end - s2->start;
	double dot = v1.x * v2.x + v1.y * v2.y;
	double mag_v1 = std::sqrt(v1.x * v1.x + v1.y * v1.y);
	double mag_v2 = std::sqrt(v2.x * v2.x + v2.y * v2.y);
	double cosineTheta = dot / (mag_v1 * mag_v2);
	return cosineTheta >= -0.866025 && cosineTheta <= 0.866025;*/

	return abs(fmod(s1->dir(), 180) - fmod(s2->dir(), 180)) > 30;


}

bool isClose(Point pos1, Point pos2)
{
	return (pos1 - pos2).length() < 0.0001;
}

struct PointHash {
	size_t operator()(const Point& p) const {
		return std::hash<double>()(p.x) ^ std::hash<double>()(p.y);
	}
};

void populateSegmentLinks(std::vector<Segment*>& segments) {
	// Map for storing segments based on their start or end points
	std::unordered_map<Point, std::vector<Segment*>, PointHash> segmentsMap;

	// Populate the map
	for (Segment* segment : segments) {
		segmentsMap[segment->start].push_back(segment);
		segmentsMap[segment->end].push_back(segment);
	}

	// Populate links for each segment
	for (Segment* segment : segments) {
		// Populate links_b for the segment's start point
		if (segmentsMap.find(segment->start) != segmentsMap.end()) {
			segment->links_b = segmentsMap[segment->start];
		}

		// Populate links_f for the segment's end point
		if (segmentsMap.find(segment->end) != segmentsMap.end()) {
			segment->links_f = segmentsMap[segment->end];
		}

		// Removing current segment from its own links_b and links_f
		segment->links_b.erase(std::remove(segment->links_b.begin(), segment->links_b.end(), segment), segment->links_b.end());
		segment->links_f.erase(std::remove(segment->links_f.begin(), segment->links_f.end(), segment), segment->links_f.end());
	}
}

void GenerateIntersections(std::vector<Segment*>& segments, std::vector<Intersection*>& intersections)
{
	std::unordered_set<Point> seenPoints{};

	for (auto segment : segments) {

		auto func = [&segment, &seenPoints, &intersections](Point pos, std::vector<Segment*> links, bool isStart) {
			if (seenPoints.find(pos) == seenPoints.end()) {
				links.push_back(segment);
				Intersection* intersection = new Intersection(links, pos);
				intersections.push_back(intersection);
				seenPoints.insert(pos);
			}
		};
		
		func(segment->start, segment->links_b, true);
		func(segment->end, segment->links_f, false);

	}
}
