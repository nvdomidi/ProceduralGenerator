#include "MapGen.h"


bool localConstraints(Segment* segment, std::vector<Segment*>& segments, Quadtree<Segment*>& qTree) {
	if (Config::IGNORE_CONFLICTS) return true;

	Action action = { 0, nullptr, 0.0 };
	DebugData* debugData = new DebugData();

	for (auto other : qTree.retrieve(segment->limits())) {

		// Intersection check
		if (action.priority <= 4) {

			LineSegmentIntersection* intersection = segment->intersectWith(other);


			if (intersection != nullptr) {
				if (action.func == nullptr || intersection->t < action.t) {
					action.t = intersection->t;
					action.priority = 4;

					action.func = [other, segment, &segments, &qTree, intersection, &debugData]() -> bool {



						// If intersecting lines are too similar, don't continue
						if (Math::minDegreeDifference(other->dir(), segment->dir()) < Config::MINIMUM_INTERSECTION_DEVIATION) {
							return false;
						}
						Point intersectionPoint{ intersection->x, intersection->y };
						other->split(intersectionPoint, segment, segments, qTree);
						segment->end = Point{ intersection->x, intersection->y };
						segment->q.severed = true;
						(*debugData).intersections.push_back(*intersection);

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

				action.func = [point, other, segment, &segments, &qTree, &debugData]() {
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
					(*debugData).snaps.push_back({ point.x, point.y });


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

				action.func = [segment, point, other, &segments, &qTree, &debugData]() {
					segment->end = point;
					segment->q.severed = true;
					// if intersecting lines are too closely aligned don't continue
					if (Math::minDegreeDifference(other->dir(), segment->dir()) < Config::MINIMUM_INTERSECTION_DEVIATION) {
						return false;
					}

					other->split(point, segment, segments, qTree);
					(*debugData).intersectionsRadius.push_back(Point{ point.x, point.y });
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
		auto templateFunc = [=](double direction, double length, double t, MetaInfo* q = nullptr) {
			return Segment::usingDirection(previousSegment->end, t, *q, previousSegment->dir() + direction, length);
		};

		auto templateContinue = [=](double direction) {
			return templateFunc(direction, previousSegment->length(), 0.0, &previousSegment->q);
		};

		auto templateBranch = [=](double direction) {
			return templateFunc(direction, Config::DEFAULT_SEGMENT_LENGTH, previousSegment->q.highway ? Config::NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY : 0, &previousSegment->q);
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
	Heatmap heatmap
) {
	Segment* minSegment = priorityQ.dequeue();

	if (minSegment == nullptr) throw std::runtime_error("no segment remaining");
	bool accepted = localConstraints(minSegment, segments, qTree);

	if (accepted) {

		if (minSegment->setupBranchLinks != nullptr) {
			minSegment->setupBranchLinks();
		};
		segments.push_back(minSegment);
		Bounds minSegmentLimits = minSegment->limits();
		qTree.insert(minSegmentLimits, minSegment);
		std::vector<Segment*> newSegments = globalGoalsGenerate(minSegment, heatmap);
		for (auto newSegment : newSegments) {
			newSegment->t = minSegment->t + 1 + newSegment->t;
			priorityQ.enqueue(newSegment);
		}
	}
}