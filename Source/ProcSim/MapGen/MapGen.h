#pragma once

#include <iostream>
#include <vector>
#include <functional>
#include <algorithm>
#include <climits>
#include <memory>

#include "Config.h"
#include "Quadtree.h"
#include "Math.h"
#include "SimplexNoise.h"
#include <random>

struct MetaInfo {
	bool highway;
	int color;
	bool severed;
};

class Segment {

public:
	/** time-step delay before this road is evaluated */
	double t;
	/** meta-information relevant to global goals */
	MetaInfo q;
	/**
	* links backwards and forwards:
	* each segment has a direction given by how the road network grows
	*
	* the backwards links are which segments merge with this road segment at it's this.start point,
	* the forwards links are which segments split off at the end point
	*/
	std::vector<Segment*> links_b {};
	std::vector<Segment*> links_f {};
	double width;
	std::function<void()> setupBranchLinks = nullptr;
	Point start;
	Point end;

	Segment* prev;

	Segment(Point start, Point end, double t = 0, MetaInfo q = {}) : start(start), end(end), t(t), q(q) {
		width = q.highway ? Config::HIGHWAY_SEGMENT_WIDTH : Config::DEFAULT_SEGMENT_WIDTH;
	}

	double dir() {
		double dx = end.x - start.x;
		double dy = end.y - start.y;
		return -1 * Math::sign(Math::crossProduct({ 0, 1 }, { dx, dy })) * Math::angleBetween({ 0, 1 }, { dx, dy });
	}

	double length() {
		return sqrt(pow(end.x - start.x, 2) + pow(end.y - start.y, 2));
	}

	Bounds limits() {
		double minX = std::min(start.x, end.x);
		double minY = std::min(start.y, end.y);
		double w = std::abs(start.x - end.x);
		double h = std::abs(start.y - end.y);
		return { minX, minY, w, h };
	}

	void debugLinks() {
		q.color = 0x00ff00;
		for (auto link : links_b)
			link->q.color = 0xff0000;
		for (auto link : links_f)
			link->q.color = 0x0000ff;
	}

	// check if the start of this segment is in the backwards links or forward links
	// should pretty much always be true (todo: is it ever not?)
	bool startIsBackwards() {
		if (!links_b.empty()) {
			bool ba = Math::equalV(links_b[0]->start, start) || Math::equalV(links_b[0]->end, start);
			if (!ba)
				std::cout << "warning: backward " << ba << std::endl;
			return ba;
		}
		else {
			if (links_f.empty())
				throw std::runtime_error("No backwards or forward links: impossible");

			return Math::equalV(links_f[0]->start, end) || Math::equalV(links_f[0]->end, end);
		}
	}

	std::vector<Segment*> linksForEndContaining(Segment* segment, bool& front) {
		auto it = std::find(links_b.begin(), links_b.end(), segment);
		if (it != links_b.end())
		{
			front = false;
			return links_b;
		}
		it = std::find(links_f.begin(), links_f.end(), segment);
		if (it != links_f.end())
		{
			front = true;
			return links_f;
		}

		//std::cout << "segment not found in linksForEndContaining function" << std::endl;
		return {};
	}

	/**
   * split this segment into two segments, connecting the given segment to the newly created crossing
   *
   * left example in https://phiresky.github.io/procedural-cities/img/20151213214559.png
   *
   * @param point the coordinates the split will be at
   * @param thirdSegment the third segment that will be joined to the newly created crossing
   * @param segmentList the full list of all segments (new segment will be added here)
   * @param qTree quadtree for faster finding of segments (new segment will be added here)
   */
	void split(Point point, Segment* thirdSegment, std::vector<Segment*>& segmentList, Quadtree<Segment*>& qTree) {
		Segment* splitPart = clone();
		bool startIsBackwards = this->startIsBackwards();
		segmentList.push_back(splitPart);
		Bounds pRect = splitPart->limits();
		qTree.insert(pRect, splitPart);
		splitPart->end = point;
		start = point;
		splitPart->links_b = links_b;
		splitPart->links_f = links_f;

		Segment* firstSplit;
		std::vector<Segment*> fixLinks;
		Segment* secondSplit;

		if (startIsBackwards) {
			firstSplit = splitPart;
			secondSplit = this;
			fixLinks = splitPart->links_b;
		}
		else {
			firstSplit = this;
			secondSplit = splitPart;
			fixLinks = splitPart->links_f;
		}
		for (auto link : fixLinks) {
			auto it = std::find(link->links_b.begin(), link->links_b.end(), this);


			if (it != link->links_b.end()) {
				*it = splitPart;
			}
			else {
				it = std::find(link->links_f.begin(), link->links_f.end(), this);
				if (it == link->links_f.end())
				{
					throw std::runtime_error("impossible, link is either backwards or forwards");
				}

				*it = splitPart;
			}


		}
		firstSplit->links_f = { thirdSegment, secondSplit };
		secondSplit->links_b = { thirdSegment, firstSplit };
		thirdSegment->links_f.push_back(firstSplit);
		thirdSegment->links_f.push_back(secondSplit);
	}

	Segment* clone() {
		return new Segment(start, end, t, q);
	}

	static Segment* usingDirection(const Point& start, double t, const MetaInfo& q = {}, double dir = 90, double length = Config::DEFAULT_SEGMENT_LENGTH) {
		Point end = {
			start.x + length * std::sin((dir * M_PI) / 180),
			start.y + length * std::cos((dir * M_PI) / 180)
		};
		return new Segment(start, end, t, q);
	}

	LineSegmentIntersection* intersectWith(const Segment* s) {
		return Math::doLineSegmentsIntersect(start, end, s->start, s->end, true);
	}

};


class Heatmap {
public:

	double offset_x1, offset_x2, offset_y1, offset_y2;
	bool completely_random;
	
	std::unique_ptr<unsigned char[]> image;
	int width{};
	int height{};

	// Copy constructor
	Heatmap(const Heatmap& other) : width(other.width), height(other.height), offset_x1(other.offset_x1),
		offset_x2(other.offset_x2), offset_y1(other.offset_y1),
		offset_y2(other.offset_y2), completely_random(other.completely_random) {
		image = std::make_unique<unsigned char[]>(width * height);
		std::copy(other.image.get(), other.image.get() + (width * height), image.get());
	}



	Heatmap(int width, int height) : width(width), height(height) {
		image = std::make_unique<unsigned char[]>(width * height);
		std::random_device rd;
		std::mt19937 generator(rd());
		std::uniform_real_distribution<double> distribution(0.0, 300.0);
		std::uniform_real_distribution<double> distribution2(50.0, 300.0);


		double denominator = distribution2(generator);

		if (Config::COMPLETELYRANDOM)
			denominator /= 10;

		offset_x1 = distribution(generator);
		offset_x2 = distribution(generator);
		offset_y1 = distribution(generator);
		offset_y2 = distribution(generator);



		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {

				double value1 = (SimplexNoise::noise(x / denominator, y / denominator) + 1) / 2.0;
				double value2 = (SimplexNoise::noise(x / (denominator * 2) + offset_x1, y / (denominator * 2) + offset_y1) + 1) / 2.0;
				double value3 = (SimplexNoise::noise(x / (denominator * 2) + offset_x2, y / (denominator * 2) + offset_y2) + 1) / 2.0;

				this->image[y * width + x] = static_cast<int>(255 * pow((value1 * value2 + value3) / 2, 2));
			}
		}
	}

	Heatmap(unsigned char* loadedImage, int width, int height) : width(width), height(height) {
		image = std::unique_ptr<unsigned char[]>(loadedImage);
	}

	std::unique_ptr<unsigned char[]> getImage() {
		return std::move(image);
	}

	void setImage(std::unique_ptr<unsigned char[]> img) {
		image = std::move(img);
	}

	double popOnRoad(const Segment& r) {
		return (populationAt(r.start.x, r.start.y) + populationAt(r.end.x, r.end.y)) / 2.0;
	}

	double populationAt(double x, double y) {
		// To generate title page of the presentation: if(x < 7000 && y < 3500 && x > -7000 && y > 2000) return 0; else if(1) return Math.random()/4+config.NORMAL_BRANCH_POPULATION_THRESHOLD;


		if (x < Config::minx || x > Config::maxx || y < Config::miny || y > Config::maxy)
			return 0.0;


		//int newx = static_cast<int>((x - Config::minx) * (this->width / (Config::maxx - Config::minx)));
		
		int newx = static_cast<int>((Config::maxx - x) * (this->width / (Config::maxx - Config::minx)));

		int newy = static_cast<int>((Config::maxy - y) * (this->height / (Config::maxy - Config::miny)));
		int idx = newy * width + newx;

		/*UE_LOG(LogTemp, Warning, TEXT("We want to see population at: %f, %f"), x, y);
		UE_LOG(LogTemp, Warning, TEXT("the image coords newx, newy: %d, %d"), newx, newy);
		UE_LOG(LogTemp, Warning, TEXT("The index: %d"), idx);
		UE_LOG(LogTemp, Warning, TEXT("The image size: %d"), (this->width * this->height));*/

		auto aa = this->image[idx];
		return static_cast<double>(aa) / 255.0;


	}
};

struct DebugData {
	std::vector<Point> snaps;
	std::vector<Point> intersectionsRadius;
	std::vector<LineSegmentIntersection> intersections;
};

struct Action {
	int priority;
	std::function<bool()> func = nullptr;
	double t;
};

template<typename T>
class PriorityQueue {
public:
	std::vector<T> elements;
	std::function<int(T)> getPriority;

	PriorityQueue(std::function<int(T)> priorityFunc) : getPriority(priorityFunc) {}

	void enqueue(const T item) {
		elements.push_back(item);
	}

	T dequeue() {
		int minT = INT_MAX;
		int minT_i = 0;

		for (size_t i = 0; i < elements.size(); ++i) {
			int t = getPriority(elements[i]);
			if (t < minT) {
				minT = t;
				minT_i = i;
			}
		}

		T minValue = elements[minT_i];
		elements.erase(elements.begin() + minT_i);
		return minValue;
	}

	bool empty() const {
		return elements.empty();
	}
};

struct GeneratorResult {
	std::vector<Segment> segments;
	std::vector<Segment> priorityQ;
	Quadtree<Segment> qTree;
};

bool localConstraints(Segment* segment, std::vector<Segment*>& segments, Quadtree<Segment*>& qTree);
std::vector<Segment*> globalGoalsGenerate(Segment* previousSegment, Heatmap heatmap);
std::vector<Segment*> makeInitialSegments();

void generationStep(
	PriorityQueue<Segment*>& priorityQ,
	std::vector<Segment*>& segments,
	Quadtree<Segment*>& qTree,
	Heatmap heatmap
);
