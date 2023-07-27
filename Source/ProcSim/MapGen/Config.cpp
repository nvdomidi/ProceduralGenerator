#include "Config.h"
#include "Math.h"

const double Config::DEFAULT_SEGMENT_LENGTH = 300;
const double Config::HIGHWAY_SEGMENT_LENGTH = 400;
const double Config::DEFAULT_SEGMENT_WIDTH = 6;
const double Config::HIGHWAY_SEGMENT_WIDTH = 16;
const double Config::DEFAULT_BRANCH_PROBABILITY = 0.4;
const double Config::HIGHWAY_BRANCH_PROBABILITY = 0.02;
const double Config::HIGHWAY_BRANCH_POPULATION_THRESHOLD = 0.1;
const double Config::NORMAL_BRANCH_POPULATION_THRESHOLD = 0.1;
const int Config::NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY = 10;
const int Config::HIGHWAY_POPULATION_SAMPLE_SIZE = 1;
const int Config::MINIMUM_INTERSECTION_DEVIATION = 30;
const int Config::SEGMENT_COUNT_LIMIT = 20000;
const int Config::ROAD_SNAP_DISTANCE = 50;
const Config::QuadTreeParams Config::QUADTREE_PARAMS = { -2e4, -2e4, 4e4, 4e4 };
const int Config::QUADTREE_MAX_OBJECTS = 10;
const int Config::QUADTREE_MAX_LEVELS = 10;
const bool Config::ONLY_HIGHWAYS = false;
const bool Config::IGNORE_CONFLICTS = false;
const int Config::DELAY_BETWEEN_TIME_STEPS = 0;
const bool Config::TWO_SEGMENTS_INITIALLY = true;
const bool Config::START_WITH_NORMAL_STREETS = false;
double Config::minx = -20000;
double Config::miny = -20000;
double Config::maxx = 20000;
double Config::maxy = 20000;
bool Config::COMPLETELYRANDOM = true;
ESTRAIGHTNESS Config::STRAIGHTNESS = ESTRAIGHTNESS::SE_STRAIGHT;

double Config::RANDOM_BRANCH_ANGLE() {
    switch (Config::STRAIGHTNESS) {
    case ESTRAIGHTNESS::SE_CURVED:
        return Math::randomNearCubic(5);
    case ESTRAIGHTNESS::SE_STRAIGHT:
        return Math::randomNearCubic(3);
    case ESTRAIGHTNESS::SE_VERYSTRAIGHT:
        return Math::randomNearCubic(1);
    default:
        return Math::randomNearCubic(3);
    }
}

double Config::RANDOM_STRAIGHT_ANGLE() {
    switch (Config::STRAIGHTNESS) {
    case ESTRAIGHTNESS::SE_CURVED:
        return Math::randomNearCubic(20);
    case ESTRAIGHTNESS::SE_STRAIGHT:
        return Math::randomNearCubic(15);
    case ESTRAIGHTNESS::SE_VERYSTRAIGHT:
        return Math::randomNearCubic(3);
    default:
        return Math::randomNearCubic(15);
    }
}

const char* Config::IMGPATH = "";
//const char* Config::IMGPATH = "C:\\Users\\Navid\\Desktop\\map.png";

const bool Config::COLORED = true;