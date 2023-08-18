#include "Config.h"
#include "Math.h"

const float Config::DEFAULT_SEGMENT_LENGTH = 300;
const float Config::HIGHWAY_SEGMENT_LENGTH = 400;
const float Config::DEFAULT_SEGMENT_WIDTH = 6;
const float Config::HIGHWAY_SEGMENT_WIDTH = 16;
const float Config::DEFAULT_HEIGHT = 0.4;
float Config::DEFAULT_ROADPART_LENGTH = 22.86;
const float Config::DEFAULT_BRANCH_PROBABILITY = 0.4;
const float Config::HIGHWAY_BRANCH_PROBABILITY = 0.02;
const float Config::HIGHWAY_BRANCH_POPULATION_THRESHOLD = 0.1;
const float Config::NORMAL_BRANCH_POPULATION_THRESHOLD = 0.1;
const int Config::NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY = 10;
const int Config::HIGHWAY_POPULATION_SAMPLE_SIZE = 1;
const int Config::MINIMUM_INTERSECTION_DEVIATION = 30;
int Config::SEGMENT_COUNT_LIMIT = 20000;
const int Config::ROAD_SNAP_DISTANCE = 50;
const Config::QuadTreeParams Config::QUADTREE_PARAMS = { -2e4, -2e4, 4e4, 4e4 };
const int Config::QUADTREE_MAX_OBJECTS = 10;
const int Config::QUADTREE_MAX_LEVELS = 10;
const bool Config::ONLY_HIGHWAYS = false;
const bool Config::IGNORE_CONFLICTS = false;
const int Config::DELAY_BETWEEN_TIME_STEPS = 0;
const bool Config::TWO_SEGMENTS_INITIALLY = true;
const bool Config::START_WITH_NORMAL_STREETS = false;
float Config::minx = -20000;
float Config::miny = -20000;
float Config::maxx = 20000;
float Config::maxy = 20000;
bool Config::COMPLETELYRANDOM = true;
ESTRAIGHTNESS Config::STRAIGHTNESS = ESTRAIGHTNESS::SE_STRAIGHT;

float Config::RANDOM_BRANCH_ANGLE() {
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

float Config::RANDOM_STRAIGHT_ANGLE() {
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