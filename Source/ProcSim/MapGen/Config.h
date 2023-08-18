#pragma once
#include "ProcSim/Utils/RoadData.h"

class Config {
public:
    static const float DEFAULT_SEGMENT_LENGTH;
    static const float HIGHWAY_SEGMENT_LENGTH;
    static const float DEFAULT_SEGMENT_WIDTH;
    static const float HIGHWAY_SEGMENT_WIDTH;
    static float DEFAULT_ROADPART_LENGTH;
    static const float DEFAULT_HEIGHT;

    /** branch with 90° angle + randomness */
    static float RANDOM_BRANCH_ANGLE();
    /** sample possible highway continuation with this angle */
    static float RANDOM_STRAIGHT_ANGLE();
    /** probability of branching normal streets (including from highways) */
    static const float DEFAULT_BRANCH_PROBABILITY;
    /** probability of branching from highways */
    static const float HIGHWAY_BRANCH_PROBABILITY;
    /** minimum population to allow branching */
    static const float HIGHWAY_BRANCH_POPULATION_THRESHOLD;
    static const float NORMAL_BRANCH_POPULATION_THRESHOLD;
    /** time steps every normal street that branches from a highway is delayed */
    static const int NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY;
    /** number of possible new segments to search for maximum population */
    static const int HIGHWAY_POPULATION_SAMPLE_SIZE;
    /** ignore intersections with less than this degrees angle */
    static const int MINIMUM_INTERSECTION_DEVIATION;
    /** stop generation after x steps */
    static int SEGMENT_COUNT_LIMIT;
    /** maximum distance to connect roads */
    static const int ROAD_SNAP_DISTANCE;

    static const struct QuadTreeParams { float x4; float y; float width; float height; } QUADTREE_PARAMS;
    static const int QUADTREE_MAX_OBJECTS;
    static const int QUADTREE_MAX_LEVELS;
    /////////////////////////////////////static const bool DEBUG;
    /** disallow branching of normal streets from highways */
    static const bool ONLY_HIGHWAYS;
    /** disable collision checks (ignore local constraints evaluation) */
    static const bool IGNORE_CONFLICTS;
    /** delay when finding new time step in priority queue */
    static const int DELAY_BETWEEN_TIME_STEPS;
    /** grow to the right and left initially */
    static const bool TWO_SEGMENTS_INITIALLY;
    /** instead of starting with highways */
    static const bool START_WITH_NORMAL_STREETS;
    /* mapsize */
    static float minx;
    static float miny;
    static float maxx;
    static float maxy;

    /* Code Configurations added by Navid*/
    /* if true, the heatmap will be very noisy and random*/
    static bool COMPLETELYRANDOM;
    /* determines how curved we want our roads to be*/
    static ESTRAIGHTNESS STRAIGHTNESS;
    /* path to input image, no image is used if empty*/
    static const char* IMGPATH;
    /* show segments colored */
    static const bool COLORED;


};