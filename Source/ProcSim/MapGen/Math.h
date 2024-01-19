#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>

# define M_PI           3.14159265358979323846

class Point {
public:

    Point() = default;

    Point(const Point&) = default;

    Point& operator=(const Point&) = default;

    Point(double x, double y) : x(x), y(y) {}

    Point operator*(const double scalar) {
        return { scalar * x, scalar * y };
    }

    Point operator/(const double scalar) {
        return { x / scalar, y / scalar };
    }


    Point operator+(const Point& other) {
        return {x+other.x, y+other.y};
    }

    Point operator-(const Point& other) {
        return { x - other.x, y - other.y };
    }

    bool operator==(const Point& other) const {
        return (x == other.x && y == other.y);
    }
    
    double length() const {
        return sqrt(pow(x, 2) + pow(y, 2));
    }

    double dot(const Point& other) {
        return x * other.x + y * other.y;
    }


    double x;
    double y;
};

namespace std {
    template<>
    struct hash<Point> {
        std::size_t operator()(const Point& p) const noexcept {
            // Hash combine idiom
            std::size_t h1 = std::hash<double>{}(p.x);
            std::size_t h2 = std::hash<double>{}(p.y);
            return h1 ^ (h2 << 1);
        }
    };
}

// Function to check if three points are collinear
inline bool areCollinear(Point p1, Point p2, Point p3) {
    int area = p1.x * (p2.y - p3.y) + p2.x * (p3.y - p1.y) + p3.x * (p1.y - p2.y);
    return area == 0;
}

// Function to check if four points are collinear
inline bool areFourPointsCollinear(Point p1, Point p2, Point p3, Point p4) {
    // Check for every combination of three points
    return areCollinear(p1, p2, p3) &&
        areCollinear(p1, p2, p4) &&
        areCollinear(p1, p3, p4) &&
        areCollinear(p2, p3, p4);
}


struct LineSegmentIntersection {
    double x;
    double y;
    double t;
};

class Math {
public:
    static Point subtractPoints(const Point& p1, const Point& p2) {
        return { p1.x - p2.x, p1.y - p2.y };
    }

    static double crossProduct(const Point& a, const Point& b) {
        return a.x * b.y - a.y * b.x;
    }

    static LineSegmentIntersection* doLineSegmentsIntersect(const Point& a, const Point& b, const Point& p, const Point& d, bool c) {
        Point bb = subtractPoints(b, a);
        Point dd = subtractPoints(d, p);
        double f = crossProduct(subtractPoints(p, a), bb);
        double k = crossProduct(bb, dd);
        if ((0 == f && 0 == k) || 0 == k) {
            return nullptr;
        }
        f /= k;
        double e = crossProduct(subtractPoints(p, a), dd) / k;
        bool intersect = c
            ? (0.001 < e && 0.999 > e && 0.001 < f && 0.999 > f)
            : (0 <= e && 1 >= e && 0 <= f && 1 >= f);
        if (intersect) {
            LineSegmentIntersection* result = new LineSegmentIntersection();
            result->x = a.x + e * bb.x;
            result->y = a.y + e * bb.y;
            result->t = e;
            return result;
        }
        return nullptr;
    }

    static double minDegreeDifference(double val1, double val2) {
        double bottom = std::fmod(std::abs(val1 - val2), 180);
        return std::min(bottom, std::abs(bottom - 180));
    }

    static bool equalV(const Point& a, const Point& b) {
        Point e = subtractPoints(a, b);
        return 1e-8 > lengthV2(e);
    }

    static double dotProduct(const Point& a, const Point& b) {
        return a.x * b.x + a.y * b.y;
    }

    static double length(const Point& a, const Point& b) {
        return lengthV(subtractPoints(b, a));
    }

    static double length2(const Point& a, const Point& b) {
        return lengthV2(subtractPoints(b, a));
    }

    static double lengthV(const Point& a) {
        return std::sqrt(lengthV2(a));
    }

    static double lengthV2(const Point& a) {
        return a.x * a.x + a.y * a.y;
    }

    static double angleBetween(const Point& a, const Point& b) {
        double angleRad = std::acos((a.x * b.x + a.y * b.y) / (lengthV(a) * lengthV(b)));
        return (180 * angleRad) / M_PI;

    }

    static int sign(double a) {
        return (0 < a) ? 1 : ((0 > a) ? -1 : 0);
    }

    static Point fractionBetween(const Point& a, const Point& b, double e) {
        Point bb = subtractPoints(b, a);
        return { a.x + bb.x * e, a.y + bb.y * e };
    }

    static double randomRange(double a, double b) {
        return std::rand() / (RAND_MAX + 1.0) * (b - a) + a;
    }

    static Point addPoints(const Point& a, const Point& b) {
        return { a.x + b.x, a.y + b.y };
    }

    static struct DistanceToLineResult {
        double distance2;
        Point pointOnLine;
        double lineProj2;
        double length2;
    } distanceToLine(const Point& a, const Point& b, const Point& e) {
        Point d = subtractPoints(a, b);
        Point ee = subtractPoints(e, b);
        struct DistanceToLineResult result;
        result.distance2 = length2(addPoints(b, project(d, ee).projected), a);
        result.pointOnLine = addPoints(b, project(d, ee).projected);
        result.lineProj2 = sign(project(d, ee).dotProduct) * lengthV2(project(d, ee).projected);
        result.length2 = lengthV2(ee);
        return result;
    }
    static struct ProjectResult {
        double dotProduct;
        Point projected;
    } project(const Point& a, const Point& b) {
        double e = dotProduct(a, b);
        struct ProjectResult result;
        result.dotProduct = e;
        result.projected = multVScalar(b, e / lengthV2(b));
        return result;
    }

    static Point multVScalar(const Point& a, double b) {
        return { a.x * b, a.y * b };
    }

    static Point divVScalar(const Point& a, double b) {
        return { a.x / b, a.y / b };
    }

    static double lerp(double a, double b, double x) {
        return a * (1 - x) + b * x;
    }

    static Point lerpV(const Point& a, const Point& b, double x) {
        return { lerp(a.x, b.x, x), lerp(a.y, b.y, x) };
    }

    static double randomNearCubic(double b) {
        double d = std::pow(std::abs(b), 3);
        double c = 0;
        while (c == 0 || std::rand() / (RAND_MAX + 1.0) < std::pow(std::abs(c), 3) / d) {
            c = randomRange(-b, b);
        }
        return c;
    }

};