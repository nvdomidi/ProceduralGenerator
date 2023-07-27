#pragma once

#include <vector>

#include "Math.h"

struct Bounds {
    double x;
    double y;
    double width;
    double height;
};

template<typename T>
class Quadtree {
private:
    struct Node {
        enum class Type { Leaf, Inner };
        Type type;
        std::vector<Bounds> objects;
        std::vector<T> objectsO;
        Quadtree<T>* topLeft;
        Quadtree<T>* topRight;
        Quadtree<T>* bottomLeft;
        Quadtree<T>* bottomRight;
    };

    Node node;
    Bounds bounds;
    int max_objects;
    int max_levels;
    int level;

public:
    Quadtree(const Bounds bounds, int max_objects = 10, int max_levels = 4, int level = 0)
        : bounds(bounds), max_objects(max_objects), max_levels(max_levels), level(level) {
        node.type = Node::Type::Leaf;
        node.objects = std::vector<Bounds>();
        node.objectsO = std::vector<T>();
    }

    void split() {
        int lvl = this->level + 1;
        double width = bounds.width / 2;
        double height = bounds.height / 2;
        double x = bounds.x;
        double y = bounds.y;

        if (node.type != Node::Type::Leaf)
            throw std::runtime_error("Can't split non-leaf");

        node.type = Node::Type::Inner;
        node.topRight = new Quadtree<T>(Bounds{ x + width, y, width, height }, max_objects, max_levels, lvl);
        node.topLeft = new Quadtree<T>(Bounds{ x, y, width, height }, max_objects, max_levels, lvl);
        node.bottomLeft = new Quadtree<T>(Bounds{ x, y + height, width, height }, max_objects, max_levels, lvl);
        node.bottomRight = new Quadtree<T>(Bounds{ x + width, y + height, width, height }, max_objects, max_levels, lvl);

        for (size_t i = 0; i < node.objects.size(); i++) {
            const Bounds rect = node.objects[i];
            const T obj = node.objectsO[i];
            //if (rect == nullptr)
            //    throw std::runtime_error("Impossible");

            for (auto subnode : getRelevantNodes(rect)) {
                subnode->insert(rect, obj);
            }
        }

        node.objects.clear();
        node.objectsO.clear();
    }

    std::vector<Quadtree<T>*> getRelevantNodes(const Bounds r) {
        if (node.type == Node::Type::Leaf)
            throw std::runtime_error("Can't get relevant nodes in leaf");

        double midX = bounds.x + bounds.width / 2;
        double midY = bounds.y + bounds.height / 2;

        std::vector<Quadtree<T>*> qs;

        bool isTop = r.y <= midY;
        bool isBottom = r.y + r.height > midY;

        if (r.x <= midX) {
            // left
            if (isTop) qs.push_back(node.topLeft);
            if (isBottom) qs.push_back(node.bottomLeft);
        }
        if (r.x + r.width > midX) {
            // right
            if (isTop) qs.push_back(node.topRight);
            if (isBottom) qs.push_back(node.bottomRight);
        }

        return qs;
    }

    void insert(const Bounds pRect, const T obj) {
        if (node.type != Node::Type::Leaf) {
            for (Quadtree<T>* subnode : getRelevantNodes(pRect)) {
                subnode->insert(pRect, obj);
            }
            return;
        }

        node.objects.push_back(pRect);
        node.objectsO.push_back(obj);

        if (node.objects.size() > max_objects && level < max_levels) {
            split();
        }
    }

    std::vector<T> retrieve(const Bounds pRect) {
        if (node.type == Node::Type::Leaf) {
            return node.objectsO;
        }

        std::vector<T> relevant;

        for (auto subnode : getRelevantNodes(pRect)) {
            auto subnodeObjects = subnode->retrieve(pRect);
            relevant.insert(relevant.end(), subnodeObjects.begin(), subnodeObjects.end());
        }

        return relevant;
    }

};