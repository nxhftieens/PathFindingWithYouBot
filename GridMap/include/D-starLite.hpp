#ifndef DSTARLITE_HEADER_HPP
#define DSTARLITE_HEADER_HPP

#include <queue>
#include <set>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <limits>

#include "../include/Element.hpp"

const float INF = std::numeric_limits<float>::infinity();

struct Key
{
    float k1, k2;
    Key(float k1 = std::numeric_limits<float>::infinity(),
    float k2 = std::numeric_limits<float>::infinity())
    : k1(k1), k2(k2) {}

    bool operator==(const Key& other) const
    {
        return k1 == other.k1 && k2 == other.k2;
    }

    bool operator<(const Key& other) const
    {
        if (k1 == other.k1)
            return k2 > other.k2;
        return k1 > other.k1;
    }

    bool operator>(const Key& other) const
    {
        if (k1 == other.k1)
            return k2 < other.k2;
        return k1 < other.k1;
    }
};

struct NodeDS 
{
	Position pos;
	float g;
	float rhs;
	float h;
	Key key;
	std::vector<NodeDS*> successors;
	std::vector<NodeDS*> predecessors;
	NodeDS(const Position& pos, float g, float rhs)
		: pos(pos), g(g), rhs(rhs), successors(), predecessors()
	{
	}
};

struct NodeDSComparator
{
    bool operator()(const NodeDS* lhs, const NodeDS* rhs) const
    {
        if (lhs->key < rhs->key)
            return false;
        if (rhs->key < lhs->key)
            return true;
        if (lhs->pos.x != rhs->pos.x)
            return lhs->pos.x > rhs->pos.x;
        return lhs->pos.y > rhs->pos.y;
    }
};

struct PositionComparator
{
    bool operator()(const Position& lhs, const Position& rhs) const
    {
        if (lhs.x == rhs.x)
            return lhs.y > rhs.y;
        return lhs.x > rhs.x;
    }
};

struct OpenListComparator
{
    bool operator()(const std::pair<Key, Position>& lhs, const std::pair<Key, Position>& rhs) const
    {
        if (rhs.first < lhs.first)
            return true;
        if (lhs.first < rhs.first)
            return false;
        if (lhs.second.x != rhs.second.x)
            return lhs.second.x > rhs.second.x;
        return lhs.second.y > rhs.second.y;
    }
};

class DStarLite
{
private:
    std::vector<std::vector<CellType>> gridMap;
    Position startCell, targetCell;
    int rows, cols;
	std::map<Position, NodeDS*, PositionComparator> allNodeDSs;
	std::set<NodeDS*, NodeDSComparator> openList;

    float km;

    std::vector<Position> path;
    Position lastStart;    

    float heuristic(const Position& current, const Position& startCell);

    Key calculateKey(NodeDS* u);

    void updateVertex(NodeDS* u);    

    bool isValidCell(const Position& pos) const;

	float costFunction(NodeDS* u, NodeDS* v);

public:
    void computeShortestPath();

    DStarLite(const std::vector<std::vector<CellType>>& grid,
        const Position& startCell,
        const Position& targetCell);
    
    std::vector<Position> getPath();

    void initialize(const std::vector<std::vector<CellType>>& grid,
        const Position& start,
        const Position& target);

	void updateObstacle(const Position& obstaclePos, const Position& newStart);
};
#endif // !DSTARLITE_HEADER_HPP
