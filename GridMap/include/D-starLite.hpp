#ifndef DSTARLITE_HEADER_HPP
#define DSTARLITE_HEADER_HPP

#include <queue>
#include <set>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>

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

struct PositionComparator
{
	bool operator()(const Position& lhs, const Position& rhs) const
	{
		if (lhs.x == rhs.x)
			return lhs.y > rhs.y;
		return lhs.x > rhs.x;
	}
};

class DStarLite
{
private:
    std::vector<std::vector<CellType>> gridMap;
    Position startCell, targetCell;
    int rows, cols;

    std::map<Position, float, PositionComparator> g, rhs;
    std::map<Position, Key, PositionComparator> keys;
    std::set<std::pair<Key, Position>, OpenListComparator> openList;
    float km;

    std::vector<Position> path;
    Position lastStart;

    std::set<Position, PositionComparator> visitedNodes;

    float heuristic(const Position& a, const Position& b);

    Key calculateKey(const Position& u);

    void updateVertex(const Position& u);

    

    bool isValidCell(const Position& pos) const;

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
