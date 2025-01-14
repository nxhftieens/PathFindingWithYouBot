#ifndef ASTAR_HEADER_HPP
#define ASTAR_HEADER_HPP

#include <queue>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "../include/Element.hpp"

struct Node
{
    Position pos;
    float gCost;
    float hCost;
    float fCost;
    Node* parent;
    Node(const Position& pos, float g, float h, Node* p = nullptr)
        : pos(pos), gCost(g), hCost(h), fCost(g + h), parent(p) 
    {
    }
    bool operator<(const Node& other) const
    {
        return fCost > other.fCost;
    }
};

class AStar
{
private:
	std::vector<std::vector<CellType>> gridMap;
	Position startCell, targetCell;
	int rows, cols;
	std::vector<Position> path;
    std::vector<std::vector<bool>> visited;

    /*static bool compareNodes(const Node* a, const Node* b)
    {
        return a->fCost > b->fCost;
    };*/
	
    struct
    {
		bool operator()(Node* a, Node* b) const
		{
			return a->fCost > b->fCost;
		}
	} compareNodes;

	std::priority_queue<Node*, std::vector<Node*>, decltype(compareNodes)> openList;

	float heuristic(const Position& a, const Position& b);

    void clearOpenList();

    void findShortestPath();

public:

    AStar(const std::vector<std::vector<CellType>>& grid, const Position& start, const Position& target);
    std::vector<Position> getPath();
};

#endif // !A-Star_HEADER_HPP