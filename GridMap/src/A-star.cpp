#include "../include/A-star.hpp"

AStar::AStar(const std::vector<std::vector<CellType>>& grid, const Position& start, const Position& target)
    : gridMap(grid), startCell(start), targetCell(target)
{
    rows = grid.size();
    cols = grid[0].size();
    visited.resize(rows, std::vector<bool>(cols, false));
}

//const bool AStar::compareNodes(const Node* a, const Node* b)
//{
//    return a->fCost > b->fCost;
//}

float AStar::heuristic(const Position& a, const Position& b)
{
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

void AStar::clearOpenList()
{
    while (!openList.empty())
    {
        delete openList.top();
        openList.pop();
    }
}

void AStar::findShortestPath()
{
    Node* startNode = new Node(startCell, 0.0f, heuristic(startCell, targetCell));
    openList.push(startNode);

    while (!openList.empty())
    {
        Node* currentNode = openList.top();

        openList.pop();

        if (visited[currentNode->pos.y][currentNode->pos.x])
        {
            delete currentNode;
            continue;
        }

        visited[currentNode->pos.y][currentNode->pos.x] = true;

        if (currentNode->pos == targetCell)
        {
            Node* pathNode = currentNode;
            while (pathNode != nullptr)
            {
                path.push_back(pathNode->pos);
                pathNode = pathNode->parent;
            }
            std::reverse(path.begin(), path.end());
            clearOpenList();
            return;
        }

        for (const auto& dir : directions)
        {
            Position neighbor = { currentNode->pos.x + dir.x, currentNode->pos.y + dir.y };
            if (neighbor.x < 0 || neighbor.x >= cols || neighbor.y < 0 || neighbor.y >= rows)
                continue;
            
            if (gridMap[neighbor.y][neighbor.x] == CellType::Obstacle || visited[neighbor.y][neighbor.x])
                continue;

            float gCost = 0.0f;

            if (dir.x || dir.y) 
            {
                gCost = currentNode->gCost + 1.0f;
            }
            else
            {
                gCost = currentNode->gCost + std::sqrt(2);
            }

            Node* neighborNode = new Node(neighbor, gCost, heuristic(neighbor, targetCell), currentNode);
            openList.push(neighborNode);
        }        
    }
    return;
}

std::vector<Position> AStar::getPath()
{
	findShortestPath();
	if (path.empty())
	{
		std::cout << "No path found" << std::endl;
	}
    return path;
}
