#include "../include/GridMap.hpp"

#include <queue>
#include <set>
#include <algorithm>

// Function to generate path using A* algorithm
std::vector<sf::Vector2f> GeneratePath(const std::vector<std::vector<CellType>>& grid, 
                                        const sf::Vector2i& startCell, 
                                        const sf::Vector2i& targetCell, 
                                        int cellSize, 
                                        int rows, 
                                        int cols)
{

    struct Node
    {
        sf::Vector2i position;
        int gCost;
        float hCost;
        float fCost;
        Node* parent;

        Node(const sf::Vector2i& pos, int g, float h, Node* p = nullptr)
            : position(pos), gCost(g), hCost(h), fCost(g + h), parent(p) {
        }
    };

    // Priority queue with custom comparator
    auto compareNodes = [](const Node* a, const Node* b)
        {
            return a->fCost > b->fCost;
        };

    std::priority_queue<Node*, std::vector<Node*>, decltype(compareNodes)> openList(compareNodes);

    // 2D vector to keep track of visited positions
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));

    // Add start node to open list
    Node* startNode = new Node(startCell, 0, abs(startCell.x - targetCell.x) + abs(startCell.y - targetCell.y));
    openList.push(startNode);

    while (!openList.empty())
    {
        Node* currentNode = openList.top();
        openList.pop();

        int x = currentNode->position.x;
        int y = currentNode->position.y;

        // Skip if already visited
        if (visited[y][x])
        {
            delete currentNode;
            continue;
        }

        visited[y][x] = true;

        // Check if target reached
        if (currentNode->position == targetCell)
        {
            // Reconstruct path
            std::vector<sf::Vector2f> path;
            Node* pathNode = currentNode;
            while (pathNode != nullptr)
            {
                // Convert cell position to pixel coordinates (center of the cell)
                sf::Vector2f pixelPos = sf::Vector2f(
                    (pathNode->position.x + 0.5f) * static_cast<float>(cellSize),
                    (pathNode->position.y + 0.5f) * static_cast<float>(cellSize)
                );
                path.push_back(pixelPos);
                pathNode = pathNode->parent;
            }
            std::reverse(path.begin(), path.end());

            // Clean up nodes
            delete currentNode;
            while (!openList.empty())
            {
                delete openList.top();
                openList.pop();
            }

            return path;
        }

        // Explore neighbors (up, down, left, right)
        for (const auto& direction : directions)
        {
            sf::Vector2i neighborPos = currentNode->position + direction;

            // Ensure neighbor is within bounds
            if (neighborPos.x < 0 || neighborPos.x >= cols || neighborPos.y < 0 || neighborPos.y >= rows)
                continue;

            // Skip if neighbor is an obstacle or already visited
            if (grid[neighborPos.y][neighborPos.x] == CellType::Obstacle || visited[neighborPos.y][neighborPos.x])
                continue;

            int gCost = currentNode->gCost + 1;
            int dx = abs(neighborPos.x - targetCell.x);
            int dy = abs(neighborPos.y - targetCell.y);
			float hCost = dx + dy + (std::sqrt(2)- 2) * std::min(dx, dy); // Diagonal distance heuristic

            Node* neighborNode = new Node(neighborPos, gCost, hCost, currentNode);
            openList.push(neighborNode);
        }

        // No need to delete currentNode here since it's used by its children
    }

    // Clean up remaining nodes in openList
    while (!openList.empty())
    {
        delete openList.top();
        openList.pop();
    }

    // No path found
    return {};
}
