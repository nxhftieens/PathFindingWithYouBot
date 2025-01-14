#include "../include/GridMap.hpp"

#include <queue>
#include <unordered_map>
#include <map>
#include <set>

class DStarLite
{
private:
    const float INF = std::numeric_limits<float>::infinity();

    // Key structure for the priority queue
    struct Key {
        float k1;
        float k2;

        Key(float k1 = std::numeric_limits<float>::infinity(),
            float k2 = std::numeric_limits<float>::infinity())
            : k1(k1), k2(k2) {
        }

        bool operator==(const Key& other) const {
            return k1 == other.k1 && k2 == other.k2;
        }

        bool operator<(const Key& other) const {
            if (k1 == other.k1)
                return k2 > other.k2;
			return k1 > other.k1;
        }
    };

    // Custom comparator for the open list
    struct OpenListComparator {
        bool operator()(const std::pair<Key, sf::Vector2i>& lhs, const std::pair<Key, sf::Vector2i>& rhs) const {
            if (lhs.first < rhs.first)
                return true;
            if (rhs.first < lhs.first)
                return false;
            // If keys are equal, compare positions
            if (lhs.second.x != rhs.second.x)
                return lhs.second.x < rhs.second.x;
            return lhs.second.y < rhs.second.y;
        }
    };

    struct Vector2iComparator
    {
        bool operator()(const sf::Vector2i& lhs, const sf::Vector2i& rhs) const
        {
            if (lhs.x == rhs.x)
                return lhs.y < rhs.y;
            return lhs.x < rhs.x;
        }
    };

    sf::Vector2i startCell;
    sf::Vector2i targetCell;
    std::vector<std::vector<CellType>> grid;
    int cellSize;
    int rows;
    int cols;

    std::map<sf::Vector2i, float, Vector2iComparator> g, rhs;
    std::map<sf::Vector2i, Key, Vector2iComparator> keys;
    std::set<std::pair<Key, sf::Vector2i>, OpenListComparator> openList;
    float km;

    std::vector<sf::Vector2f> path;
    sf::Vector2i lastStart;

    std::vector<sf::Vector2i> directions = {
        { 1,  0}, // Right
        {-1,  0}, // Left
        { 0,  1}, // Down
        { 0, -1}, // Up
        { 1,  1}, // Diagonal Right Down
        {-1,  1}, // Diagonal Left Down
        { 1, -1}, // Diagonal Right Up
        {-1, -1}  // Diagonal Left Up
    };

    // Calculate the heuristic cost
    float heuristic(const sf::Vector2i& a, const sf::Vector2i& b) const {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    // Calculate cell key
    Key calculateKey(const sf::Vector2i& pos) const {
        float minCost = std::min(g.at(pos), rhs.at(pos));
        return Key(minCost + heuristic(startCell, pos) + km, minCost);
    }

    void updateVertex(const sf::Vector2i& u) {
        if (g[u] != rhs[u] && openList.count({ calculateKey(u), u }) == 0) {
            openList.insert({ calculateKey(u), u });
        }
        else if (g[u] == rhs[u]) {
            openList.erase({ keys[u], u });
        }
        keys[u] = calculateKey(u);
    }

    void computeShortestPath() {
        while (!openList.empty()) {
            auto [k_old, u] = *openList.begin();
            openList.erase(openList.begin());

            Key k_new = calculateKey(u);
            if (k_old < k_new) {
                openList.insert({ k_new, u });
                continue;
            }

            if (g[u] > rhs[u]) {
                g[u] = rhs[u];
                for (const auto& direction : directions) {
                    sf::Vector2i neighborPos = u + direction;
                    if (isValidCell(neighborPos)) {
                        updateVertex(neighborPos);
                    }
                }
            }
            else {
                g[u] = INF;
                updateVertex(u);
                for (const auto& direction : directions) {
                    sf::Vector2i neighborPos = u + direction;
                    if (isValidCell(neighborPos)) {
                        updateVertex(neighborPos);
                    }
                }
            }
        }
    }

    bool isValidCell(const sf::Vector2i& pos) const {
        return pos.x >= 0 && pos.x < cols &&
            pos.y >= 0 && pos.y < rows &&
            grid[pos.y][pos.x] != CellType::Obstacle;
    }

public:
    DStarLite(const std::vector<std::vector<CellType>>& grid,
        const sf::Vector2i& startCell,
        const sf::Vector2i& targetCell,
        int cellSize,
        int rows,
        int cols)
        : grid(grid), startCell(startCell), targetCell(targetCell),
        cellSize(cellSize), rows(rows), cols(cols), km(0), lastStart(startCell)
    {
        if (startCell.x != -1 && targetCell.x != -1)
        {
            // Initialize g, rhs, keys, and openList
            for (int y = 0; y < rows; ++y)
            {
                for (int x = 0; x < cols; ++x)
                {
                    sf::Vector2i pos(x, y);
                    g[pos] = INF;
                    rhs[pos] = INF;
                }
            }
            rhs[targetCell] = 0;
            Key key = calculateKey(targetCell);
            keys[targetCell] = key;
            openList.insert({ key, targetCell });
        }
    }

    std::vector<sf::Vector2f> findPath() {
        computeShortestPath();

        sf::Vector2i currentPos = startCell;
        lastStart = startCell;
        path.clear();

        while (currentPos != targetCell) {
            path.push_back(sf::Vector2f(currentPos.x * cellSize + cellSize / 2.0f,
                currentPos.y * cellSize + cellSize / 2.0f));

            float minCost = INF;
            sf::Vector2i nextCell = currentPos;

            for (const auto& direction : directions) {
                sf::Vector2i neighborPos = currentPos + direction;
                if (isValidCell(neighborPos)) {
                    float cost = (direction.x == 0 || direction.y == 0) ? 1.0f : std::sqrt(2.0f);
                    if (g[neighborPos] + cost < minCost) {
                        minCost = g[neighborPos] + cost;
                        nextCell = neighborPos;
                    }
                }
            }

            if (nextCell == currentPos) {
                // No path found
                return {};
            }

            currentPos = nextCell;
        }

        path.push_back(sf::Vector2f(targetCell.x * cellSize + cellSize / 2.0f,
            targetCell.y * cellSize + cellSize / 2.0f));

        // Reverse the path to get from start to target
        std::reverse(path.begin(), path.end());

        return path;
    }

    void updateParameters(const std::vector<std::vector<CellType>>& grid,
        const sf::Vector2i& startCell,
        const sf::Vector2i& targetCell,
        int cellSize,
        int rows,
        int cols)
    {
        this->grid = grid;
        this->startCell = startCell;
        this->targetCell = targetCell;
        this->cellSize = cellSize;
        this->rows = rows;
        this->cols = cols;

        // Reinitialize internal variables
        km = 0;
        lastStart = startCell;

        // Reset g, rhs, and keys
        g.clear();
        rhs.clear();
        keys.clear();

        for (int y = 0; y < rows; ++y)
        {
            for (int x = 0; x < cols; ++x)
            {
                sf::Vector2i pos(x, y);
                g[pos] = INF;
                rhs[pos] = INF;
            }
        }

        rhs[targetCell] = 0;
        Key key = calculateKey(targetCell);
        keys[targetCell] = key;

        // Clear and reinitialize the open list
        openList.clear();
        openList.insert({ key, targetCell });
    }

    void updateObstacle(const sf::Vector2i& obstaclePos) {
        // Assume the obstacle is added
        grid[obstaclePos.y][obstaclePos.x] = CellType::Obstacle;
        rhs[obstaclePos] = INF;
        g[obstaclePos] = INF;
        updateVertex(obstaclePos);
        km += heuristic(lastStart, startCell);
        computeShortestPath();
    }
};
