#include "../include/D-starLite.hpp"
#include <iomanip> 

DStarLite::DStarLite(const std::vector<std::vector<CellType>>& grid, const Position& start, const Position& target)
    : gridMap(grid), startCell(start), targetCell(target)
{
    rows = grid.size();
    cols = grid[0].size();
    km = 0.0f;

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x)
        {
            Position pos(x, y);
            g[pos] = INF;
            rhs[pos] = INF;
        }
    }

    rhs[targetCell] = 0.0f;
    keys[targetCell] = calculateKey(targetCell);
    openList.insert({ keys[targetCell], targetCell });
}

float DStarLite::heuristic(const Position& a, const Position& b)
{
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

Key DStarLite::calculateKey(const Position& u)
{
    float gValue = std::min(g[u], rhs[u]);
    return Key(gValue + heuristic(u, startCell) + km, gValue);
}

void DStarLite::updateVertex(const Position& u)
{
    if (g[u] != rhs[u] && openList.contains({ keys[u], u }))
    {
        openList.erase(openList.find({ keys[u], u }));
		keys[u] = calculateKey(u);
		openList.insert({ keys[u], u });
	}
	else if (g[u] != rhs[u] && !openList.contains({ keys[u], u }))
	{
		keys[u] = calculateKey(u);
		openList.insert({ keys[u], u });
	}
    else if (g[u] == rhs[u] && openList.contains({ keys[u], u }))
    {
        openList.erase(openList.find({ keys[u], u }));
    }
}

void DStarLite::computeShortestPath()
{
    
    while (!openList.empty() && (openList.begin()->first > calculateKey(startCell) || rhs[startCell] != g[startCell]))
    {
		auto topElement = openList.begin();
        Position topPosition = topElement->second;
        // Debugging output
        std::cout << "Processing node: (" << topPosition.x << ", " << topPosition.y << ")\n";

        Key kOld = topElement->first;
        Key kNew = calculateKey(topPosition);

        if (kOld < kNew)
        {
            openList.erase(topElement);
            openList.insert({ kNew, topPosition });
        }
        else if (g[topPosition] > rhs[topPosition])
        {
            g[topPosition] = rhs[topPosition];
            openList.erase(topElement);
            for (const auto& dir : directions)
            {
                Position neighbor = { topPosition.x + dir.x, topPosition.y + dir.y };
                if (isValidCell(neighbor))
                {
                    if (neighbor != targetCell)
                    {
                        float cost = (dir.x == 0 || dir.y == 0) ? 1.0f : std::sqrt(2.0f);
                        if (gridMap[neighbor.y][neighbor.x] == CellType::Obstacle)
                            cost = INF;
                        rhs[neighbor] = std::min(rhs[neighbor], g[topPosition] + cost);
                    }
                    updateVertex(neighbor);
                }
            }
        }
        else
        {
            float gOld = g[topPosition];
            g[topPosition] = INF;
            for (const auto& dir : directions)
            {
                float cost = (dir.x == 0 || dir.y == 0) ? 1.0f : std::sqrt(2.0f);
                Position neighbor = { topPosition.x + dir.x, topPosition.y + dir.y };
                if (gridMap[neighbor.y][neighbor.x] == CellType::Obstacle)
                    cost = INF;
                if (isValidCell(neighbor))
                {
                    if (rhs[neighbor] == cost + gOld)
                    {
                        if (neighbor != targetCell)
                        {
                            rhs[topPosition] = std::min(rhs[topPosition], cost + gOld);
                        }
                    }
                    updateVertex(neighbor);
                }
            }
        }	
    }    
}

bool DStarLite::isValidCell(const Position& pos) const
{
    return pos.x >= 0 && pos.x < cols && pos.y >= 0 && pos.y < rows;
}

void DStarLite::initialize(const std::vector<std::vector<CellType>>& grid,
        const Position& start,
        const Position& target)
{
	gridMap = grid;
	startCell = start;
	targetCell = target;
    rows = grid.size();
    cols = grid[0].size();

	g.clear();
	rhs.clear();
	openList.clear();
	keys.clear();
    path.clear();
    km = 0.0f;

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x)
        {
            Position pos(x, y);
            g[pos] = INF;
            rhs[pos] = INF;
        }
    }

    rhs[targetCell] = 0.0f;
    keys[targetCell] = calculateKey(targetCell);
    openList.insert({ keys[targetCell], targetCell });

    computeShortestPath();
    lastStart = startCell;
}

std::vector<Position> DStarLite::getPath()
{
    Position currentPos = startCell;
    path.clear();

	

    while (currentPos != targetCell)
    {
        path.push_back(currentPos);

        float minCost = INF;
        Position nextCell = currentPos;
        //std::cout << "11111" << std::endl;

        for (const auto& dir : directions)
        {
            Position neighborPos = { currentPos.x + dir.x, currentPos.y + dir.y };
            if (isValidCell(neighborPos))
            {
                float cost = (dir.x == 0 || dir.y == 0) ? 1.0f : std::sqrt(2.0f);
                if (gridMap[neighborPos.y][neighborPos.x] == CellType::Obstacle)
                {
                    cost = INF;
                }

                if (g[neighborPos] + cost < minCost)
                {
                    minCost = g[neighborPos] + cost;
                    nextCell = neighborPos;
                }
            }
        }
        if (minCost == INF)
        {
            std::cout << "No path found" << std::endl;
            return {};
        }
        std::cout << "Insert path cell: " << nextCell.x << ", " << nextCell.y << std::endl;
        path.push_back(nextCell);
        currentPos = nextCell;
    }
    path.push_back(targetCell);
    return path;
}

void DStarLite::updateObstacle(const Position& obstacle, const Position& newStart)
{
    gridMap[obstacle.y][obstacle.x] = CellType::Obstacle;
    rhs[obstacle] = INF;
    //g[obstacle] = INF;

    km += heuristic(startCell, newStart);

    /*auto it = openList.begin();
    while (it != openList.end())
    {
        if (it->second == obstacle)
        {
            it = openList.erase(it);
        }
        else
        {
            ++it;
        }
    }*/

    std::cout << "G map: " << std::endl;
    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            std::cout << std::setw(5) << std::setprecision(2) << std::fixed << g[Position(x, y)] << " | ";
        }
        std::cout << std::endl;
    }
    std::cout << "RHS map: " << std::endl;
    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            std::cout << std::setw(5) << std::setprecision(2) << std::fixed << rhs[Position(x, y)] << " | ";
        }
        std::cout << std::endl;
    }

	startCell = newStart;

    // Update all predecessors of the obstacle cell
    for (const auto& dir : directions)
    {
        Position neighbor = { obstacle.x + dir.x, obstacle.y + dir.y };
        float cOld = (dir.x == 0 || dir.y == 0) ? 1.0f : std::sqrt(2.0f);
        if (isValidCell(neighbor) && neighbor != targetCell)
        {
            for (const auto& dir2 : directions)
            {
                Position neighbor2 = { neighbor.x + dir2.x, neighbor.y + dir2.y };
                if (isValidCell(neighbor2))
                {
                    float cost = (dir2.x == 0 || dir2.y == 0) ? 1.0f : std::sqrt(2.0f);
                    if (gridMap[neighbor2.y][neighbor2.x] == CellType::Obstacle)
                        cost = INF;
                    rhs[neighbor] = std::min(rhs[neighbor], g[neighbor2] + cost);
                }
            }
            std::cout << "Updating vertex: (" << neighbor.x << ", " << neighbor.y << ")\n";
            std::cout << "New RHS: " << rhs[neighbor] << std::endl;
            //updateVertex(neighbor);
        }
		if (neighbor == targetCell)
		{
            static bool oneTime = false;
			if (!oneTime)
			{
                rhs[targetCell] = 0.0f;
                keys[targetCell] = calculateKey(targetCell);
                openList.insert({ keys[targetCell], targetCell });
                oneTime = true;
			}
		}
    }

    std::cout << "G map: " << std::endl;
    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            std::cout << std::setw(5) << std::setprecision(2) << std::fixed << g[Position(x, y)] << " | ";
        }
        std::cout << std::endl;
    }
    std::cout << "RHS map: " << std::endl;
    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            std::cout << std::setw(5) << std::setprecision(2) << std::fixed << rhs[Position(x, y)] << " | ";
        }
        std::cout << std::endl;
    }

    computeShortestPath();

    std::cout << "G map after: " << std::endl;
    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            std::cout << std::setw(5) << std::setprecision(2) << std::fixed << g[Position(x, y)] << " | ";
        }
        std::cout << std::endl;
    }
    std::cout << "RHS map: " << std::endl;
    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            std::cout << std::setw(5) << std::setprecision(2) << std::fixed << rhs[Position(x, y)] << " | ";
        }
        std::cout << std::endl;
    }

}