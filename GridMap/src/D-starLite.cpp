#include "../include/D-starLite.hpp"
#include <iomanip> 

DStarLite::DStarLite(const std::vector<std::vector<CellType>>& grid, const Position& start, const Position& target)
    : gridMap(grid), startCell(start), targetCell(target)
{
    rows = grid.size();
    cols = grid[0].size();
    km = 0.0f;
}

float DStarLite::heuristic(const Position& current, const Position& startCell)
{
	Position posA = current;
	Position posB = startCell;
    return std::sqrt(std::pow(posA.x - posB.x, 2) + std::pow(posA.y - posB.y, 2));
}

Key DStarLite::calculateKey(NodeDS* u)
{
    float gValue = std::min(u->g, u->rhs);
    return Key(gValue + heuristic(u->pos, startCell) + km, gValue);
}

void DStarLite::updateVertex(NodeDS* u)
{
    if (u->g != u->rhs && openList.contains(u))
    {
		//u->g = u->rhs;
        u->key = calculateKey(u);
	}
	else if (u->g != u->rhs && !openList.contains(u))
	{
		//u->g = u->rhs;
		u->key = calculateKey(u);
		openList.insert(u);
	}
    else if (u->g == u->rhs && openList.contains(u))
    {
        openList.erase(u);
    }
}

void DStarLite::computeShortestPath()
{
	if (!openList.empty())
	{
		NodeDS* startNodeDS = allNodeDSs[startCell];
        NodeDS* u = *openList.begin();
		std::cout << "StartNode key: " << startNodeDS->key.k1 << "  " << startNodeDS->key.k2 << std::endl;
        std::cout << "Top key: " << u->key.k1 << "  " << u->key.k2 << std::endl;

        while (u->key > calculateKey(startNodeDS) || startNodeDS->rhs > startNodeDS->g)
        {
            Position topPosition = u->pos;
            // Debugging output
            //std::cout << "Processing NodeDS: (" << topPosition.x << ", " << topPosition.y << ")\n";

            Key kOld = u->key;
            Key kNew = calculateKey(u);

            if (kOld > kNew)
            {
				u->key = kNew;
            }
            else if (u->g > u->rhs)
            {
                u->g = u->rhs;
                for (const auto& pred : u->predecessors)
                {    
					if (pred->pos != targetCell)
					{
						pred->rhs = std::min(pred->rhs, u->g + costFunction(pred, u));
					}
                    updateVertex(pred);                    
                }
                openList.erase(u);
			}
			else
			{
                std::cout << "Okay till here\n";
				float gOld = u->g;
				u->g = INF;
				std::vector<NodeDS*> preds = u->predecessors;
				preds.push_back(u);
				for (const auto& pred : preds)
				{
                    if (pred->rhs == costFunction(pred, u) + gOld)
                    {
                        if (pred->pos != targetCell)
                        {
							for (const auto& succ : pred->successors)
							{
								pred->rhs = std::min(pred->rhs, costFunction(pred, succ) + succ->g);
							}
                        }
                    }
                    updateVertex(pred);
				}
			}
            u = *openList.begin();
        }
	}

    std::cout << "G map: " << std::endl;
    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            std::cout << std::setw(5) << std::setprecision(2) << std::fixed << allNodeDSs[Position(x, y)]->g << " | ";
        }
        std::cout << std::endl;
    }
    std::cout << "RHS map: " << std::endl;
    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            std::cout << std::setw(5) << std::setprecision(2) << std::fixed << allNodeDSs[Position(x, y)]->rhs << " | ";
        }
        std::cout << std::endl;
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
	openList.clear();
	allNodeDSs.clear();
    path.clear();
    km = 0.0f;

	for (int y = 0; y < rows; ++y)
	{
		for (int x = 0; x < cols; ++x)
		{
			NodeDS* newNodeDS = new NodeDS(Position(x, y), INF, INF);
			newNodeDS->key = calculateKey(newNodeDS);
			allNodeDSs[Position(x, y)] = newNodeDS;
		}
	}

	//Linking neighbors
	for (int y = 0; y < rows; ++y)
	{
		for (int x = 0; x < cols; ++x)
		{
			NodeDS* currentNodeDS = allNodeDSs[Position(x, y)];
			for (const auto& dir : directions)
			{
				Position neighbor = { x + dir.x, y + dir.y };
				if (isValidCell(neighbor))
				{
                    currentNodeDS->successors.push_back(allNodeDSs[neighbor]);
                    allNodeDSs[neighbor]->predecessors.push_back(currentNodeDS);
				}
			}
		}
	}

	allNodeDSs[targetCell]->rhs = 0.0f;
    allNodeDSs[targetCell]->key = calculateKey(allNodeDSs[targetCell]);
    openList.insert(allNodeDSs[targetCell]);
	std::cout << openList.size() << std::endl;

    std::cout << "Initialzed" << std::endl;
    computeShortestPath();
}

float DStarLite::costFunction(NodeDS* u, NodeDS* v)
{
	if (gridMap[v->pos.y][v->pos.x] == CellType::Obstacle)
		return INF;
	if (u->pos.x == v->pos.x && u->pos.y == v->pos.y)
		return 0.0f;
	return (u->pos.x == v->pos.x || u->pos.y == v->pos.y) ? 1.0f : std::sqrt(2.0f);
}

std::vector<Position> DStarLite::getPath()
{
    Position currentPos = startCell;
    path.clear();

    std::cout << "G map: " << std::endl;
    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            std::cout << std::setw(5) << std::setprecision(2) << std::fixed << allNodeDSs[Position(x, y)]->g << " | ";
        }
        std::cout << std::endl;
    }
    std::cout << "RHS map: " << std::endl;
    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            std::cout << std::setw(5) << std::setprecision(2) << std::fixed << allNodeDSs[Position(x, y)]->rhs << " | ";
        }
        std::cout << std::endl;
    }

    while (currentPos != targetCell)
    {        
        float minCost = INF;
        Position nextCell = currentPos;	

		for (const auto& succ : allNodeDSs[currentPos]->successors)
		{
			float cost = costFunction(allNodeDSs[currentPos], succ) + succ->g;
			if (cost < minCost)
			{
				minCost = cost;
				nextCell = succ->pos;
			}
		}
        if (minCost == INF)
        {
            //std::cout << "No path found" << std::endl;
            return {};
        }
        path.push_back(currentPos);
        currentPos = nextCell;        
    }
    path.push_back(targetCell);
    return path;
}

void DStarLite::updateObstacle(const Position& obstacle, const Position& newStart)
{
    km += heuristic(startCell, newStart);
    startCell = newStart;

    gridMap[obstacle.y][obstacle.x] = CellType::Obstacle;

	for (const auto& neighbor : allNodeDSs[obstacle]->successors)
	{
		if (neighbor->pos != targetCell)
		{
			neighbor->rhs = std::min(neighbor->rhs, costFunction(allNodeDSs[obstacle], neighbor) + allNodeDSs[obstacle]->g);
        }
		updateVertex(neighbor);
	}

    computeShortestPath();
}