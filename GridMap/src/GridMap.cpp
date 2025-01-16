// GridMap.cpp : Defines the entry point for the application.
//

#include "../include/GridMap.hpp"


GridMap::GridMap(const int& cellSize, const int& rows, const int& cols)
    : cellSize(cellSize), rows(rows), cols(cols)
{
    gridMap.resize(rows, std::vector<CellType>(cols, CellType::Empty));
}

void GridMap::eventHandler(sf::RenderWindow& window, Position& selectedCell, Robot& robot, DStarLite& dstarlite)
{
    while(const std::optional<sf::Event> event = window.pollEvent())
    {
        if (event->is<sf::Event::Closed>())
            window.close();

        if (event->is<sf::Event::MouseButtonPressed>())
        {
            auto mouseEvent = event->getIf<sf::Event::MouseButtonPressed>();
            int x = mouseEvent->position.x / cellSize;
            int y = mouseEvent->position.y / cellSize;

            // Check if the cell is within bounds
            if (x >= 0 && x < cols && y >= 0 && y < rows)
            {
                // Set the selected cell
                selectedCell = Position(x, y);
            }
        }

        if (event->is<sf::Event::KeyPressed>())
        {
            auto keyEvent = event->getIf<sf::Event::KeyPressed>();

            // Check if a cell is selected
            if (selectedCell.x != -1 && selectedCell.y != -1)
            {
                // Determine the action based on the key pressed
                switch (keyEvent->code)
                {
                case sf::Keyboard::Key::S:
                    // Set the start point
                    startCell = selectedCell;
                    dstarlite.initialize(startCell, targetCell);
					robot.updatePos(startCell, cellSize);
                    robot.seenObstacles.clear();
                    robot.resetPathLength();
                    getPixelPath(dstarlite, robot);
                    break;
                case sf::Keyboard::Key::T:
                    // Set the target point
                    targetCell = selectedCell;
                    dstarlite.initialize(startCell, targetCell);
					robot.updatePos(startCell, cellSize);
                    robot.seenObstacles.clear();
                    robot.resetPathLength();
                    getPixelPath(dstarlite, robot);
                    break;
                case sf::Keyboard::Key::O:
                    // Toggle obstacle
                    if (gridMap[selectedCell.y][selectedCell.x] == CellType::Empty)
                        gridMap[selectedCell.y][selectedCell.x] = CellType::Obstacle;
                    else
                        gridMap[selectedCell.y][selectedCell.x] = CellType::Empty;
                    break;
                case sf::Keyboard::Key::E:
                    // Clear the cell
                    if (startCell == selectedCell)
                        startCell = Position(-1, -1);
                    if (targetCell == selectedCell)
                        targetCell = Position(-1, -1);
                    gridMap[selectedCell.y][selectedCell.x] = CellType::Empty;
                    break;
                case sf::Keyboard::Key::R:
                    // Generate a new random map
                    randomize();
                    dstarlite.initialize(startCell, targetCell);
					robot.updatePos(startCell, cellSize);
                    robot.seenObstacles.clear();
                    robot.resetPathLength();
                    getPixelPath(dstarlite, robot);
                    break;
                default:
                    break;
                }

                // Reset the selected cell after action
                selectedCell = Position(-1, -1);
            }
            else
            {
                // If no cell is selected
                if (keyEvent->code == sf::Keyboard::Key::R)
                {
                    // Generate a new random map
                    randomize();
                    dstarlite.initialize(startCell, targetCell);
					robot.updatePos(startCell, cellSize);
                    robot.seenObstacles.clear();
                    robot.resetPathLength();
                    getPixelPath(dstarlite, robot);
                }
                if (keyEvent->code == sf::Keyboard::Key::P)
                {
					// Generate the path
                    dstarlite.initialize(startCell, targetCell);
                    robot.updatePos(startCell, cellSize);
                    robot.seenObstacles.clear();
                    robot.resetPathLength();
					getPixelPath(dstarlite, robot);
                }
                if (keyEvent->code == sf::Keyboard::Key::M)
                {
					// Toggle the moving flag
					robot.movingFlagToggle();
					std::cout << "Moving flag toggled"  << std::endl;
                }
            }
        }
    }
}

void GridMap::visualize() 
{
    const unsigned int gridWidth = cols * cellSize;
    const unsigned int gridHeight = rows * cellSize;

    // Create the SFML window
    sf::RenderWindow window(sf::VideoMode({ gridWidth, gridHeight }), "Grid Map");
    
    Position selectedCell(-1, -1);

	Robot robot(startCell, 100.0f, cellSize * 4, cellSize * 3, cellSize * 2, 0);
	robot.updatePos(startCell, cellSize);

	AStar astar(gridMap, startCell, targetCell);
	DStarLite dstarlite(Position(cols, rows));
	dstarlite.initialize(startCell, targetCell);

    while (window.isOpen())
    {
        window.clear(sf::Color::White);

        eventHandler(window, selectedCell, robot, dstarlite);
        
        if (startCell.x != -1 && startCell.y != -1)
        {
            sf::CircleShape startPoint(static_cast<float>(cellSize) / 2.0f);
            startPoint.setFillColor(sf::Color::Blue);
            startPoint.setPosition(sf::Vector2f(static_cast<float>(startCell.x * cellSize), static_cast<float>(startCell.y * cellSize)));
            window.draw(startPoint);
        }

        if (targetCell.x != -1 && targetCell.y != -1)
        {
            sf::CircleShape targetPoint(static_cast<float>(cellSize) / 2.0f);
            targetPoint.setFillColor(sf::Color::Red);
            targetPoint.setPosition(sf::Vector2f(static_cast<float>(targetCell.x * cellSize), static_cast<float>(targetCell.y * cellSize)));
            window.draw(targetPoint);
        }

        if (selectedCell.x != -1 && selectedCell.y != -1)
        {
            sf::RectangleShape selectedPoint(sf::Vector2f(static_cast<float>(cellSize), static_cast<float>(cellSize)));
            selectedPoint.setFillColor(sf::Color::Cyan);
            selectedPoint.setPosition(sf::Vector2f(static_cast<float>(selectedCell.x * cellSize), static_cast<float>(selectedCell.y * cellSize)));
            window.draw(selectedPoint); 
        }		
        for (int y = 0; y < rows; ++y)
        {
            for (int x = 0; x < cols; ++x)
            {                
                if (gridMap[y][x] == CellType::Obstacle)
                {
                    sf::RectangleShape cellShape(sf::Vector2f(static_cast<float>(cellSize), static_cast<float>(cellSize)));
                    cellShape.setPosition(sf::Vector2f(static_cast<float>(x * cellSize), static_cast<float>(y * cellSize)));
                    cellShape.setFillColor(sf::Color::Black);
                    window.draw(cellShape);
                }
            }
        }

        if (!pixelPath.empty())
        {
            sf::VertexArray pathLines(sf::PrimitiveType::LineStrip, pixelPath.size());
            for (size_t i = 0; i < pixelPath.size(); ++i)
            {
                pathLines[i].position = pixelPath[i];
                pathLines[i].color = sf::Color::Green;
            }
            window.draw(pathLines);
        }


        for (const auto& obs : robot.seenObstacles)
        {
            sf::CircleShape obstacleShape(static_cast<float>(cellSize) / 2.0f);
            obstacleShape.setFillColor(sf::Color(118, 152, 224, 170));
            obstacleShape.setPosition(sf::Vector2f(obs.x * static_cast<float>(cellSize), obs.y * static_cast<float>(cellSize)));
            window.draw(obstacleShape);
        }
        std::vector<std::vector<CellType>> inflatedGrid = inflateObstacles(robot.getSafetyOffset());
        robot.lookAhead(inflatedGrid, pixelPath, gridPath, cellSize);
        if (robot.foundObstacle())
            getPixelPath(dstarlite, robot);        
        robot.move(pixelPath);
		robot.draw(window, cellSize);
        window.display();
    }
}

void GridMap::getPixelPath(DStarLite& dstarlite, Robot& robot)
{    
    if (robot.foundObstacle())
    {
		robot.seenObstacles.push_back(robot.getObstaclePos());
        robot.resetObstacleFlag();
        robot.resetPathLength();  
		dstarlite.updateCell(robot.getObstaclePos(), -1);
		dstarlite.updateStart(robot.getGridPosition(cellSize));
    }
    pixelPath.clear();
    gridPath.clear();
	dstarlite.replan();
    gridPath = dstarlite.getPath();

    for (const auto& pos : gridPath)
    {
        pixelPath.push_back(sf::Vector2f((pos.x + 0.5f) * static_cast<float>(cellSize),
            (pos.y + 0.5f) * static_cast<float>(cellSize)));
    }
    robot.calculatePathLength(pixelPath);
}

void GridMap::getPixelPath(AStar& astar)
{
    pixelPath.clear();
    gridPath.clear();

    gridPath = astar.getPath();
    for (const auto& pos : gridPath)
    {
        pixelPath.push_back(sf::Vector2f((pos.x + 0.5f) * static_cast<float>(cellSize),
            (pos.y + 0.5f) * static_cast<float>(cellSize)));
    }
}

void GridMap::randomize()
{
    // Reset grid
    for (int y = 0; y < rows; ++y)
    {
        std::fill(gridMap[y].begin(), gridMap[y].end(), CellType::Empty);
    }

    // Reset start and target positions
    startCell = Position(-1, -1);
    targetCell = Position(-1, -1);

    // Random number generators
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distRow(0, rows - 1);
    std::uniform_int_distribution<> distCol(0, cols - 1);

    int numberOfShapes = 20; // Adjust the number of obstacle shapes

    for (int i = 0; i < numberOfShapes; ++i)
    {
        int shapeType = i % 6;

        int posX = distCol(gen);
        int posY = distRow(gen);
        int sizeX = distCol(gen) % (cols / 4) + cols / 10;
        int sizeY = distRow(gen) % (rows / 4) + rows / 10;

        int radius = std::min(sizeX, sizeY) / 2;
        int centerX = posX + radius;
        int centerY = posY + radius;

        switch (shapeType)
        {
        case 0: // Rectangle
            for (int y = posY; y < posY + sizeY && y < rows; ++y)
                for (int x = posX; x < posX + sizeX && x < cols; ++x)
                    gridMap[y][x] = CellType::Obstacle;
            break;
        case 1: // Square
            sizeX = sizeY = std::min(sizeX, sizeY);
            for (int y = posY; y < posY + sizeY && y < rows; ++y)
                for (int x = posX; x < posX + sizeX && x < cols; ++x)
                    gridMap[y][x] = CellType::Obstacle;
            break;
        case 2: // Circle                    
            for (int y = std::max(0, centerY - radius); y <= std::min(rows - 1, centerY + radius); ++y)
            {
                for (int x = std::max(0, centerX - radius); x <= std::min(cols - 1, centerX + radius); ++x)
                {
                    int dx = x - centerX;
                    int dy = y - centerY;
                    if (dx * dx + dy * dy <= radius * radius)
                        gridMap[y][x] = CellType::Obstacle;
                }
            }
            break;
        case 3: // U-shape
            for (int y = posY; y < posY + sizeY && y < rows; ++y)
            {
                for (int x = posX; x < posX + sizeX && x < cols; ++x)
                {
                    if (x < posX + 4 || x >= posX + sizeX - 4 || (y >= posY + sizeY - 4))
                        gridMap[y][x] = CellType::Obstacle;
                }
            }
            break;
        case 4: //Triangle
            for (int y = 0; y < sizeY && posY + y < rows; ++y)
            {
                int startX = posX + sizeX / 3 - y * sizeX / (3 * sizeY);
                int endX = posX + sizeX / 3 + y * sizeX / (3 * sizeY);
                for (int x = startX; x <= endX && x < cols; ++x)
                {
                    if (x >= 0)
                        gridMap[posY + y][x] = CellType::Obstacle;
                }
            }
            break;
        default:
            break;
        }
    }

    // Random start position
    std::uniform_int_distribution<> distRowStart(0, rows - 1);
    std::uniform_int_distribution<> distColStart(0, cols - 1);
    do
    {
        startCell.x = distColStart(gen);
        startCell.y = distRowStart(gen);
    } while (gridMap[startCell.y][startCell.x] == CellType::Obstacle);

    // Random target position
    std::uniform_int_distribution<> distRowTarget(0, rows - 1);
    std::uniform_int_distribution<> distColTarget(0, cols - 1);
    do
    {
        targetCell.x = distColTarget(gen);
        targetCell.y = distRowTarget(gen);
    } while (gridMap[targetCell.y][targetCell.x] == CellType::Obstacle || targetCell == startCell);
    setWall();
}

std::vector<std::vector<CellType>> GridMap::inflateObstacles(const float& inflationRadius)
{
	int offsetCells = static_cast<int>(std::ceil(inflationRadius / cellSize));

    std::vector<std::vector<CellType>> inflatedGrid = gridMap;

    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            if (gridMap[y][x] == CellType::Obstacle)
            {
                // Inflate the obstacle
                for (int dy = -offsetCells; dy <= offsetCells; ++dy)
                {
                    for (int dx = -offsetCells; dx <= offsetCells; ++dx)
                    {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < cols && ny >= 0 && ny < rows)
                        {
                            inflatedGrid[ny][nx] = CellType::Obstacle;
                        }
                    }
                }
            }
        }
    }
    // Update the grid map with the inflated obstacles
    return inflatedGrid;
}

void GridMap::setWall()
{
    for (int y = 0; y < rows; ++y)
    {
        for (int x = 0; x < cols; ++x)
        {
            if (x == 0 || x == cols - 1 || y == 0 || y == rows - 1)
                gridMap[y][x] = CellType::Obstacle;
        }
    }
}

Robot::Robot(const Position& pos, const float& speed, const float& visibleRadius, const float& length, const float& width, const float& offset)
	: pos(pos), speed(speed), visibleRadius(visibleRadius), length(length), width(width)
{
    totalPathLength = 0.0f;
    accumulatedDistance = 0.0f;
	robotClock.restart();

    safetyOffset = 0.5f * std::sqrt(length * length + width * width) + offset;
}

void Robot::calculatePathLength(const std::vector<sf::Vector2f>& path)
{	
	if (!path.empty())
	{
		for (size_t i = 0; i < path.size() - 1; ++i)
		{
			float segmentLengthPixels = std::hypot(
				path[i + 1].x - path[i].x,
				path[i + 1].y - path[i].y
			);
			segmentLengths.push_back(segmentLengthPixels);
			totalPathLength += segmentLengthPixels;
		}
	}
}

void Robot::resetPathLength()
{
	segmentLengths.clear();
	totalPathLength = 0.0f;
    accumulatedDistance = 0.0f;
    robotClock.restart();
}

void Robot::move(const std::vector<sf::Vector2f>& pixelPath)
{
    float deltaTime = robotClock.restart().asSeconds();
    if (!pixelPath.empty() && !segmentLengths.empty() && moving)
    {
        float distanceToMove = speed * deltaTime;
        accumulatedDistance += distanceToMove;

		float distanceCovered = 0.0f;
		int robotPathIndex = 0;

        // Find the current segment based on accumulated distance
        while (robotPathIndex < segmentLengths.size() && accumulatedDistance > distanceCovered + segmentLengths[robotPathIndex])
        {
            distanceCovered += segmentLengths[robotPathIndex];
            ++robotPathIndex;
        }

        // Interpolate the robot's position between the two points
        if (robotPathIndex < segmentLengths.size())
        {   
            if (robotPathIndex + 1 < pixelPath.size())
            {
                float segmentProgress = (accumulatedDistance - distanceCovered) / segmentLengths[robotPathIndex];
                float x = pixelPath[robotPathIndex].x + (pixelPath[robotPathIndex + 1].x - pixelPath[robotPathIndex].x) * segmentProgress;
                float y = pixelPath[robotPathIndex].y + (pixelPath[robotPathIndex + 1].y - pixelPath[robotPathIndex].y) * segmentProgress;
                pos.x = static_cast<int>(x);
                pos.y = static_cast<int>(y);
            }
            else
            {
				pos = Position(pixelPath.back().x, pixelPath.back().y);
            }
        }
        else
        {
            pos = Position(pixelPath.back().x, pixelPath.back().y);
        }
    }
}

void Robot::draw(sf::RenderWindow& window, const int& cellSize)
{
    sf::CircleShape visibleArea(visibleRadius);
    visibleArea.setFillColor(sf::Color(224, 131, 179, 100));
    visibleArea.setPosition(sf::Vector2f(static_cast<float>(pos.x - visibleRadius), static_cast<float>(pos.y - visibleRadius)));
    window.draw(visibleArea);

    /*sf::RectangleShape robotShape(sf::Vector2f(width, length));
    robotShape.setFillColor(sf::Color::Magenta);
    robotShape.setOrigin(sf::Vector2f(width / 2.0f, length / 2.0f));
    robotShape.setPosition(sf::Vector2f(pos.x, pos.y));
    window.draw(robotShape);*/

	sf::CircleShape robotShape(length / 2.0f);
	robotShape.setFillColor(sf::Color::Magenta);
	robotShape.setOrigin(sf::Vector2f(length / 2.0f, length / 2.0f));
	robotShape.setPosition(sf::Vector2f(pos.x, pos.y));
	window.draw(robotShape);
}

void Robot::updatePos(const Position& start, const int& cellSize)
{
    pos.x = (start.x + 0.5f) * static_cast<float>(cellSize);
    pos.y = (start.y + 0.5f) * static_cast<float>(cellSize);
}

void Robot::lookAhead(std::vector<std::vector<CellType>>& gridMap, const std::vector<sf::Vector2f>& pixelPath, const std::vector<Position>& gridPath, const int& cellSize)
{
	if (!pixelPath.empty() && !gridPath.empty() && pos != Position(pixelPath.back().x, pixelPath.back().y))
	{
        std::vector<Position> visiblePath;
        for (size_t i = 0; i < pixelPath.size() - 1; ++i)
        {
            if (std::pow(pixelPath[i].x - pos.x, 2) + std::pow(pixelPath[i].y - pos.y, 2) <= std::pow(visibleRadius, 2))
            {
                visiblePath.push_back(gridPath[i]);
				//std::cout << "Visible path: " << gridPath[i].x << " " << gridPath[i].y << std::endl;
            }
        }
        for (auto it = visiblePath.begin(); it != visiblePath.end();)
        {
            if (gridMap[it->y][it->x] == CellType::Obstacle)
            {
                sawObstacle = true;
                obstaclePos = *it;
                std::cout << "Obstacle found at: " << it->x << " " << it->y << std::endl;
                return;
            }
            
            if (std::pow((it->x + 0.5f) * static_cast<float>(cellSize) - pos.x, 2) + std::pow((it->y + 0.5f) * static_cast<float>(cellSize) - pos.y, 2) > std::pow(visibleRadius, 2))
            {
                it = visiblePath.erase(it);
            }
            else 
            {
				++it;
            }
        }
        if (visiblePath.empty())
        {
            sawObstacle = false;
        }
	}
}

void GridMap::setGridMapFromImg(const std::string& imgPath, const std::string& dirForCalibration, const int& cellSizeInPixel)
{
    gridMap = mapFromImage(imgPath, dirForCalibration);
	rows = gridMap.size();
	cols = gridMap[0].size();
	for (int y = 0; y < rows; ++y)
	{
		for (int x = 0; x < cols; ++x)
		{
			if (gridMap[y][x] == CellType::Start)
			{
				startCell = Position(x, y);
			}
			if (gridMap[y][x] == CellType::Target)
			{
				targetCell = Position(x, y);
			}			
		}
	}
	cellSize = cellSizeInPixel;
}

int main()
{
    // Define the size of the window and grid
    /*int cellSize = 5;
    int rows = 60;
    int cols = 80;
    GridMap gridMap(cellSize, rows, cols);
    gridMap.randomize();
    gridMap.visualize();*/

    int cellSize = 5;
    string imgPath = "D:/SourceCode/PathFindingWithYouBot/Detect/asd.png";
	string dirForCalibration = "D:/SourceCode/PathFindingWithYouBot/Detect/CamCalib";

	GridMap gridMapFromImg(cellSize, 0, 0);
	gridMapFromImg.setGridMapFromImg(imgPath, dirForCalibration, cellSize);

    gridMapFromImg.visualize();
    return 0;
}

// TODO:
// 1. Add dimensions for the robot
// 2. Add safety distance for the robot
// 3. Use camera with markers to simulate the visible area of the robot
