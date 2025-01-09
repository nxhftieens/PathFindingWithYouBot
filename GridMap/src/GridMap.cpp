// GridMap.cpp : Defines the entry point for the application.
//

#include "../include/GridMap.hpp"
#include "../src/PathFinding.cpp"

int main()
{
    // Define the size of the window and grid
    int cellSize = 10;
    int rows = 120;
    int cols = 160;
    GridMapVisualize(cellSize, rows, cols);
    return 0;
}

// Function to visualize the grid map
void GridMapVisualize(int cellSize,
                        int rows,
                        int cols)
{
    const unsigned int gridWidth = cols * cellSize;
    const unsigned int gridHeight = rows * cellSize;

    // Create the SFML window
    sf::RenderWindow window(sf::VideoMode({ gridWidth, gridHeight }), "Grid Map");

    // Create a 2D vector to hold the cell types
    std::vector<std::vector<CellType>> grid(rows, std::vector<CellType>(cols, CellType::Empty));

    // Variables to store start and target cell positions
    sf::Vector2i startCell(-1, -1);
    sf::Vector2i targetCell(-1, -1);

    // Generate random map
    GenerateRandomMap(grid, startCell, targetCell, rows, cols);

    // Variable to store the selected cell
    sf::Vector2i selectedCell(-1, -1);

    // Variable to store the path generation time
    double pathGenerationTime = 0.0;

    // Variable to store the generated path
    std::vector<sf::Vector2f> path;

    // Flag to indicate if the path needs to be updated
    bool pathNeedsUpdate = true;

    //Flag to indicate if the robot need to move
    bool robotNeedsMove = false;

    // Variables for robot movement
    size_t robotPathIndex = 0;
    float robotSpeed = 100.0f; // Speed in grids per second
    sf::Clock robotClock;
    float accumulatedDistance = 0.0f;
    float totalPathLength = 0.0f;
    float distanceCovered = 0.0f;
    std::vector<float> segmentLengths;

    // Main loop
    while (window.isOpen())
    {
        while (const std::optional<sf::Event> event = window.pollEvent())
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
                    selectedCell = sf::Vector2i(x, y);
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
                        pathNeedsUpdate = true;
                        break;
                    case sf::Keyboard::Key::T:
                        // Set the target point
                        targetCell = selectedCell;
                        pathNeedsUpdate = true;
                        break;
                    case sf::Keyboard::Key::O:
                        // Toggle obstacle
                        if (grid[selectedCell.y][selectedCell.x] == CellType::Empty)
                            grid[selectedCell.y][selectedCell.x] = CellType::Obstacle;
                        else
                            grid[selectedCell.y][selectedCell.x] = CellType::Empty;
                        pathNeedsUpdate = true;
                        break;
                    case sf::Keyboard::Key::E:
                        // Clear the cell
                        if (startCell == selectedCell)
                            startCell = sf::Vector2i(-1, -1);
                        if (targetCell == selectedCell)
                            targetCell = sf::Vector2i(-1, -1);
                        grid[selectedCell.y][selectedCell.x] = CellType::Empty;
                        pathNeedsUpdate = true;
                        break;
                    case sf::Keyboard::Key::R:
                        // Generate a new random map
                        GenerateRandomMap(grid, startCell, targetCell, rows, cols);
                        pathNeedsUpdate = true;
                        break;
                    default:
                        break;
                    }

                    // Reset the selected cell after action
                    selectedCell = sf::Vector2i(-1, -1);
                }
                else
                {
                    // If no cell is selected
                    if (keyEvent->code == sf::Keyboard::Key::R)
                    {
                        // Generate a new random map
                        GenerateRandomMap(grid, startCell, targetCell, rows, cols);
                        pathNeedsUpdate = true;
                    }
                    if (keyEvent->code == sf::Keyboard::Key::Space)
                    {
                        // Toggle robot movement
                        robotNeedsMove = !robotNeedsMove;
                    }
                    if (keyEvent->code == sf::Keyboard::Key::C)
                    {
                        // Reset robot movement variables
                        robotPathIndex = 0;
                        accumulatedDistance = 0.0f;
                        totalPathLength = 0.0f;
                        distanceCovered = 0.0f;
                    }
                }
            }
        }

        // Clear the window
        window.clear(sf::Color::White);

        // Draw obstacles
        for (int y = 0; y < rows; ++y)
        {
            for (int x = 0; x < cols; ++x)
            {
                if (grid[y][x] == CellType::Obstacle)
                {
                    sf::RectangleShape cellShape(sf::Vector2f(static_cast<float>(cellSize), static_cast<float>(cellSize)));
                    cellShape.setPosition(sf::Vector2f(static_cast<float>(x * cellSize), static_cast<float>(y * cellSize)));
                    cellShape.setFillColor(sf::Color::Black);
                    window.draw(cellShape);
                }
            }
        }

        // Draw start point
        if (startCell.x != -1 && startCell.y != -1)
        {
            sf::CircleShape startPoint(static_cast<float>(cellSize) / 2.0f);
            startPoint.setFillColor(sf::Color::Blue);
            startPoint.setPosition(sf::Vector2f(static_cast<float>(startCell.x * cellSize), static_cast<float>(startCell.y * cellSize)));
            window.draw(startPoint);
        }

        // Draw target point
        if (targetCell.x != -1 && targetCell.y != -1)
        {
            sf::CircleShape targetPoint(static_cast<float>(cellSize) / 2.0f);
            targetPoint.setFillColor(sf::Color::Red);
            targetPoint.setPosition(sf::Vector2f(static_cast<float>(targetCell.x * cellSize), static_cast<float>(targetCell.y * cellSize)));
            window.draw(targetPoint);
        }

        // Draw the path if start and target are set
        if (pathNeedsUpdate && startCell.x != -1 && targetCell.x != -1)
        {
            auto startTime = std::chrono::high_resolution_clock::now();

            path = GeneratePathAStar(grid, startCell, targetCell, cellSize, rows, cols);

			auto endTime = std::chrono::high_resolution_clock::now();
			pathGenerationTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
            window.setTitle("Grid Map - Path Generation Time: " + std::to_string(pathGenerationTime) + " ms");
            
            // Reset robot movement variables
            robotPathIndex = 0;
            accumulatedDistance = 0.0f;
            totalPathLength = 0.0f;
            distanceCovered = 0.0f;
            segmentLengths.clear();
            robotClock.restart();

            // Calculate segment lengths
            if (!path.empty())
            {
                for (size_t i = 0; i < path.size() - 1; ++i)
                {
                    float segmentLengthPixels = std::hypot(
                        path[i + 1].x - path[i].x,
                        path[i + 1].y - path[i].y
                    );
                    //float segmentLengthGrids = segmentLengthPixels / static_cast<float>(cellSize);
                    segmentLengths.push_back(segmentLengthPixels);
                    totalPathLength += segmentLengthPixels;
                }
            }

            pathNeedsUpdate = false;
        }

        // Draw the path
        if (!path.empty())
        {
            sf::VertexArray pathLines(sf::PrimitiveType::LineStrip, path.size());
            for (size_t i = 0; i < path.size(); ++i)
            {
                pathLines[i].position = path[i];
                pathLines[i].color = sf::Color::Green;
            }
            window.draw(pathLines);
        }

        // Update and draw the robot
        if (!path.empty() && !segmentLengths.empty())
        {
            float deltaTime = robotClock.restart().asSeconds();

            if (robotNeedsMove) {
                // Move the robot along the path
                float distanceToMove = robotSpeed * deltaTime;
                accumulatedDistance += distanceToMove;

                // Find the current segment based on accumulated distance
                while (robotPathIndex < segmentLengths.size() && accumulatedDistance > distanceCovered + segmentLengths[robotPathIndex])
                {
                    distanceCovered += segmentLengths[robotPathIndex];
                    ++robotPathIndex;
                }
            }
            
            /*
            // Check if robot has reached the end of the path
            if (robotPathIndex >= segmentLengths.size() - 1)
            {
                // Optionally, reset the robot to start again
                robotPathIndex = 0;
                accumulatedDistance = 0.0f;
                distanceCovered = 0.0f;
            }
            */

            // Interpolate the robot's position between the two points
            sf::Vector2f robotPosition;
            robotPosition.x = -1;
            robotPosition.y = -1;
            if (robotPathIndex <= segmentLengths.size() - 1)
            {
                float segmentProgress = (accumulatedDistance - distanceCovered) / segmentLengths[robotPathIndex];
                robotPosition = path[robotPathIndex] + (path[robotPathIndex + 1] - path[robotPathIndex]) * segmentProgress;
            }
            else
            {
                robotPosition = path.back();
            }

            // Draw the robot as a circle
            sf::CircleShape robotShape(static_cast<float>(cellSize) / 2.0f);
            robotShape.setFillColor(sf::Color::Magenta);
            robotShape.setPosition(robotPosition - sf::Vector2f(static_cast<float>(cellSize) / 2.0f, static_cast<float>(cellSize) / 2.0f));
            window.draw(robotShape);
        }

        // Highlight the selected cell
        if (selectedCell.x != -1 && selectedCell.y != -1)
        {
            sf::RectangleShape highlight(sf::Vector2f(static_cast<float>(cellSize), static_cast<float>(cellSize)));
            highlight.setPosition(sf::Vector2f(static_cast<float>(selectedCell.x * cellSize), static_cast<float>(selectedCell.y * cellSize)));
            highlight.setFillColor(sf::Color(255, 255, 0, 128)); // Semi-transparent yellow
            window.draw(highlight);
        }

        // Display the contents of the window
        window.display();
    }
}

// Function to generate random start, target, and obstacles
void GenerateRandomMap(std::vector<std::vector<CellType>>& grid,
                        sf::Vector2i& startCell,
                        sf::Vector2i& targetCell,
                        int rows,
                        int cols)
{
    // Reset grid
    for (int y = 0; y < rows; ++y)
    {
        std::fill(grid[y].begin(), grid[y].end(), CellType::Empty);
    }

    // Reset start and target positions
    startCell = sf::Vector2i(-1, -1);
    targetCell = sf::Vector2i(-1, -1);

    // Random number generators
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distRow(0, rows - 1);
    std::uniform_int_distribution<> distCol(0, cols - 1);

    // Generate shapes
    int numberOfShapes = 20; // Adjust the number of obstacle shapes
    for (int i = 0; i < numberOfShapes; ++i)
    {
        // Randomly choose a shape type
        int shapeType = i % 6; // 0: rectangle, 1: square, 2: circle, 3: ellipse, 4: U-shape, 5: V-shape, 6: triangle

        // Random position and size
        int posX = distCol(gen);
        int posY = distRow(gen);
        int sizeX = distCol(gen) % (cols / 4) + cols / 10;
        int sizeY = distRow(gen) % (rows / 4) + rows / 10;

        switch (shapeType)
        {
        case 0: // Rectangle
            for (int y = posY; y < posY + sizeY && y < rows; ++y)
                for (int x = posX; x < posX + sizeX && x < cols; ++x)
                    grid[y][x] = CellType::Obstacle;
            break;

        case 1: // Square
            sizeX = sizeY = std::min(sizeX, sizeY);
            for (int y = posY; y < posY + sizeY && y < rows; ++y)
                for (int x = posX; x < posX + sizeX && x < cols; ++x)
                    grid[y][x] = CellType::Obstacle;
            break;

        case 2: // Circle
        {
            int radius = std::min(sizeX, sizeY) / 2;
            int centerX = posX + radius;
            int centerY = posY + radius;
            for (int y = std::max(0, centerY - radius); y <= std::min(rows - 1, centerY + radius); ++y)
            {
                for (int x = std::max(0, centerX - radius); x <= std::min(cols - 1, centerX + radius); ++x)
                {
                    int dx = x - centerX;
                    int dy = y - centerY;
                    if (dx * dx + dy * dy <= radius * radius)
                        grid[y][x] = CellType::Obstacle;
                }
            }
        }
        break;

        case 3: // Ellipse
        {
            int radiusX = sizeX / 2;
            int radiusY = sizeY / 2;
            int centerX = posX + radiusX;
            int centerY = posY + radiusY;
            for (int y = std::max(0, centerY - radiusY); y <= std::min(rows - 1, centerY + radiusY); ++y)
            {
                for (int x = std::max(0, centerX - radiusX); x <= std::min(cols - 1, centerX + radiusX); ++x)
                {
                    int dx = x - centerX;
                    int dy = y - centerY;
                    if ((dx * dx) * (radiusY * radiusY) + (dy * dy) * (radiusX * radiusX) <= (radiusX * radiusX) * (radiusY * radiusY))
                        grid[y][x] = CellType::Obstacle;
                }
            }
        }
        break;

        case 4: // U-shape
            for (int y = posY; y < posY + sizeY && y < rows; ++y)
            {
                for (int x = posX; x < posX + sizeX && x < cols; ++x)
                {
                    if (x < posX + 2 || x >= posX + sizeX - 2 || (y >= posY + sizeY - 2))
                        grid[y][x] = CellType::Obstacle;
                }
            }
            break;

        //case 5: // V-shape
        //    for (int y = 0; y < sizeY && posY + y < rows; ++y)
        //    {
        //        int offset = y / 2;
        //        if (posX + offset < cols)
        //            grid[posY + y][posX + offset] = CellType::Obstacle;
        //        if (posX + sizeX - offset - 1 < cols)
        //            grid[posY + y][posX + sizeX - offset - 1] = CellType::Obstacle;
        //    }
        //    break;

        case 5: // Triangle
            for (int y = 0; y < sizeY && posY + y < rows; ++y)
            {
                int startX = posX + sizeX / 2 - y * sizeX / (2 * sizeY);
                int endX = posX + sizeX / 2 + y * sizeX / (2 * sizeY);
                for (int x = startX; x <= endX && x < cols; ++x)
                {
                    if (x >= 0)
                        grid[posY + y][x] = CellType::Obstacle;
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
    } while (grid[startCell.y][startCell.x] == CellType::Obstacle);

    // Random target position
    std::uniform_int_distribution<> distRowTarget(0, rows - 1);
    std::uniform_int_distribution<> distColTarget(0, cols - 1);
    do
    {
        targetCell.x = distColTarget(gen);
        targetCell.y = distRowTarget(gen);
    } while (grid[targetCell.y][targetCell.x] == CellType::Obstacle || targetCell == startCell);
}