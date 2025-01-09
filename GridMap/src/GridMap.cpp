// GridMap.cpp : Defines the entry point for the application.
//

#include "../include/GridMap.hpp"

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
void GridMapVisualize(int cellSize, int rows, int cols)
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

            path = GeneratePath(grid, startCell, targetCell, cellSize, rows, cols);

			auto endTime = std::chrono::high_resolution_clock::now();
			pathGenerationTime = std::chrono::duration<double, std::milli>(endTime - startTime).count();
            window.setTitle("Grid Map - Path Generation Time: " + std::to_string(pathGenerationTime) + " ms");
            
            pathNeedsUpdate = false;
        }

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

// Function to generate path using A* algorithm
std::vector<sf::Vector2f> GeneratePath(const std::vector<std::vector<CellType>>& grid, const sf::Vector2i& startCell, const sf::Vector2i& targetCell, int cellSize, int rows, int cols)
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
			float hCost = dx + dy + (1.414 - 2) * std::min(dx, dy); // Diagonal distance heuristic

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

// Function to generate random start, target, and obstacles
void GenerateRandomMap(std::vector<std::vector<CellType>>& grid, sf::Vector2i& startCell, sf::Vector2i& targetCell, int rows, int cols)
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