// client.cpp

#include <SFML/Network.hpp>
#include <iostream>
#include <vector>
#include <iomanip>

int main()
{
    sf::TcpSocket socket;

    // Connect to the server
    if (socket.connect(sf::IpAddress(127, 0, 0, 1), 53000) != sf::Socket::Status::Done)
    {
        std::cerr << "Error connecting to server." << std::endl;
        return -1;
    }	

    // Send initial command to indicate this is a client
    sf::Packet commandPacket;
    int32_t command = 2; // Command 2 indicates a client
    commandPacket << command;

	if (socket.send(commandPacket) != sf::Socket::Status::Done)
	{
		std::cerr << "Error sending command to server." << std::endl;
		socket.disconnect();
		return -1;
	}

    std::cout << "Connected to server as client." << std::endl;

    while (true)
    {
        // Wait for data from the server
        sf::Packet dataPacket;
        sf::Socket::Status status = socket.receive(dataPacket);

        if (status == sf::Socket::Status::Disconnected)
        {
            std::cerr << "Disconnected from server." << std::endl;
            break;
        }
        else if (status != sf::Socket::Status::Done)
        {
            std::cerr << "Error receiving data from server." << std::endl;
            break;
        }

        std::cout << "Data received from server." << std::endl;
		int32_t rows, cols;
		dataPacket >> rows >> cols;
        std::cout << "Rows: " << rows << " Cols: " << cols << std::endl;

		uint32_t obstacleCount;
        dataPacket >> obstacleCount;

        std::vector<sf::Vector2i> obstaclePositions;
        for (uint32_t i = 0; i < obstacleCount; ++i)
        {
            int32_t x, y;
            dataPacket >> x >> y;
            obstaclePositions.emplace_back(x, y);
        }

		int32_t startX, startY, targetX, targetY;
		dataPacket >> startX >> startY >> targetX >> targetY;
        std::cout << "Start: (" << startX << ", " << startY << ") "
            << "Target: (" << targetX << ", " << targetY << ")" << std::endl;

		uint32_t pathSize;
		dataPacket >> pathSize;
		std::cout << "Path size: " << pathSize << std::endl;

        std::vector<sf::Vector2i> pathData;
        for (uint32_t i = 0; i < pathSize; ++i)
        {
            int32_t x, y;
            dataPacket >> x >> y;
            pathData.emplace_back(x, y);
        }

        // Create a grid initialized with empty spaces
        std::vector<std::vector<char>> grid(rows, std::vector<char>(cols, ' '));

        // Mark the obstacle positions
        for (const auto& pos : obstaclePositions)
        {
            if (pos.x >= 0 && pos.x < cols && pos.y >= 0 && pos.y < rows)
                grid[pos.y][pos.x] = '#'; // Use '#' to represent obstacles
        }

        // Mark the start and target positions
        if (startX >= 0 && startX < cols && startY >= 0 && startY < rows)
            grid[startY][startX] = 'S';

        if (targetX >= 0 && targetX < cols && targetY >= 0 && targetY < rows)
            grid[targetY][targetX] = 'T';

        // Mark the path positions
        for (const auto& pos : pathData)
        {
            if (pos.x >= 0 && pos.x < cols && pos.y >= 0 && pos.y < rows)
            {
                if (grid[pos.y][pos.x] == ' ' || grid[pos.y][pos.x] == '*')
                    grid[pos.y][pos.x] = '*'; // Use '*' to represent the path
            }
        }

        // Print the grid to console
        for (int y = 0; y < rows; ++y)
        {
            for (int x = 0; x < cols; ++x)
            {
                std::cout << std::setw(2) << grid[y][x];
            }
            std::cout << std::endl;
        }

        std::cout << "Waiting for new data..." << std::endl << std::endl;

    }

    // Disconnect from the server
    socket.disconnect();
    return 0;    
}
