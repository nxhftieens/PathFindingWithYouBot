// server.cpp
#include "../include/Server.hpp"

std::mutex dataMutex;
std::condition_variable dataCondition;
bool newDataAvailable = false;

int32_t rows = 0, cols = 0;
int32_t startX = -1, startY = -1, targetX = -1, targetY = -1;
std::vector<sf::Vector2i> pathData;
std::vector<sf::Vector2i> obstaclePositions;

// List of connected clients
std::list<sf::TcpSocket*> clients;


void handleClient(sf::TcpSocket* client)
{

    std::cout << "Client connected." << std::endl;

    while (true)
    { 
        // Wait for new data to be available
        std::unique_lock<std::mutex> lock(dataMutex);
        dataCondition.wait(lock, [] { return newDataAvailable; });

        // Prepare the packet to send
        sf::Packet sendPacket;

        // Send grid map size
        sendPacket << rows << cols;

        // Send obstacle data
        sendPacket << static_cast<uint32_t>(obstaclePositions.size());
        for (const auto& pos : obstaclePositions)
        {
            sendPacket << pos.x << pos.y;
        }

        // Send start and target positions
        sendPacket << startX << startY << targetX << targetY;

        // Send path size
        sendPacket << static_cast<uint32_t>(pathData.size());

        // Send path data
        for (const auto& pos : pathData)
        {
            sendPacket << pos.x << pos.y;
        }

        // Send the packet to the client
        if (client->send(sendPacket) != sf::Socket::Status::Done)
        {
            std::cerr << "Error sending data to client." << std::endl;
            // Remove client from the list and exit the loop
            clients.remove(client);
            client->disconnect();
            delete client;
            break;
        }
        else
        {
            std::cout << "Data sent to client." << std::endl;
        }

        // Reset the new data flag
        newDataAvailable = false;
    }    
}

// Function to handle publisher connections
void handlePublisher(sf::TcpSocket* publisher)
{
    std::cout << "Publisher connected." << std::endl;

    while (true)
    {
        // Receive data from the publisher
        sf::Packet packet;
        if (publisher->receive(packet) != sf::Socket::Status::Done)
        {
            std::cerr << "Error receiving data from publisher. Disconnecting publisher." << std::endl;
            publisher->disconnect();
            delete publisher;
            break;
        }

        // Lock the mutex to update data
        {
            std::lock_guard<std::mutex> lock(dataMutex);

            // Receive grid map size
            packet >> rows >> cols;

            // Receive obstacle data
            uint32_t obstacleCount;
            packet >> obstacleCount;

            obstaclePositions.clear();
            for (uint32_t i = 0; i < obstacleCount; ++i)
            {
                int32_t x, y;
                packet >> x >> y;
                obstaclePositions.emplace_back(x, y);
            }

            // Receive start and target positions
            packet >> startX >> startY >> targetX >> targetY;

            // Receive path size
            uint32_t pathSize;
            packet >> pathSize;

            // Receive path data
            pathData.clear();
            for (uint32_t i = 0; i < pathSize; ++i)
            {
                int32_t x, y;
                packet >> x >> y;
                pathData.emplace_back(x, y);
            }

            std::cout << "Data received from publisher." << std::endl;

            // Set the flag and notify all clients
            newDataAvailable = true;
        }

        dataCondition.notify_all(); // Notify all clients that new data is available
    }
}

int main()
{
    sf::TcpListener listener;

    // Bind the listener to a port
    if (listener.listen(53000) != sf::Socket::Status::Done)
    {
        std::cerr << "Error binding the listener to port." << std::endl;
        return -1;
    }

    std::cout << "Server is listening on port 53000..." << std::endl;


    while (true)
    {
        // Accept new connections
        sf::TcpSocket* socket = new sf::TcpSocket;
        if (listener.accept(*socket) == sf::Socket::Status::Done)
        {
            // Receive the initial command to determine if it's a publisher or client
            sf::Packet packet;
            if (socket->receive(packet) != sf::Socket::Status::Done)
            {
                std::cerr << "Error receiving initial command. Disconnecting socket." << std::endl;
                socket->disconnect();
                delete socket;
                continue;
            }

            int32_t command;
            packet >> command;

            if (command == 1) // Publisher
            {
                // Handle the publisher in a separate thread
                std::thread(&handlePublisher, socket).detach();
            }
            else if (command == 2) // Client
            {
                // Add the client to the list
                {
                    std::lock_guard<std::mutex> lock(dataMutex);
                    clients.push_back(socket);
                }

                // Handle the client in a separate thread
                std::thread(&handleClient, socket).detach();
            }
            else
            {
                std::cerr << "Unknown command. Disconnecting socket." << std::endl;
                socket->disconnect();
                delete socket;
            }
        }
        else
        {
            delete socket;
        }
    }
    return 0;
}
