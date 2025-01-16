#ifndef SERVER_HEADER_HPP
#define SERVER_HEADER_HPP

#include <SFML/Network.hpp>
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>
#include <list>


void handleClient(sf::TcpSocket* client);
void handlePublisher(sf::TcpSocket* publisher);


#endif // SERVER_HEADER_HPP