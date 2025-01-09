// GridMap.h : Include file for standard system include files,
// or project specific include files.

#ifndef GRIDMAP_HEADER_HPP
#define GRIDMAP_HEADER_HPP

#include <iostream>
#include <vector>
#include <SFML/Graphics.hpp>
#include <random>

#include <chrono>

enum class CellType {
	Empty,
	Start,
	Target,
	Obstacle,
	Path
};

// Define directions for neighbor nodes (up, down, left, right)
const std::vector<sf::Vector2i> directions = {
	{ 1,  0}, // Right
	{-1,  0}, // Left
	{ 0,  1}, // Down
	{ 0, -1}, // Up
	{ 1,  1}, // Diagonal Right Down
	{-1,  1}, // Diagonal Left Down
	{ 1, -1}, // Diagonal Right Up
	{-1, -1}  // Diagonal Left Up
};

void GridMapVisualize(int cellSize,
                        int rows,
                        int cols);

void GenerateRandomMap(std::vector<std::vector<CellType>>& grid,
                        sf::Vector2i& startCell,
                        sf::Vector2i& targetCell,
                        int rows,
                        int cols);

std::vector<sf::Vector2f> GeneratePathAStar(const std::vector<std::vector<CellType>>& grid, 
                                            const sf::Vector2i& startCell, 
                                            const sf::Vector2i& targetCell, 
                                            int cellSize, 
                                            int rows, 
                                            int cols);


#endif // !GRIDMAP_HEADER_HPP

// TODO: Reference additional headers your program requires here.
