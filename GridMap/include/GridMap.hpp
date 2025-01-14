﻿// GridMap.h : Include file for standard system include files,
// or project specific include files.

#ifndef GRIDMAP_HEADER_HPP
#define GRIDMAP_HEADER_HPP

#include <iostream>
#include <vector>
#include <SFML/Graphics.hpp>
#include <random>

#include <chrono>

#include "../include/Element.hpp"
#include"../include/A-star.hpp"
#include"../include/D-starLite.hpp"
#include <cmath>

class Robot
{
private:
    Position pos;
    float speed;
    sf::Clock robotClock;
    std::vector<float> segmentLengths;
    float totalPathLength;
    float accumulatedDistance;
    float visibleRadius;
	bool sawObstacle = false;
	Position obstaclePos;
	bool moving = false;

public:
    Robot(const Position& pos, const float& speed, const float& visibleRadius);
    void calculatePathLength(const std::vector<sf::Vector2f>& path);
    void resetPathLength();
	void movingFlagToggle()
	{
		moving = !moving;
	}
    void move(const std::vector<sf::Vector2f>& pixelPath);
    void draw(sf::RenderWindow& window, const int& cellSize);
    void updatePos(const Position& start, const int& cellSize);
	void lookAhead(std::vector<std::vector<CellType>>& gridMap,const std::vector<sf::Vector2f>& pixelPath, const std::vector<Position>& gridPath, const int& cellSize);

    Position getObstaclePos() const
	{
		return obstaclePos;
	}
	bool foundObstacle() const
	{
		return sawObstacle;
	}
	Position getGridPosition(const int& cellSize) const
	{
		Position gridPos;
		gridPos.x = static_cast<int>(pos.x / static_cast<float>(cellSize));
		gridPos.y = static_cast<int>(pos.y / static_cast<float>(cellSize));
		return gridPos;
	}
	void resetObstacleFlag()
	{
		sawObstacle = false;
	}
};

class GridMap
{
private:
    std::vector<std::vector<CellType>> gridMap;
    Position startCell, targetCell;
    int cellSize, rows, cols;
    std::vector<sf::Vector2f> pixelPath;
	std::vector<Position> gridPath;
    
    void eventHandler(sf::RenderWindow& window, Position& selectedCell, Robot& robot, DStarLite& dstarlite);

public:
    GridMap(const int& cellSize, const int& rows, const int& cols);
    void visualize();
    void randomize();
    void getPixelPath(AStar& astar);
	void getPixelPath(DStarLite& dstarlite, Robot& robot);
};


#endif // !GRIDMAP_HEADER_HPP

// TODO: Reference additional headers your program requires here.
