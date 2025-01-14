#ifndef ELEMENT_HEADER_HPP
#define ELEMENT_HEADER_HPP

#include <vector>

enum class CellType {
	Empty,
	Start,
	Target,
	Obstacle,
};

struct Position {
	int x;
	int y;
    bool operator==(const Position& other) const
    {
        return x == other.x && y == other.y;
    }
    bool operator !=(const Position& other) const
    {
        return x != other.x || y != other.y;
    }
    Position() : x(-1), y(-1) {}
    Position(const int x, const int y) : x(x), y(y) {}
    Position(const float x, const float y) : x(static_cast<int>(x)), y(static_cast<int>(y)) {}
};

const std::vector<Position> directions = {
	{ 1,  0}, // Right
	{-1,  0}, // Left
	{ 0,  1}, // Down
	{ 0, -1}, // Up
	{ 1,  1}, // Diagonal Right Down
	{-1,  1}, // Diagonal Left Down
	{ 1, -1}, // Diagonal Right Up
	{-1, -1}  // Diagonal Left Up
};

#endif // !ELEMENT_HEADER_HPP