#ifndef DSTARLITE_HEADER_HPP
#define DSTARLITE_HEADER_HPP

#define _USE_MATH_DEFINES

#include <vector>
#include <queue>
#include <unordered_map>
#include <list>
#include <cmath>
#include <iostream>

#include "../include/Element.hpp"

struct State {
    int x;
    int y;
    std::pair<double, double> k;

    bool operator== (const State& other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator!= (const State& other) const
    {
        return x != other.x || y != other.y;
    }

    bool operator> (const State& other) const
    {
        if (k.first-0.00001 > other.k.first)
            return true;
        else if (k.first < other.k.first-0.00001)
                return false;
        return k.second > other.k.second;
    }

    bool operator<= (const State& other) const
    {
        if (k.first < other.k.first)
            return true;
        else if (k.first > other.k.first)
            return false;
        return k.second <= other.k.second + 0.00001;
    }

    bool operator< (const State& other) const
    {
        if (k.first + 0.00001 < other.k.first)
            return true;
        else if (k.first - 0.00001 > other.k.first)
            return false;
        return k.second < other.k.second;
    }
};

struct CellInfo
{
    double g;
    double rhs;
    double cost;
};

struct State_hash
{
    size_t operator() (const State& other) const
    {
        return other.x + 180120 * other.y;
    }
};

typedef std::priority_queue<State, std::vector<State>, std::greater<State>> PriorityQueue;
typedef std::unordered_map<State, CellInfo, State_hash, std::equal_to<State>> CellHash;
typedef std::unordered_map<State, float, State_hash, std::equal_to<State>> OpenHash;

class DStarLite
{
public:

    DStarLite(Position boundaries);
    void initialize(const Position& start, const Position& target);
    void updateCell(const Position& pos, const double& val);
    void updateStart(const Position& start);
    void updateTarget(const Position& target);
    bool replan();
    std::vector<Position> getPath();
    
private:
    
    std::list<State> path;
	Position boundaries;

    double C1;
    double k_m;
    State s_start, s_target, s_last;
    int maxSteps;

    PriorityQueue openList;
    OpenHash openHash;
    CellHash cellHash;

    bool close(const double& x, const double& y);
    void makeNewCell(const State& u);
    double getG(const State& u);
    double getRHS(const State& u);
    void setG(const State& u, const double& g);
    void setRHS(const State& u, const double& rhs);
    double eightCondist(const State& a, const State& b);
    int computeShortestPath();
    void updateVertex(const State& u);
    void insert(State u);
    void remove(const State& u);
    double trueDist(const State& a, const State& b);
    double heuristic(const State& a, const State& b);
    State calculateKey(State u);
    void getSucc(State u, std::list<State>& s);
    void getPred(State u, std::list<State>& s);
    double cost(const State& a, const State& b);
    bool occupied(const State& u);
    bool isValid(const State& u);
    float keyHashCode(const State& u);
};

#endif // !DSTARLITE_HEADER_HPP
