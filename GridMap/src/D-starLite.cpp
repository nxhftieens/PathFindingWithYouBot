#include "../include/D-starLite.hpp"


DStarLite::DStarLite(Position boundaries) : boundaries(boundaries)
{
    maxSteps = 100000;
    C1 = 1;
}

float DStarLite::keyHashCode(const State& u)
{
    return static_cast<float>(u.x + 1801 * u.y);
}

bool DStarLite::isValid(const State& u)
{
    OpenHash::iterator it = openHash.find(u);
    if (it == openHash.end())
        return false;
    if (!close(keyHashCode(u), it->second))
        return false;
    return true;
}

std::vector<Position> DStarLite::getPath()
{
    std::vector<Position> gridPath;
    for (const auto& pos : this->path)
    {
        gridPath.push_back(Position(pos.x, pos.y));
    }
    return gridPath;
}

bool DStarLite::occupied(const State& u)
{
    if (u.x < 0 || u.x >= boundaries.x || u.y < 0 || u.y >= boundaries.y)
        return true;
    CellHash::iterator it = cellHash.find(u);
    if (it == cellHash.end()) return false;
    return (it->second.cost < 0);
}

void DStarLite::initialize(const Position& start, const Position& target)
{
    cellHash.clear();
    path.clear();
    openHash.clear();
    while(!openList.empty())
        openList.pop();

    k_m = 0;
    s_start.x = start.x;
    s_start.y = start.y;
    s_target.x = target.x;
    s_target.y = target.y;

    CellInfo tmp;
    tmp.g = tmp.rhs = 0;
    tmp.cost = C1;

    cellHash[s_target] = tmp;

    tmp.g = tmp.rhs = heuristic(s_start, s_target);
    tmp.cost = C1;
    cellHash[s_start] = tmp;
    s_start = calculateKey(s_start);

    s_last = s_start;
}

void DStarLite::makeNewCell(const State& u)
{
    if (cellHash.find(u) != cellHash.end())
        return;
    
    CellInfo tmp;
    tmp.g = tmp.rhs = heuristic(u, s_target);
    tmp.cost = C1;
    cellHash[u] = tmp;
}

double DStarLite::getG(const State& u)
{
    if (cellHash.find(u) == cellHash.end())
        return heuristic(u, s_target);
    return cellHash[u].g;
}

double DStarLite::getRHS(const State& u)
{
    if (u == s_target)
        return 0;
    if (cellHash.find(u) == cellHash.end())
        return heuristic(u, s_target);
    return cellHash[u].rhs;
}

void DStarLite::setG(const State& u, const double& g)
{
    makeNewCell(u);
    cellHash[u].g = g;
}

void DStarLite::setRHS(const State& u, const double& rhs)
{
    makeNewCell(u);
    cellHash[u].rhs = rhs;
}

double DStarLite::eightCondist(const State& a, const State& b)
{
    double temp;
    double min = fabs(a.x - b.x);
    double max = fabs(a.y - b.y);
    if (min > max)
    {
        double temp = min;
        min = max;
        max = temp;
    }
    return ((M_SQRT2 - 1) * min + max);
}

int DStarLite::computeShortestPath()
{
	//std::cout << "Compute once" << std::endl;
    std::list<State> s;
    std::list<State>::iterator it;

    if (openList.empty())
        return 1;
    
    int k = 0;
    while (!openList.empty() && (openList.top() < (s_start = calculateKey(s_start))) || (getRHS(s_start) != getG(s_start)))
    {
        if (k++ > maxSteps)
        {
            std::cerr << "At max steps" << std::endl;
            return -1;
        }

        State u;

        bool test = (getRHS(s_start) != getG(s_start));

        while(1)
        {
            if (openList.empty())
                return 1;
            u = openList.top();
            openList.pop();

            if (!isValid(u))
                continue;
            if (!(u < s_start) && (!test))
                return 2;
            break;
        }

        OpenHash::iterator curr = openHash.find(u);
        openHash.erase(curr);

        State k_old = u;

        if (k_old < calculateKey(u))
        {
            insert(u);
        }
        else if (getG(u) > getRHS(u))
        {
            setG(u, getRHS(u));
            getPred(u, s);
            for (it = s.begin(); it != s.end(); it++)
            {
                updateVertex(*it);
            }
        }
        else 
        {
            setG(u, INFINITY);
            getPred(u, s);
            for (it = s.begin(); it != s.end(); it++)
            {
                updateVertex(*it);
            }
            updateVertex(u);
        }
    }
    return 0;
}

bool DStarLite::close(const double& x, const double& y)
{
    if (isinf(x) && isinf(y))
        return true;
    return (fabs(x - y) < 0.00001);
}

void DStarLite::updateVertex(const State& u)
{
    std::list<State> s;
    std::list<State>::iterator it;

    if (u != s_target)
    {
        getSucc(u, s);
        double tmp = INFINITY;
        double tmp2;

        for (it = s.begin(); it != s.end(); it++)
        {
            tmp2 = getG(*it) + cost(u, *it);
            if (tmp2 < tmp)
                tmp = tmp2;
        }
        if (!close(getRHS(u), tmp))
            setRHS(u, tmp);
    }

    if (!close(getG(u), getRHS(u)))
        insert(u);
}

void DStarLite::insert(State u)
{
    OpenHash::iterator it;
    float csum;

    u = calculateKey(u);
    it = openHash.find(u);
    csum = keyHashCode(u);

    // return if cell is already in list. TODO: this should be
    // uncommented except it introduces a bug, I suspect that there is a
    // bug somewhere else and having duplicates in the openList queue
    // hides the problem...
    //if ((it != openHash.end()) && (close(csum,it->second))) return;

    openHash[u] = csum;
    openList.push(u);
}

void DStarLite::remove(const State& u)
{
    OpenHash::iterator it = openHash.find(u);
    if (it == openHash.end())
    {
        return;
    }
    openHash.erase(it);
}

double DStarLite::trueDist(const State& a, const State& b)
{
    float x = a.x - b.x;
    float y = a.y - b.y;
    return sqrt(x * x + y * y);
}

double DStarLite::heuristic(const State& a, const State& b)
{
    return eightCondist(a, b);
}

State DStarLite::calculateKey(State u)
{
    double val = fmin(getG(u), getRHS(u));
    u.k.first = val + heuristic(u, s_start) + k_m;
    u.k.second = val;
    return u;
}

double DStarLite::cost(const State& a, const State& b)
{
    int xd = fabs(a.x - b.x);
    int yd = fabs(a.y - b.y);
    double scale = 1;

    if (xd + yd > 1)
        scale = M_SQRT2;
    
    if (cellHash.count(a) == 0) 
        return scale*C1;
    return scale*cellHash[a].cost;
}

void DStarLite::updateCell(const Position& pos, const double& val)
{
    State u;
    u.x = pos.x;
    u.y = pos.y;

    if ((u == s_start) || (u == s_target))
        return;
    
    makeNewCell(u);
    cellHash[u].cost = val;

    updateVertex(u);
}

void DStarLite::getSucc(State u, std::list<State>& s)
{
    s.clear();
    u.k.first = -1;
    u.k.second = -1;

    if (occupied(u))
        return;

    u.x += 1;
    s.push_front(u);
    u.y += 1;
    s.push_front(u);
    u.x -= 1;
    s.push_front(u);
    u.x -= 1;
    s.push_front(u);
    u.y -= 1;
    s.push_front(u);
    u.y -= 1;
    s.push_front(u);
    u.x += 1;
    s.push_front(u);
    u.x += 1;
    s.push_front(u);
}

void DStarLite::getPred(State u, std::list<State>& s)
{
    s.clear();
    u.k.first  = -1;
    u.k.second = -1;

    u.x += 1;
    if (!occupied(u)) s.push_front(u);
    u.y += 1;
    if (!occupied(u)) s.push_front(u);
    u.x -= 1;
    if (!occupied(u)) s.push_front(u);
    u.x -= 1;
    if (!occupied(u)) s.push_front(u);
    u.y -= 1;
    if (!occupied(u)) s.push_front(u);
    u.y -= 1;
    if (!occupied(u)) s.push_front(u);
    u.x += 1;
    if (!occupied(u)) s.push_front(u);
    u.x += 1;
    if (!occupied(u)) s.push_front(u);
}

void DStarLite::updateStart(const Position& start)
{
    s_start.x = start.x;
    s_start.y = start.y;

    k_m += heuristic(s_last, s_start);

    s_start = calculateKey(s_start);
    s_last = s_start;
}

void DStarLite::updateTarget(const Position& target)
{
    std::list<std::pair<Position, double>> toAdd;
    std::pair<Position, double> tmp;

    CellHash::iterator it;
    std::list<std::pair<Position, double>>::iterator kk;

    for (it = cellHash.begin(); it != cellHash.end(); it++)
    {
        if (!close(it->second.cost, C1))
        {
            tmp.first.x = it->first.x;
            tmp.first.y = it->first.y;
            tmp.second = it->second.cost;
            toAdd.push_back(tmp);
        }
    }

    cellHash.clear();
    openHash.clear();

    while(!openList.empty())
        openList.pop();
    
    k_m = 0;

    s_target.x = target.x;
    s_target.y = target.y;

    CellInfo tmp2;
    tmp2.g = tmp2.rhs = 0;
    tmp2.cost = C1;

    cellHash[s_target] = tmp2;

    tmp2.g = tmp2.rhs = heuristic(s_start, s_target);
    tmp2.cost = C1;
    cellHash[s_start] = tmp2;
    s_start = calculateKey(s_start);

    s_last = s_start;

    for (kk = toAdd.begin(); kk != toAdd.end(); kk++)
    {
        updateCell(kk->first, kk->second);
    }
}

bool DStarLite::replan()
{
    path.clear();

    int res = computeShortestPath();

    if (res < 0)
    {
        std::cerr << "NO PATH FOUND" << std::endl;
        return false;
    }

    std::list<State> n;
    std::list<State>::iterator it;

    State curr = s_start;

    if (isinf(getG(s_start)))
    {
        std::cerr << "NO PATH FOUND" << std::endl;
        return false;
    }

    while (curr != s_target)
    {
        path.push_back(curr);
        getSucc(curr, n);

        if (n.empty())
        {
            std::cerr << "NO PATH FOUND" << std::endl;
            return false;
        }

        double cmin = INFINITY;
        double tmin = INFINITY;
        State smin;

        for (it = n.begin(); it != n.end(); it++)
        {
            double val = cost(curr, *it);
            double val2 = trueDist(*it, s_target) + trueDist(s_start, *it);
            val += getG(*it);

            if (close(val, cmin))
            {
                if (tmin > val2)
                {
                    tmin = val2;
                    cmin = val;
                    smin = *it;
                }
            }
            else if (val < cmin)
            {
                tmin = val2;
                cmin = val;
                smin = *it;
            }
        }
        n.clear();        
        curr = smin;
    }
    path.push_back(s_target);
    return true;
}