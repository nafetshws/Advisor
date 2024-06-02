#include <iostream>
#include <string>
#include "../../include/mms.hpp"
#include "../../include/floodfill.hpp"

// with help of https://www.geeksforgeeks.org/shortest-path-unweighted-graph/

void optimisePath()
{
    g.bfsInit();
    g.print(Maze::endCell, Maze::startCell);
}

void Graph::bfsInit()
{
    for (auto edge : edges)
    {
        graph[edge.first].push_back(edge.second);
        graph[edge.second].push_back(edge.first);
    }

    for (Cell *cell : vertices)
    {
        // std::cerr << "c(" << (int)cell->x << "," << (int)cell->y << ")" << std::endl;
        MMS::setColor((int)cell->x, (int)cell->y, 'c');
        dist[cell] = 1e9;
    }
};

void Graph::print(Cell *S, Cell *D)
{

    Graph::bfs(S);

    std::vector<Cell *> path;
    Cell *currentNode = D;
    path.push_back(D);

    while (currentNode != NULL)
    {
        path.push_back(par[currentNode]);
        currentNode = par[currentNode];
    }

    for (auto cell : path)
    {
        // std::cerr << "(" << (int)cell->x << "," << (int)cell->y << ")" << std::endl;
        MMS::setColor((int)cell->x, (int)cell->y, 'b');
    }
    std::cerr << "Laenge des kuerzesten Pfades: " << path.size() << std::endl;
};

void Graph::bfs(Cell *S)
{
    std::queue<Cell *> q;
    dist[S] = 0;
    q.push(S);

    while (!q.empty())
    {
        Cell *node = q.front();
        // std::cerr << "ddd(" << (int)node->x << "," << (int)node->y << ")" << std::endl;
        q.pop();

        for (auto neighbor : graph[node])
        {
            if (dist[neighbor] == 1e9)
            {
                par[neighbor] = node;
                dist[neighbor] = dist[node] + 1;
                q.push(neighbor);
            }
        }
    }
}