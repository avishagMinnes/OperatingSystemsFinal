#include <queue>
#include <vector>
#include <iostream>
#include "IMSTSolver.cpp"
#include <limits>

class PrimSolver : public IMSTSolver {
public:
    std::list<std::pair<int, int>> solve(int numVertices,
                                         const std::vector<std::pair<int, std::pair<int, double>>>& edges) override {
        std::list<std::pair<int, int>> mstEdges;

        auto adjList = buildAdjacencyList(numVertices, edges);

        std::vector<bool> inMST(numVertices, false);
        std::vector<double> key(numVertices, std::numeric_limits<double>::infinity());
        std::vector<int> parent(numVertices, -1);

        key[0] = 0;
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
        pq.push({0, 0});

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();
            inMST[u] = true;

            for (const auto& neighbor : adjList[u]) {
                int v = neighbor.first;
                double weight = neighbor.second;

                if (!inMST[v] && weight < key[v]) {
                    key[v] = weight;
                    pq.push({key[v], v});
                    parent[v] = u;
                }
            }
        }

        for (int i = 1; i < numVertices; ++i) {
            if (parent[i] != -1) {
                mstEdges.push_back({parent[i], i});
            }
        }

        std::cout << "Prim's Algorithm executed\n";
        return mstEdges;
    }

private:
    std::vector<std::vector<std::pair<int, double>>> buildAdjacencyList(
            int numVertices,
            const std::vector<std::pair<int, std::pair<int, double>>>& edges) {
        std::vector<std::vector<std::pair<int, double>>> adjList(numVertices);
        for (const auto& edge : edges) {
            int u = edge.first;
            int v = edge.second.first;
            double w = edge.second.second;
            adjList[u].push_back({v, w});
            adjList[v].push_back({u, w});
        }
        return adjList;
    }
};

