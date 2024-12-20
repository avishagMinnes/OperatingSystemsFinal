#include <iostream>
#include <vector>
#include <list>
#include "IMSTSolver.cpp"

class BoruvkaSolver : public IMSTSolver {
public:
    std::list<std::pair<int, int>> solve(int numVertices,
                                         const std::vector<std::pair<int, std::pair<int, double>>>& edges) override {
        std::list<std::pair<int, int>> mstEdges;

        std::vector<int> component(numVertices);
        for (int i = 0; i < numVertices; ++i)
            component[i] = i;

        int numComponents = numVertices;

        while (numComponents > 1) {
            std::vector<int> cheapestEdge(numVertices, -1);

            for (int i = 0; i < edges.size(); ++i) {
                int u = edges[i].first;
                int v = edges[i].second.first;
                double weight = edges[i].second.second;

                if (component[u] != component[v]) {
                    if (cheapestEdge[component[u]] == -1 || weight < edges[cheapestEdge[component[u]]].second.second) {
                        cheapestEdge[component[u]] = i;
                    }
                    if (cheapestEdge[component[v]] == -1 || weight < edges[cheapestEdge[component[v]]].second.second) {
                        cheapestEdge[component[v]] = i;
                    }
                }
            }

            for (int i = 0; i < numVertices; ++i) {
                if (cheapestEdge[i] != -1) {
                    int u = edges[cheapestEdge[i]].first;
                    int v = edges[cheapestEdge[i]].second.first;
                    double weight = edges[cheapestEdge[i]].second.second;

                    if (component[u] != component[v]) {
                        mstEdges.push_back({u, v});
                        numComponents--;

                        int oldComponent = component[v], newComponent = component[u];
                        for (int j = 0; j < numVertices; ++j) {
                            if (component[j] == oldComponent)
                                component[j] = newComponent;
                        }
                    }
                }
            }
        }

        std::cout << "Boruvka's Algorithm executed\n";
        return mstEdges;
    }
};

