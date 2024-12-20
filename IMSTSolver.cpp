#pragma once

#include <vector>
#include <list>
#include <utility> // for std::pair

/**
 * Interface for Minimum Spanning Tree (MST) Solvers.
 *
 * @param numVertices: Number of vertices in the graph.
 * @param edges: A vector of edges represented as (source, (destination, weight)).
 * @return A list of edges (source, destination) forming the MST.
 */
class IMSTSolver {
public:
    virtual std::list<std::pair<int, int>> solve(
            int numVertices,
            const std::vector<std::pair<int, std::pair<int, double>>>& edges) = 0;

    virtual ~IMSTSolver() = default;
};



