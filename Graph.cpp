#include <unordered_map>
#include <vector>
#include <list>
#include <iostream>
#include "Edge.cpp"

class Graph {
private:
    // Adjacency list: each vertex maps to a list of pairs (neighbor vertex, weight)
    std::unordered_map<int, std::list<std::pair<int, double>>> adjList;
    int numVertices;
    int numEdges;

public:
    Graph(int numVertices) : numVertices(numVertices), numEdges(0) {}

    // Add a directed edge from vertex u to vertex v with weight w
    void addEdge(int u, int v, double w) {
        adjList[u].push_back(std::make_pair(v, w));
        numEdges++;
    }

    // Remove an edge from u to v
    void removeEdge(int u, int v) {
        auto& neighbors = adjList[u];
        neighbors.remove_if([v](const std::pair<int, double>& edge) {
            return edge.first == v;
        });
        numEdges--;
    }

    // Update the weight of an edge
    void updateEdgeWeight(int u, int v, double newWeight) {
        auto& neighbors = adjList[u];
        for (auto& neighbor : neighbors) {
            if (neighbor.first == v) {
                neighbor.second = newWeight;
                return;
            }
        }
        std::cerr << "Edge from " << u << " to " << v << " not found." << std::endl;
    }

    // Check if an edge exists
    bool edgeExists(int u, int v) const {
        if (adjList.find(u) != adjList.end()) {
            for (const auto& neighbor : adjList.at(u)) {
                if (neighbor.first == v) {
                    return true;
                }
            }
        }
        return false;
    }

    // Add a vertex to the graph
    void addVertex(int id) {
        if (adjList.find(id) == adjList.end()) {
            adjList[id] = std::list<std::pair<int, double>>();
            numVertices++;
        }
    }

    // Remove a vertex from the graph
    void removeVertex(int id) {
        adjList.erase(id);
        for (auto& [vertex, neighbors] : adjList) {
            neighbors.remove_if([id](const std::pair<int, double>& edge) {
                return edge.first == id;
            });
        }
        numVertices--;
    }

    // Get all edges in the graph as a vector of pairs
    std::vector<std::pair<int, std::pair<int, double>>> getEdges() const {
        std::vector<std::pair<int, std::pair<int, double>>> edges;
        for (const auto& [u, neighbors] : adjList) {
            for (const auto& [v, weight] : neighbors) {
                edges.push_back({u, {v, weight}});
            }
        }
        return edges;
    }

    // Get all the neighbors of a given vertex
    std::list<std::pair<int, double>> getNeighbors(int u) const {
        if (adjList.find(u) != adjList.end()) {
            return adjList.at(u);
        }
        return std::list<std::pair<int, double>>();  // Return empty list if u is not found
    }

    // Get the number of vertices in the graph
    int getNumVertices() const {
        return numVertices;
    }

    // Get the number of edges in the graph
    int getNumEdges() const {
        return numEdges;
    }

    // Print the graph for debugging
    void printGraph() const {
        for (const auto& [vertex, neighbors] : adjList) {
            std::cout << "Vertex " << vertex << " -> ";
            for (const auto& [neighbor, weight] : neighbors) {
                std::cout << "(" << neighbor << ", " << weight << ") ";
            }
            std::cout << std::endl;
        }
    }
};

// Example usage
int main() {
    Graph graph(5);  // Create a graph with 5 vertices

    // Add directed edges with weights
    graph.addEdge(0, 1, 10.0);
    graph.addEdge(0, 3, 5.0);
    graph.addEdge(1, 2, 1.0);
    graph.addEdge(3, 4, 2.0);

    // Print the edges
    std::vector<std::pair<int, std::pair<int, double>>> edges = graph.getEdges();
    std::cout << "Edges in the graph:" << std::endl;
    for (const auto& edge : edges) {
        std::cout << edge.first << " -> " << edge.second.first 
                  << " (weight: " << edge.second.second << ")" << std::endl;
    }

    // Update edge weight
    graph.updateEdgeWeight(0, 3, 8.0);

    // Check if an edge exists
    std::cout << "Edge 0 -> 3 exists: " << graph.edgeExists(0, 3) << std::endl;

    // Print the graph
    graph.printGraph();

    return 0;
}


