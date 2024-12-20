#include <unordered_map>
#include <vector>
#include <algorithm>
#include <limits>
#include <list>
#include <queue>
#include <iostream>

class MSTTree {
private:
    // Adjacency list for the tree
    std::unordered_map<int, std::list<std::pair<int, double>>> treeAdjList;
    int numVertices;
    double totalWeight;

public:
    MSTTree(int numVertices) : numVertices(numVertices), totalWeight(0.0) {}

    // Add an edge to the tree
    void addEdge(int u, int v, double w) {
        treeAdjList[u].push_back(std::make_pair(v, w));
        treeAdjList[v].push_back(std::make_pair(u, w));  // Since this is undirected MST
        totalWeight += w;
    }

    // Get total weight of the tree (sum of all edge weights)
    double getTotalWeight() const {
        return totalWeight;
    }

    // Calculate the longest distance between two vertices (tree diameter)
    double findTreeDiameter() {
        // First DFS to find the farthest node from an arbitrary start node
        int farthestNode = 0;
        double maxDistance = 0.0;
        std::vector<bool> visited(numVertices, false);
        dfs(0, -1, 0.0, visited, maxDistance, farthestNode);

        // Reset visited array for second DFS
        std::fill(visited.begin(), visited.end(), false);

        // Second DFS from the farthest node to calculate the diameter
        maxDistance = 0.0;
        dfs(farthestNode, -1, 0.0, visited, maxDistance);
        return maxDistance;
    }

    // Calculate the shortest distance between two vertices (using BFS)
    double shortestDistanceBetweenTwoVertices(int start, int end) {
        if (start == end) {
            return 0.0;  // distance from a vertex to itself is 0
        }

        std::vector<double> dist(numVertices, std::numeric_limits<double>::infinity());
        std::vector<bool> visited(numVertices, false);
        std::queue<int> q;

        dist[start] = 0.0;
        visited[start] = true;
        q.push(start);

        while (!q.empty()) {
            int u = q.front();
            q.pop();

            for (const auto& neighbor : treeAdjList[u]) {
                int v = neighbor.first;
                double w = neighbor.second;

                if (!visited[v]) {
                    double newDist = dist[u] + w;
                    if (newDist < dist[v]) {
                        dist[v] = newDist;
                    }
                    visited[v] = true;
                    q.push(v);
                }
            }
        }

        return dist[end];
    }

    // Calculate the average distance between all pairs of vertices
    double averageDistanceBetweenVertices() {
        double totalDistance = 0.0;
        int count = 0;

        for (int i = 0; i < numVertices; ++i) {
            for (int j = i + 1; j < numVertices; ++j) {
                double dist = shortestDistanceBetweenTwoVertices(i, j);
                totalDistance += dist;
                count++;
            }
        }

        return count > 0 ? totalDistance / count : 0.0;
    }

    // Debug utility to print the tree
    void printTree() const {
        for (const auto& [vertex, neighbors] : treeAdjList) {
            std::cout << "Vertex " << vertex << " -> ";
            for (const auto& [neighbor, weight] : neighbors) {
                std::cout << "(" << neighbor << ", " << weight << ") ";
            }
            std::cout << std::endl;
        }
    }

private:
    // Helper DFS function
    void dfs(int u, int parent, double currentDistance, std::vector<bool>& visited, double& maxDistance, int& farthestNode) {
        visited[u] = true;
        if (currentDistance > maxDistance) {
            maxDistance = currentDistance;
            farthestNode = u;
        }
        for (const auto& neighbor : treeAdjList[u]) {
            int v = neighbor.first;
            double w = neighbor.second;
            if (v != parent && !visited[v]) {
                dfs(v, u, currentDistance + w, visited, maxDistance, farthestNode);
            }
        }
    }

    // Overloaded DFS function for tree diameter calculation
    void dfs(int u, int parent, double currentDistance, std::vector<bool>& visited, double& maxDistance) {
        visited[u] = true;
        maxDistance = std::max(maxDistance, currentDistance);

        for (const auto& neighbor : treeAdjList[u]) {
            int v = neighbor.first;
            double w = neighbor.second;
            if (v != parent && !visited[v]) {
                dfs(v, u, currentDistance + w, visited, maxDistance);
            }
        }
    }
};

// Example usage
//int main() {
//    MSTTree mstTree(5);  // Create an MST tree with 5 vertices
//
//    // Add edges to the MST
//    mstTree.addEdge(0, 1, 10.0);
//    mstTree.addEdge(0, 3, 5.0);
//    mstTree.addEdge(1, 2, 1.0);
//    mstTree.addEdge(3, 4, 2.0);
//
//    // Print the tree
//    std::cout << "Tree structure:" << std::endl;
//    mstTree.printTree();
//
//    // Get total weight of the MST
//    std::cout << "Total weight of MST: " << mstTree.getTotalWeight() << std::endl;
//
//    // Calculate longest distance between two vertices
//    double longestDistance = mstTree.findTreeDiameter();
//    std::cout << "Longest distance between two vertices: " << longestDistance << std::endl;
//
//    // Calculate average distance between all pairs of vertices
//    double avgDistance = mstTree.averageDistanceBetweenVertices();
//    std::cout << "Average distance between vertices: " << avgDistance << std::endl;
//
//    return 0;
//}



