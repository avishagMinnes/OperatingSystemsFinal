#include "Vertex.cpp"

class Edge {
public:
    Vertex source;
    Vertex destination;
    double weight;

    Edge(Vertex source, Vertex destination, double weight)
            : source(source), destination(destination), weight(weight) {}

    // Compare edges based on weight (useful for Kruskal's algorithm)
    bool operator<(const Edge& other) const {
        return this->weight < other.weight;
    }

    // For debug purposes
    friend std::ostream& operator<<(std::ostream& os, const Edge& edge) {
        os << "Edge(Source: " << edge.source.id
           << ", Destination: " << edge.destination.id
           << ", Weight: " << edge.weight << ")";
        return os;
    }
};

