class Vertex {
public:
    int id;

    Vertex(int id) : id(id) {}

    bool operator==(const Vertex& other) const {
        return this->id == other.id;
    }

    // For debug purposes
    friend std::ostream& operator<<(std::ostream& os, const Vertex& vertex) {
        os << "Vertex(ID: " << vertex.id << ")";
        return os;
    }
};
