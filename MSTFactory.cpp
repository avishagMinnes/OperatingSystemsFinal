#include <memory>
#include <string>
#include <vector>
#include <list>
#include <iostream>
#include "IMSTSolver.cpp"
#include "BoruvkaSolver.cpp"
#include "PrimSolver.cpp"

class MSTFactory {
public:
    static std::unique_ptr<IMSTSolver> createSolver(const std::string& algorithmType) {
        if (algorithmType == "Boruvka") {
            return std::make_unique<BoruvkaSolver>();
        } else if (algorithmType == "Prim") {
            return std::make_unique<PrimSolver>();
        } else {
            throw std::invalid_argument("Unknown algorithm type.");
        }
    }
};

//int main() {
//    int numVertices = 5;
//    std::vector<std::pair<int, std::pair<int, double>>> edges = {
//            {0, {1, 10.0}}, {0, {3, 5.0}}, {1, {2, 1.0}}, {3, {4, 2.0}}, {2, {4, 3.0}}
//    };
//
//    try {
//        // Example: Using Boruvka's algorithm
//        std::unique_ptr<IMSTSolver> solver = MSTFactory::createSolver("Boruvka");
//        std::list<std::pair<int, int>> mst = solver->solve(numVertices, edges);
//
//        std::cout << "MST Edges (Boruvka):\n";
//        for (const auto& edge : mst) {
//            std::cout << edge.first << " - " << edge.second << "\n";
//        }
//
//        // Example: Using Prim's algorithm
//        solver = MSTFactory::createSolver("Prim");
//        mst = solver->solve(numVertices, edges);
//
//        std::cout << "MST Edges (Prim):\n";
//        for (const auto& edge : mst) {
//            std::cout << edge.first << " - " << edge.second << "\n";
//        }
//
//    } catch (const std::exception& e) {
//        std::cerr << "Error: " << e.what() << std::endl;
//    }
//
//    return 0;
//}


