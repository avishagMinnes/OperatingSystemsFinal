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

