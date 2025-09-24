#include <iostream>
#include <set>
#include <cmath>

#include "data_structures.hpp"

polygon_t read_triangle() {
    std::vector<point_t> points(3);
    point_t point{};
    for (size_t i = 0; i < 3; i++) {
        std::cin >> point.x >> point.y >> point.z;
        if (!std::cin.good()) {
            std::cerr << "failed to read elem from cin\n";
            return polygon_t{{}};
        }
        points[i] = point;
    }
    return polygon_t{points};
}

int main() {
    uint triangles_amount;

    std::cin >> triangles_amount;
    if (!std::cin.good()) {
        std::cerr << "failed to read triangles amount from cin\n";
        return EXIT_FAILURE;
    }

    std::vector<polygon_t> triangles;
    triangles.reserve(triangles_amount);
    for (size_t i = 0; i < triangles_amount; i++) {
        triangles.push_back(read_triangle());
    }

    std::set<size_t> intersected;
    for (size_t i = 0; i < triangles_amount; i++) {
        for (size_t j = i + 1; j < triangles_amount; j++) {
            if (triangles[i].are_intersecting(triangles[j])) {
                intersected.insert(i);
                intersected.insert(j);
            }
        }
    }

    for (auto pos : intersected) {
        std::cout << pos 