#include <iostream>
#include <vector>
#include <gtest/gtest.h>

#include "data_structures.hpp"

void check_point(const point_t& p, const plane_t& plane) {
    double val = plane.a * p.x + plane.b * p.y + plane.c * p.z + plane.d;
    EXPECT_NEAR(val, 0.0, 1e-6);
}

TEST(PlaneTest, PlaneConstructor) {
    struct PlaneTestData {
        point_t p1, p2, p3;
    };

    PlaneTestData test_cases[] = {
        {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}},
        {{0, 0, 1}, {1, 0, 1}, {0, 1, 1}},
        {{1, 2, 1}, {2, 0, 1}, {0, 2, 1}},
        {{12, 20.120, 1.002}, {10.09, -15.33, 9}, {0.3, 0.888, 1.123}},
        {{1999, -887, -0.0001}, {100.13, 288.88, 885}, {-9854, 9.888, 3.13}},
        {{9.000001, 0.0000001, 0.0000001}, {0.0000003, 0.0000003, 9}, {0.0000001, 0.0000001, 0.0000001}}
    };

    for (const auto& data : test_cases) {
        plane_t plane(data.p1, data.p2, data.p3);
        check_point(data.p1, plane);
        check_point(data.p2, plane);
        check_point(data.p3, plane);
    }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
