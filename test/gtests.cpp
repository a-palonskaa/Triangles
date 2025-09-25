#include <iostream>
#include <vector>
#include <cmath>

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

TEST(PointOnSegment, PointOnSegment) {
    struct PointOnSegmentTestData {
        point_t refp, p1, p2;
        bool present;
    };

    PointOnSegmentTestData test_cases[] = {
        {{0, 0, 0}, {0, 0, 0}, {1, 1, 1}, true},
        {{2, 2, 2}, {1, 1, 1}, {3, 3, 3}, true},
        {{3, 3, 3}, {1, 1, 1}, {2, 2, 2}, false},
        {{0, 0, 1}, {-1, 0, 0}, {1, 0, 2}, true},
        {{4, 3, 1}, {-1, 0, 0}, {1, 0, 2}, false}
    };

    for (const auto& data : test_cases) {
        EXPECT_TRUE(point_on_segment(data.refp, data.p1, data.p2) == data.present);
    }
}

TEST(Solve2x2Equation, Solve2x2Equation) {
    double t1_got = 0, t2_got = 0;

    struct Solve2x2EqTestData {
        double a11, a12, a21, a22;
        double b1, b2;
        double t1, t2;
    };

    Solve2x2EqTestData test_cases[] = {
        {1, 0, 1, 0, 2, 3, NAN, NAN},
        {1, 2, 1, 3, 3, 4, 1, 1}
    };

    for (const auto& t : test_cases) {
        solve_2x2_equation(t.a11, t.a12, t.a21, t.a22, t.b1, t.b2, &t1_got, &t2_got);
        EXPECT_TRUE(t1_got == t.t1 || std::isnan(t1_got) && std::isnan(t.t1));
        EXPECT_TRUE(t2_got == t.t2 || std::isnan(t2_got) && std::isnan(t.t2));

    }
}

TEST(PolygonT, isInside) {
    struct PolygonIsInsideTestData {
        std::vector<point_t> v;
        point_t refp;
        bool is_inside;
    };

    PolygonIsInsideTestData test_cases[] = {
        {std::vector<point_t>{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}, {0.5, 0.5, 0}, true},
        {std::vector<point_t>{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}, {1.5, 0.5, 0}, false},
    };

    for (const auto& data : test_cases) {
        polygon_t polygon{data.v};
        EXPECT_TRUE(polygon.is_inside(data.refp) == data.is_inside);
    }


}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
