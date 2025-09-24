#pragma once

#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>

inline constexpr double tolerance = 0.00001;

inline bool are_equal(double n1, double n2) {
    return std::abs(n1 - n2) < tolerance;
}

namespace point_plane_relation {
    enum type {
        ABOVE_PLANE = 0,
        ON_PLANE    = 1,
        BELOW_PLANE = 2
    };
};

struct point_t {
    double x = NAN, y = NAN, z = NAN;

    point_t() {};
    point_t(double xp, double yp, double zp) : x(xp), y(yp), z(zp) {};

    bool is_valid() const {
        return !(x == NAN || y == NAN || z == NAN);
    }

    bool equal(const point_t& refp) const {
        return (std::abs(x - refp.x) < tolerance) &&
               (std::abs(y - refp.y) < tolerance) &&
               (std::abs(z - refp.z) < tolerance);
    }
};

inline double dot(const point_t& u, const point_t& v) {
    return u.x * v.x + u.y * v.y + u.z * v.z;
}

inline point_t cross(const point_t& u, const point_t& v) {
    return point_t{
        u.y * v.z - u.z * v.y,
        u.z * v.x - u.x * v.z,
        u.x * v.y - u.y * v.x
    };
}

// ax+by+cz+d=0
struct plane_t {
    double a = 0, b = 0, c = 0, d = 0;

    plane_t(const point_t& p1, const point_t& p2, const point_t& p3) {
        point_t u{p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
        point_t v{p3.x - p1.x, p3.y - p1.y, p3.z - p1.z};
        point_t p = cross(u, v);

        a = p.x;
        b = p.y;
        c = p.z;
        d = -(a * p1.x + b * p1.y + c * p1.z);
    }

    void print_plane_params() const;
    point_plane_relation::type classify_point_location(const point_t& p) const;
    point_t find_cross_line_intersection_point(const point_t& p1, const point_t& p2) const;
};

//ХУЙНЯ -  naming
//ХУЙНЯ - polygon != triangle, maybe divide
struct polygon_t {
    std::vector<point_t> vertices;

    polygon_t(std::vector<point_t> v) : vertices(std::move(v)) {};

    bool are_intersecting(const polygon_t& ref_polygon) const;
    bool is_inside(const point_t& refp) const;
    bool is_plane_line_intersection(const point_t& p1, const point_t& p2) const;
    bool segments_intersect_in_plane(const point_t& p1, const point_t& p2,
                                     const point_t& q1, const point_t& q2) const;
    bool are_non_collinear_segments_intersects_in_plane(const point_t& p1, const point_t& p2,
                                                        const point_t& q1, const point_t& q2) const;
};


