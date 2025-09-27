#pragma once

#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>
#include <array>

inline constexpr double tolerance = 0.00001;

inline bool are_equal(double n1, double n2) {
    return std::abs(n1 - n2) < tolerance;
}

inline bool is_null(double n1) {
    return std::abs(n1) < tolerance;
}

inline void solve_2x2_equation(const double& a11, const double& a12, const double& a21, const double& a22,
                        const double& b1,  const double& b2, double* t1, double* t2) {
    double det1 = a11 * a22 - a12 * a21;
    if (is_null(det1)) {
        *t1 = NAN;
        *t2 = NAN;
        return;
    }
    double det2 = b1 * a22 - b2 * a12;
    double det3 = a11 * b2 - a21 * b1;

    *t1 = det2 / det1;
    *t2 = det3 / det1;
}

inline void solve_appropriate_2x2(const double& a11, const double& a12, const double& a21, const double& a22,
    const double& a31, const double& a32, const double& b1, const double& b2, const double& b3, double* t1, double* t2) {
    if (is_null(a11 * a22 - a21 * a12)) {
        solve_2x2_equation(a21, a22, a31, a32, b2, b3, t1, t2);
    } else {
        solve_2x2_equation(a11, a12, a21, a22, b1, b2, t1, t2);
    }
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

    bool is_point_on_segment( const point_t& p1, const point_t& p2) const {
        double t = 0;
        double a11 = p2.x - p1.x, a21 = p2.y - p1.y, a31 = p2.z - p1.z;
        double b1 = x - p1.x, b2 = y - p1.y, b3 = z - p1.z;

        if (!are_equal(a11 * b2, a21 * b1) && are_equal(a21 * b3, a31 * b2)) {
            return false;
        }
        else {
            if (!are_equal(a11, 0)) {
                t = b1 / a11;
            }
            else if (!are_equal(a21, 0)) {
                t = b2 / a21;
            }
            else {
                t = b3 / a31;
            }
        }
        return (t > -tolerance && t < 1 + tolerance);
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
    point_t find_cross_line_intersection_point(const point_plane_relation::type& rel1,
const point_plane_relation::type& rel2, const point_t& p1, const point_t& p2) const;
};

struct triangle_t {
private:
    static constexpr size_t size_ = 3;
public:
    std::array<point_t, size_> vertices;

   triangle_t(const std::array<point_t, size_>& v) : vertices(v) {}


    bool are_intersecting(const triangle_t& ref_triangle) const;
    bool is_inside(const point_plane_relation::type& rel, const point_t& refp) const;
    bool is_plane_line_intersection(const point_plane_relation::type& rel1,
    const point_plane_relation::type& rel2, const point_t& p1, const point_t& p2) const;
    bool segments_intersect_in_plane(const point_t& p1, const point_t& p2,
                                     const point_t& q1, const point_t& q2) const;
    bool are_non_collinear_segments_intersects(const point_t& p1, const point_t& p2,
                                               const point_t& q1, const point_t& q2) const;
};

