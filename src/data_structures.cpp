#include <vector>
#include <cassert>
#include <cmath>
#include <array>

#include "data_structures.hpp"

//----------------triangle_t-methods--------------------------------------------------------------

bool triangle_t::are_intersecting(const triangle_t& ref_triangle) const {
    plane_t plane1(vertices[0], vertices[1], vertices[2]);

    point_plane_relation::type point_rels[3];
    for (size_t i = 0; i < 3; i++) {
        point_rels[i] = plane1.classify_point_location(ref_triangle.vertices[i]);
    }

    for (size_t i = 0, j = 1; i < size_ - 1; i++, j = (i + 1) % size_) {
        if (is_inside(point_rels[i], ref_triangle.vertices[i])) {
            return true;
        }

        if (is_plane_line_intersection(point_rels[i], point_rels[j],
            ref_triangle.vertices[i], ref_triangle.vertices[j])) {
            return true;
        }

        point_t intersection = plane1.find_cross_line_intersection_point(point_rels[i], point_rels[j],
            ref_triangle.vertices[i], ref_triangle.vertices[j]);
        if (is_inside(point_plane_relation::ON_PLANE, intersection)){
            return true;
        }
    }
    return false;
}

// point in on polygon plane, barycentric coordinates
bool triangle_t::is_inside(const point_plane_relation::type& rel, const point_t& refp) const {
    if (rel != point_plane_relation::ON_PLANE) {
        return false;
    }

    if (!refp.is_valid()) {
        return false;
    }

    const point_t& a = vertices[0], b = vertices[1], c = vertices[2];

    point_t v0{c.x - a.x, c.y - a.y, c.z - a.z};
    point_t v1{b.x - a.x, b.y - a.y, b.z - a.z};
    point_t v2{refp.x - a.x, refp.y - a.y, refp.z - a.z};

    double d01 = v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
    double d00 = v0.x * v0.x + v0.y * v0.y + v0.z * v0.z;
    double d11 = v1.x * v1.x + v1.y * v1.y + v1.z * v1.z;
    double d20 = v2.x * v0.x + v2.y * v0.y + v2.z * v0.z;
    double d21 = v2.x * v1.x + v2.y * v1.y + v2.z * v1.z;

    double d = d00 * d11 - d01 * d01;
    if (is_null(d)) {
        return false;
    }

    double beta  = (d11 * d20 - d01 * d21) / d;
    double gamma = (d00 * d21 - d01 * d20) / d;
    double alpha = 1.0 - beta - gamma;

    return (alpha > -tolerance && beta > -tolerance && gamma > -tolerance );
}

// line in on polygon plane, no dots inside triangle
bool triangle_t::is_plane_line_intersection(const point_plane_relation::type& rel1,
    const point_plane_relation::type& rel2, const point_t& p1, const point_t& p2) const {
    if (rel1 != point_plane_relation::ON_PLANE || rel2 != point_plane_relation::ON_PLANE) {
        return false;
    }

    return (segments_intersect_in_plane(p1, p2, vertices[0], vertices[1]) ||
            segments_intersect_in_plane(p1, p2, vertices[1], vertices[2]) ||
            segments_intersect_in_plane(p1, p2, vertices[2], vertices[0]));
}

// Check if two segments in the same plane intersect
bool triangle_t::segments_intersect_in_plane(const point_t& p1, const point_t& p2,
                                const point_t& q1, const point_t& q2) const {
    point_t dir1 = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
    point_t dir2 = {q2.x - q1.x, q2.y - q1.y, q2.z - q1.z};
    point_t cross_prod = cross(dir1, dir2);

    if (cross_prod.equal({0, 0, 0})) {
        return q1.is_point_on_segment(p1, p2) || q2.is_point_on_segment(p1, p2) ||
               p1.is_point_on_segment(q1, q2) || p2.is_point_on_segment(q1, q2);
    }
    return are_non_collinear_segments_intersects(p1, p2, q1, q2);
}

// Check if two non collinear segments in the same plane intersect
bool triangle_t::are_non_collinear_segments_intersects(const point_t& p1, const point_t& p2,
                                                    const point_t& q1, const point_t& q2) const {
    double t1 = 0, t2 = 0;
    double a11 = p2.x - p1.x, a12 = q1.x - q2.x, b1 = q1.x - p1.x;
    double a21 = p2.y - p1.y, a22 = q1.y - q2.y, b2 = q1.y - p1.y;
    double a31 = p2.z - p1.z, a32 = q1.z - q2.z, b3 = q1.z - p1.z;

    solve_appropriate_2x2(a11, a12, a21, a22, a31, a32, b1, b2, b3, &t1, &t2);
    if (std::isnan(t1) || std::isnan(t2)) {
        return false;
    }
    return (t1 > -tolerance && t1 < 1 + tolerance && t2 > -tolerance && t2 < 1 + tolerance);
}

//----------------plane_t-methods--------------------------------------------------------------

void plane_t::print_plane_params() const {
    std::cout << a << ' ' << b << ' ' << c << ' ' << d << '\n';
    if (!std::cin.good()) {
        std::cerr << "failed to read elem from cin\n";
    }
}

point_plane_relation::type plane_t::classify_point_location(const point_t& p) const {
    double val = a * p.x + b * p.y + c * p.z + d;
    return (std::abs(val) < tolerance) ? point_plane_relation::ON_PLANE :
                                         (val > 0) ? point_plane_relation::ABOVE_PLANE :
                                                     point_plane_relation::BELOW_PLANE;
}

// line cross the plane and it is known for sure
point_t plane_t::find_cross_line_intersection_point(const point_plane_relation::type& rel1,
const point_plane_relation::type& rel2, const point_t& p1, const point_t& p2) const {
    if (!(rel1 == point_plane_relation::ABOVE_PLANE && rel2 == point_plane_relation::BELOW_PLANE ||
        rel1 == point_plane_relation::BELOW_PLANE && rel2 == point_plane_relation::ABOVE_PLANE)) {
        return point_t{NAN, NAN, NAN};
    }

    point_t v12{p2.x - p1.x, p2.y - p1.y, p2.z - p1.z}; // directional vector of a line  l = v12t + p1
    double t = -(a * p1.x + b * p1.y + c * p1.z + d) / (a * v12.x + b * v12.y + c * v12.z); // t correcponding to intersection

    return point_t{
        p1.x + t * v12.x,
        p1.y + t * v12.y,
        p1.z + t * v12.z
    };
}

