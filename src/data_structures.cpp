#include <vector>
#include <cassert>
#include <cmath>

#include "data_structures.hpp"

static bool point_on_segment(const point_t& refp, const point_t& p1, const point_t& p2) {
    double t = 0;
    double a11 = p2.x - p1.x, a21 = p2.y - p1.y, a31 = p2.z - p1.z;
    double b1 = refp.x - p1.x, b2 = refp.y - p1.y, b3 = refp.y - p1.y;

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

static void solve_2x2_equation(const double& a11, const double& a12, const double& a21, const double& a22,
                        const double& b1,  const double& b2, double* t1, double* t2) {
    double det1 = a11 * a22 - a12 * a21;
    if (are_equal(det1, 0)) {
        *t1 = NAN;
        *t2 = NAN;
    }
    double det2 = b1 * a22 - b2 * a12;
    double det3 = a11 * b2 - a21 * b1;

    *t1 = det2 / det1;
    *t2 = det3 / det1;
}

//----------------polygon_t-methods--------------------------------------------------------------

bool polygon_t::are_intersecting(const polygon_t& ref_polygon) const {
    assert(vertices.size() == 3); //ХУЙНЯ - production??
    plane_t plane1(vertices[0], vertices[1], vertices[2]);

    std::vector<point_plane_relation::type> vals; //ХУЙНЯ - shitty naming
    for (const auto& v : ref_polygon.vertices) {
        vals.push_back(plane1.classify_point_location(v));
    }
//ХУЙНЯ - too much, non-readable
//ХУЙНЯ - iteration by vals  - change to size_ or smth
    for (size_t i = 0, j = 1; i < 2; i++, j = (i + 1) % 3) {
        if (vals[i] == point_plane_relation::ON_PLANE && is_inside(ref_polygon.vertices[i])) {
            return true;
        } else if ((vals[i] == point_plane_relation::ON_PLANE && vals[j] == point_plane_relation::ON_PLANE) &&
            is_plane_line_intersection(ref_polygon.vertices[i], ref_polygon.vertices[j])) {
            return true;
        } else if (((vals[i] == point_plane_relation::ABOVE_PLANE && vals[j] == point_plane_relation::BELOW_PLANE ||
            vals[i] == point_plane_relation::BELOW_PLANE && vals[j] == point_plane_relation::ABOVE_PLANE)) &&
            is_inside(plane1.find_cross_line_intersection_point(ref_polygon.vertices[i], ref_polygon.vertices[j]))) {
            return true;
        }
    }
    return false;
}

// point in on polygon plane, barycentric coordinates
bool polygon_t::is_inside(const point_t& refp) const {
    assert(vertices.size() == 3); //ХУЙНЯ - shit

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
    if (are_equal(d, 0)) { //ХУЙНЯ - maybe create is_null for doubles
        return false;
    }

    double beta  = (d11 * d20 - d01 * d21) / d;
    double gamma = (d00 * d21 - d01 * d20) / d;
    double alpha = 1.0 - beta - gamma;

    return (alpha > -tolerance && beta > -tolerance && gamma > -tolerance );
}

// line in on polygon plane, no dots inside triangle
bool polygon_t::is_plane_line_intersection(const point_t& p1, const point_t& p2) const {
    return (segments_intersect_in_plane(p1, p2, vertices[0], vertices[1]) ||
            segments_intersect_in_plane(p1, p2, vertices[1], vertices[2]) ||
            segments_intersect_in_plane(p1, p2, vertices[2], vertices[0]));
}

// Check if two segments in the same plane intersect
bool polygon_t::segments_intersect_in_plane(const point_t& p1, const point_t& p2,
                                const point_t& q1, const point_t& q2) const {
    point_t dir1 = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
    point_t dir2 = {q2.x - q1.x, q2.y - q1.y, q2.z - q1.z};
    point_t cross_prod = cross(dir1, dir2);

    if (cross_prod.equal({0, 0, 0})) {
        return point_on_segment(q1, p1, p2) || point_on_segment(q2, p1, p2) ||
               point_on_segment(p1, q1, q2) || point_on_segment(p2, q1, q2);
    }
    return are_non_collinear_segments_intersects_in_plane(p1, p2, q1, q2);
}

//ХУЙНЯ - damn long name i am tiref of reading
// Check if two non collinear segments in the same plane intersect
bool polygon_t::are_non_collinear_segments_intersects_in_plane(const point_t& p1, const point_t& p2,
                                                    const point_t& q1, const point_t& q2) const {
    double t1 = 0, t2 = 0;
    double a11 = p2.x - p1.x, a12 = q1.x - q2.x, b1 = q1.x - p1.x;
    double a21 = p2.y - p1.y, a22 = q1.y - q2.y, b2 = q1.y - p1.y;
    double a31 = p2.z - p1.z, a32 = q1.z - q2.z, b3 = q1.z - p1.z;

//ХУЙНЯ - looks copypasting
    if (are_equal(a11 * a22 - a21 * a12, 0)) {
        solve_2x2_equation(a21, a22, a31, a32, b2, b3, &t1, &t2);
        if (std::isnan(t1) || std::isnan(t2)) {
            return false;
        }
    }
    else {
        solve_2x2_equation(a11, a12, a21, a22, b1, b2, &t1, &t2);
        if (std::isnan(t1) || std::isnan(t2)) {
            return false;
        }
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
point_t plane_t::find_cross_line_intersection_point(const point_t& p1, const point_t& p2) const {
    point_t v12{p2.x - p1.x, p2.y - p1.y, p2.z - p1.z}; // directional vector of a line  l = v12t + p1
    double t = -(a * p1.x + b * p1.y + c * p1.z + d) / (a * v12.x + b * v12.y + c * v12.z); // t correcponding to intersection

    return point_t{
        p1.x + t * v12.x,
        p1.y + t * v12.y,
        p1.z + t * v12.z
    };
}

