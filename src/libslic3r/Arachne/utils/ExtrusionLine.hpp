//Copyright (c) 2020 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.


#ifndef UTILS_EXTRUSION_LINE_H
#define UTILS_EXTRUSION_LINE_H

#include "ExtrusionJunction.hpp"
#include "../../Polyline.hpp"
#include "../../Polygon.hpp"
#include "../../BoundingBox.hpp"
#include "../../ExtrusionEntity.hpp"
#include "../../../clipper/clipper_z.hpp"

namespace Slic3r {
class ThickPolyline;
}

namespace Slic3r::Arachne
{

/*!
 * Represents a polyline (not just a line) that is to be extruded with variable
 * line width.
 *
 * This polyline is a sequence of \ref ExtrusionJunction, with a bit of metadata
 * about which inset it represents.
 */
struct ExtrusionLine
{
    /*!
     * Which inset this path represents, counted from the outside inwards.
     *
     * The outer wall has index 0.
     */
    size_t inset_idx;

    /*!
     * If a thin piece needs to be printed with an odd number of walls (e.g. 5
     * walls) then there will be one wall in the middle that is not a loop. This
     * field indicates whether this path is such a line through the middle, that
     * has no companion line going back on the other side and is not a closed
     * loop.
     */
    bool is_odd;

    /*!
     * Whether this is a closed polygonal path
     */
    bool is_closed;

    /*!
     * Gets the number of vertices in this polygon.
     * \return The number of vertices in this polygon.
     */
    size_t size() const { return junctions.size(); }

    /*!
     * Whether there are no junctions.
     */
    bool empty() const { return junctions.empty(); }

    /*!
     * The list of vertices along which this path runs.
     *
     * Each junction has a width, making this path a variable-width path.
     */
    std::vector<ExtrusionJunction> junctions;

    ExtrusionLine(const size_t inset_idx, const bool is_odd);
    ExtrusionLine() : inset_idx(-1), is_odd(true), is_closed(false) {}
    ExtrusionLine(const ExtrusionLine &other) : inset_idx(other.inset_idx), is_odd(other.is_odd), is_closed(other.is_closed), junctions(other.junctions) {}

    ExtrusionLine &operator=(ExtrusionLine &&other)
    {
        junctions = std::move(other.junctions);
        inset_idx = other.inset_idx;
        is_odd    = other.is_odd;
        is_closed = other.is_closed;
        return *this;
    }

    ExtrusionLine &operator=(const ExtrusionLine &other)
    {
        junctions = other.junctions;
        inset_idx = other.inset_idx;
        is_odd    = other.is_odd;
        is_closed = other.is_closed;
        return *this;
    }

    std::vector<ExtrusionJunction>::const_iterator begin() const { return junctions.begin(); }
    std::vector<ExtrusionJunction>::const_iterator end() const { return junctions.end(); }
    std::vector<ExtrusionJunction>::const_reverse_iterator rbegin() const { return junctions.rbegin(); }
    std::vector<ExtrusionJunction>::const_reverse_iterator rend() const { return junctions.rend(); }
    std::vector<ExtrusionJunction>::const_reference front() const { return junctions.front(); }
    std::vector<ExtrusionJunction>::const_reference back() const { return junctions.back(); }
    const ExtrusionJunction &operator[](unsigned int index) const { return junctions[index]; }
    ExtrusionJunction &operator[](unsigned int index) { return junctions[index]; }
    std::vector<ExtrusionJunction>::iterator begin() { return junctions.begin(); }
    std::vector<ExtrusionJunction>::iterator end() { return junctions.end(); }
    std::vector<ExtrusionJunction>::reference front() { return junctions.front(); }
    std::vector<ExtrusionJunction>::reference back() { return junctions.back(); }

    template<typename... Args> void emplace_back(Args &&...args) { junctions.emplace_back(args...); }
    void remove(unsigned int index) { junctions.erase(junctions.begin() + index); }
    void insert(size_t index, const ExtrusionJunction &p) { junctions.insert(junctions.begin() + index, p); }

    template<class iterator>
    std::vector<ExtrusionJunction>::iterator insert(std::vector<ExtrusionJunction>::const_iterator pos, iterator first, iterator last)
    {
        return junctions.insert(pos, first, last);
    }

    void clear() { junctions.clear(); }
    void reverse() { std::reverse(junctions.begin(), junctions.end()); }

    /*!
     * Sum the total length of this path.
     */
    int64_t getLength() const;
    bool    is_zero_length() const;
    int64_t polylineLength() const { return getLength(); }

    /*!
     * Put all junction locations into a polygon object.
     *
     * When this path is not closed the returned Polygon should be handled as a polyline, rather than a polygon.
     */
    Polygon toPolygon() const
    {
        assert(this->is_closed);
        Polygon ret;
        for (const ExtrusionJunction &j : junctions) {
            // only copy a point if it's far enough
            if (ret.points.empty() || !j.p.coincides_with_epsilon(ret.points.back())) {
                ret.points.emplace_back(j.p);
            }
        }
        // be sure the last point is the one that is kept
        if (ret.points.back() != junctions.back().p) {
            ret.points.back() = junctions.back().p;
        }
        // a polygon doesn't repeat the last point
        if (ret.points.back().coincides_with_epsilon(ret.points.front())) {
            ret.points.pop_back();
        }

        return ret;
    }

    /*!
     * Put all junction locations into a polygon object.
     *
     * When this path is not closed the returned Polygon should be handled as a polyline, rather than a polygon.
     */
    Polyline toPolyline() const
    {
        Polyline ret;
        for (const ExtrusionJunction &j : junctions) {
            // only copy a point if it's far enough
            if (ret.points.empty() || !j.p.coincides_with_epsilon(ret.points.back())) {
                ret.points.emplace_back(j.p);
            }
        }
        // be sure the last point is the one that is kept
        if (ret.points.back() != junctions.back().p) {
            ret.points.back() = junctions.back().p;
        }
        // if loop, be sure to have the exact same point in front & back.
        if (ret.points.back().coincides_with_epsilon(ret.points.front())) {
            assert(this->is_closed);
            ret.points.back() = ret.points.front();
        }

        return ret;
    }

    /*!
     * Get the minimal width of this path
     */
    coord_t getMinimalWidth() const;

    /*!
     * Removes vertices of the ExtrusionLines to make sure that they are not too high
     * resolution.
     *
     * This removes junctions which are connected to line segments that are shorter
     * than the `smallest_line_segment`, unless that would introduce a deviation
     * in the contour of more than `allowed_error_distance`.
     *
     * Criteria:
     * 1. Never remove a junction if either of the connected segments is larger than \p smallest_line_segment
     * 2. Never remove a junction if the distance between that junction and the final resulting polygon would be higher
     *    than \p allowed_error_distance
     * 3. The direction of segments longer than \p smallest_line_segment always
     *    remains unaltered (but their end points may change if it is connected to
     *    a small segment)
     * 4. Never remove a junction if it has a distinctively different width than the next junction, as this can
     *    introduce unwanted irregularities on the wall widths.
     *
     * Simplify uses a heuristic and doesn't necessarily remove all removable
     * vertices under the above criteria, but simplify may never violate these
     * criteria. Unless the segments or the distance is smaller than the
     * rounding error of 5 micron.
     *
     * Vertices which introduce an error of less than 5 microns are removed
     * anyway, even if the segments are longer than the smallest line segment.
     * This makes sure that (practically) co-linear line segments are joined into
     * a single line segment.
     * \param smallest_line_segment Maximal length of removed line segments.
     * \param allowed_error_distance If removing a vertex introduces a deviation
     *         from the original path that is more than this distance, the vertex may
     *         not be removed.
     * \param maximum_extrusion_area_deviation The maximum extrusion area deviation allowed when removing intermediate
     *        junctions from a straight ExtrusionLine
     */
    void simplify(int64_t smallest_line_segment_squared, int64_t allowed_error_distance_squared, int64_t maximum_extrusion_area_deviation);

    /*!
     * Computes and returns the total area error (in μm²) of the AB and BC segments of an ABC straight ExtrusionLine
     * when the junction B with a width B.w is removed from the ExtrusionLine. The area changes due to the fact that the
     * new simplified line AC has a uniform width which equals to the weighted average of the width of the subsegments
     * (based on their length).
     *
     * \param A Start point of the 3-point-straight line
     * \param B Intermediate point of the 3-point-straight line
     * \param C End point of the 3-point-straight line
     * */
    static int64_t calculateExtrusionAreaDeviationError(ExtrusionJunction A, ExtrusionJunction B, ExtrusionJunction C);

    bool is_contour() const;

    double area() const;
};

static inline Slic3r::ThickPolyline to_thick_polyline(const Arachne::ExtrusionLine &line_junctions)
{
    assert(line_junctions.size() >= 2);
    Slic3r::ThickPolyline out;
    out.points.emplace_back(line_junctions.front().p);
    out.points_width.emplace_back(line_junctions.front().w);

    for (auto it = line_junctions.begin() + 1; it != line_junctions.end(); ++it) {
        if (!it->p.coincides_with_epsilon(out.points.back())) {
            out.points.emplace_back(it->p);
            out.points_width.emplace_back(it->w);
        }
    }
    return out;
}

static inline Slic3r::ThickPolyline to_thick_polyline(const ClipperLib_Z::Path &path)
{
    assert(path.size() >= 2);
    Slic3r::ThickPolyline out;
    out.points.emplace_back(path.front().x(), path.front().y());
    out.points_width.emplace_back(path.front().z());

    for (auto it = path.begin() + 1; it != path.end(); ++it) {
        if (!Point{ it->x(), it->y() }.coincides_with_epsilon(out.points.back())) {
            out.points.emplace_back(it->x(), it->y());
            out.points_width.emplace_back(it->z());
        }
    }
    // Don't create 1-element polyline.
    if(out.points.size() <2)
        return {};

    assert(out.points.back().coincides_with_epsilon(Point{ path.back().x(), path.back().y() }));
    out.points.back() = Point{ path.back().x(), path.back().y() };

    assert(out.points.front().x() == path.front().x());
    assert(out.points.front().y() == path.front().y());
    assert(out.points.back().x() == path.back().x());
    assert(out.points.back().y() == path.back().y());
    return out;
}

static inline Polygon to_polygon(const ExtrusionLine &line)
{
    Polygon out;
    assert(line.junctions.size() >= 3);
    assert(line.junctions.front().p == line.junctions.back().p);
    out.points.reserve(line.junctions.size() - 1);
    for (auto it = line.junctions.begin(); it != line.junctions.end() - 1; ++it)
        out.points.emplace_back(it->p);
    return out;
}

#if 0
static BoundingBox get_extents(const ExtrusionLine &extrusion_line)
{
    BoundingBox bbox;
    for (const ExtrusionJunction &junction : extrusion_line.junctions)
        bbox.merge(junction.p);
    return bbox;
}

static BoundingBox get_extents(const std::vector<ExtrusionLine> &extrusion_lines)
{
    BoundingBox bbox;
    for (const ExtrusionLine &extrusion_line : extrusion_lines)
        bbox.merge(get_extents(extrusion_line));
    return bbox;
}

static BoundingBox get_extents(const std::vector<const ExtrusionLine *> &extrusion_lines)
{
    BoundingBox bbox;
    for (const ExtrusionLine *extrusion_line : extrusion_lines) {
        assert(extrusion_line != nullptr);
        bbox.merge(get_extents(*extrusion_line));
    }
    return bbox;
}

static Points to_points(const ExtrusionLine &extrusion_line)
{
    Points points;
    points.reserve(extrusion_line.junctions.size());
    for (const ExtrusionJunction &junction : extrusion_line.junctions)
        points.emplace_back(junction.p);
    return points;
}

static std::vector<Points> to_points(const std::vector<const ExtrusionLine *> &extrusion_lines)
{
    std::vector<Points> points;
    for (const ExtrusionLine *extrusion_line : extrusion_lines) {
        assert(extrusion_line != nullptr);
        points.emplace_back(to_points(*extrusion_line));
    }
    return points;
}
#endif

using VariableWidthLines = std::vector<ExtrusionLine>; //<! The ExtrusionLines generated by libArachne

} // namespace Slic3r::Arachne
#endif // UTILS_EXTRUSION_LINE_H
