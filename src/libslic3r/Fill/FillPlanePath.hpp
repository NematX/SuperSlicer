///|/ Copyright (c) Prusa Research 2016 - 2022 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2016 Alessandro Ranellucci @alranel
///|/
///|/ ported from lib/Slic3r/Fill/Rectilinear.pm:
///|/ Copyright (c) Prusa Research 2016 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2011 - 2015 Alessandro Ranellucci @alranel
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_FillPlanePath_hpp_
#define slic3r_FillPlanePath_hpp_

#include <map>

#include "../libslic3r.h"

#include "FillBase.hpp"

namespace Slic3r {

// The original Perl code used path generators from Math::PlanePath library:
// http://user42.tuxfamily.org/math-planepath/
// http://user42.tuxfamily.org/math-planepath/gallery.html

class FillPlanePath : public Fill
{
public:
    FillPlanePath() : Fill() { can_fill_surface_single = true; }
    ~FillPlanePath() override = default;

protected:
    void _fill_surface_single(
        const FillParams                &params, 
        unsigned int                     thickness_layers,
        const std::pair<float, Point>   &direction, 
        ExPolygon                        expolygon, 
        Polylines                       &polylines_out) const override;

    float _layer_angle(size_t idx) const override { return 0.f; }
    virtual bool centered() const = 0;

    friend class InfillPolylineClipper;
    class InfillPolylineOutput {
    public:
        InfillPolylineOutput(const double scale_out) : m_scale_out(scale_out) {}

        void            reserve(size_t n) { m_out.reserve(n); }
        void            add_point(const Vec2d& pt) { m_out.emplace_back(this->scaled(pt)); }
        Points&& result() { return std::move(m_out); }
        virtual bool    clips() const { return false; }

    protected:
        const Point     scaled(const Vec2d &fpt) const { return { coord_t(floor(fpt.x() * m_scale_out + 0.5)), coord_t(floor(fpt.y() * m_scale_out + 0.5)) }; }

        // Output polyline.
        Points          m_out;

    private:
        // Scaling coefficient of the generated points before tested against m_bbox and clipped by bbox.
        double          m_scale_out;
    };

    virtual void generate(coord_t min_x, coord_t min_y, coord_t max_x, coord_t max_y, const coordf_t resolution, InfillPolylineOutput &output) const = 0;
};

class FillArchimedeanChords : public FillPlanePath
{
public:
    Fill* clone() const override { return new FillArchimedeanChords(*this); };
    ~FillArchimedeanChords() override = default;

protected:
    bool centered() const override { return true; }
    void generate(coord_t min_x, coord_t min_y, coord_t max_x, coord_t max_y, const coordf_t resolution, InfillPolylineOutput &output) const override;
};

class FillHilbertCurve : public FillPlanePath
{
public:
    Fill* clone() const override { return new FillHilbertCurve(*this); };
    ~FillHilbertCurve() override = default;

protected:
    bool centered() const override { return false; }
    void generate(coord_t min_x, coord_t min_y, coord_t max_x, coord_t max_y, const coordf_t resolution, InfillPolylineOutput &output) const override;
};

class FillOctagramSpiral : public FillPlanePath
{
public:
    Fill* clone() const override { return new FillOctagramSpiral(*this); };
    ~FillOctagramSpiral() override = default;

protected:
    bool centered() const override { return true; }
    void generate(coord_t min_x, coord_t min_y, coord_t max_x, coord_t max_y, const coordf_t resolution, InfillPolylineOutput &output) const override;
};

} // namespace Slic3r

#endif // slic3r_FillPlanePath_hpp_
