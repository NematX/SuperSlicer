///|/ Copyright (c) Prusa Research 2023 Enrico Turri @enricoturri1966, Lukáš Matěna @lukasmatena
///|/ Copyright (c) BambuStudio 2023 manch1n @manch1n
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_ConflictChecker_hpp_
#define slic3r_ConflictChecker_hpp_

#include "libslic3r/Print.hpp"

#include <queue>
#include <vector>
#include <optional>


//used only in Print::process() after brim generation, to check for object collision
// arc is done just after, so we don't need to worry about it
namespace Slic3r {

struct LineWithID
{
    Line _line;
    int  _obj_id;
    int  _inst_id;
    ExtrusionRole _role;

    LineWithID(const Line& line, int obj_id, int inst_id, const ExtrusionRole& role) :
        _line(line), _obj_id(obj_id), _inst_id(inst_id), _role(role) {}
};

using LineWithIDs = std::vector<LineWithID>;

class LinesBucket
{
private:
    double   _curHeight  = 0.0;
    unsigned _curPileIdx = 0;

    std::vector<ExtrusionPaths> _piles;
    int                         _id;
    Points                      _offsets;

public:
    LinesBucket(std::vector<ExtrusionPaths> &&paths, int id, Points offsets) : _piles(paths), _id(id), _offsets(offsets) {}
    LinesBucket(LinesBucket &&) = default;

    bool valid() const { return _curPileIdx < _piles.size(); }
    void raise()
    {
        if (valid()) {
            if (_piles[_curPileIdx].empty() == false) { _curHeight += _piles[_curPileIdx].front().height(); }
            _curPileIdx++;
        }
    }
    double      curHeight() const { return _curHeight; }
    LineWithIDs curLines() const
    {
        LineWithIDs lines;
        for (const ExtrusionPath &path : _piles[_curPileIdx]) {
            Polyline check_polyline;
            for (int idx_offset = 0; idx_offset < (int)_offsets.size(); ++idx_offset) {
                check_polyline = path.as_polyline().to_polyline();
                check_polyline.translate(_offsets[idx_offset]);
                for (size_t idx_pt = 1; idx_pt < check_polyline.size(); ++idx_pt) {
                    lines.emplace_back(Line(check_polyline.points[idx_pt - 1], check_polyline.points[idx_pt]), _id, idx_offset, path.role());
                }
            }
        }
        return lines;
    }

    friend bool operator>(const LinesBucket &left, const LinesBucket &right) { return left._curHeight > right._curHeight; }
    friend bool operator<(const LinesBucket &left, const LinesBucket &right) { return left._curHeight < right._curHeight; }
    friend bool operator==(const LinesBucket &left, const LinesBucket &right) { return left._curHeight == right._curHeight; }
};

struct LinesBucketPtrComp
{
    bool operator()(const LinesBucket *left, const LinesBucket *right) { return *left > *right; }
};

class LinesBucketQueue
{
private:
    std::vector<LinesBucket>                                                           _buckets;
    std::priority_queue<LinesBucket *, std::vector<LinesBucket *>, LinesBucketPtrComp> _pq;
    std::map<int, const void *>                                                        _idToObjsPtr;
    std::map<const void *, int>                                                        _objsPtrToId;

public:
    void        emplace_back_bucket(std::vector<ExtrusionPaths> &&paths, const void *objPtr, Points offset);
    void        build_queue();
    bool        valid() const { return _pq.empty() == false; }
    const void *idToObjsPtr(int id)
    {
        if (_idToObjsPtr.find(id) != _idToObjsPtr.end())
            return _idToObjsPtr[id];
        else
            return nullptr;
    }
    double      removeLowests();
    LineWithIDs getCurLines() const;
};

void getExtrusionPathsFromEntity(const ExtrusionEntityCollection *entity, ExtrusionPaths &paths);

ExtrusionPaths getExtrusionPathsFromLayer(LayerRegionPtrs layerRegionPtrs);

ExtrusionPaths getExtrusionPathsFromSupportLayer(SupportLayer *supportLayer);

std::pair<std::vector<ExtrusionPaths>, std::vector<ExtrusionPaths>> getAllLayersExtrusionPathsFromObject(PrintObject *obj);

struct ConflictComputeResult
{
    int _obj1;
    int _obj2;

    ConflictComputeResult(int o1, int o2) : _obj1(o1), _obj2(o2) {}
    ConflictComputeResult() = default;
};

using ConflictComputeOpt = std::optional<ConflictComputeResult>;

using ConflictObjName = std::optional<std::pair<std::string, std::string>>;

struct ConflictChecker
{
    static ConflictResultOpt  find_inter_of_lines_in_diff_objs(SpanOfConstPtrs<PrintObject> objs, const WipeTowerData& wtd);
    static ConflictComputeOpt find_inter_of_lines(const LineWithIDs &lines);
    static ConflictComputeOpt line_intersect(const LineWithID &l1, const LineWithID &l2);
};

} // namespace Slic3r

#endif
