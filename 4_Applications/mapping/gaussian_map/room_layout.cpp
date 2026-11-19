#include "4_Applications/mapping/gaussian_map/room_layout.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

namespace vista::application {
namespace {
using Point3 = models::RoomLayoutPosition;
using Point2 = std::array<double, 2>;
constexpr double pi = 3.14159265358979323846;
constexpr int angle_bins = 72;
constexpr double angle_step = 2*pi/angle_bins;
constexpr double offset_step = .15;
constexpr double max_room_radius = 12;
constexpr std::size_t max_walls = 8;
constexpr std::size_t max_segments = 96;
constexpr double sample_gap_m = .30;
constexpr double max_inferred_gap_m = .45;
constexpr double min_opening_width_m = .65;
constexpr double floor_support_height_m = .18;
constexpr double ceiling_grid_m = .18;
constexpr double ceiling_band_m = .12;
constexpr std::size_t max_ceiling_levels = 6;
constexpr std::size_t max_ceiling_segments = 32;

template<std::size_t N>
double dot(const std::array<double,N>& a, const std::array<double,N>& b) {
    double value = 0;
    for (std::size_t i = 0; i < N; ++i) value += a[i]*b[i];
    return value;
}
Point3 cross(const Point3& a, const Point3& b) {
    return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
}
Point2 tangent(const Point2& n) { return {-n[1],n[0]}; }

struct FloorBasis {
    Point3 origin, x, y, up;
    Point3 world(const Point2& p, double h) const {
        auto out = origin;
        for (std::size_t i = 0; i < 3; ++i) out[i] += p[0]*x[i]+p[1]*y[i]+h*up[i];
        return out;
    }
};

struct WallHit {
    Point2 position, normal;
    double height;
};
struct Span {
    double low{}, high{};
    double length() const { return high-low; }
};

struct WallSpan {
    Span extent;
    std::array<const WallSpan*,2> joins{};
};

struct Wall {
    Point2 normal;
    double offset{};
    std::vector<std::pair<double,double>> hits; // Tangential coordinate and height.
    double low{}, high{}, low_height{}, high_height{};
    double trace_height{};
    std::vector<WallSpan> spans;

    Point2 at(double along) const {
        const auto axis = tangent(normal);
        return {normal[0]*offset+axis[0]*along,normal[1]*offset+axis[1]*along};
    }
};

void fit_wall(const std::vector<WallHit>& hits, double tolerance, Wall& wall) {
    Point2 mean{};
    double xx = 0, xy = 0, yy = 0;
    std::vector<const WallHit*> support;
    for (const auto& hit : hits) {
        if (std::abs(dot(hit.position,wall.normal)-wall.offset)>tolerance ||
            std::abs(dot(hit.normal,wall.normal))<.94) continue;
        mean[0] += hit.position[0];
        mean[1] += hit.position[1];
        xx += hit.position[0]*hit.position[0];
        xy += hit.position[0]*hit.position[1];
        yy += hit.position[1]*hit.position[1];
        support.push_back(&hit);
    }
    if (support.size()<8) return;

    const auto n = static_cast<double>(support.size());
    mean[0] /= n;
    mean[1] /= n;
    const auto angle = .5*std::atan2(2*(xy/n-mean[0]*mean[1]),
        (xx-yy)/n-mean[0]*mean[0]+mean[1]*mean[1]);
    Point2 normal{-std::sin(angle),std::cos(angle)};
    if (std::abs(dot(normal,wall.normal))<.94) return;
    if (dot(normal,mean)<0) { normal[0] *= -1; normal[1] *= -1; }
    const auto offset = dot(normal,mean);
    if (offset<.18) return;
    const auto axis = tangent(normal);
    wall.normal = normal;
    wall.offset = offset;
    wall.low = wall.low_height = std::numeric_limits<double>::infinity();
    wall.high = wall.high_height = -wall.low;
    for (const auto& hit : hits) {
        if (std::abs(dot(hit.position,normal)-offset)>tolerance ||
            std::abs(dot(hit.normal,normal))<.94) continue;
        const auto along = dot(hit.position,axis);
        wall.low = std::min(wall.low,along);
        wall.high = std::max(wall.high,along);
        wall.low_height = std::min(wall.low_height,hit.height);
        wall.high_height = std::max(wall.high_height,hit.height);
        wall.hits.emplace_back(along,hit.height);
    }
    std::sort(wall.hits.begin(),wall.hits.end());
}

struct HorizontalBin {
    std::size_t hits{};
    double height{}, min_x{}, max_x{}, min_y{}, max_y{};
    void add(const Point2& p, double z) {
        if (hits==0) {
            min_x = max_x = p[0];
            min_y = max_y = p[1];
        }
        ++hits;
        height += z;
        min_x = std::min(min_x,p[0]); max_x = std::max(max_x,p[0]);
        min_y = std::min(min_y,p[1]); max_y = std::max(max_y,p[1]);
    }
};
std::optional<double> level(const std::map<int,HorizontalBin>& bins) {
    const HorizontalBin* best{};
    for (const auto& item : bins) {
        const auto& b = item.second;
        if (b.hits>=12 && b.max_x-b.min_x>=1.2 && b.max_y-b.min_y>=1.2 &&
            (!best || b.hits>best->hits)) best = &b;
    }
    if (best) return best->height/best->hits;
    return std::nullopt;
}

std::optional<Point2> intersect(const Wall& left, const Wall& right) {
    const auto& a = left.normal;
    const auto& b = right.normal;
    const double det = a[0]*b[1]-a[1]*b[0];
    if (std::abs(det)<.30) return std::nullopt;
    const Point2 p{(left.offset*b[1]-a[1]*right.offset)/det,
                   (a[0]*right.offset-left.offset*b[0])/det};
    if (dot(p,p)>max_room_radius*max_room_radius) return std::nullopt;
    return p;
}

std::pair<double,double> height_support(const Wall& wall, const Span& extent) {
    double low = std::numeric_limits<double>::infinity(), high = -low;
    const auto start=std::lower_bound(wall.hits.begin(),wall.hits.end(),extent.low,
        [](const auto& hit, double value) {return hit.first<value;});
    for (auto it=start; it!=wall.hits.end() && it->first<=extent.high; ++it) {
        low = std::min(low,it->second);
        high = std::max(high,it->second);
    }
    return {low,high};
}

std::vector<Span> measured_spans(const Wall& wall, double min_height, double max_height, double min_length) {
    std::vector<Span> spans;
    for (const auto& [along,height] : wall.hits) {
        if (height<min_height || height>max_height) continue;
        if (spans.empty() || along-spans.back().high>sample_gap_m) spans.push_back({along,along});
        else spans.back().high=along;
    }
    spans.erase(std::remove_if(spans.begin(),spans.end(),[=](const auto& span) {
        return span.length()<min_length;
    }),spans.end());
    return spans;
}

void find_wall_spans(Wall& wall, const std::optional<double>& floor) {
    constexpr double min_length = .35;
    auto spans=measured_spans(wall,wall.low_height,wall.high_height,min_length);
    // A lintel can cover an opening in plan view. Look for support at ordinary
    // wall height on either side before treating the intervening area as a wall.
    const auto lower_limit=floor ? *floor+1.65 : wall.low_height+.90;
    const auto lower=measured_spans(wall,wall.low_height,lower_limit,min_length);
    std::vector<Span> pieces;
    for (auto span : spans) {
        for (std::size_t i=1; i<lower.size(); ++i) {
            const Span opening{lower[i-1].high,lower[i].low};
            if (opening.length()<min_opening_width_m || opening.low<span.low || opening.high>span.high)
                continue;
            pieces.push_back({span.low,opening.low});
            span.low=opening.high;
        }
        pieces.push_back(span);
    }
    for (const auto& piece : pieces) {
        const auto [low,high]=height_support(wall,piece);
        if (piece.length()>=min_length && high-low>=.25) wall.spans.push_back({piece});
    }
}

struct WallEnd {
    WallSpan* span{};
    std::size_t side{};
    double along{}, distance{max_inferred_gap_m};
};

WallEnd nearest_end(Wall& wall, const Point2& p) {
    const auto along=dot(tangent(wall.normal),p);
    WallEnd end;
    for (auto& span : wall.spans) {
        const auto consider=[&](std::size_t side, double coordinate) {
            const auto distance=std::abs(along-coordinate);
            if (distance<=end.distance) end={&span,side,coordinate,distance};
        };
        consider(0,span.extent.low);
        consider(1,span.extent.high);
    }
    return end;
}

struct CornerCandidate {
    Wall* a;
    Wall* b;
    WallEnd left, right;
    Point2 position;
    double low_height, high_height;
};

bool is_closed(const std::vector<Wall>& walls) {
    const WallSpan* first{};
    std::size_t count{};
    for (const auto& wall : walls) {
        for (const auto& span : wall.spans) {
            if (!span.joins[0] || !span.joins[1]) return false;
            if (!first) first=&span;
            ++count;
        }
    }
    if (count<3) return false;
    const WallSpan* previous=first;
    const WallSpan* current=first->joins[0];
    std::size_t visited=1;
    while (current!=first && visited<count) {
        const auto next=current->joins[current->joins[0]==previous ? 1 : 0];
        previous=current;
        current=next;
        ++visited;
    }
    return current==first && visited==count;
}

void segment(models::RoomLayout& result, const Point3& start, const Point3& end, bool inferred) {
    if (result.segments.size()>=max_segments) return;
    const Point3 d{start[0]-end[0],start[1]-end[1],start[2]-end[2]};
    if (dot(d,d)>.0004) result.segments.push_back({start,end,inferred});
}
void trace(models::RoomLayout& result, const FloorBasis& basis, const Wall& wall,
           const Span& extent, double height, const std::vector<Span>& supported) {
    const auto draw=[&](double from, double to, bool inferred) {
        segment(result,basis.world(wall.at(from),height),basis.world(wall.at(to),height),inferred);
    };
    double cursor=extent.low;
    for (const auto& span : supported) {
        const auto a=std::max(extent.low,span.low), b=std::min(extent.high,span.high);
        if (a>=b || b<=cursor) continue;
        if (a>cursor) draw(cursor,a,true);
        draw(std::max(cursor,a),b,false);
        cursor=b;
    }
    if (cursor<extent.high) draw(cursor,extent.high,true);
}

void draw_wall(models::RoomLayout& result, const FloorBasis& basis, const Wall& wall,
               const std::optional<double>& floor) {
    const auto near_floor=floor && result.floor==models::RoomLayoutFloor::measured ?
        measured_spans(wall,*floor-.06,*floor+floor_support_height_m,.12) : std::vector<Span>{};
    const auto near_trace=measured_spans(wall,wall.trace_height-.30,wall.trace_height+.15,.12);
    for (std::size_t i=0; i<wall.spans.size(); ++i) {
        const auto& extent=wall.spans[i].extent;
        trace(result,basis,wall,extent,wall.trace_height,near_trace);
        if (floor && *floor<wall.trace_height) trace(result,basis,wall,extent,*floor,near_floor);
        if (i==0) continue;
        const Span gap{wall.spans[i-1].extent.high,extent.low};
        if (gap.length()>max_inferred_gap_m) continue;
        trace(result,basis,wall,gap,wall.trace_height,{});
        if (floor && *floor<wall.trace_height) trace(result,basis,wall,gap,*floor,{});
    }
}
using CeilingBands = std::map<int,std::vector<std::pair<Point2,double>>>;
using GridCell = std::array<int,2>;

struct CeilingCell {
    Point2 sum{};
    double height{};
    std::size_t count{};
    bool visited{};

    Point2 center() const { return {sum[0]/count,sum[1]/count}; }
};

double orientation(const Point2& a, const Point2& b, const Point2& p) {
    return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]);
}

std::vector<Point2> ceiling_hull(std::vector<Point2> points) {
    std::sort(points.begin(),points.end());
    std::vector<Point2> hull;
    for (const auto& p : points) {
        while (hull.size()>=2 && orientation(hull[hull.size()-2],hull.back(),p)<=0) hull.pop_back();
        hull.push_back(p);
    }
    const auto lower_size=hull.size();
    for (auto it=points.rbegin()+1; it!=points.rend(); ++it) {
        while (hull.size()>lower_size && orientation(hull[hull.size()-2],hull.back(),*it)<=0) hull.pop_back();
        hull.push_back(*it);
    }
    hull.pop_back();
    // Adjacent steps from scan sampling often describe one straight edge.
    for (std::size_t i=0; hull.size()>3 && i<hull.size();) {
        const auto& a=hull[(i+hull.size()-1)%hull.size()];
        const auto& b=hull[(i+1)%hull.size()];
        const auto length=std::hypot(b[0]-a[0],b[1]-a[1]);
        if (length>0 && std::abs(orientation(a,b,hull[i]))/length<=.08) {
            hull.erase(hull.begin()+static_cast<std::ptrdiff_t>(i));
            if (i>0) --i;
        } else ++i;
    }
    return hull;
}

void trace_ceiling_edge(models::RoomLayout& layout, const FloorBasis& basis, const std::vector<Point2>& points,
                        const Point2& a, const Point2& b, double height) {
    const auto length=std::hypot(b[0]-a[0],b[1]-a[1]);
    if (length<.38) return;
    const Point2 axis{(b[0]-a[0])/length,(b[1]-a[1])/length};
    const auto normal=tangent(axis);
    std::vector<double> boundary;
    std::size_t interior=0;
    for (const auto& p : points) {
        const Point2 d{p[0]-a[0],p[1]-a[1]};
        const auto along=dot(d,axis), across=dot(d,normal);
        if (along<-.02 || along>length+.02) continue;
        if (std::abs(across)<=.10) boundary.push_back(std::clamp(along,0.,length));
        if (across>=.23 && across<=.60) ++interior;
    }
    if (boundary.size()<3 || interior<3) return;
    std::sort(boundary.begin(),boundary.end());
    const auto at=[&](double t) {return basis.world({a[0]+t*axis[0],a[1]+t*axis[1]},height);};
    std::optional<double> end;
    for (auto first=boundary.begin(); first!=boundary.end();) {
        auto next=first+1;
        while (next!=boundary.end() && *next-*(next-1)<=sample_gap_m) ++next;
        const auto last=*(next-1);
        if (next-first>=3 && last-*first>=.33) {
            if (end && *first-*end<=max_inferred_gap_m) segment(layout,at(*end),at(*first),true);
            segment(layout,at(*first),at(last),false);
            end=last;
        }
        first=next;
    }
}

models::RoomLayout trace_ceiling_band(const FloorBasis& basis, const CeilingBands& bands, int level_bin) {
    std::map<GridCell,CeilingCell> grid;
    // Combine nearby samples around the peak without joining distinct levels.
    for (auto it=bands.lower_bound(level_bin-1); it!=bands.end() && it->first<=level_bin+1; ++it) {
        for (const auto& [xy,height] : it->second) {
            if (std::abs(height-level_bin*ceiling_band_m)>.13) continue;
            const GridCell key{static_cast<int>(std::floor(xy[0]/ceiling_grid_m)),
                               static_cast<int>(std::floor(xy[1]/ceiling_grid_m))};
            auto& cell=grid[key];
            cell.sum[0]+=xy[0];cell.sum[1]+=xy[1];cell.height+=height;++cell.count;
        }
    }
    models::RoomLayout layout;
    for (auto& [start, cell] : grid) {
        if (cell.visited) continue;
        cell.visited=true;
        std::vector<GridCell> pending{start};
        std::vector<Point2> points;
        double height_sum{};
        for (std::size_t i=0; i<pending.size(); ++i) {
            const auto key=pending[i];
            const auto& current=grid.at(key);
            points.push_back(current.center());height_sum+=current.height/current.count;
            for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) {
                auto it=grid.find({key[0]+dx,key[1]+dy});
                if (it==grid.end() || it->second.visited) continue;
                it->second.visited=true;
                pending.push_back(it->first);
            }
        }
        if (points.size()<9) continue;
        const auto height=height_sum/points.size();
        const auto hull=ceiling_hull(points);
        for (std::size_t i=0; i<hull.size(); ++i)
            trace_ceiling_edge(layout,basis,points,hull[i],hull[(i+1)%hull.size()],height);
    }
    return layout;
}

void append_ceiling_traces(models::RoomLayout& layout, const FloorBasis& basis, const CeilingBands& bands,
                           const std::function<bool()>& cancelled) {
    std::vector<std::pair<std::size_t,int>> peaks;
    for (const auto& [bin,samples] : bands) if (samples.size()>=9) peaks.emplace_back(samples.size(),bin);
    std::sort(peaks.rbegin(),peaks.rend());
    std::vector<int> processed;
    std::vector<std::vector<models::RoomLayoutSegment>> levels;
    for (const auto& [_,bin] : peaks) {
        if (cancelled && cancelled()) return;
        if (std::any_of(processed.begin(),processed.end(),[=](int other) {return std::abs(bin-other)<=2;})) continue;
        processed.push_back(bin);
        auto layer=trace_ceiling_band(basis,bands,bin);
        if (!layer.segments.empty()) levels.push_back(std::move(layer.segments));
        if (levels.size()>=max_ceiling_levels || processed.size()>=2*max_ceiling_levels) break;
    }
    const auto length=[](const auto& line) {
        const Point3 d{line.end[0]-line.start[0],line.end[1]-line.start[1],line.end[2]-line.start[2]};
        return dot(d,d);
    };
    for (auto& level : levels) {
        std::stable_sort(level.begin(),level.end(),[&](const auto& a,const auto& b) {
            if (a.inferred!=b.inferred) return !a.inferred;
            return length(a)>length(b);
        });
    }
    // A large upper ceiling must not exhaust the budget before a lower patch.
    std::vector<models::RoomLayoutSegment> traces;
    for (std::size_t rank=0; traces.size()<max_ceiling_segments; ++rank) {
        const auto before=traces.size();
        for (const auto& level : levels) {
            if (rank<level.size()) traces.push_back(level[rank]);
            if (traces.size()>=max_ceiling_segments) break;
        }
        if (traces.size()==before) break;
    }
    const auto keep=std::min(layout.segments.size(),max_segments-traces.size());
    layout.segments.resize(keep);
    layout.segments.insert(layout.segments.end(),traces.begin(),traces.end());
}
} // namespace

models::RoomLayout estimate_room_layout(const std::vector<RoomLayoutSample>& samples,
    const RoomLayoutReference& reference, double voxel_size_m, const std::function<bool()>& cancelled) {
    models::RoomLayout result;
    const auto length = std::sqrt(dot(reference.world_up,reference.world_up));
    if (!(length>0) || !std::isfinite(length)) return result;
    FloorBasis basis{reference.lidar_position,{},{},reference.world_up};
    for (auto& v : basis.up) v /= length;
    const Point3 seed = std::abs(basis.up[2])>.85 ? Point3{0,1,0} : Point3{0,0,1};
    basis.x = cross(seed,basis.up);
    const auto x_length = std::sqrt(dot(basis.x,basis.x));
    for (auto& v : basis.x) v /= x_length;
    basis.y = cross(basis.up,basis.x);

    std::vector<WallHit> vertical;
    std::map<std::pair<int,int>,std::size_t> votes;
    std::map<int,HorizontalBin> floor_bins, ceiling_bins;
    CeilingBands overhead;
    for (const auto& sample : samples) {
        const Point3 p{sample.position[0]-basis.origin[0],sample.position[1]-basis.origin[1],sample.position[2]-basis.origin[2]};
        const Point3 n{sample.normal[0],sample.normal[1],sample.normal[2]};
        const Point2 xy{dot(p,basis.x),dot(p,basis.y)};
        const auto h = dot(p,basis.up), nz = std::abs(dot(n,basis.up));
        if (!(dot(xy,xy)<=max_room_radius*max_room_radius) || !(std::abs(h)<8)) continue;
        if (nz>=.96) {
            const auto bin = static_cast<int>(std::floor(h/.12+.5));
            if (h<-.25) floor_bins[bin].add(xy,h);
            if (h>.55) ceiling_bins[bin].add(xy,h);
            if (h>.32) overhead[bin].emplace_back(xy,h);
        }
        if (nz>.30) continue;
        Point2 normal{dot(n,basis.x),dot(n,basis.y)};
        const auto norm = std::sqrt(dot(normal,normal));
        if (!(norm>.5)) continue;
        for (auto& v : normal) v /= norm;
        if (dot(normal,xy)<0) {normal[0] *= -1; normal[1] *= -1;}
        const auto angle_bin = static_cast<int>(std::lround(std::atan2(normal[1],normal[0])/angle_step));
        for (int shift=-1; shift<=1; ++shift) {
            const int cell = (angle_bin+shift+angle_bins)%angle_bins;
            const Point2 direction{std::cos(cell*angle_step),std::sin(cell*angle_step)};
            const auto d = dot(direction,xy);
            if (d>=.18) ++votes[{cell,static_cast<int>(std::lround(d/offset_step))}];
        }
        vertical.push_back({xy,normal,h});
    }

    using Peak = std::pair<std::pair<int,int>,std::size_t>;
    std::vector<Peak> peaks(votes.begin(),votes.end());
    std::stable_sort(peaks.begin(),peaks.end(),[](const Peak& a,const Peak& b){return a.second>b.second;});
    std::vector<Wall> walls;
    const auto tolerance = std::clamp(voxel_size_m*2.0,.085,.16);
    for (std::size_t i=0; i<std::min<std::size_t>(48,peaks.size()) && walls.size()<max_walls; ++i) {
        if (cancelled && cancelled()) return {};
        if (peaks[i].second<6) break;
        const auto [angle, offset] = peaks[i].first;
        Wall wall;
        wall.normal = {std::cos(angle*angle_step),std::sin(angle*angle_step)};
        wall.offset = offset*offset_step;
        fit_wall(vertical,tolerance,wall);
        // Broad support at multiple heights removes tiny object edges.
        if (wall.hits.size()<12 || wall.high-wall.low<.85 || wall.high_height-wall.low_height<.45)
            continue;
        if (std::any_of(walls.begin(),walls.end(),[&](const Wall& item) {
            return dot(item.normal,wall.normal)>.985 &&
                std::abs(item.offset-wall.offset)<2*tolerance;
        }))
            continue;
        walls.push_back(std::move(wall));
    }
    auto floor = level(floor_bins);
    if (floor) result.floor = models::RoomLayoutFloor::measured;
    else if (reference.floor_datum) {
        const auto& datum = *reference.floor_datum;
        const auto h = dot(Point3{datum[0]-basis.origin[0],datum[1]-basis.origin[1],datum[2]-basis.origin[2]},basis.up);
        if (h<-.15 && h>-8) { floor=h;result.floor=models::RoomLayoutFloor::estimated; }
    }
    const auto ceiling=level(ceiling_bins);
    for (auto& wall : walls) {
        wall.trace_height=ceiling && std::abs(*ceiling-wall.high_height)<=.30 ? *ceiling : wall.high_height;
        find_wall_spans(wall,floor);
    }

    // Match only nearby ends with common height support. Other walls do not
    // define a convex boundary around the sensor in open or irregular layouts.
    std::vector<CornerCandidate> candidates;
    for (std::size_t i=0; i<walls.size(); ++i) {
        auto& a=walls[i];
        for (std::size_t j=i+1; j<walls.size(); ++j) {
            auto& b=walls[j];
            const auto p=intersect(a,b);
            if (!p) continue;
            const auto left=nearest_end(a,*p), right=nearest_end(b,*p);
            if (!left.span || !right.span) continue;
            const auto a_heights=height_support(a,{left.along-.28,left.along+.28});
            const auto b_heights=height_support(b,{right.along-.28,right.along+.28});
            const auto low=std::max(a_heights.first,b_heights.first), high=std::min(a_heights.second,b_heights.second);
            if (high-low>=.20) candidates.push_back({&a,&b,left,right,*p,low,high});
        }
    }
    std::stable_sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b) {
        return a.left.distance+a.right.distance<b.left.distance+b.right.distance;
    });
    for (const auto& candidate : candidates) {
        if (result.corners.size()>=max_walls) break;
        const auto& [a,b,left,right,position,low,high]=candidate;
        if (left.span->joins[left.side] || right.span->joins[right.side]) continue;
        left.span->joins[left.side]=right.span;
        right.span->joins[right.side]=left.span;
        const auto top=std::min(a->trace_height,b->trace_height);
        const bool inferred=left.distance>.22 || right.distance>.22 || top-high>.26;
        const auto at=[&](double height) {return basis.world(position,height);};
        result.corners.push_back({at(top),inferred});
        segment(result,basis.world(a->at(left.along),a->trace_height),at(top),true);
        segment(result,basis.world(b->at(right.along),b->trace_height),at(top),true);
        if (!floor || *floor>=top) continue;
        segment(result,basis.world(a->at(left.along),*floor),at(*floor),true);
        segment(result,basis.world(b->at(right.along),*floor),at(*floor),true);
        const auto from=std::max(*floor,low), to=std::min(top,high);
        if (inferred || from>=to) segment(result,at(*floor),at(top),true);
        else {
            segment(result,at(*floor),at(from),true);
            segment(result,at(from),at(to),false);
            segment(result,at(to),at(top),true);
        }
    }
    result.closed=is_closed(walls);
    for (const auto& wall : walls) draw_wall(result,basis,wall,floor);
    append_ceiling_traces(result,basis,overhead,cancelled);
    return result;
}

} // namespace vista::application
