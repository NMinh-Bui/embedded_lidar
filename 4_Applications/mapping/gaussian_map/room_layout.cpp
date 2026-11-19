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
struct Wall {
    Point2 normal;
    double offset{};
    std::vector<std::pair<double,double>> hits; // Tangential coordinate and height.
    double low{}, high{}, low_height{}, high_height{};
    double angle() const { return std::atan2(normal[1], normal[0]); }
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

std::optional<Point2> intersect(const Wall& left, const Wall& right, const std::vector<Wall>& walls) {
    const auto& a = left.normal;
    const auto& b = right.normal;
    const double det = a[0]*b[1]-a[1]*b[0];
    // The angularly adjacent walls must form a finite interior corner.
    if (det<.30) return std::nullopt;
    const Point2 p{(left.offset*b[1]-a[1]*right.offset)/det,
                   (a[0]*right.offset-left.offset*b[0])/det};
    if (dot(p,p)>max_room_radius*max_room_radius ||
        std::any_of(walls.begin(),walls.end(),[&](const auto& w){return dot(w.normal,p)>w.offset+.20;}))
        return std::nullopt;
    return p;
}
bool within_support(const Wall& w, const Point2& p, double allowance) {
    const auto along = dot(tangent(w.normal),p);
    return along>=w.low-allowance && along<=w.high+allowance;
}
std::pair<double,double> height_support(const Wall& wall, const Point2& p) {
    const auto along = dot(tangent(wall.normal),p);
    double low = std::numeric_limits<double>::infinity(), high = -low;
    for (const auto& hit : wall.hits) if (std::abs(hit.first-along)<=.28) {
        low = std::min(low,hit.second);
        high = std::max(high,hit.second);
    }
    return {low,high};
}
struct Corner {
    Point2 xy;
    double top{};
    bool inferred{};
};

void segment(models::RoomLayout& result, const Point3& start, const Point3& end, bool inferred) {
    if (result.segments.size()>=max_segments) return;
    const Point3 d{start[0]-end[0],start[1]-end[1],start[2]-end[2]};
    if (dot(d,d)>.0004) result.segments.push_back({start,end,inferred});
}
void rail(models::RoomLayout& result, const FloorBasis& basis, const Wall& wall,
          double low, double high, double h, bool estimated) {
    const auto axis = tangent(wall.normal);
    const auto world = [&](double t) {
        return basis.world({wall.normal[0]*wall.offset+axis[0]*t,
                            wall.normal[1]*wall.offset+axis[1]*t}, h);
    };
    if (estimated) {
        segment(result,world(low),world(high),true);
        return;
    }
    // Merge measured runs across normal sample spacing, leaving wider holes dashed.
    double cursor = low;
    for (auto it = wall.hits.begin(); it!=wall.hits.end();) {
        const auto start = it->first;
        auto end = start;
        for (++it; it!=wall.hits.end() && it->first-end<=.30; ++it) end = it->first;
        const auto a = std::max(low,start-.10), b = std::min(high,end+.10);
        if (a>=b || b<=cursor) continue;
        if (a>cursor) segment(result,world(cursor),world(a),true);
        segment(result,world(std::max(cursor,a)),world(b),false);
        cursor = b;
    }
    if (cursor<high) segment(result,world(cursor),world(high),true);
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
        if (std::any_of(walls.begin(),walls.end(),[&](const Wall& item){return dot(item.normal,wall.normal)>.95;}))
            continue;
        walls.push_back(std::move(wall));
    }
    if (walls.size()<2) return result;
    std::sort(walls.begin(),walls.end(),[](const auto& a,const auto& b){return a.angle()<b.angle();});

    auto floor = level(floor_bins);
    if (floor) result.floor = models::RoomLayoutFloor::measured;
    else if (reference.floor_datum) {
        const auto& datum = *reference.floor_datum;
        const auto h = dot(Point3{datum[0]-basis.origin[0],datum[1]-basis.origin[1],datum[2]-basis.origin[2]},basis.up);
        if (h<-.15 && h>-8) { floor=h;result.floor=models::RoomLayoutFloor::estimated; }
    }
    const auto ceiling = level(ceiling_bins);
    std::vector<std::optional<Corner>> corners(walls.size());
    std::vector<std::optional<Point2>> candidates(walls.size());
    std::size_t found = 0;
    const auto add_corner = [&](std::size_t i, const Point2& xy, bool inferred) {
        const auto& a = walls[i];
        const auto& b = walls[(i+1)%walls.size()];
        const auto a_heights = height_support(a,xy), b_heights = height_support(b,xy);
        const auto support_top = std::min(a_heights.second,b_heights.second);
        const auto top = ceiling.value_or(std::min(a.high_height,b.high_height));
        const bool extrapolated = inferred || support_top<top-.26;
        corners[i] = Corner{xy,top,extrapolated};
        result.corners.push_back({basis.world(xy,top),extrapolated});
        ++found;
    };
    for (std::size_t i=0; i<walls.size(); ++i) {
        const auto& a = walls[i];
        const auto& b = walls[(i+1)%walls.size()];
        candidates[i] = intersect(a,b,walls);
        if (candidates[i] && within_support(a,*candidates[i],.65) && within_support(b,*candidates[i],.65))
            add_corner(i,*candidates[i],!within_support(a,*candidates[i],.22)||!within_support(b,*candidates[i],.22));
    }
    if (walls.size()==4 && found==3) {
        bool rectangle=true;
        for (std::size_t i=0; i<walls.size(); ++i)
            rectangle = rectangle && std::abs(dot(walls[i].normal,walls[(i+1)%walls.size()].normal))<.14;
        if (rectangle) for (std::size_t i=0; i<walls.size(); ++i)
            if (!corners[i] && candidates[i]) add_corner(i,*candidates[i],true);
    }
    result.closed = walls.size()>=3 && found==walls.size();
    if (!found) return result;

    for (std::size_t i=0; i<walls.size(); ++i) {
        if (!corners[i] || !floor || *floor>=corners[i]->top) continue;
        const auto& corner = *corners[i];
        const auto a = height_support(walls[i],corner.xy);
        const auto b = height_support(walls[(i+1)%walls.size()],corner.xy);
        const auto lo = std::max({*floor,a.first,b.first});
        const auto hi = std::min({corner.top,a.second,b.second});
        const auto point = [&](double h){return basis.world(corner.xy,h);};
        if (corner.inferred || lo>=hi) segment(result,point(*floor),point(corner.top),true);
        else {
            segment(result,point(*floor),point(lo),true);
            segment(result,point(lo),point(hi),false);
            segment(result,point(hi),point(corner.top),true);
        }
    }
    for (std::size_t i=0; i<walls.size(); ++i) {
        const auto& wall = walls[i];
        const auto& previous = corners[(i+walls.size()-1)%walls.size()];
        const auto& next = corners[i];
        if (!previous && !next) continue;
        const auto axis = tangent(wall.normal);
        double a = previous ? dot(axis,previous->xy) : wall.low;
        double b = next ? dot(axis,next->xy) : wall.high;
        if (a>b) std::swap(a,b);
        const auto top = ceiling.value_or(wall.high_height);
        // Keep rail heights consistent at shared corner endpoints when no ceiling is available.
        if (ceiling) rail(result,basis,wall,a,b,top,false);
        else {
            const auto start = previous ? basis.world(previous->xy,previous->top) :
                basis.world({wall.offset*wall.normal[0]+wall.low*axis[0],wall.offset*wall.normal[1]+wall.low*axis[1]},top);
            const auto end = next ? basis.world(next->xy,next->top) :
                basis.world({wall.offset*wall.normal[0]+wall.high*axis[0],wall.offset*wall.normal[1]+wall.high*axis[1]},top);
            segment(result,start,end,true);
        }
        if (floor) rail(result,basis,wall,a,b,*floor,result.floor==models::RoomLayoutFloor::estimated);
    }
    return result;
}

} // namespace vista::application
