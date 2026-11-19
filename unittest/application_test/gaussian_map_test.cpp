#include "unittest/test.hpp"
#include "4_Applications/mapping/gaussian_map/gaussian_map.hpp"
#include "2_Transport/storage/room_map_tiles.hpp"
#include "config.hpp"
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include "4_Applications/mapping/room_map/room_map.hpp"

namespace {
using Store=vista::transport::RoomMapTileStore;
using vista::models::GaussianSurfaceMode;
std::unique_ptr<Store> store() {
    auto s=std::make_unique<Store>(vista::fs::path{},.1F,.4F,1000,8,2,2);
    s->enable_surface_snapshots(true);return s;
}
void add(Store& s,int x,int y,int z=0) {
    s.insert({x,y,z},{{(x+.25F)*.1F,(y+.25F)*.1F,(z+.25F)*.1F,77},2,1,0});
}
}
VISTA_TEST(gaussian_regions_are_full_immutable_and_ignore_evidence_only_updates) {
    auto s=store();for(int x=0;x<8;++x) for(int y=0;y<4;++y) add(*s,x,y);
    auto first=s->view_source(1)->surface_snapshot();
    std::size_t count{};for(const auto& r:first->regions) r.second->read([&](const auto&){++count;});
    VISTA_CHECK(count==32); // More than both LOD reservoirs combined.
    const auto revision=first->geometry_revision;
    s->find({0,0,0})->observations++;s->mark_changed({0,0,0});
    auto same=s->view_source(2)->surface_snapshot();
    VISTA_CHECK(same->geometry_revision==revision && same->regions==first->regions);
    s->erase({0,0,0});auto latest=s->view_source(3)->surface_snapshot();
    VISTA_CHECK(latest->geometry_revision>revision);
    count=0;for(const auto& r:first->regions) r.second->read([&](const auto&){++count;});
    VISTA_CHECK(count==32); // Old revision survives atomic page replacement/deletion.
}

VISTA_TEST(gaussian_cache_cleanup_waits_for_retained_view_and_preserves_sibling_files) {
    auto s=store();add(*s,0,0);
    auto snapshot=s->view_source(1)->surface_snapshot();
    const auto parent=vista::fs::temp_directory_path()/("vista-gaussian-cleanup-"+
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    vista::fs::create_directory(parent);const auto sentinel=parent/"RoomMap.pcd";
    {vista::io::ofstream out(sentinel);out<<"keep";}
    std::shared_ptr<const vista::models::GaussianMapViewSource> held;
    vista::fs::path cache;
    {
        vista::application::GaussianMapBuilder builder(parent/"tiles.gaussian");
        held=builder.update(*snapshot);
        for(const auto& item:vista::fs::directory_iterator(parent)) if(item.is_directory()) cache=item.path();
        VISTA_CHECK(!cache.empty());
    }
    vista::models::RoomMapViewRequest request;request.point_budget=10;
    VISTA_CHECK(held->select_splats(request).size()==1 && vista::fs::exists(cache));
    held.reset();VISTA_CHECK(!vista::fs::exists(cache) && vista::fs::exists(sentinel));
    vista::fs::remove(sentinel);vista::fs::remove(parent);
}
VISTA_TEST(gaussian_surfaces_fit_planes_across_tiles_and_propagate_deletions) {
    auto s=store();for(int x=-4;x<8;++x) for(int y=-4;y<8;++y) add(*s,x,y);
    auto snapshot=s->view_source(1)->surface_snapshot();
    vista::application::GaussianMapBuilder builder;
    auto first=builder.update(*snapshot);
    vista::models::RoomMapViewRequest request;request.point_budget=1000;
    auto points=first->select_splats(request);VISTA_CHECK(points.size()==144);
    for(const auto& p:points) {
        VISTA_CHECK(p.covariance[0]>0 && p.covariance[3]>0 && p.covariance[5]>0);
        if(p.center.x>0 && p.center.x<.6F && p.center.y>0 && p.center.y<.6F)
            VISTA_CHECK(p.covariance[5]<p.covariance[0]*.1F);
    }
    const auto built=builder.rebuilt_tiles();builder.update(*snapshot);
    VISTA_CHECK(builder.rebuilt_tiles()==built);
    request.point_budget=7;VISTA_CHECK(first->select_splats(request).size()==7);
    for(int x=0;x<4;++x) for(int y=0;y<4;++y) s->erase({x,y,0});
    auto revised=s->view_source(2)->surface_snapshot();
    auto second=builder.update(*revised);request.point_budget=1000;
    VISTA_CHECK(second->select_splats(request).size()==128);
    VISTA_CHECK(first->select_splats(request).size()==144); // In-flight viewer remains coherent.
    VISTA_CHECK(builder.rebuilt_tiles()>built);
    vista::application::GaussianMapBuilder rebuilt;
    const auto fresh=rebuilt.update(*revised)->select_splats(request);
    const auto incremental=second->select_splats(request);
    VISTA_CHECK(fresh.size()==incremental.size());
    for(std::size_t i=0;i<fresh.size();++i) {
        VISTA_CHECK(fresh[i].center==incremental[i].center);
        for(unsigned c=0;c<6;++c) VISTA_CHECK(std::abs(fresh[i].covariance[c]-incremental[i].covariance[c])<1e-7F);
    }
    auto empty=*snapshot;empty.regions.clear();
    VISTA_CHECK(builder.update(empty)->select_splats(request).empty());
}
VISTA_TEST(gaussian_cancelled_update_keeps_last_complete_revision_and_packet_layout) {
    auto s=store();for(int x=0;x<4;++x) for(int y=0;y<4;++y) add(*s,x,y);
    vista::application::GaussianMapBuilder builder;
    auto source=s->view_source(1)->surface_snapshot();auto first=builder.update(*source);
    add(*s,5,0);auto updated=s->view_source(2)->surface_snapshot();
    VISTA_CHECK(!builder.update(*updated,[]{return true;}));
    vista::models::RoomMapViewRequest request;request.point_budget=1000;
    VISTA_CHECK(first->select_splats(request).size()==16);
    auto points=builder.update(*updated)->select_splats(request);
    auto packet=vista::application::encode_gaussian_map(points,42,123);
    VISTA_CHECK(packet.size()==32+17*44 && std::memcmp(packet.data(),"GSP1",4)==0);
    std::uint32_t count{},stride{};std::memcpy(&count,packet.data()+8,4);std::memcpy(&stride,packet.data()+12,4);
    VISTA_CHECK(count==17 && stride==44);
}
VISTA_TEST(gaussian_configuration_retires_only_live_socket_and_preserves_sensor_health_path) {
    const auto config=vista::parse_device_config_text("Lidar: UnitreeL2\nConnectionMode: serial\nSerialPort: COM6\nBaudRate: 4000000\nGrafanaEnabled: 1\nRoomMapEnabled: 1\nGaussianMapEnabled: 1\n");
    const auto runtime=config.runtime_config();
    VISTA_CHECK(!config.pointcloud_websocket.enabled && !runtime.threads.pointcloud_websocket.enabled);
    VISTA_CHECK(runtime.threads.live_preprocessing.enabled && runtime.threads.grafana_bridge.enabled);
    VISTA_CHECK(runtime.threads.gaussian_mapping.enabled && runtime.threads.gaussian_map_websocket.enabled);
    VISTA_CHECK(config.room_map.surface_snapshots && config.gaussian_map_websocket.port==8767);
}

VISTA_TEST(gaussian_loaded_reference_preserves_floor_and_deduplicates_repeated_points) {
    vista::application::RoomMapConfig config;config.surface_snapshots=true;
    config.voxel_size_m=.1F;config.tile_size_m=.4F;
    vista::application::RoomMapAccumulator map(config);
    vista::models::RoomMapFrame frame;
    for(int x=-4;x<8;++x) for(int y=-4;y<8;++y) {
        frame.points.push_back({(x+.25F)*.1F,(y+.25F)*.1F,0,33});
        frame.points.push_back(frame.points.back());
    }
    map.restore(frame);
    vista::application::GaussianMapBuilder builder;
    auto view=builder.update(*map.view_source()->surface_snapshot());
    vista::models::RoomMapViewRequest request;request.point_budget=1000;
    const auto points=view->select_splats(request);
    VISTA_CHECK(points.size()==144);
    for(const auto& p:points) VISTA_CHECK(p.center.z==0);
}

VISTA_TEST(gaussian_bulk_page_records_preserve_unsigned_intensity_and_sparse_view_selection) {
    auto s=store();for(int x=-5;x<8;++x)for(int y=-3;y<6;++y){add(*s,x,y);
        s->find({x,y,0})->point.intensity=255;s->mark_changed({x,y,0});}
    vista::application::GaussianMapBuilder builder;const auto view=builder.update(*s->view_source(1)->surface_snapshot());
    vista::models::RoomMapViewRequest request;request.point_budget=1000;const auto all=view->select_splats(request);
    VISTA_CHECK(all.size()==117);for(const auto& p:all)VISTA_CHECK(p.center.intensity==255);
    request.point_budget=7;const auto sparse=view->select_splats(request);VISTA_CHECK(sparse.size()==7);
    for(std::size_t i=0;i<sparse.size();++i){const auto index=static_cast<std::size_t>((i+.5L)*all.size()/sparse.size());
        VISTA_CHECK(sparse[i].center==all[index].center && sparse[i].covariance==all[index].covariance && sparse[i].opacity==all[index].opacity);}
}

VISTA_TEST(gaussian_rotated_plane_has_thin_normal_covariance_across_region_boundaries) {
    auto s=store();
    for(int x=-6;x<7;++x) for(int y=-6;y<7;++y) {
        const float px=(x+.25F)*.1F,py=(y+.25F)*.1F,pz=.3F*px+.4F*py;
        s->insert({x,y,static_cast<std::int64_t>(std::floor(pz/.1))},{{px,py,pz,55},2,1,0});
    }
    vista::application::GaussianMapBuilder builder;
    auto view=builder.update(*s->view_source(1)->surface_snapshot());
    vista::models::RoomMapViewRequest request;request.point_budget=1000;
    const auto points=view->select_splats(request);VISTA_CHECK(points.size()==169);
    const double length=std::sqrt(1.25),n[3]{-.3/length,-.4/length,1/length};
    unsigned validated{};
    for(const auto& p:points) if(std::abs(p.center.x)<.4 && std::abs(p.center.y)<.4) {
        const auto& c=p.covariance;
        const auto normal_variance=c[0]*n[0]*n[0]+2*c[1]*n[0]*n[1]+2*c[2]*n[0]*n[2]+
            c[3]*n[1]*n[1]+2*c[4]*n[1]*n[2]+c[5]*n[2]*n[2];
        VISTA_CHECK(normal_variance>0 && normal_variance<(c[0]+c[3]+c[5])*.03);
        ++validated;
    }
    VISTA_CHECK(validated>20);
}

VISTA_TEST(gaussian_optional_snapshot_failure_does_not_break_room_map_checkpoint) {
    auto s=store();add(*s,0,0);auto first=s->view_source(1);
    vista::fs::path leaf;
    for(const auto& entry:vista::fs::recursive_directory_iterator(s->directory()))
        if(entry.path().filename()=="cells.bin") leaf=entry.path().parent_path();
    VISTA_CHECK(!leaf.empty());
    // Force only the optional immutable link/copy to fail; the map page is writable.
    vista::fs::create_directories(leaf/"surface-2.bin");
    s->find({0,0,0})->point.intensity=99;s->mark_changed({0,0,0});
    auto latest=s->view_source(2);
    VISTA_CHECK(!latest->surface_snapshot()->error.empty());
    vista::models::RoomMapViewRequest request;request.point_budget=10;
    const auto geometry=latest->select_view(request);
    VISTA_CHECK(geometry.points.size()==1 && geometry.points[0].intensity==99);
    vista::application::GaussianMapBuilder builder;
    VISTA_CHECK_THROWS(builder.update(*latest->surface_snapshot()));
}

// Optional real-map validation without storing a capture/test fixture in Git.
VISTA_TEST(gaussian_edges_remove_flat_interiors_without_tile_seams_and_keep_boundaries) {
    auto s=store();for(int x=-12;x<=12;++x) for(int y=-12;y<=12;++y) add(*s,x,y);
    add(*s,40,40); // An isolated return is not an edge.
    vista::application::GaussianMapBuilder builder;
    const auto snapshot=s->view_source(1)->surface_snapshot();
    const auto view=builder.update(*snapshot);
    vista::models::RoomMapViewRequest request;request.point_budget=2000;
    const auto full=view->select_splats(request);VISTA_CHECK(full.size()==626);
    request.surface_mode=GaussianSurfaceMode::edges;const auto edges=view->select_splats(request);
    VISTA_CHECK(edges.size()>20 && edges.size()<full.size()/2);
    for(const auto& point:edges) {
        VISTA_CHECK(std::abs(point.center.x)>.9F || std::abs(point.center.y)>.9F);
        VISTA_CHECK(point.center.x<3 && point.center.y<3);
    }
    const auto built=builder.rebuilt_tiles();request.point_budget=7;
    VISTA_CHECK(view->select_splats(request).size()==7);
    VISTA_CHECK(builder.rebuilt_tiles()==built);
    request.surface_mode=GaussianSurfaceMode::full_surface;request.point_budget=2000;
    const auto restored=view->select_splats(request);VISTA_CHECK(restored.size()==full.size());
    for(std::size_t i=0;i<full.size();++i) VISTA_CHECK(restored[i].center==full[i].center && restored[i].covariance==full[i].covariance);
}

VISTA_TEST(gaussian_edges_keep_supported_creases_and_rebuild_after_neighbor_deletions) {
    auto s=store();
    for(int x=0;x<=16;++x) for(int y=-10;y<=10;++y) add(*s,x,y);
    for(int z=1;z<=16;++z) for(int y=-10;y<=10;++y) add(*s,0,y,z);
    vista::application::GaussianMapBuilder builder;
    const auto first=builder.update(*s->view_source(1)->surface_snapshot());
    vista::models::RoomMapViewRequest request;request.point_budget=2000;request.surface_mode=GaussianSurfaceMode::edges;
    const auto old=first->select_splats(request);unsigned crease{};
    for(const auto& p:old) if(p.center.x<.2F && p.center.z<.2F && std::abs(p.center.y)<.5F) ++crease;
    VISTA_CHECK(crease>=8);
    for(int x=0;x<8;++x) for(int y=-3;y<=3;++y) s->erase({x,y,0});
    const auto source=s->view_source(2)->surface_snapshot();
    const auto updated=builder.update(*source);vista::application::GaussianMapBuilder fresh;
    const auto expected=fresh.update(*source)->select_splats(request),actual=updated->select_splats(request);
    VISTA_CHECK(actual.size()==expected.size());
    for(std::size_t i=0;i<actual.size();++i) VISTA_CHECK(actual[i].center==expected[i].center && actual[i].covariance==expected[i].covariance);
    VISTA_CHECK(first->select_splats(request).size()==old.size());
    auto empty=*source;empty.regions.clear();VISTA_CHECK(builder.update(empty)->select_splats(request).empty());
}

namespace {
using vista::application::RoomLayoutSample;

std::vector<RoomLayoutSample> room_layout_samples(bool west, bool floor, bool hidden_corner = false) {
    std::vector<RoomLayoutSample> points;
    for (int along=-15; along<=15; ++along) for (int h=2; h<=22; ++h) {
        const auto a=along*.1F, z=h*.1F;
        if (!hidden_corner || along<5) points.push_back({{1.5F,a,z},{1,0,0}});
        if (!hidden_corner || along<5) points.push_back({{a,1.5F,z},{0,1,0}});
        points.push_back({{a,-1.5F,z},{0,1,0}});
        if (west) points.push_back({{-1.5F,a,z},{1,0,0}});
    }
    for (int x=-14; x<=14; ++x) for (int y=-14; y<=14; ++y) {
        points.push_back({{x*.1F,y*.1F,2.3F},{0,0,1}});
        if (floor) points.push_back({{x*.1F,y*.1F,0},{0,0,1}});
    }
    // A monitor-sized vertical patch inside the room should not become a wall.
    for (int y=-3; y<=3; ++y) for (int h=6; h<=10; ++h)
        points.push_back({{.5F,y*.1F,h*.1F},{1,0,0}});
    return points;
}
}
VISTA_TEST(gaussian_room_layout_fits_room_walls_and_projects_along_gravity) {
    auto points=room_layout_samples(true,true);
    // Rotate the room and gravity in world coordinates. The floor is tilted
    // relative to world Z, so a hardcoded vertical axis gives wrong feet.
    constexpr double a=.24;
    const auto rotate=[a](const std::array<float,3>& p) {
        return std::array<float,3>{p[0],static_cast<float>(p[1]*std::cos(a)-p[2]*std::sin(a)),
            static_cast<float>(p[1]*std::sin(a)+p[2]*std::cos(a))};
    };
    for (auto& p : points) {p.position=rotate(p.position);p.normal=rotate(p.normal);}
    vista::application::RoomLayoutReference reference;
    reference.lidar_position={0,-.7*std::sin(a),.7*std::cos(a)};
    reference.world_up={0,-std::sin(a),std::cos(a)};
    const auto layout=vista::application::estimate_room_layout(points,reference,.05);
    VISTA_CHECK(layout.closed && layout.corners.size()==4);
    VISTA_CHECK(layout.floor==vista::models::RoomLayoutFloor::measured);
    std::size_t feet=0;
    for (const auto& segment : layout.segments) {
        const auto height=[&](const auto& p) {return -p[1]*std::sin(a)+p[2]*std::cos(a);};
        if (std::abs(height(segment.start))>.03 || height(segment.end)<.12) continue;
        VISTA_CHECK_NEAR(segment.start[0],segment.end[0],1e-6);
        const auto dy=segment.end[1]-segment.start[1], dz=segment.end[2]-segment.start[2];
        VISTA_CHECK_NEAR(dy*std::cos(a)+dz*std::sin(a),0,1e-6);
        ++feet;
    }
    VISTA_CHECK(feet==4);
}
VISTA_TEST(gaussian_room_layout_keeps_open_room_and_labels_estimated_floor) {
    vista::application::RoomLayoutReference reference;
    reference.lidar_position={0,0,.7};
    reference.floor_datum=vista::models::RoomLayoutPosition{0,0,0};
    const auto layout=vista::application::estimate_room_layout(room_layout_samples(false,false),reference,.05);
    VISTA_CHECK(!layout.closed && layout.corners.size()==2);
    VISTA_CHECK(layout.floor==vista::models::RoomLayoutFloor::estimated);
    for (const auto& segment : layout.segments)
        if (std::min(segment.start[2],segment.end[2])==0) VISTA_CHECK(segment.inferred);
    reference.floor_datum.reset();
    const auto unknown=vista::application::estimate_room_layout(room_layout_samples(false,false),reference,.05);
    VISTA_CHECK(unknown.floor==vista::models::RoomLayoutFloor::unknown && unknown.corners.size()==2);
    for (const auto& segment : unknown.segments)
        VISTA_CHECK(segment.start[2]>1 && segment.end[2]>1);
}
VISTA_TEST(gaussian_room_layout_leaves_large_corner_holes_open) {
    vista::application::RoomLayoutReference reference;
    reference.lidar_position={0,0,.7};
    const auto layout=vista::application::estimate_room_layout(room_layout_samples(true,true,true),reference,.05);
    VISTA_CHECK(!layout.closed && layout.corners.size()==3);
    for (const auto& corner : layout.corners)
        VISTA_CHECK(corner.position[0]<1 || corner.position[1]<1);
}

VISTA_TEST(gaussian_room_layout_projects_an_isolated_wall_and_marks_measured_base) {
    using vista::models::RoomLayoutFloor;
    std::vector<RoomLayoutSample> wall;
    for (int y=-16; y<=16; ++y) for (int z=y<=-5 ? 0 : 5; z<=22; ++z)
        wall.push_back({{2,y*.1F,z*.1F},{1,0,0}});
    vista::application::RoomLayoutReference reference;
    reference.lidar_position={0,0,.7};
    const auto unknown=vista::application::estimate_room_layout(wall,reference,.05);
    VISTA_CHECK(unknown.floor==RoomLayoutFloor::unknown && unknown.corners.empty() && !unknown.closed);
    VISTA_CHECK(unknown.segments.size()==1);
    const auto& upper=unknown.segments.front();
    VISTA_CHECK(!upper.inferred);
    VISTA_CHECK_NEAR(upper.start[2],2.2,1e-5);
    VISTA_CHECK_NEAR(upper.start[2],upper.end[2],1e-6);

    reference.floor_datum=vista::models::RoomLayoutPosition{0,0,0};
    const auto estimated=vista::application::estimate_room_layout(wall,reference,.05);
    VISTA_CHECK(estimated.floor==RoomLayoutFloor::estimated && estimated.segments.size()==2);
    const auto& base=estimated.segments.back();
    VISTA_CHECK(base.inferred);
    VISTA_CHECK_NEAR(base.start[2],0,1e-6);
    VISTA_CHECK_NEAR(base.end[2],0,1e-6);
    for (unsigned axis=0; axis<2; ++axis) {
        VISTA_CHECK_NEAR(upper.start[axis],base.start[axis],1e-6);
        VISTA_CHECK_NEAR(upper.end[axis],base.end[axis],1e-6);
    }

    for (int x=-12; x<=23; ++x) for (int y=-18; y<=18; ++y)
        wall.push_back({{x*.1F,y*.1F,0},{0,0,1}});
    const auto measured=vista::application::estimate_room_layout(wall,reference,.05);
    VISTA_CHECK(measured.floor==RoomLayoutFloor::measured);
    VISTA_CHECK(measured.segments.size()==3);
    const auto& supported=measured.segments[1];
    VISTA_CHECK(!supported.inferred && measured.segments[2].inferred);
    VISTA_CHECK_NEAR(supported.start[2],0,1e-6);
    VISTA_CHECK_NEAR(supported.end[1],-.5,1e-6);
    VISTA_CHECK_NEAR(measured.segments[2].end[1],upper.end[1],1e-6);
}

VISTA_TEST(gaussian_room_layout_preserves_open_doorways_and_dashes_short_gaps) {
    std::vector<RoomLayoutSample> points;
    for (int y=-30; y<=30; ++y) {
        if (y>=13 && y<=15) continue; // 40 cm gap in the samples.
        for (int z=(y>-5 && y<5 ? 21 : 2); z<=24; ++z)
            points.push_back({{3,y*.1F,z*.1F},{1,0,0}}); // Wall above the doorway is still visible.
    }
    vista::application::RoomLayoutReference reference;
    reference.lidar_position={0,0,.7};
    reference.floor_datum=vista::models::RoomLayoutPosition{0,0,0};
    const auto layout=vista::application::estimate_room_layout(points,reference,.05);
    VISTA_CHECK(!layout.closed && layout.corners.empty());
    std::size_t short_gaps=0;
    for (const auto& segment : layout.segments) {
        const auto lo=std::min(segment.start[1],segment.end[1]);
        const auto hi=std::max(segment.start[1],segment.end[1]);
        VISTA_CHECK(lo>=.49 || hi<=-.49);
        if (lo<1.4 && hi>1.4) {
            VISTA_CHECK(segment.inferred);
            VISTA_CHECK_NEAR(hi-lo,.4,1e-6);
            ++short_gaps;
        }
    }
    VISTA_CHECK(short_gaps==2);
}

VISTA_TEST(gaussian_room_layout_keeps_parallel_walls_and_matches_oblique_ends) {
    std::vector<RoomLayoutSample> points;
    for (int i=0; i<=25; ++i) for (int z=5; z<=22; ++z) {
        const auto t=i*.1F, h=z*.1F;
        points.push_back({{2,-t,h},{1,0,0}});
        points.push_back({{2+.6F*t,.8F*t,h},{.8F,-.6F,0}});
        points.push_back({{5,t,h},{1,0,0}});
    }
    vista::application::RoomLayoutReference reference;
    reference.lidar_position={0,0,.7};
    reference.floor_datum=vista::models::RoomLayoutPosition{0,0,0};
    const auto layout=vista::application::estimate_room_layout(points,reference,.05);
    VISTA_CHECK(!layout.closed && layout.corners.size()==1);
    VISTA_CHECK_NEAR(layout.corners[0].position[0],2,.06);
    VISTA_CHECK_NEAR(layout.corners[0].position[1],0,.06);
    for (const double offset : {2.,5.}) {
        VISTA_CHECK(std::any_of(layout.segments.begin(),layout.segments.end(),[=](const auto& line) {
            return std::abs(line.start[0]-offset)<.03 && std::abs(line.end[0]-offset)<.03 &&
                std::abs(line.start[2])<.01 && std::abs(line.end[1]-line.start[1])>2.;
        }));
    }
}
VISTA_TEST(gaussian_room_layout_traces_short_low_ceiling_below_a_higher_ceiling_without_walls) {
    std::vector<RoomLayoutSample> points;
    for (int x=-8; x<=8; ++x) for (int y=-8; y<=8; ++y)
        points.push_back({{x*.05F,y*.05F,1.48F},{0,0,1}});
    for (int x=20; x<=64; ++x) for (int y=-22; y<=22; ++y)
        points.push_back({{x*.05F,y*.05F,3.38F},{0,0,1}});
    // A surface below the LiDAR and an isolated overhead strip are not ceiling patches.
    for (int x=-10; x<=10; ++x) for (int y=-10; y<=10; ++y)
        points.push_back({{x*.1F,y*.1F,.4F},{0,0,1}});
    for (int x=-24; x<=24; ++x) points.push_back({{x*.05F,3.F,2.3F},{0,0,1}});
    vista::application::RoomLayoutReference reference;
    reference.lidar_position={0,0,.7};
    reference.floor_datum=vista::models::RoomLayoutPosition{0,0,0};
    const auto layout=vista::application::estimate_room_layout(points,reference,.05);
    VISTA_CHECK(layout.corners.empty() && !layout.closed);
    unsigned low=0, high=0;
    for (const auto& line : layout.segments) {
        VISTA_CHECK_NEAR(line.start[2],line.end[2],1e-6);
        if (std::abs(line.start[2]-1.48)<.03) ++low;
        else if (std::abs(line.start[2]-3.38)<.03) ++high;
        else VISTA_CHECK(false);
    }
    VISTA_CHECK(low>=2 && high>=2);
}

VISTA_TEST(gaussian_room_layout_shares_the_segment_budget_across_six_ceiling_levels) {
    constexpr std::array<float,6> heights{1.46F,2.04F,2.65F,3.21F,3.81F,4.39F};
    std::vector<RoomLayoutSample> points;
    for (std::size_t i=0; i<heights.size(); ++i) {
        const auto half_width=8+static_cast<int>(i);
        for (const float center : {-2.F,2.F})
            for (int x=-half_width; x<=half_width; ++x)
                for (int y=-half_width; y<=half_width; ++y)
                    points.push_back({{center+x*.05F,y*.05F,heights[i]},{0,0,1}});
    }
    vista::application::RoomLayoutReference reference;
    reference.lidar_position={0,0,.7};
    const auto layout=vista::application::estimate_room_layout(points,reference,.05);
    VISTA_CHECK(layout.corners.empty() && !layout.closed);
    VISTA_CHECK(layout.segments.size()==32);
    for (const auto height : heights) {
        const auto traces=std::count_if(layout.segments.begin(),layout.segments.end(),[=](const auto& line) {
            return !line.inferred && std::abs(line.start[2]-height)<.03 && std::abs(line.end[2]-height)<.03;
        });
        VISTA_CHECK(traces>=4);
    }
}

VISTA_TEST(gaussian_room_layout_leaves_the_gap_between_ceiling_patches_open) {
    std::vector<RoomLayoutSample> points;
    for (int x=-18; x<=18; ++x) {
        if (std::abs(x)<8) continue;
        for (int y=-10; y<=10; ++y)
            points.push_back({{x*.1F,y*.08F,2.F},{0,0,1}});
    }
    vista::application::RoomLayoutReference reference;
    reference.lidar_position={0,0,.7};
    const auto layout=vista::application::estimate_room_layout(points,reference,.05);
    bool left=false, right=false;
    for (const auto& line : layout.segments) {
        VISTA_CHECK_NEAR(line.start[2],2,1e-6);
        VISTA_CHECK_NEAR(line.end[2],2,1e-6);
        VISTA_CHECK(line.start[0]*line.end[0]>0);
        left=left || line.start[0]<0;
        right=right || line.start[0]>0;
    }
    VISTA_CHECK(left && right);
}

VISTA_TEST(gaussian_room_layout_reuses_planar_cache_and_replaces_removed_walls) {
    auto cache=std::make_unique<Store>(vista::fs::path{},.1F,2.F,20000,32,2,2);
    cache->enable_surface_snapshots(true);
    for (int along=-15; along<=15; ++along) for (int h=1; h<=24; ++h) {
        add(*cache,15,along,h);add(*cache,-15,along,h);
        add(*cache,along,15,h);add(*cache,along,-15,h);
    }
    for (int x=-15; x<=15; ++x) for (int y=-15; y<=15; ++y) {
        add(*cache,x,y,0);add(*cache,x,y,25);
    }
    vista::application::GaussianMapBuilder builder;
    vista::application::RoomLayoutReference reference;
    reference.lidar_position={0,0,.7};
    const auto first=builder.update(*cache->view_source(1)->surface_snapshot(),{},reference);
    VISTA_CHECK(first->room_layout()->closed && first->room_layout()->corners.size()==4);
    vista::models::RoomMapViewRequest view;
    view.surface_mode=GaussianSurfaceMode::corners;
    VISTA_CHECK(first->select_splats(view).empty());
    for (int along=-14; along<=14; ++along) for (int h=1; h<=24; ++h) cache->erase({-15,along,h});
    const auto second=builder.update(*cache->view_source(2)->surface_snapshot(),{},reference);
    VISTA_CHECK(!second->room_layout()->closed && second->room_layout()->corners.size()==2);
    VISTA_CHECK(first->room_layout()->closed && first->room_layout()->corners.size()==4);
}

VISTA_TEST(gaussian_optional_real_pcd_fixture) {
    const auto path=std::getenv("VISTA_GAUSSIAN_TEST_PCD");if(!path) return;
    vista::application::RoomMapConfig config;config.surface_snapshots=true;
    vista::application::RoomMapAccumulator map(config);
    const auto started=std::chrono::steady_clock::now();map.load(path);
    auto snapshot=map.view_source()->surface_snapshot();
    vista::application::GaussianMapBuilder builder;
    vista::application::RoomLayoutReference reference;
    reference.lidar_position={0,0,.7};
    reference.floor_datum=vista::models::RoomLayoutPosition{0,0,0};
    auto view=builder.update(*snapshot,{},reference);
    vista::models::RoomMapViewRequest request;request.point_budget=100000;
    const auto points=view->select_splats(request);
    VISTA_CHECK(!points.empty() && points.size()<=request.point_budget);
    for(const auto& p:points) VISTA_CHECK(std::isfinite(p.covariance[0]) && p.covariance[0]>0);
    std::cout<<"Gaussian real PCD: "<<map.status().point_count<<" source points, "<<points.size()
        <<" visible splats, "<<builder.rebuilt_tiles()<<" tiles, "
        <<std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()<<" s (load + fit + view)\n";
    request.surface_mode=GaussianSurfaceMode::edges;const auto edges=view->select_splats(request);
    VISTA_CHECK(edges.size()<=request.point_budget);
    std::cout<<"Gaussian real PCD edges: "<<edges.size()<<" visible splats, "
        <<vista::application::encode_gaussian_map(edges,1,1).size()<<" bytes\n";
    request.surface_mode=GaussianSurfaceMode::corners;
    VISTA_CHECK(view->select_splats(request).empty());
    const auto& layout=*view->room_layout();
    VISTA_CHECK(layout.corners.size()<=8 && layout.segments.size()<=96);
    std::cout<<"Gaussian real PCD layout: "<<layout.corners.size()<<" corners, "
        <<layout.segments.size()<<" segments, closed="<<layout.closed<<'\n';
}
