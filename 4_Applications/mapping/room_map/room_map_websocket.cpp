#include "4_Applications/pointcloud_websocket/pointcloud_websocket.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include "2_Transport/messaging/websocket_server.hpp"
#include "models/topics.hpp"
#include "4_Applications/mapping/mapping_pose_control.hpp"
#include "4_Applications/mapping/gaussian_map/gaussian_map.hpp"

namespace vista::application {
namespace {
using models::GaussianSurfaceMode;

std::string mapping_state(const std::shared_ptr<MappingPoseControl>& control) {
    const auto status = control->status();
    const auto& mount = status.active.mount;
    std::ostringstream json;
    json.imbue(std::locale::classic());
    json.precision(9);
    json << std::boolalpha << R"({"type":"mapping-state","enabled":true,"generation":)"
         << status.active.generation << R"(,"pending":)" << status.pending
         << R"(,"lioEnabled":)" << status.active.lio_enabled << R"(,"mount":[)"
         << mount.mount_x_m << ',' << mount.mount_y_m << ',' << mount.mount_z_m << ','
         << mount.mount_roll_deg << ',' << mount.mount_pitch_deg << ',' << mount.mount_yaw_deg << "]}";
    return json.str();
}

void request_mapping_change(const std::string& message, const std::shared_ptr<MappingPoseControl>& control) {
    if (!control) throw std::invalid_argument("Remote mapping controls are disabled");
    if (message.size() > 320) throw std::invalid_argument("Invalid mapping command");
    std::istringstream input(message);
    input.imbue(std::locale::classic());
    std::string prefix, command, token, extra;
    std::uint64_t generation{};
    if (!(input >> prefix >> command >> generation >> token) || prefix != "MAPPING")
        throw std::invalid_argument("Invalid mapping command");

    if (command == "MODE") {
        std::string mode;
        if (!(input >> mode) || (mode != "0" && mode != "1") || (input >> extra))
            throw std::invalid_argument("Invalid LIO mode");
        control->request(token, generation, std::nullopt, mode == "1");
    } else if (command == "APPLY") {
        MountingConfig mount;
        if (!(input >> mount.mount_x_m >> mount.mount_y_m >> mount.mount_z_m
                    >> mount.mount_roll_deg >> mount.mount_pitch_deg >> mount.mount_yaw_deg) || (input >> extra))
            throw std::invalid_argument("Mount requires six finite numeric values");
        control->request(token, generation, mount, std::nullopt);
    } else throw std::invalid_argument("Invalid mapping command");
}

const char* surface_mode_name(GaussianSurfaceMode mode) {
    switch (mode) {
        case GaussianSurfaceMode::edges: return "edges";
        case GaussianSurfaceMode::corners: return "corners";
        case GaussianSurfaceMode::full_surface: return "full_surface";
    }
    return "full_surface";
}
}

models::RoomMapViewRequest parse_room_map_view_request(const std::string& text,std::size_t budget) {
    if(text.size()>2048 || !budget || budget>2'000'000) throw std::invalid_argument("invalid room-map view request size/budget");
    std::istringstream input(text);input.imbue(std::locale::classic());
    std::string command;double requested{};
    models::RoomMapViewRequest result;
    if(!(input>>command>>requested>>result.viewport_height) || command!="VIEW" ||
        !std::isfinite(requested) || requested<1 || requested>2'000'000 || std::floor(requested)!=requested ||
        !std::isfinite(result.viewport_height) || result.viewport_height<1 || result.viewport_height>8192)
        throw std::invalid_argument("invalid room-map camera header");
    result.point_budget=std::min(static_cast<std::size_t>(requested),budget);
    for(auto& coordinate:result.camera)
        if(!(input>>coordinate) || !std::isfinite(coordinate) || std::fabs(coordinate)>1e12)
            throw std::invalid_argument("invalid room-map camera coordinate");
    for(auto& plane:result.planes) {
        for(auto& value:plane)
            if(!(input>>value) || !std::isfinite(value) || std::fabs(value)>1e12)
                throw std::invalid_argument("invalid room-map frustum plane");
        const auto length=std::sqrt(plane[0]*plane[0]+plane[1]*plane[1]+plane[2]*plane[2]);
        if(length<1e-12) throw std::invalid_argument("empty room-map frustum plane");
        for(auto& value:plane) value/=length;
    }
    std::string extra;
    if(input>>extra) {
        if(extra=="EDGES") result.surface_mode=GaussianSurfaceMode::edges;
        else if(extra=="CORNERS") {
            result.surface_mode=GaussianSurfaceMode::corners;
            if (input>>extra) {
                std::istringstream radius_text(extra);
                radius_text.imbue(std::locale::classic());
                double radius{};
                char extra_character{};
                if (!(radius_text>>radius) || (radius_text>>extra_character) ||
                    !std::isfinite(radius) || radius<0 || radius>models::maximum_gaussian_corner_radius_m)
                    throw std::invalid_argument("invalid Gaussian corner radius");
                result.gaussian_corner_radius_m=radius;
            }
        }
        else if(extra!="FULL") throw std::invalid_argument("invalid surface mode");
        if(input>>extra) throw std::invalid_argument("unexpected room-map view fields");
    }
    result.has_camera=true;return result;
}

platform::WorkerHandle spawn_room_map_websocket(platform::MessageBus& bus,
    platform::ThreadConfig thread,platform::StopToken stop,PointCloudWebSocketConfig config,
    PointCloudWebSocketCompletion complete) {
    validate_pointcloud_websocket_config(config);
    auto subscriber=bus.subscribe<models::RoomMapViewMessage>(
        config.gaussian_map?models::topics::gaussian_map_view:models::topics::room_map_view,thread.name);
    return platform::spawn_worker(std::move(thread),[stop,config=std::move(config),
        subscriber=std::move(subscriber),complete=std::move(complete)]() mutable {
        try {
            struct ClientView {
                models::RoomMapViewRequest request;
                std::string requested_signature,sent_signature,sent_control_state;
                std::uint64_t sent_revision{};
                models::RoomMapState sent_state{models::RoomMapState::empty};
                bool delivered{};
                GaussianSurfaceMode sent_mode{GaussianSurfaceMode::full_surface};
                std::optional<double> sent_corner_radius;
                std::chrono::steady_clock::time_point next_send{};
            };
            PointCloudWebSocketReport report;
            std::unique_ptr<transport::WebSocketServer> server;
            std::unordered_map<std::uint64_t,ClientView> views;
            std::shared_ptr<const models::RoomMapViewMessage> latest;
            auto retry=std::chrono::steady_clock::now();
            while(!stop.is_stop_requested()) {
                std::shared_ptr<const models::RoomMapViewMessage> message;
                const auto received=subscriber.receive_for(message,std::chrono::milliseconds(25));
                if(received==platform::ReceiveStatus::closed) break;
                if(received==platform::ReceiveStatus::message) {latest=message;++report.received_messages;}
                for(unsigned i=0;i<32 && subscriber.try_receive(message)==platform::ReceiveStatus::message;++i) {
                    latest=message;++report.received_messages;
                }
                const auto now=std::chrono::steady_clock::now();
                if(!server && now>=retry) {
                    try {
                        server=std::make_unique<transport::WebSocketServer>(transport::WebSocketServer::listen({
                            config.bind_address,config.port,config.maximum_clients,
                            std::chrono::milliseconds(1000),std::chrono::milliseconds(250)}));
                        views.clear();
                        std::cout<<(config.gaussian_map?"Gaussian map":"Room-map LOD")<<" WebSocket listening at ws://"<<config.bind_address<<':'<<config.port<<'\n';
                    } catch(const std::exception& error) {
                        ++report.server_failures;retry=now+config.retry_interval;
                        std::cerr<<"Room-map WebSocket listen failed: "<<error.what()<<'\n';
                    }
                }
                if(!server) continue;
                try {
                    server->poll_accept();
                    const auto ids=server->client_ids();
                    for(auto it=views.begin();it!=views.end();) {
                        if(std::find(ids.begin(),ids.end(),it->first)==ids.end()) it=views.erase(it);else ++it;
                    }
                    for(const auto id:ids) {
                        auto insertion=views.try_emplace(id);
                        if(insertion.second) insertion.first->second.request.point_budget=config.maximum_points;
                    }
                    for(const auto& incoming:server->poll_text()) {
                        const auto found=views.find(incoming.client_id);if(found==views.end()) continue;
                        if (!config.gaussian_map && incoming.text.rfind("MAPPING ", 0) == 0) {
                            try {
                                request_mapping_change(incoming.text, config.mapping_control);
                            } catch (const std::invalid_argument& error) {
                                server->send_text(incoming.client_id,
                                    std::string(R"({"type":"mapping-error","message":")") + error.what() + "\"}");
                            }
                            continue;
                        }
                        try {
                            auto request=parse_room_map_view_request(incoming.text,config.maximum_points);
                            if(request.surface_mode!=GaussianSurfaceMode::full_surface && !config.gaussian_map)
                                throw std::invalid_argument("surface mode requires Gaussian map");
                            if (request.surface_mode==GaussianSurfaceMode::corners && !request.gaussian_corner_radius_m)
                                request.gaussian_corner_radius_m=config.gaussian_corner_radius_m;
                            found->second.request=request;
                            found->second.requested_signature=incoming.text;
                        } catch(const std::exception&) {
                            server->send_text(incoming.client_id,"{\"type\":\"roommap-error\",\"message\":\"Invalid VIEW request\"}");
                        }
                    }
                    if (!config.gaussian_map && config.mapping_control && !views.empty()) {
                        const auto control_state = mapping_state(config.mapping_control);
                        for (auto& entry : views) {
                            auto& sent = entry.second.sent_control_state;
                            if (sent != control_state && server->send_text(entry.first, control_state))
                                sent = control_state;
                        }
                    }
                    if(!latest) continue;
                    // Each client has its own camera and bounded view. Frozen maps
                    // send only on camera changes/reconnect, not every polling tick.
                    for(auto& entry:views) {
                        auto& view=entry.second;
                        const bool map_changed=!view.delivered || view.sent_revision!=latest->revision ||
                            view.sent_state!=latest->state;
                        const bool mode_changed=view.delivered && (view.sent_mode!=view.request.surface_mode ||
                            view.sent_corner_radius!=view.request.gaussian_corner_radius_m);
                        if(!map_changed && !mode_changed && (now<view.next_send ||
                            view.sent_signature==view.requested_signature)) continue;
                        // The producer has ALREADY checkpointed this revision at
                        // RoomMapPublishIntervalMilliseconds. Do not add a second
                        // delay before showing it. Only same-map camera requests
                        // are throttled (using that same configuration value).
                        models::RoomMapFrame cloud;
                        std::vector<models::GaussianMapPoint> splats;
                        std::array<double,6> bounds{-1,-1,-1,1,1,1};
                        auto state=latest->state;
                        try {
                            if(latest->source) {
                                bounds=latest->source->bounds();
                                if(config.gaussian_map) {
                                    const auto source=std::dynamic_pointer_cast<const models::GaussianMapViewSource>(latest->source);
                                    if(!source) throw std::runtime_error("invalid Gaussian map source");
                                    splats=source->select_splats(view.request);
                                } else cloud=latest->source->select_view(view.request);
                            }
                        } catch(const std::exception& error) {
                            // Optional rendering/storage failures must not stop sensor workers.
                            ++report.server_failures;state=models::RoomMapState::storage_error;
                            cloud.points.clear();std::cerr<<"Room-map LOD query failed: "<<error.what()<<'\n';
                            splats.clear();
                        }
                        std::ostringstream meta;meta.imbue(std::locale::classic());meta.precision(15);
                        meta<<"{\"type\":\"roommap\",\"revision\":"<<latest->revision
                            <<",\"totalPoints\":"<<latest->point_count<<",\"pointBudget\":"<<config.maximum_points
                            <<",\"state\":"<<static_cast<unsigned>(state)
                            <<",\"mappingGeneration\":"<<latest->mapping_generation<<",\"bounds\":[";
                        for(unsigned i=0;i<6;++i) {if(i) meta<<',';meta<<bounds[i];}meta<<"]";
                        if(config.gaussian_map) {
                            meta<<",\"supportsEdges\":true,\"supportsCorners\":true,\"surfaceMode\":\""
                                <<surface_mode_name(view.request.surface_mode)<<'"'
                                <<",\"defaultCornerRadiusMeters\":"<<config.gaussian_corner_radius_m;
                            if (view.request.gaussian_corner_radius_m)
                                meta<<",\"cornerRadiusMeters\":"<<*view.request.gaussian_corner_radius_m;
                        }
                        meta<<'}';
                        cloud.timestamp_ns=latest->timestamp_ns;
                        models::RoomMapMessage frame(latest->revision,
                            latest->timestamp_ns,std::move(cloud));
                        auto packet=config.gaussian_map?encode_gaussian_map(splats,latest->revision,latest->timestamp_ns):
                            encode_lpc1_pointcloud(frame,view.request.point_budget);
                        if(server->send_text(entry.first,meta.str()) && server->send_binary(entry.first,packet.data(),packet.size())) {
                            ++report.broadcast_frames;++report.client_deliveries;
                            report.encoded_points+=config.gaussian_map?splats.size():frame.payload.points.size();
                            view.delivered=true;view.sent_revision=latest->revision;view.sent_state=latest->state;
                            view.sent_mode=view.request.surface_mode;
                            view.sent_corner_radius=view.request.gaussian_corner_radius_m;
                            view.sent_signature=view.requested_signature;
                        }
                        // Bound repeated camera queries without delaying a new map.
                        view.next_send=std::chrono::steady_clock::now()+config.publish_interval;
                    }
                } catch(const std::exception& error) {
                    ++report.server_failures;server.reset();views.clear();retry=now+config.retry_interval;
                    std::cerr<<"Room-map WebSocket restarting: "<<error.what()<<'\n';
                }
            }
            report.dropped_input_messages=subscriber.dropped_messages();complete(report,{});
        } catch(const std::exception& error) {complete(std::nullopt,error.what());}
          catch(...) {complete(std::nullopt,"unknown room-map WebSocket failure");}
    });
}
} // namespace vista::application
