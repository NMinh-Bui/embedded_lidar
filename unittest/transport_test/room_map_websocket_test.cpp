#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <vector>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif
#include "2_Transport/messaging/websocket_server.hpp"
#include "4_Applications/mapping/room_map/room_map.hpp"
#include "4_Applications/pointcloud_websocket/pointcloud_websocket.hpp"
#include "models/topics.hpp"
#include "1_Platform/timing/measurement_clock.hpp"
#include "2_Transport/storage/room_map_storage.hpp"
#include "4_Applications/mapping/lio/lio.hpp"
#include "4_Applications/mapping/mapping_pose_control.hpp"
#include "4_Applications/settings/runtime_settings.hpp"
#include "4_Applications/processing/pointcloud/pointcloud_processing.hpp"
#include <cmath>
#include "unittest/test.hpp"
#include "4_Applications/mapping/gaussian_map/gaussian_map.hpp"
#include "4_Applications/perception/foreground/foreground.hpp"

namespace {
using namespace std::chrono_literals;
#ifdef _WIN32
using Socket=SOCKET;constexpr Socket invalid=INVALID_SOCKET;
struct Runtime {Runtime(){WSADATA data{};if(WSAStartup(MAKEWORD(2,2),&data)) throw std::runtime_error("WSAStartup");}~Runtime(){WSACleanup();}};
void close_test_socket(Socket socket){closesocket(socket);}
#else
using Socket=int;constexpr Socket invalid=-1;
struct Runtime {};
void close_test_socket(Socket socket){close(socket);}
#endif
struct Client {
    Socket socket{invalid};
    ~Client(){if(socket!=invalid) close_test_socket(socket);}
    void connect(std::uint16_t port) {
        socket=::socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
        if(socket==invalid) throw std::runtime_error("test client socket");
#ifdef _WIN32
        DWORD timeout=3000;setsockopt(socket,SOL_SOCKET,SO_RCVTIMEO,reinterpret_cast<const char*>(&timeout),sizeof(timeout));
#else
        timeval timeout{3,0};setsockopt(socket,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
#endif
        sockaddr_in address{};address.sin_family=AF_INET;address.sin_port=htons(port);
        address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        if(::connect(socket,reinterpret_cast<sockaddr*>(&address),sizeof(address))) throw std::runtime_error("test connect");
        const std::string request="GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n";
        send_bytes(std::vector<std::uint8_t>(request.begin(),request.end()));
    }
    void send_bytes(const std::vector<std::uint8_t>& bytes) {
        std::size_t sent{};while(sent<bytes.size()) {
            const auto n=send(socket,reinterpret_cast<const char*>(bytes.data()+sent),static_cast<int>(bytes.size()-sent),0);
            if(n<=0) throw std::runtime_error("test send");sent+=static_cast<std::size_t>(n);
        }
    }
    std::vector<std::uint8_t> read_bytes(std::size_t count) {
        std::vector<std::uint8_t> bytes(count);std::size_t read{};
        while(read<count) {
            const auto n=recv(socket,reinterpret_cast<char*>(bytes.data()+read),static_cast<int>(count-read),0);
            if(n<=0) throw std::runtime_error("test read timeout/close");read+=static_cast<std::size_t>(n);
        }return bytes;
    }
    void handshake() {
        std::string header;
        while(header.find("\r\n\r\n")==std::string::npos && header.size()<8192)
            header.push_back(static_cast<char>(read_bytes(1)[0]));
        if(header.find("101 Switching Protocols")==std::string::npos) throw std::runtime_error("test handshake");
    }
    std::pair<unsigned,std::vector<std::uint8_t>> read_frame() {
        const auto header=read_bytes(2);std::uint64_t length=header[1]&127U;
        if(length==126) {const auto n=read_bytes(2);length=n[0]*256U+n[1];}
        else if(length==127) {const auto n=read_bytes(8);length=0;for(const auto byte:n) length=(length<<8)|byte;}
        if(length>1000000 || (header[1]&128U)) throw std::runtime_error("invalid test frame");
        return {header[0]&15U,read_bytes(static_cast<std::size_t>(length))};
    }
};
std::uint16_t available_port() {
    Client probe;probe.socket=::socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    if(bind(probe.socket,reinterpret_cast<sockaddr*>(&address),sizeof(address))) throw std::runtime_error("test bind");
#ifdef _WIN32
    int size=sizeof(address);
#else
    socklen_t size=sizeof(address);
#endif
    if(getsockname(probe.socket,reinterpret_cast<sockaddr*>(&address),&size)) throw std::runtime_error("getsockname");
    return ntohs(address.sin_port);
}
std::vector<std::uint8_t> masked_text(const std::string& text) {
    std::vector<std::uint8_t> result{0x81};
    if(text.size()<126) result.push_back(static_cast<std::uint8_t>(0x80U|text.size()));
    else {result.push_back(0xfe);result.push_back(static_cast<std::uint8_t>(text.size()>>8));result.push_back(static_cast<std::uint8_t>(text.size()));}
    const std::array<std::uint8_t,4> mask{1,2,3,4};result.insert(result.end(),mask.begin(),mask.end());
    for(std::size_t i=0;i<text.size();++i) result.push_back(static_cast<std::uint8_t>(text[i])^mask[i%4]);return result;
}
}

VISTA_TEST(foreground_websocket_replaces_clears_stale_data_and_does_not_replay_old_objects) {
    Runtime runtime;
    vista::platform::MessageBus bus;vista::platform::StopToken stop;
    auto publisher=bus.publisher<vista::models::ForegroundFrame>(vista::models::topics::foreground_points);
    vista::application::PointCloudWebSocketConfig config;
    config.port=available_port();config.maximum_points=10;config.publish_interval=500ms;
    std::string failure;
    auto worker=vista::application::spawn_foreground_websocket(bus,{"test-foreground-socket",5},stop,config,
        [&](auto,auto error){failure=std::move(error);});
    struct Finish {vista::platform::StopToken& stop;vista::platform::MessageBus& bus;vista::platform::WorkerHandle& worker;
        ~Finish(){stop.request_stop();bus.close();worker.join();}} finish{stop,bus,worker};
    std::this_thread::sleep_for(100ms);
    Client client;client.connect(config.port);client.handshake();
    const auto metadata=client.read_frame();VISTA_CHECK(metadata.first==1);
    const std::string waiting(metadata.second.begin(),metadata.second.end());
    VISTA_CHECK(waiting.find("foreground-status")!=std::string::npos && waiting.find("\"active\":false")!=std::string::npos);
    VISTA_CHECK(client.read_frame().second.size()==32); // Nothing retained at startup.
    vista::models::ForegroundFrame frame;frame.source="test";frame.sequence=42;
    frame.received_monotonic_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    frame.points={{1,2,3,99}};frame.input_points=1;frame.reason="foreground; \"test\"\n";
    const auto sent_at=std::chrono::steady_clock::now();publisher.publish(frame);
    bool saw_motion=false;
    bool explained=false;
    for(int i=0;i<20 && !saw_motion;++i) {
        const auto packet=client.read_frame();
        if(packet.first==1) {const std::string text(packet.second.begin(),packet.second.end());
            explained=text.find("\"active\":true")!=std::string::npos && text.find("\\\"test\\\"\\n")!=std::string::npos;}
        else saw_motion=packet.second.size()==48;
    }
    VISTA_CHECK(saw_motion);
    VISTA_CHECK(explained);
    // The socket forwards a sampled result immediately, without another 500 ms
    // publication timer, and does not repeatedly transmit unchanged active data.
    VISTA_CHECK(std::chrono::steady_clock::now()-sent_at<400ms);
    fd_set ready;FD_ZERO(&ready);FD_SET(client.socket,&ready);timeval quiet{0,100000};
    VISTA_CHECK(select(static_cast<int>(client.socket)+1,&ready,nullptr,nullptr,&quiet)==0);
    publisher.publish(vista::models::ForegroundFrame{});
    bool cleared=false;
    for(int i=0;i<20 && !cleared;++i) {const auto packet=client.read_frame();cleared=packet.first==2 && packet.second.size()==32;}
    VISTA_CHECK(cleared);
    frame.received_monotonic_ns-=2'000'000'000ULL;publisher.publish(frame);
    std::this_thread::sleep_for(100ms);
    Client late;late.connect(config.port);late.handshake();
    const auto stale=late.read_frame();VISTA_CHECK(stale.first==1);
    const std::string reason(stale.second.begin(),stale.second.end());
    VISTA_CHECK(reason.find("older than 500 ms")!=std::string::npos && reason.find("\"active\":false")!=std::string::npos);
    VISTA_CHECK(late.read_frame().second.size()==32); // Delayed queue arrival is not fresh evidence.
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(failure.empty());
}

VISTA_TEST(websocket_server_isolates_clients_buffers_partial_reads_and_rejects_oversized_requests) {
    Runtime runtime;
    auto server=vista::transport::WebSocketServer::listen({"127.0.0.1",available_port(),4,1000ms,250ms});
    Client first,second;first.connect(server.config().port);server.poll_accept();first.handshake();
    second.connect(server.config().port);server.poll_accept();second.handshake();
    const auto ids=server.client_ids();VISTA_CHECK(ids.size()==2 && ids[0]!=ids[1]);
    const auto frame=masked_text("VIEW first");
    first.send_bytes({frame.begin(),frame.begin()+3});
    std::this_thread::sleep_for(5ms);VISTA_CHECK(server.poll_text().empty());
    first.send_bytes({frame.begin()+3,frame.end()});second.send_bytes(masked_text("VIEW second"));
    std::vector<vista::transport::WebSocketTextMessage> requests;
    for(unsigned i=0;i<20 && requests.size()<2;++i) {
        auto values=server.poll_text();requests.insert(requests.end(),values.begin(),values.end());std::this_thread::sleep_for(5ms);
    }
    VISTA_CHECK(requests.size()==2);
    VISTA_CHECK(requests[0].client_id!=requests[1].client_id);
    VISTA_CHECK(server.send_text(ids[0],"first-only"));
    const auto response=first.read_frame();VISTA_CHECK(response.first==1);
    VISTA_CHECK(std::string(response.second.begin(),response.second.end())=="first-only");
    second.send_bytes(masked_text(std::string(2049,'x')));
    for(unsigned i=0;i<20 && server.client_count()>1;++i){server.poll_text();std::this_thread::sleep_for(5ms);}
    VISTA_CHECK(server.client_count()==1);
    const std::array<std::uint8_t,4> bytes{'L','P','C','1'};
    VISTA_CHECK(server.send_binary(ids[0],bytes.data(),bytes.size()));
    const auto binary=first.read_frame();VISTA_CHECK(binary.first==2 && binary.second.size()==4);
}

VISTA_TEST(room_map_websocket_replays_to_late_client_and_serves_camera_view) {
    Runtime runtime;
    auto options=vista::application::RoomMapConfig{};options.voxel_size_m=1;options.tile_size_m=1;
    options.minimum_observations=1;options.freeze_after=0ms;
    vista::application::RoomMapAccumulator map(options);
    map.integrate({1,{{1.25F,0.25F,0.25F,1,0,0,0},{-1.25F,0.25F,0.25F,2,0,0,0}}},0ms);map.freeze();map.flush();
    vista::platform::MessageBus bus;vista::platform::StopToken stop;
    auto publisher=bus.publisher<vista::models::RoomMapViewMessage>(vista::models::topics::room_map_view);
    // Selecting the map topic must never subscribe with a LiDAR message type.
    vista::application::PointCloudWebSocketConfig config;config.input_topic=vista::models::topics::room_map;
    config.port=available_port();config.maximum_points=10;config.publish_interval=20ms;
    std::string failure;
    auto worker=vista::application::spawn_pointcloud_websocket(bus,{"test-room-lod",4},stop,config,
        [&](auto,auto error){failure=std::move(error);});
    struct Finish {vista::platform::StopToken& stop;vista::platform::MessageBus& bus;vista::platform::WorkerHandle& worker;
        ~Finish(){stop.request_stop();bus.close();worker.join();}} finish{stop,bus,worker};
    publisher.publish({map.view_source(),7,1,2,vista::models::RoomMapState::frozen});
    std::this_thread::sleep_for(100ms);
    Client client;client.connect(config.port);client.handshake();
    const auto metadata=client.read_frame();
    VISTA_CHECK(metadata.first==1);
    VISTA_CHECK(std::string(metadata.second.begin(),metadata.second.end()).find("\"totalPoints\":2")!=std::string::npos);
    const auto overview=client.read_frame();VISTA_CHECK(overview.first==2 && overview.second.size()==64);
    client.send_bytes(masked_text("VIEW 10 600 0 0 10 1 0 0 0 -1 0 0 100 0 1 0 100 0 -1 0 100 0 0 1 100 0 0 -1 100"));
    VISTA_CHECK(client.read_frame().first==1);
    const auto view=client.read_frame();VISTA_CHECK(view.first==2 && view.second.size()==48);
    float x{};std::memcpy(&x,view.second.data()+32,4);VISTA_CHECK(x>0);
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(failure.empty());
}

VISTA_TEST(room_map_controls_reset_fixed_mount_and_keep_previous_pcd_separate) {
    using namespace vista;
    using namespace vista::application;
    Runtime runtime;
    platform::MessageBus bus;
    platform::StopToken stop;
    const std::string token = "test_only_mapping_token_01234567";
    const auto control = std::make_shared<MappingPoseControl>(
        MappingPose{MountingConfig{0, 0, 0.7F, 0, 0, 0}, true}, token);
    auto decoded = bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_decoded);
    auto worlds = bus.subscribe<devices::LidarPointCloudMessage>(models::topics::pointcloud_world, "mount-world");
    auto poses = bus.subscribe<models::LocalizationStatus>(models::topics::localization_status, "mount-pose");
    auto views = bus.subscribe<models::RoomMapViewMessage>(models::topics::room_map_view, "mount-map");
    const auto root = vista::fs::temp_directory_path() /
        ("vista-mount-reset-" + std::to_string(platform::monotonic_timestamp_ns()));
    vista::fs::create_directory(root);
    struct RemoveTestFiles {
        vista::fs::path path;
        ~RemoveTestFiles() { std::error_code ignored; vista::fs::remove_all(path, ignored); }
    } remove{root};
    RoomMapConfig room;
    room.enabled = true;
    room.minimum_observations = 1;
    room.publish_interval = 20ms;
    room.freeze_after = 0ms;
    room.file = root / "RoomMap.pcd";
    room.storage_directory = root / "tiles";
    room.mapping_control = control;
    PointCloudWebSocketConfig socket;
    socket.port = available_port();
    socket.room_map_lod = true;
    socket.mapping_control = control;
    LioConfig lio;
    lio.enabled = true;
    lio.initial_position_m[2] = 0.7;
    PreprocessingConfig preprocessing;
    preprocessing.input_world_coordinates = true;
    std::string map_error, lio_error, socket_error, preprocess_error;
    std::vector<platform::WorkerHandle> workers;
    struct FinishWorkers {
        platform::MessageBus& bus;
        platform::StopToken& stop;
        std::vector<platform::WorkerHandle>& workers;
        void join() { stop.request_stop(); bus.close(); for (auto& worker : workers) worker.join(); workers.clear(); }
        ~FinishWorkers() { join(); }
    } finish{bus, stop, workers};
    workers.push_back(spawn_room_map_worker(bus, {"mount-room",4}, stop, room,
        [&](auto, auto error) { map_error = std::move(error); }));
    workers.push_back(spawn_preprocessing_worker(bus, {"mount-clean",3}, stop, preprocessing,
        [&](auto, auto error) { preprocess_error = std::move(error); }));
    workers.push_back(spawn_lio_worker(bus, {"mount-lio",3}, stop, lio,
        [&](auto, auto error) { lio_error = std::move(error); }, control));
    workers.push_back(spawn_pointcloud_websocket(bus, {"mount-socket",4}, stop, socket,
        [&](auto, auto error) { socket_error = std::move(error); }));
    std::this_thread::sleep_for(100ms);
    Client client;
    client.connect(socket.port);
    client.handshake();

    const auto read_control = [&](const std::string& expected) {
        for (int i = 0; i < 25; ++i) {
            const auto packet = client.read_frame();
            if (packet.first != 1) continue;
            std::string text(packet.second.begin(), packet.second.end());
            if (text.find(expected) != std::string::npos) return text;
        }
        throw std::runtime_error("missing mapping WebSocket response: " + expected);
    };
    const auto wait_map = [&](std::uint64_t generation, std::uint64_t points) {
        std::shared_ptr<const models::RoomMapViewMessage> message;
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (std::chrono::steady_clock::now() < deadline) {
            if (views.receive_for(message, 100ms) == platform::ReceiveStatus::message &&
                message->mapping_generation == generation && message->point_count == points) return;
        }
        throw std::runtime_error("missing generation-specific Room Map checkpoint");
    };
    const auto scan = [&](std::uint64_t sequence) {
        decoded.publish(devices::LidarPointCloudMessage("unitree-l2", sequence, std::nullopt, 1,
            models::PointCloudFrame{1, {{1.0F, 0, 0, 12, 0, 0, 0}}},
            platform::monotonic_timestamp_ns()));
        std::shared_ptr<const devices::LidarPointCloudMessage> world;
        if (worlds.receive_for(world, 1s) != platform::ReceiveStatus::message)
            throw std::runtime_error("fixed mount produced no world cloud");
        return world;
    };

    VISTA_CHECK(read_control("\"generation\":0").find("\"lioEnabled\":true") != std::string::npos);
    client.send_bytes(masked_text("MAPPING MODE 0 invalid 0"));
    VISTA_CHECK(read_control("\"type\":\"mapping-error\"").find("Invalid control token") != std::string::npos);
    client.send_bytes(masked_text("MAPPING MODE 0 " + token + " 0"));
    VISTA_CHECK(read_control("\"generation\":1").find("\"lioEnabled\":false") != std::string::npos);
    const auto first = scan(1);
    VISTA_CHECK(first->payload.mapping_generation == 1);
    VISTA_CHECK(std::abs(first->payload.points.at(0).z - 0.7F) < 1e-5F);
    wait_map(1, 1);

    client.send_bytes(masked_text("MAPPING APPLY 0 " + token + " 10 0 2 0 0 90"));
    VISTA_CHECK(read_control("\"type\":\"mapping-error\"").find("Active pose changed") != std::string::npos);
    client.send_bytes(masked_text("MAPPING APPLY 1 " + token + " 10 0 2 0 0 900"));
    VISTA_CHECK(read_control("\"type\":\"mapping-error\"").find("Mount rotation") != std::string::npos);
    client.send_bytes(masked_text("MAPPING APPLY 1 " + token + " 10 0 2 0 0 90"));
    VISTA_CHECK(read_control("\"generation\":2").find("\"mount\":[10,0,2,0,0,90]") != std::string::npos);
    wait_map(2, 0);
    VISTA_CHECK(transport::load_room_map_snapshot(room.file, 10).points.size() == 1);
    const auto second = scan(2);
    VISTA_CHECK(second->payload.mapping_generation == 2);
    const auto& point = second->payload.points.at(0);
    VISTA_CHECK(std::abs(point.x - 10.0F) < 1e-5F && std::abs(point.y - 1.0F) < 1e-5F &&
                std::abs(point.z - 2.0F) < 1e-5F);
    wait_map(2, 1);

    client.send_bytes(masked_text("MAPPING MODE 2 " + token + " 1"));
    VISTA_CHECK(read_control("\"generation\":3").find("\"lioEnabled\":true") != std::string::npos);
    std::shared_ptr<const models::LocalizationStatus> pose;
    while (poses.receive_for(pose, 1s) == platform::ReceiveStatus::message && pose->mapping_generation != 3) {}
    VISTA_CHECK(pose && pose->mapping_generation == 3 && !pose->pose_valid &&
                pose->state == models::LocalizationState::initializing);
    VISTA_CHECK(std::abs(pose->position_m[0] - 10.0) < 1e-5);
    wait_map(3, 0);

    finish.join();
    VISTA_CHECK(map_error.empty() && lio_error.empty() && socket_error.empty() && preprocess_error.empty());
    unsigned pcd_count = 0;
    for (const auto& entry : vista::fs::directory_iterator(root)) {
        if (entry.path().extension() == ".pcd") ++pcd_count;
    }
    VISTA_CHECK(pcd_count == 2);
}

VISTA_TEST(settings_apply_validates_complete_candidate_and_keeps_defaults_in_memory) {
    using namespace vista;
    using namespace vista::application;
    const std::string text = "Lidar: quanergy-m8\nIP (m8): 127.0.0.1\nTCPPort: 4141\n"
        "LioImuAccelerationCovariance: 0.1\nLioImuGyroscopeCovariance: 0.12\n";
    const auto initial = parse_device_config_text(text);
    auto apply = [&](std::string group, SettingValues values) {
        return validate_settings_request(text,initial,{1,0,std::move(group),std::move(values)},{});
    };
    const auto snapshot = runtime_setting_values(initial);
    VISTA_CHECK(snapshot.at("LioImuAccelerationCovariance")=="0.1");
    VISTA_CHECK(snapshot.at("LioImuGyroscopeCovariance")=="0.12");
    VISTA_CHECK(snapshot.at("Camera")=="None" && snapshot.at("Radar")=="None");
    auto paired = apply("lio_noise",{{"LioImuAccelerationCovariance","0.2"},{"LioImuGyroscopeCovariance","0.2"}});
    VISTA_CHECK(paired.changes.size()==2);
    VISTA_CHECK(settings_change_line("LioImuGyroscopeCovariance",paired.changes.at("LioImuGyroscopeCovariance"))==
        "[settings] LioImuGyroscopeCovariance \"0.12\" -> \"0.2\"");
    VISTA_CHECK(parse_device_config_text(text).lio.imu_gyroscope_covariance != paired.config.lio.imu_gyroscope_covariance);
    const auto lidar = apply("lidar",{{"SensorPort","4142"},{"SensorIP","127.0.0.2"}});
    VISTA_CHECK(lidar.config.sensor_port==4142 && lidar.config.sensor_ip=="127.0.0.2");
    VISTA_CHECK(lidar.text.find("TCPPort:")==std::string::npos);
    auto rejected = [&](const char* group, SettingValues values) {
        try { (void)apply(group,std::move(values)); } catch (const std::invalid_argument&) { return true; }
        return false;
    };
    VISTA_CHECK(rejected("lio_noise",{{"LioImuAccelerationCovariance","0.2"},{"LioImuGyroscopeCovariance","-1"}}));
    VISTA_CHECK(rejected("logging",{{"LioEnabled","0"}}));
    VISTA_CHECK(rejected("camera",{{"Camera","RealSense"}}));
    VISTA_CHECK(rejected("room_map_io",{{"RoomMapWebSocketPort","9988"}}));
    VISTA_CHECK(rejected("logging",{{"RawLoggingEnabled","1\nLidar: unitree-l2"}}));
}

VISTA_TEST(settings_websocket_authentication_and_session_state_are_shared) {
    using namespace vista;
    using namespace vista::application;
    [[maybe_unused]] Runtime runtime;
    platform::StopToken stop;
    const auto initial = parse_device_config_text("Lidar: quanergy-m8\nSensorIP: 127.0.0.1\nSensorPort: 4141\n");
    RuntimeSettings settings(initial);
    const auto port = available_port();
    const std::string token = "test_only_settings_token_01234567";
    auto worker = spawn_settings_server(stop,"127.0.0.1",port,token,settings);
    struct Finish {
        platform::StopToken& stop;
        platform::WorkerHandle& worker;
        ~Finish() { stop.request_stop(); worker.join(); }
    } finish{stop,worker};
    auto read = [](Client& client, const std::string& match) {
        for (int attempt=0;attempt<8;++attempt) {
            const auto message = client.read_frame();
            const std::string text(message.second.begin(),message.second.end());
            if (message.first==1 && text.find(match)!=std::string::npos) return text;
        }
        throw std::runtime_error("Expected settings message was not sent");
    };
    Client operator_view;
    operator_view.connect(port);
    operator_view.handshake();
    operator_view.send_bytes(masked_text("APPLY 0 logging\nRawLoggingEnabled=1"));
    VISTA_CHECK(read(operator_view,"Enter the control token").find("settings")==std::string::npos);
    operator_view.send_bytes(masked_text("AUTH not-the-token"));
    read(operator_view,"Invalid control token");
    operator_view.send_bytes(masked_text("AUTH "+token));
    VISTA_CHECK(read(operator_view,"\"type\":\"state\"").find("\"RawLoggingEnabled\":\"0\"")!=std::string::npos);
    read(operator_view,"\"type\":\"ports\"");
    operator_view.send_bytes(masked_text("APPLY 0 logging\nRawLoggingEnabled=1"));
    std::optional<SettingsRequest> request;
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (!request && std::chrono::steady_clock::now()<deadline) { request=settings.take(); std::this_thread::sleep_for(10ms); }
    VISTA_CHECK(request && request->group=="logging" && request->values.at("RawLoggingEnabled")=="1");
    auto active=initial; active.raw_logging_enabled=true;
    settings.complete(*request,&active,"Applied");
    VISTA_CHECK(read(operator_view,"\"revision\":1").find("\"RawLoggingEnabled\":\"1\"")!=std::string::npos);
    read(operator_view,"\"type\":\"result\"");
    operator_view.send_bytes(masked_text("APPLY 0 logging\nRawLoggingEnabled=0"));
    read(operator_view,"Configuration has changed");

    Client reopened;
    reopened.connect(port);
    reopened.handshake();
    reopened.send_bytes(masked_text("AUTH "+token));
    VISTA_CHECK(read(reopened,"\"revision\":1").find("\"RawLoggingEnabled\":\"1\"")!=std::string::npos);
    VISTA_CHECK(!initial.raw_logging_enabled && !settings.take());
}

VISTA_TEST(gaussian_worker_and_websocket_use_full_map_checkpoint_and_clear_load_errors) {
    Runtime runtime;
    vista::application::RoomMapConfig options;
    options.voxel_size_m=1;options.tile_size_m=1;options.minimum_observations=1;
    options.freeze_after=0ms;options.surface_snapshots=true;
    vista::application::RoomMapAccumulator map(options);
    map.integrate({1,{{1.25F,.25F,.25F,77,0,0,0},{-1.25F,.25F,.25F,88,0,0,0}}},0ms);
    map.freeze();map.flush();
    vista::platform::MessageBus bus;vista::platform::StopToken stop;
    auto publisher=bus.publisher<vista::models::RoomMapViewMessage>(vista::models::topics::room_map_view);
    auto observer=bus.subscribe<vista::models::RoomMapViewMessage>(vista::models::topics::gaussian_map_view,"test-gaussian-output");
    vista::application::PointCloudWebSocketConfig config;config.gaussian_map=true;config.room_map_lod=true;
    config.port=available_port();config.maximum_points=10;config.publish_interval=20ms;
    std::string build_error,socket_error;
    std::optional<vista::application::GaussianMapReport> report;
    auto socket=vista::application::spawn_pointcloud_websocket(bus,{"test-gaussian-websocket",4},stop,config,
        [&](auto,auto error){socket_error=std::move(error);});
    auto worker=vista::application::spawn_gaussian_map_worker(bus,{"test-gaussian-builder",4},stop,{},
        [&](auto result,auto error){report=result;build_error=std::move(error);});
    struct Finish {
        vista::platform::StopToken& stop;vista::platform::MessageBus& bus;
        vista::platform::WorkerHandle& worker;vista::platform::WorkerHandle& socket;
        ~Finish(){stop.request_stop();bus.close();worker.join();socket.join();}
    } finish{stop,bus,worker,socket};
    auto source=map.view_source();
    publisher.publish({source,7,123,2,vista::models::RoomMapState::frozen,111,5});
    std::shared_ptr<const vista::models::RoomMapViewMessage> output;
    VISTA_CHECK(observer.receive_for(output,3s)==vista::platform::ReceiveStatus::message);
    VISTA_CHECK(output->source && output->state==vista::models::RoomMapState::frozen);
    VISTA_CHECK(output->mapping_generation==5 && output->checkpoint_monotonic_ns==111);
    const auto revision=output->revision;
    std::this_thread::sleep_for(100ms);
    Client client;client.connect(config.port);client.handshake();
    VISTA_CHECK(client.read_frame().first==1);
    const auto frame=client.read_frame();
    VISTA_CHECK(frame.first==2 && frame.second.size()==32+2*44 && std::memcmp(frame.second.data(),"GSP1",4)==0);
    client.send_bytes(masked_text("VIEW 10 600 0 0 10 1 0 0 0 -1 0 0 100 0 1 0 100 0 -1 0 100 0 0 1 100 0 0 -1 100"));
    VISTA_CHECK(client.read_frame().first==1);
    VISTA_CHECK(client.read_frame().second.size()==32+44);
    publisher.publish({source,8,124,2,vista::models::RoomMapState::frozen,111,5});
    VISTA_CHECK(observer.receive_for(output,2s)==vista::platform::ReceiveStatus::message);
    VISTA_CHECK(output->revision==revision); // No changed geometry: no rebuild/resend.
    publisher.publish({nullptr,9,125,0,vista::models::RoomMapState::load_error});
    VISTA_CHECK(client.read_frame().first==1);
    VISTA_CHECK(client.read_frame().second.size()==32);
    VISTA_CHECK(!stop.is_stop_requested());
    stop.request_stop();bus.close();worker.join();socket.join();
    VISTA_CHECK(build_error.empty() && socket_error.empty() && report && report->completed_revisions==1);
}

VISTA_TEST(lidar_pose_websocket_reports_tracking_last_known_and_fixed_mount) {
    Runtime runtime;
    using namespace vista;
    for (const bool gaussian : {false, true}) {
        platform::MessageBus bus;
        platform::StopToken stop;
        auto map = bus.publisher<models::RoomMapViewMessage>(
            gaussian ? models::topics::gaussian_map_view : models::topics::room_map_view);
        auto poses = bus.publisher<models::LidarPoseStatus>(models::topics::lidar_pose);
        application::PointCloudWebSocketConfig config;
        config.room_map_lod = true;
        config.gaussian_map = gaussian;
        config.port = available_port();
        const models::LidarPose pose{{1, 2, 3}, {0, 0, 0, 1}};
        if (gaussian) config.fixed_lidar_pose = pose;
        auto worker = application::spawn_pointcloud_websocket(bus, {"lidar-pose-test", 4}, stop, config,
            [](auto, auto) {});
        struct Finish {
            platform::StopToken& stop;
            platform::MessageBus& bus;
            platform::WorkerHandle& worker;
            ~Finish() { stop.request_stop(); bus.close(); worker.join(); }
        } finish{stop, bus, worker};
        const auto generation = gaussian ? 0U : 7U;
        map.publish({nullptr, 1, 1, 0, models::RoomMapState::empty, 0, generation});
        std::this_thread::sleep_for(120ms);
        Client client;
        client.connect(config.port);
        client.handshake();
        const auto receive_text = [&] {
            const auto packet = client.read_frame();
            VISTA_CHECK(packet.first == 1);
            return std::string(packet.second.begin(), packet.second.end());
        };
        VISTA_CHECK(receive_text().find(R"("supportsLidarPose":true)") != std::string::npos);
        VISTA_CHECK(client.read_frame().first == 2);
        client.send_bytes(masked_text("LIDAR POSE"));
        const auto initial = receive_text();
        VISTA_CHECK(initial.find(gaussian ? R"("state":"fixed_mount")" : R"("state":"unavailable")") != std::string::npos);
        if (gaussian) {
            VISTA_CHECK(initial.find(R"("position":[1,2,3])") != std::string::npos);
            continue;
        }
        poses.publish({models::LidarPoseState::tracking, pose, generation});
        const auto tracked = receive_text();
        VISTA_CHECK(tracked.find(R"("state":"tracking")") != std::string::npos);
        VISTA_CHECK(tracked.find(R"("mappingGeneration":7)") != std::string::npos);
        VISTA_CHECK(tracked.find(R"("position":[1,2,3])") != std::string::npos);
        VISTA_CHECK(tracked.find(R"("orientation":[0,0,0,1])") != std::string::npos);
        poses.publish({models::LidarPoseState::last_known, pose, generation});
        const auto held = receive_text();
        VISTA_CHECK(held.find(R"("state":"last_known")") != std::string::npos);
        VISTA_CHECK(held.find(R"("position":[1,2,3])") != std::string::npos);
        poses.publish({models::LidarPoseState::unavailable, std::nullopt, 8});
        const auto reset = receive_text();
        VISTA_CHECK(reset.find(R"("state":"unavailable")") != std::string::npos);
        VISTA_CHECK(reset.find(R"("mappingGeneration":8)") != std::string::npos);
        VISTA_CHECK(reset.find(R"("position")") == std::string::npos);
    }
}

VISTA_TEST(gaussian_surface_mode_parser_preserves_legacy_and_rejects_invalid_modes) {
    const std::string view="VIEW 100 600 0 0 10 1 0 0 100 -1 0 0 100 0 1 0 100 0 -1 0 100 0 0 1 100 0 0 -1 100";
    using vista::application::parse_room_map_view_request;
    using vista::models::GaussianSurfaceMode;
    VISTA_CHECK(parse_room_map_view_request(view,100).surface_mode==GaussianSurfaceMode::full_surface);
    VISTA_CHECK(parse_room_map_view_request(view+" FULL",100).surface_mode==GaussianSurfaceMode::full_surface);
    VISTA_CHECK(parse_room_map_view_request(view+" EDGES",100).surface_mode==GaussianSurfaceMode::edges);
    VISTA_CHECK(parse_room_map_view_request(view+" CORNERS",100).surface_mode==GaussianSurfaceMode::corners);
    VISTA_CHECK(!parse_room_map_view_request(view+" CORNERS",100).gaussian_corner_radius_m);
    VISTA_CHECK(parse_room_map_view_request(view+" CORNERS 1.5",100).gaussian_corner_radius_m==1.5);
    VISTA_CHECK(parse_room_map_view_request(view+" CORNERS 0",100).gaussian_corner_radius_m==0.0);
    VISTA_CHECK_THROWS(parse_room_map_view_request(view+" CORNERS -0.1",100));
    VISTA_CHECK_THROWS(parse_room_map_view_request(view+" CORNERS 4",100));
    VISTA_CHECK_THROWS(parse_room_map_view_request(view+" CORNERS 1.5extra",100));
    VISTA_CHECK_THROWS(parse_room_map_view_request(view+" POINTS",100));
    VISTA_CHECK_THROWS(parse_room_map_view_request(view+" EDGES extra",100));
}

VISTA_TEST(gaussian_websocket_switches_modes_on_frozen_map_without_camera_cooldown_or_cross_client_effects) {
    struct Source final : vista::models::GaussianMapViewSource {
        std::array<double,6> bounds() const override {return {-10,-10,-10,10,10,10};}
        std::vector<vista::models::GaussianMapPoint> select_splats(const vista::models::RoomMapViewRequest& request) const override {
            vista::models::GaussianMapPoint point;point.center={1,2,3,77};point.covariance={.1F,0,0,.1F,0,.1F};
            using vista::models::GaussianSurfaceMode;
            switch(request.surface_mode) {
                case GaussianSurfaceMode::edges: return {point,point};
                case GaussianSurfaceMode::corners:
                    return request.gaussian_corner_radius_m.value_or(0)>1.5 ?
                        std::vector<vista::models::GaussianMapPoint>{point,point} :
                        std::vector<vista::models::GaussianMapPoint>{point};
                case GaussianSurfaceMode::full_surface: return {point,point,point};
            }
            return {};
        }
    };
    Runtime runtime;vista::platform::MessageBus bus;vista::platform::StopToken stop;
    auto publisher=bus.publisher<vista::models::RoomMapViewMessage>(vista::models::topics::gaussian_map_view);
    vista::application::PointCloudWebSocketConfig config;config.gaussian_map=true;config.room_map_lod=true;
    config.port=available_port();config.publish_interval=10s;config.maximum_points=10;
    config.gaussian_corner_radius_m=1.25;
    std::string failure;
    auto worker=vista::application::spawn_pointcloud_websocket(bus,{"test-edge-mode",4},stop,config,
        [&](auto,auto error){failure=std::move(error);});
    struct Finish {vista::platform::StopToken& stop;vista::platform::MessageBus& bus;vista::platform::WorkerHandle& worker;
        ~Finish(){stop.request_stop();bus.close();worker.join();}} finish{stop,bus,worker};
    publisher.publish({std::make_shared<Source>(),1,1,3,vista::models::RoomMapState::frozen});
    std::this_thread::sleep_for(100ms);
    Client client,second;client.connect(config.port);client.handshake();
    VISTA_CHECK(client.read_frame().first==1);VISTA_CHECK(client.read_frame().second.size()==32+3*44);
    second.connect(config.port);second.handshake();
    VISTA_CHECK(second.read_frame().first==1);VISTA_CHECK(second.read_frame().second.size()==32+3*44);
    const std::string view="VIEW 10 600 0 0 10 1 0 0 100 -1 0 0 100 0 1 0 100 0 -1 0 100 0 0 1 100 0 0 -1 100";
    const auto start=std::chrono::steady_clock::now();client.send_bytes(masked_text(view+" EDGES"));
    const auto metadata=client.read_frame();const std::string text(metadata.second.begin(),metadata.second.end());
    VISTA_CHECK(text.find("\"supportsEdges\":true")!=std::string::npos && text.find("\"surfaceMode\":\"edges\"")!=std::string::npos);
    VISTA_CHECK(client.read_frame().second.size()==32+2*44);
    client.send_bytes(masked_text(view+" CORNERS"));
    const auto corner_metadata=client.read_frame();
    const std::string corner_text(corner_metadata.second.begin(),corner_metadata.second.end());
    VISTA_CHECK(corner_text.find("\"supportsCorners\":true")!=std::string::npos && corner_text.find("\"surfaceMode\":\"corners\"")!=std::string::npos);
    VISTA_CHECK(corner_text.find("\"defaultCornerRadiusMeters\":1.25")!=std::string::npos);
    VISTA_CHECK(corner_text.find("\"cornerRadiusMeters\":1.25")!=std::string::npos);
    VISTA_CHECK(client.read_frame().second.size()==32+44);
    client.send_bytes(masked_text(view+" CORNERS 2"));
    const auto wider_metadata=client.read_frame();
    const std::string wider_text(wider_metadata.second.begin(),wider_metadata.second.end());
    VISTA_CHECK(wider_text.find("\"cornerRadiusMeters\":2")!=std::string::npos);
    VISTA_CHECK(client.read_frame().second.size()==32+2*44);
    client.send_bytes(masked_text(view));
    VISTA_CHECK(client.read_frame().first==1);VISTA_CHECK(client.read_frame().second.size()==32+3*44);
    VISTA_CHECK(std::chrono::steady_clock::now()-start<2s);
    publisher.publish({std::make_shared<Source>(),2,2,3,vista::models::RoomMapState::frozen});
    VISTA_CHECK(second.read_frame().first==1);VISTA_CHECK(second.read_frame().second.size()==32+3*44);
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(failure.empty());
}

VISTA_TEST(room_map_websocket_sends_new_checkpoint_without_second_interval_delay) {
    struct Source final : vista::models::RoomMapViewSource {
        mutable std::atomic<unsigned> queries{};
        vista::models::RoomMapFrame select_view(const vista::models::RoomMapViewRequest&) const override {
            ++queries;return {1,{{1,2,3,1}}};
        }
        std::array<double,6> bounds() const override {return {0,0,0,4,4,4};}
    };
    Runtime runtime;
    vista::platform::MessageBus bus;vista::platform::StopToken stop;
    auto publisher=bus.publisher<vista::models::RoomMapViewMessage>(vista::models::topics::room_map_view);
    vista::application::PointCloudWebSocketConfig config;config.room_map_lod=true;
    config.port=available_port();config.publish_interval=10s;config.maximum_points=10;
    std::string failure;
    auto worker=vista::application::spawn_pointcloud_websocket(bus,{"test-checkpoint-send",4},stop,config,
        [&](auto,auto error){failure=std::move(error);});
    struct Finish {vista::platform::StopToken& stop;vista::platform::MessageBus& bus;vista::platform::WorkerHandle& worker;
        ~Finish(){stop.request_stop();bus.close();worker.join();}} finish{stop,bus,worker};
    auto source=std::make_shared<Source>();
    publisher.publish({source,1,1,1,vista::models::RoomMapState::building});
    std::this_thread::sleep_for(100ms);
    Client client;client.connect(config.port);client.handshake();
    VISTA_CHECK(client.read_frame().first==1);VISTA_CHECK(client.read_frame().first==2);
    VISTA_CHECK(source->queries==1);
    publisher.publish({source,1,2,1,vista::models::RoomMapState::building});
    std::this_thread::sleep_for(100ms);
    VISTA_CHECK(source->queries==1); // Heartbeat: no disk query or geometry resend.
    const auto start=std::chrono::steady_clock::now();
    publisher.publish({source,2,3,1,vista::models::RoomMapState::building});
    const auto metadata=client.read_frame();
    VISTA_CHECK(metadata.first==1);
    VISTA_CHECK(std::string(metadata.second.begin(),metadata.second.end()).find("\"revision\":2")!=std::string::npos);
    VISTA_CHECK(client.read_frame().first==2);
    VISTA_CHECK(std::chrono::steady_clock::now()-start<2s); // Not the 10 s camera cooldown.
    VISTA_CHECK(source->queries==2);
    stop.request_stop();bus.close();worker.join();VISTA_CHECK(failure.empty());
}
