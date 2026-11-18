#include "4_Applications/settings/runtime_settings.hpp"
#include "2_Transport/messaging/websocket_server.hpp"
#include "2_Transport/storage/room_map_storage.hpp"
#include "4_Applications/mapping/mapping_pose_control.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cwchar>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <set>
#include <sstream>
#include <type_traits>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

namespace vista::application {
namespace {
using namespace std::chrono_literals;

std::string value(const std::string& input) { return input; }
std::string value(const char* input) { return input; }
std::string value(bool input) { return input ? "1" : "0"; }
template <typename T> std::string value(T input) {
    std::ostringstream encoded;
    encoded.imbue(std::locale::classic());
    if constexpr (std::is_floating_point_v<T>) {
        // Use a short decimal whenever it recovers the same binary value.
        for (auto precision = std::numeric_limits<T>::digits10;
             precision <= std::numeric_limits<T>::max_digits10; ++precision) {
            encoded.str({});
            encoded << std::setprecision(precision) << input;
            std::istringstream decoded(encoded.str());
            decoded.imbue(std::locale::classic());
            T round_trip{};
            if (decoded >> round_trip && round_trip == input) break;
        }
    } else encoded << input;
    return encoded.str();
}
template <typename T> std::string value(const std::optional<T>& input) {
    return input ? value(*input) : "";
}
auto sec(std::chrono::milliseconds duration) {
    return std::chrono::duration_cast<std::chrono::seconds>(duration).count();
}

struct Field {
    std::string_view key, group;
    bool editable;
    std::string (*read)(const AppConfig&);
};

const Field fields[]{
    {"RawLoggingEnabled", "logging", true, [](const AppConfig& c) { return value(c.raw_logging_enabled); }},
    {"PointCloudLoggingEnabled", "logging", true, [](const AppConfig& c) { return value(c.pointcloud_logging_enabled); }},
    {"Lidar", "lidar", true, [](const AppConfig& c) { return value(devices::to_string(c.lidar_type)); }},
    {"ReconnectIntervalSeconds", "lidar", true, [](const AppConfig& c) { return value(sec(c.lidar_reconnect_interval)); }},
    {"SensorIP", "lidar", true, [](const AppConfig& c) { return value(c.sensor_ip); }},
    {"SensorPort", "lidar", true, [](const AppConfig& c) { return value(c.sensor_port); }},
    {"ConnectionMode", "lidar", true, [](const AppConfig& c) { return value(devices::to_string(c.unitree_connection_mode)); }},
    {"LocalIP", "lidar", true, [](const AppConfig& c) { return value(c.unitree_local_ip); }},
    {"LocalPort", "lidar", true, [](const AppConfig& c) { return value(c.unitree_local_port); }},
    {"SerialPort", "lidar", true, [](const AppConfig& c) { return value(c.unitree_serial_port); }},
    {"BaudRate", "lidar", true, [](const AppConfig& c) { return value(c.unitree_baud_rate); }},
    {"InitializeOnConnect", "lidar", true, [](const AppConfig& c) { return value(c.unitree_initialize_on_connect); }},
    {"WorkMode", "lidar", true, [](const AppConfig& c) { return value(c.unitree_work_mode); }},
    {"UsbSerial", "lidar", true, [](const AppConfig& c) { return value(c.usb_serial); }},
    {"DepthWidth", "lidar", true, [](const AppConfig& c) { return value(c.depth_width); }},
    {"DepthHeight", "lidar", true, [](const AppConfig& c) { return value(c.depth_height); }},
    {"DepthFps", "lidar", true, [](const AppConfig& c) { return value(c.depth_fps); }},
    {"Camera", "camera", false, [](const AppConfig& c) { (void)c; return value("None"); }},
    {"LioRobustImuEnabled", "lio_calibration", true, [](const AppConfig& c) { return value(c.lio.robust_imu_enabled); }},
    {"LioStationaryUpdateEnabled", "lio_calibration", true, [](const AppConfig& c) { return value(c.lio.stationary_update_enabled); }},
    {"LioLidarToImuTranslationX", "lio_calibration", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.lidar_to_imu_translation_m[0])); }},
    {"LioLidarToImuTranslationY", "lio_calibration", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.lidar_to_imu_translation_m[1])); }},
    {"LioLidarToImuTranslationZ", "lio_calibration", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.lidar_to_imu_translation_m[2])); }},
    {"LioLidarToImuRollDeg", "lio_calibration", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.lidar_to_imu_rpy_deg[0])); }},
    {"LioLidarToImuPitchDeg", "lio_calibration", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.lidar_to_imu_rpy_deg[1])); }},
    {"LioLidarToImuYawDeg", "lio_calibration", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.lidar_to_imu_rpy_deg[2])); }},
    {"LioScanVoxelSizeMeters", "lio_scan", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.scan_voxel_m)); }},
    {"LioMapVoxelSizeMeters", "lio_scan", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.map_voxel_m)); }},
    {"LioLocalMapRadiusMeters", "lio_scan", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.local_radius_m)); }},
    {"LioLocalMapMaxPoints", "lio_scan", true, [](const AppConfig& c) { return value(c.lio.local_max_points); }},
    {"LioScanMinPoints", "lio_scan", true, [](const AppConfig& c) { return value(c.lio.scan_min_points); }},
    {"LioScanMaxPoints", "lio_scan", true, [](const AppConfig& c) { return value(c.lio.scan_max_points); }},
    {"LioInitializationSamples", "lio_scan", true, [](const AppConfig& c) { return value(c.lio.initialization_samples); }},
    {"LioImuAccelerationCovariance", "lio_noise", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.imu_acceleration_covariance)); }},
    {"LioImuGyroscopeCovariance", "lio_noise", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.imu_gyroscope_covariance)); }},
    {"LioImuAccelerationBiasCovariance", "lio_noise", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.imu_acceleration_bias_covariance)); }},
    {"LioImuGyroscopeBiasCovariance", "lio_noise", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.imu_gyroscope_bias_covariance)); }},
    {"LioVelocityProcessCovariance", "lio_noise", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.velocity_process_covariance)); }},
    {"LioAccelerationProcessCovariance", "lio_noise", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.acceleration_process_covariance)); }},
    {"LioAngularVelocityProcessCovariance", "lio_noise", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.angular_velocity_process_covariance)); }},
    {"LioLidarMeasurementCovariance", "lio_noise", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.lidar_measurement_covariance)); }},
    {"LioImuAccelerationNorm", "lio_noise", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.imu_acceleration_norm)); }},
    {"LioImuAccelerationSaturation", "lio_noise", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.imu_acceleration_saturation)); }},
    {"LioImuGyroscopeSaturation", "lio_noise", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.imu_gyroscope_saturation)); }},
    {"LioMinRangeMeters", "lio_quality", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.minimum_range_m)); }},
    {"LioPlaneThresholdMeters", "lio_quality", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.plane_threshold_m)); }},
    {"LioMaxImuGapMilliseconds", "lio_quality", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.maximum_imu_gap_s * 1000)); }},
    {"LioMinMatchRatio", "lio_quality", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.minimum_match_ratio)); }},
    {"LioMaxResidualMeters", "lio_quality", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.maximum_residual_m)); }},
    {"LioMaxTranslationStepMeters", "lio_quality", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.maximum_translation_step_m)); }},
    {"LioMaxRotationStepDeg", "lio_quality", true, [](const AppConfig& c) { return value(static_cast<float>(c.lio.maximum_rotation_step_deg)); }},
    {"LioSynchronizationTimeoutMilliseconds", "lio_quality", true, [](const AppConfig& c) { return value(c.lio.synchronization_timeout.count()); }},
    {"Radar", "radar", false, [](const AppConfig& c) { (void)c; return value("None"); }},
    {"GrafanaPublishIntervalMilliseconds", "telemetry", true, [](const AppConfig& c) { return value(c.grafana.publish_interval.count()); }},
    {"GrafanaRetryIntervalSeconds", "telemetry", true, [](const AppConfig& c) { return value(sec(c.grafana.retry_interval)); }},
    {"SystemMonitorIntervalMilliseconds", "telemetry", true, [](const AppConfig& c) { return value(c.system_monitor.sample_interval.count()); }},
    {"RoomMapEnabled", "room_map", true, [](const AppConfig& c) { return value(c.room_map.enabled); }},
    {"RoomMapVoxelSizeMeters", "room_map", true, [](const AppConfig& c) { return value(c.room_map.voxel_size_m); }},
    {"RoomMapCacheMaxVoxels", "room_map", true, [](const AppConfig& c) { return value(c.room_map.cache_max_voxels); }},
    {"RoomMapCacheMaxTiles", "room_map", true, [](const AppConfig& c) { return value(c.room_map.cache_max_tiles); }},
    {"RoomMapTileSizeMeters", "room_map", true, [](const AppConfig& c) { return value(c.room_map.tile_size_m); }},
    {"RoomMapLodPointsPerNode", "room_map", true, [](const AppConfig& c) { return value(c.room_map.lod_points_per_node); }},
    {"RoomMapMinObservations", "room_map", true, [](const AppConfig& c) { return value(c.room_map.minimum_observations); }},
    {"RoomMapObservationIntervalMilliseconds", "room_map", true, [](const AppConfig& c) { return value(c.room_map.observation_interval.count()); }},
    {"RoomMapFreezeAfterSeconds", "room_map", true, [](const AppConfig& c) { return value(sec(c.room_map.freeze_after)); }},
    {"RoomMapPublishIntervalMilliseconds", "room_map_io", true, [](const AppConfig& c) { return value(c.room_map.publish_interval.count()); }},
    {"RoomMapLoadExisting", "room_map_io", true, [](const AppConfig& c) { return value(c.room_map.load_existing); }},
    {"RoomMapFile", "room_map_io", true, [](const AppConfig& c) { return value(c.room_map.file.u8string()); }},
    {"RoomMapFreeSpaceMinObservations", "room_map_io", true, [](const AppConfig& c) { return value(c.room_map.free_space_minimum_observations); }},
    {"RoomMapRaycastMaxRaysPerWindow", "room_map_io", true, [](const AppConfig& c) { return value(c.room_map.raycast_maximum_rays_per_window); }},
    {"RoomMapRaycastMaxRangeMeters", "room_map_io", true, [](const AppConfig& c) { return value(c.room_map.raycast_maximum_range_m); }},
    {"RoomMapRaycastSurfaceMarginMeters", "room_map_io", true, [](const AppConfig& c) { return value(c.room_map.raycast_surface_margin_m); }},
    {"RoomMapWebSocketBindAddress", "room_map_io", false, [](const AppConfig& c) { return value(c.room_map_websocket.bind_address); }},
    {"RoomMapWebSocketPort", "room_map_io", false, [](const AppConfig& c) { return value(c.room_map_websocket.port); }},
    {"RoomMapWebSocketMaxPoints", "room_map_io", true, [](const AppConfig& c) { return value(c.room_map_websocket.maximum_points); }},
    {"RoomMapWebSocketMaxClients", "room_map_io", true, [](const AppConfig& c) { return value(c.room_map_websocket.maximum_clients); }},
    {"RoomMapWebSocketRetryIntervalSeconds", "room_map_io", true, [](const AppConfig& c) { return value(sec(c.room_map_websocket.retry_interval)); }},
    {"GaussianMapEnabled", "gaussian", true, [](const AppConfig& c) { return value(c.gaussian_map.enabled); }},
    {"GaussianMapWebSocketBindAddress", "gaussian", false, [](const AppConfig& c) { return value(c.gaussian_map_websocket.bind_address); }},
    {"GaussianMapWebSocketPort", "gaussian", false, [](const AppConfig& c) { return value(c.gaussian_map_websocket.port); }},
    {"GaussianMapWebSocketMaxPoints", "gaussian", true, [](const AppConfig& c) { return value(c.gaussian_map_websocket.maximum_points); }},
    {"ForegroundEnabled", "foreground", true, [](const AppConfig& c) { return value(c.foreground.enabled); }},
    {"ForegroundBackgroundDistanceMeters", "foreground", true, [](const AppConfig& c) { return value(c.foreground.background_distance_m); }},
    {"ForegroundOutlierFilterEnabled", "foreground", true, [](const AppConfig& c) { return value(c.foreground.outlier_filter_enabled); }},
    {"ForegroundOutlierRadiusMeters", "foreground", true, [](const AppConfig& c) { return value(c.foreground.outlier_radius_m); }},
    {"ForegroundOutlierStartDistanceMeters", "foreground", true, [](const AppConfig& c) { return value(c.foreground.outlier_start_distance_m); }},
    {"ForegroundOutlierEndDistanceMeters", "foreground", true, [](const AppConfig& c) { return value(c.foreground.outlier_end_distance_m); }},
    {"ForegroundOutlierMaxRadiusMeters", "foreground", true, [](const AppConfig& c) { return value(c.foreground.outlier_max_radius_m); }},
    {"ForegroundOutlierMinNeighbors", "foreground", true, [](const AppConfig& c) { return value(c.foreground.outlier_minimum_neighbors); }},
    {"ForegroundMaxPoints", "foreground", true, [](const AppConfig& c) { return value(c.foreground.maximum_input_points); }},
    {"ForegroundPublishIntervalMilliseconds", "foreground", true, [](const AppConfig& c) { return value(c.foreground.publish_interval.count()); }},
    {"ForegroundWebSocketBindAddress", "foreground", false, [](const AppConfig& c) { return value(c.foreground_websocket.bind_address); }},
    {"ForegroundWebSocketPort", "foreground", false, [](const AppConfig& c) { return value(c.foreground_websocket.port); }},
};

std::vector<std::string> serial_ports() {
    std::set<std::string> ports;
#ifdef _WIN32
    std::vector<wchar_t> names(65'536);
    if (QueryDosDeviceW(nullptr, names.data(), static_cast<DWORD>(names.size()))) {
        for (auto name = names.data(); *name; name += std::char_traits<wchar_t>::length(name) + 1) {
            const std::wstring_view entry(name);
            if (entry.size() > 3 && entry.substr(0,3) == L"COM" &&
                entry.find_first_not_of(L"0123456789",3) == std::wstring_view::npos)
                ports.emplace("COM" + std::to_string(std::wcstoul(name + 3,nullptr,10)));
        }
    }
#else
    for (const auto& root : {vista::fs::path("/dev/serial/by-id"),vista::fs::path("/dev")}) {
        std::error_code error;
        vista::fs::directory_iterator end;
        for (vista::fs::directory_iterator entry(root,error); !error && entry != end; entry.increment(error)) {
            const auto name = entry->path().filename().string();
            if (root == "/dev" && name.rfind("ttyACM",0) != 0 && name.rfind("ttyUSB",0) != 0) continue;
            if (vista::fs::exists(entry->path(),error)) ports.insert(entry->path().u8string());
            error.clear();
        }
    }
#endif
    return {ports.begin(), ports.end()};
}

std::string json_string(std::string_view text) {
    static constexpr char digits[] = "0123456789abcdef";
    std::string result{"\""};
    for (const unsigned char byte : text) {
        if (byte == '"' || byte == '\\') { result += '\\'; result += static_cast<char>(byte); }
        else if (byte < 32) {
            result += "\\u00";
            result += digits[byte >> 4];
            result += digits[byte & 15];
        } else result += static_cast<char>(byte);
    }
    result += '"';
    return result;
}

std::string state_message(const RuntimeSettings::Snapshot& snapshot) {
    std::string result = "{\"type\":\"state\",\"revision\":" + std::to_string(snapshot.revision) +
        ",\"busy\":" + (snapshot.busy ? "true" : "false") +
        ",\"realSense\":" + (VISTA_HAS_REALSENSE ? "true" : "false") + ",\"settings\":{";
    bool first = true;
    for (const auto& item : snapshot.values) {
        if (!first) result += ',';
        first = false;
        result += json_string(item.first) + ':' + json_string(item.second);
    }
    return result + "}}";
}

bool matches_token(std::string_view input, std::string_view token) {
    if (input.size() != token.size()) return false;
    unsigned diff = 0;
    for (std::size_t i = 0; i < token.size(); ++i) diff |= input[i] ^ token[i];
    return diff == 0;
}

SettingsRequest parse_apply(std::uint64_t client, const std::string& message) {
    std::istringstream input(message);
    std::string header, command, revision, extra;
    std::getline(input, header);
    std::istringstream head(header);
    SettingsRequest request;
    request.client = client;
    if (!(head >> command >> revision >> request.group) || command != "APPLY" || (head >> extra))
        throw std::invalid_argument("Invalid Apply command");
    const auto decoded = std::from_chars(revision.data(), revision.data()+revision.size(), request.revision);
    if (decoded.ec != std::errc{} || decoded.ptr != revision.data()+revision.size())
        throw std::invalid_argument("Invalid configuration revision");
    for (std::string line; std::getline(input, line);) {
        const auto split = line.find('=');
        if (split == std::string::npos || !request.values.emplace(line.substr(0,split), line.substr(split+1)).second)
            throw std::invalid_argument("Invalid or duplicate configuration field");
    }
    if (request.values.empty() || request.values.size() > 16)
        throw std::invalid_argument("Apply expects one to sixteen changes");
    return request;
}

void send_ports(transport::WebSocketServer& server, std::uint64_t client) {
    std::string message = "{\"type\":\"ports\",\"ports\":[";
    bool first = true;
    for (const auto& port : serial_ports()) {
        if (!first) message += ',';
        first = false;
        message += json_string(port);
    }
    server.send_text(client, message + "]}");
}

vista::fs::path validated_reference(const AppConfig& config, const vista::fs::path& root) {
    // Remote operators select saved maps on the edge; canonicalize before
    // checking containment so symlinks cannot leave the map directory.
    std::error_code error;
    const auto directory = vista::fs::canonical(root / "maps", error);
    if (error) throw std::invalid_argument("The edge data/maps directory is unavailable");
    auto supplied = config.room_map.file;
    if (supplied.is_relative()) supplied = directory / supplied;
    const auto path = vista::fs::canonical(supplied, error);
    if (error || path.extension() != ".pcd" || !vista::fs::is_regular_file(path,error))
        throw std::invalid_argument("Choose an existing .pcd file in the edge data/maps directory");
    const auto relative = path.lexically_relative(directory);
    if (relative.empty() || relative.is_absolute() || *relative.begin() == "..")
        throw std::invalid_argument("Room Map PCD must be in the edge data/maps directory");
    if (!transport::stream_room_map_snapshot(path, [](const models::RoomMapPoint&){}))
        throw std::invalid_argument("The Room Map file contains no points");
    return path;
}
} // namespace

SettingValues runtime_setting_values(const AppConfig& config) {
    SettingValues result;
    for (const auto& field : fields) result.emplace(field.key, field.read(config));
    return result;
}

SettingsCandidate validate_settings_request(const std::string& text, const AppConfig& active,
    const SettingsRequest& request, const vista::fs::path& data_root) {
    for (const auto& item : request.values) {
        const auto field = std::find_if(std::begin(fields),std::end(fields),[&](const Field& field) {
            return field.key == item.first && field.group == request.group;
        });
        if (field == std::end(fields) || !field->editable)
            throw std::invalid_argument("Field is unavailable in this panel");
        if (item.second.size() > 512 ||
            std::any_of(item.second.begin(),item.second.end(),[](unsigned char c){return c < 32 || c == 127 || c == '#';}))
            throw std::invalid_argument("Invalid configuration value");
    }
    SettingsCandidate result;
    result.text = overlay_device_config(text, request.values);
    result.config = parse_device_config_text(result.text);
#if !VISTA_HAS_REALSENSE
    if (result.config.lidar_type == devices::LidarType::realsense_l515 && active.lidar_type != result.config.lidar_type)
        throw std::invalid_argument("This build has no RealSense support");
#endif
    for (const auto& field : fields) {
        const auto old_value = field.read(active);
        const auto new_value = field.read(result.config);
        if (old_value != new_value) result.changes.emplace(field.key,std::make_pair(old_value,new_value));
    }
    if (result.config.room_map.load_existing &&
        (result.changes.count("RoomMapFile") || result.changes.count("RoomMapLoadExisting"))) {
        result.config.room_map.file = validated_reference(result.config,data_root);
        const auto absolute = result.config.room_map.file.u8string();
        result.text = overlay_device_config(result.text,{{"RoomMapFile",absolute}});
        const auto old_path = active.room_map.file.u8string();
        if (old_path == absolute) result.changes.erase("RoomMapFile");
        else result.changes["RoomMapFile"] = {old_path,absolute};
    }
    if (result.changes.count("SerialPort")) {
        const auto ports = serial_ports();
        if (std::find(ports.begin(),ports.end(),result.config.unitree_serial_port) == ports.end())
            throw std::invalid_argument("Select an available serial port on the edge. Use Refresh ports.");
    }
    return result;
}

std::string numeric_setting_value(float number) { return value(number); }

std::string settings_change_line(const std::string& key, const std::pair<std::string,std::string>& change) {
    return "[settings] " + key + " " + json_string(change.first) + " -> " + json_string(change.second);
}

RuntimeSettings::RuntimeSettings(const AppConfig& initial) { state_.values = runtime_setting_values(initial); }

void RuntimeSettings::submit(SettingsRequest request) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.busy) throw std::invalid_argument("A configuration change is in progress");
    if (state_.revision != request.revision)
        throw std::invalid_argument("Configuration has changed. Review the refreshed values.");
    request_ = std::move(request);
    state_.busy = true;
}

std::optional<SettingsRequest> RuntimeSettings::take() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = std::move(request_);
    request_.reset();
    return result;
}

void RuntimeSettings::complete(const SettingsRequest& request, const AppConfig* updated, std::string message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (updated) {
        state_.values = runtime_setting_values(*updated);
        ++state_.revision;
    }
    state_.outcome = Outcome{request.client,++outcome_number_,request.group,std::move(message),updated != nullptr};
    state_.busy = false;
}

RuntimeSettings::Snapshot RuntimeSettings::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

platform::WorkerHandle spawn_settings_server(platform::StopToken stop, std::string bind_address,
    std::uint16_t port, std::string token, RuntimeSettings& settings) {
    auto server = transport::WebSocketServer::listen({std::move(bind_address),port,8,1000ms,250ms});
    std::cout << "Settings WebSocket: ws://" << server.config().bind_address << ':' << port << '\n';
    return platform::spawn_worker({"runtime-settings",4},[stop,server=std::move(server),
        token=std::move(token),&settings]() mutable {
        std::set<std::uint64_t> authenticated;
        std::uint64_t revision{}, outcome{};
        bool busy{};
        while (!stop.is_stop_requested()) {
            server.poll_accept();
            for (const auto& incoming : server.poll_text()) {
                try {
                    if (incoming.text.rfind("AUTH ",0) == 0) {
                        const auto supplied = std::string_view(incoming.text).substr(5);
                        if (!matches_token(supplied,token)) {
                            authenticated.erase(incoming.client_id);
                            throw std::invalid_argument("Invalid control token");
                        }
                        authenticated.insert(incoming.client_id);
                        server.send_text(incoming.client_id,state_message(settings.snapshot()));
                        send_ports(server,incoming.client_id);
                    } else {
                        if (!authenticated.count(incoming.client_id))
                            throw std::invalid_argument("Enter the control token first");
                        if (incoming.text == "PORTS") send_ports(server,incoming.client_id);
                        else settings.submit(parse_apply(incoming.client_id,incoming.text));
                    }
                } catch (const std::invalid_argument& error) {
                    server.send_text(incoming.client_id,
                        "{\"type\":\"error\",\"message\":" + json_string(error.what()) + "}");
                }
            }
            const auto ids = server.client_ids();
            for (auto i=authenticated.begin();i!=authenticated.end();) {
                if (std::find(ids.begin(),ids.end(),*i) == ids.end()) i=authenticated.erase(i); else ++i;
            }
            if (!authenticated.empty()) {
                const auto snapshot = settings.snapshot();
                if (snapshot.revision != revision || snapshot.busy != busy) {
                    const auto message = state_message(snapshot);
                    for (const auto id : authenticated) server.send_text(id,message);
                    revision = snapshot.revision;
                    busy = snapshot.busy;
                }
                if (snapshot.outcome && snapshot.outcome->number > outcome) {
                    const auto& last = *snapshot.outcome;
                    if (authenticated.count(last.client)) server.send_text(last.client,
                        "{\"type\":\"result\",\"group\":" + json_string(last.group) + ",\"ok\":" +
                        (last.ok ? "true" : "false") + ",\"message\":" + json_string(last.message) + "}");
                    outcome = last.number;
                }
            }
            std::this_thread::sleep_for(50ms);
        }
    });
}
} // namespace vista::application
