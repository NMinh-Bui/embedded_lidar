#include "lib.hpp"
#include "4_Applications/mapping/mapping_pose_control.hpp"
#include "4_Applications/settings/runtime_settings.hpp"
#include "1_Platform/os/paths.hpp"

#include <map>
#include <set>

namespace {
using namespace vista;
using namespace vista::application;
using namespace std::chrono_literals;

enum class Task {
    read, decode, lio, clean, live_clean, room, gaussian, foreground,
    room_socket, gaussian_socket, foreground_socket, raw_log, pcd_log,
    grafana, monitor
};

class CaptureWorkers {
public:
    explicit CaptureWorkers(const platform::StopToken& stop) : stop_(stop) {}
    ~CaptureWorkers() {
        for (auto& entry : workers_) entry.second.stop.request_local_stop();
        for (auto& entry : workers_) {
            try { entry.second.handle.join(); } catch (...) {}
        }
    }

    template <typename Report, typename Spawn>
    void start(Task task, const WorkerConfig& worker, Spawn spawn,
        std::function<void(const Report&)> on_report = {}) {
        if (!worker.enabled || workers_.count(task)) return;
        auto stop = stop_.child();
        auto result = std::make_shared<platform::WorkerResult<Report>>();
        std::vector<platform::WorkerHandle> handles;
        handles.reserve(1);
        platform::add_worker(handles, spawn(worker.thread,stop,platform::completion_for(result)));
        workers_.emplace(task, Active{std::move(handles.front()),stop,
            [worker,result,on_report=std::move(on_report)](std::vector<std::string>& errors) {
                platform::collect_worker_error(errors,worker.thread.name,result,true);
                if (on_report && result->report) on_report(*result->report);
            }});
    }

    void stop(const std::set<Task>& tasks) {
        std::vector<std::string> failures;
        // Stop the reader first. The decoder and loggers can then finish or
        // release their old subscriptions before new source packets arrive.
        for (auto task : tasks) {
            const auto found = workers_.find(task);
            if (found != workers_.end()) found->second.stop.request_local_stop();
        }
        for (auto task : tasks) join(task, failures);
        platform::throw_if_worker_failures(failures);
    }

    void join_all() {
        std::vector<std::string> failures;
        while (!workers_.empty()) join(workers_.begin()->first, failures);
        platform::throw_if_worker_failures(failures);
    }

private:
    struct Active {
        platform::WorkerHandle handle;
        platform::StopToken stop;
        std::function<void(std::vector<std::string>&)> report;
    };
    void join(Task task, std::vector<std::string>& failures) {
        const auto found = workers_.find(task);
        if (found == workers_.end()) return;
        auto& active = found->second;
        try { active.handle.join(); }
        catch (const std::exception& error) { failures.push_back(active.handle.name() + ": " + error.what()); }
        catch (...) { failures.push_back(active.handle.name() + ": unknown worker error"); }
        active.report(failures);
        workers_.erase(found);
    }

    platform::StopToken stop_;
    std::map<Task,Active> workers_;
};

std::set<Task> affected_workers(const SettingsRequest& request, const SettingChanges& changes) {
    const auto changed = [&](const char* field) { return changes.count(field) != 0; };
    std::set<Task> tasks;
    const auto reset_map = [&] {
        tasks.insert({Task::room,Task::room_socket,Task::gaussian,Task::gaussian_socket,
                      Task::foreground,Task::foreground_socket});
    };
    if (request.group == "lidar" && changes.size() == 1 && changed("ReconnectIntervalSeconds"))
        return tasks;
    if (request.group == "lidar" || request.group.rfind("lio_",0) == 0) {
        reset_map();
        tasks.insert({Task::lio,Task::clean,Task::pcd_log});
        if (request.group == "lidar")
            tasks.insert({Task::read,Task::decode,Task::live_clean,Task::raw_log,Task::grafana});
    } else if (request.group == "logging") {
        if (changed("RawLoggingEnabled")) tasks.insert(Task::raw_log);
        if (changed("PointCloudLoggingEnabled")) tasks.insert(Task::pcd_log);
    } else if (request.group == "telemetry") {
        if (changed("SystemMonitorIntervalMilliseconds")) tasks.insert(Task::monitor);
        if (changed("GrafanaRetryIntervalSeconds") || changed("GrafanaPublishIntervalMilliseconds"))
            tasks.insert(Task::grafana);
    } else if (request.group == "room_map") {
        reset_map();
    } else if (request.group == "room_map_io") {
        for (const auto& change : changes)
            if (change.first.rfind("RoomMapWebSocket",0) != 0) reset_map();
        if (changed("RoomMapWebSocketMaxPoints")) tasks.insert(Task::room_socket);
        if (changed("RoomMapWebSocketMaxClients") || changed("RoomMapWebSocketRetryIntervalSeconds"))
            tasks.insert({Task::room_socket,Task::gaussian_socket,Task::foreground_socket});
    } else if (request.group == "gaussian") {
        tasks.insert(Task::gaussian_socket);
        if (changed("GaussianMapEnabled")) tasks.insert(Task::gaussian);
    } else if (request.group == "foreground") {
        tasks.insert({Task::foreground,Task::foreground_socket});
    }
    return tasks;
}

vista::fs::path next_capture_file(const vista::fs::path& preferred) {
    vista::fs::create_directories(preferred.parent_path());
    for (std::size_t suffix=0;suffix<10'000;++suffix) {
        const auto path = suffix == 0 ? preferred : preferred.parent_path() /
            (preferred.stem().native() + vista::fs::path("_" + std::to_string(suffix)).native() +
             preferred.extension().native());
        if (platform::create_text_file_if_missing(path, {})) return path;
    }
    throw std::runtime_error("No available capture filename");
}

std::string restart_summary(const std::set<Task>& tasks) {
    if (tasks.count(Task::read)) return "Applied. LiDAR reconnecting. A new mapping session is starting.";
    if (tasks.count(Task::lio)) return "Applied. Point-LIO and the mapping session are restarting.";
    if (tasks.count(Task::room)) return "Applied. Room Map is starting a new session.";
    return "Applied for this session.";
}

void run_capture(AppConfig active, std::string device_text, const std::optional<std::string>& token) {
    const auto root = application_data_root();
    const auto session_start = std::chrono::system_clock::now();
    auto config = active;
    configure_session_output_paths(config,root,session_start);
    auto runtime = config.runtime_config();
    const bool use_control = token.has_value();
    std::shared_ptr<MappingPoseControl> mapping_control;
    if (use_control) {
        mapping_control = std::make_shared<MappingPoseControl>(
            MappingPose{config.mounting.value_or(MountingConfig{}),config.lio.enabled},*token);
    }

    platform::StopToken stop;
    platform::MessageBus bus(16,use_control ? platform::MessageBus::Lifetime::explicit_close :
        platform::MessageBus::Lifetime::last_publisher);
        // Main only defines queue capacities. Each worker creates its own
        // publisher/subscriber endpoints inside its spawn_* function.
        bus.configure_topic(
            vista::models::topics::lidar_raw,
            runtime.queues.raw_capacity);
        bus.configure_topic(
            vista::models::topics::lidar_imu,
            4096); // Bounded ~8 s at 500 Hz, including slow Debug LIO solves.
        bus.configure_topic(
            vista::models::topics::pointcloud_decoded,
            runtime.queues.decoded_capacity);
        bus.configure_topic(vista::models::topics::pointcloud_cleaned, runtime.queues.processed_capacity);
        bus.configure_topic(vista::models::topics::pointcloud_cleaned_sensor, runtime.queues.processed_capacity);
        bus.configure_topic(vista::models::topics::pointcloud_world,4);
        bus.configure_topic(vista::models::topics::localization_status,runtime.queues.telemetry_capacity);
        // Map snapshots are much larger than individual sensor packets; retain only two.
        bus.configure_topic(vista::models::topics::room_map, 2);
        bus.configure_topic(vista::models::topics::room_map_view, 2);
        bus.configure_topic(vista::models::topics::gaussian_map_view,2);
        bus.configure_topic(vista::models::topics::foreground_points,2);
        bus.configure_topic(vista::models::topics::room_map_status, runtime.queues.telemetry_capacity);
        bus.configure_topic(
            vista::models::topics::system_health,
            runtime.queues.telemetry_capacity);
        bus.configure_topic(
            vista::models::topics::worker_health,
            runtime.queues.telemetry_capacity);
        bus.configure_topic(
            vista::models::topics::storage_health,
            runtime.queues.telemetry_capacity);
        bus.configure_topic(
            vista::models::topics::power_thermal,
            runtime.queues.telemetry_capacity);
    RuntimeSettings settings(active);
    std::optional<platform::WorkerHandle> settings_server;
    std::uint64_t packets{}, points{}, dropped_decode{}, dropped_raw{}, dropped_clean{}, dropped_pcd{};
    std::optional<RoomMapReport> final_map;
    std::optional<ForegroundReport> final_foreground;
    std::optional<GrafanaBridgeReport> final_grafana;
    const auto capture_started = std::chrono::steady_clock::now();
    CaptureWorkers workers(stop);
    auto decoder_slot = std::make_shared<devices::LidarDecoderSlot>();
    auto lidar_config = config.lidar_config();
    auto reconnect_interval = std::make_shared<std::atomic<std::chrono::milliseconds::rep>>(
        config.lidar_reconnect_interval.count());

    // Each factory still declares its own subscriptions. New workers always
    // subscribe before producers start, and replacements drop old input queues.
    const auto start_workers = [&] {
        config.room_map.mapping_control.reset();
        config.room_map_websocket.mapping_control.reset();
        const bool has_imu = config.lidar_type == devices::LidarType::unitree_l2;
        if (mapping_control) {
            const auto pose = mapping_control->status().active;
            config.mounting = pose.mount;
            config.lio.initial_position_m = {pose.mount.mount_x_m,pose.mount.mount_y_m,pose.mount.mount_z_m};
            config.lio.initial_rpy_deg = {pose.mount.mount_roll_deg,pose.mount.mount_pitch_deg,pose.mount.mount_yaw_deg};
            config.lio.enabled = has_imu && pose.lio_enabled;
            // Optional branches can then be toggled without discarding the live map.
            config.room_map.surface_snapshots = config.room_map.enabled;
        }
        runtime = config.runtime_config();
        if (mapping_control && has_imu) {
            config.room_map.mapping_control = mapping_control;
            if (config.grafana.enabled && config.room_map.enabled && !config.room_map.load_existing)
                config.room_map_websocket.mapping_control = mapping_control;
            runtime.preprocessing.input_world_coordinates = true;
            runtime.threads.localization.enabled = true;
        }
        config.foreground.require_localization = runtime.threads.localization.enabled;
        platform::validate_worker_selection(config,runtime);

        const auto& t = runtime.threads;
        workers.start<ForegroundReport>(Task::foreground,t.foreground_filter,[&](auto thread,auto s,auto done) {
            return spawn_foreground_worker(bus,thread,s,config.foreground,done);
        },[&](const auto& report) { final_foreground = report; });
        workers.start<PointCloudWebSocketReport>(Task::foreground_socket,t.foreground_websocket,[&](auto thread,auto s,auto done) {
            return spawn_foreground_websocket(bus,thread,s,config.foreground_websocket,done);
        });
        workers.start<GrafanaBridgeReport>(Task::grafana,t.grafana_bridge,[&](auto thread,auto s,auto done) {
            return spawn_grafana_bridge(bus,thread,s,config.grafana,done);
        },[&](const auto& report) { final_grafana = report; });
        workers.start<PointCloudWebSocketReport>(Task::room_socket,t.room_map_websocket,[&](auto thread,auto s,auto done) {
            return spawn_pointcloud_websocket(bus,thread,s,config.room_map_websocket,done);
        });
        workers.start<PointCloudWebSocketReport>(Task::gaussian_socket,t.gaussian_map_websocket,[&](auto thread,auto s,auto done) {
            return spawn_pointcloud_websocket(bus,thread,s,config.gaussian_map_websocket,done);
        });
        workers.start<GaussianMapReport>(Task::gaussian,t.gaussian_mapping,[&](auto thread,auto s,auto done) {
            auto gaussian = config.gaussian_map;
            if (!config.room_map.storage_directory.empty()) {
                gaussian.storage_directory = config.room_map.storage_directory;
                gaussian.storage_directory += ".gaussian";
            }
            return spawn_gaussian_map_worker(bus,thread,s,gaussian,done);
        });
        workers.start<RoomMapReport>(Task::room,t.room_mapping,[&](auto thread,auto s,auto done) {
            return spawn_room_map_worker(bus,thread,s,config.room_map,done);
        },[&](const auto& report) { final_map = report; });
        workers.start<SystemMonitorReport>(Task::monitor,t.system_monitor,[&](auto thread,auto s,auto done) {
            return spawn_system_monitor(bus,thread,s,config.system_monitor,done);
        });
        workers.start<LoggerReport>(Task::pcd_log,t.pcd_logger,[&](auto thread,auto s,auto done) {
            return spawn_pcd_logger(bus,thread,s,next_capture_file(*config.pcd_path),done);
        },[&](const auto& report) { dropped_pcd += report.dropped_message_count; });
        workers.start<PreprocessingReport>(Task::clean,t.preprocessing,[&](auto thread,auto s,auto done) {
            return spawn_preprocessing_worker(bus,thread,s,runtime.preprocessing,done);
        },[&](const auto& report) { points += report.point_count; dropped_clean += report.dropped_message_count; });
        workers.start<PreprocessingReport>(Task::live_clean,t.live_preprocessing,[&](auto thread,auto s,auto done) {
            return spawn_live_preprocessing_worker(bus,thread,s,runtime.preprocessing,done);
        });
        workers.start<LioReport>(Task::lio,t.localization,[&](auto thread,auto s,auto done) {
            return spawn_lio_worker(bus,thread,s,config.lio,done,config.room_map.mapping_control);
        });
        workers.start<devices::LidarWorkerReport>(Task::decode,t.lidar_decode,[&](auto thread,auto s,auto done) {
            return devices::spawn_lidar_decode_worker(bus,thread,s,decoder_slot,done);
        },[&](const auto& report) { dropped_decode += report.dropped_message_count; });
        workers.start<LoggerReport>(Task::raw_log,t.raw_logger,[&](auto thread,auto s,auto done) {
            return spawn_raw_logger(bus,thread,s,next_capture_file(*config.raw_path),done);
        },[&](const auto& report) { dropped_raw += report.dropped_message_count; });
        workers.start<devices::LidarWorkerReport>(Task::read,t.lidar_read,[&](auto thread,auto s,auto done) {
            return devices::spawn_lidar_read_worker(bus,thread,s,[source=lidar_config] {
                return devices::Lidar::connect(source);
            },config.lidar_reconnect_interval,decoder_slot,done,reconnect_interval);
        },[&](const auto& report) { packets += report.message_count; });
    };

    const auto apply_settings = [&] {
        if (!settings_server) return;
        const auto request = settings.take();
        if (!request) return;
        try {
            auto candidate = validate_settings_request(device_text,active,*request,root);
            if (candidate.changes.empty()) {
                settings.complete(*request,&active,"Values are already active.");
                return;
            }
            const auto affected = affected_workers(*request,candidate.changes);
            auto next = candidate.config;
            configure_session_output_paths(next,root,session_start);
            auto next_source = next.lidar_config();
            if (next_source.unitree_l2 && lidar_config.unitree_l2) {
                const auto& old = *lidar_config.unitree_l2;
                auto& incoming = *next_source.unitree_l2;
                if (old.work_mode == incoming.work_mode && old.serial_port == incoming.serial_port &&
                    old.baud_rate == incoming.baud_rate && old.sensor_ip == incoming.sensor_ip &&
                    old.sensor_port == incoming.sensor_port && old.initialize_on_connect == incoming.initialize_on_connect) {
                    incoming.initialization_state = old.initialization_state;
                }
            }
            if (mapping_control && !mapping_control->suspend_if_idle())
                throw std::invalid_argument("A Mount/LIO pose reset is already pending. Apply again when it completes.");
            struct ResumePose {
                std::shared_ptr<MappingPoseControl> control;
                ~ResumePose() { if (control) control->resume(); }
            } resume{mapping_control};

            const auto previous = config;
            const auto previous_source = lidar_config;
            const auto reset_decoder = [&] {
                if (affected.count(Task::decode)) decoder_slot = std::make_shared<devices::LidarDecoderSlot>();
            };
            try {
                workers.stop(affected);
                reset_decoder();
                config = std::move(next);
                lidar_config = std::move(next_source);
                start_workers();
            } catch (...) {
                workers.stop(affected);
                reset_decoder();
                config = previous;
                lidar_config = previous_source;
                start_workers();
                throw;
            }
            reconnect_interval->store(config.lidar_reconnect_interval.count());
            for (const auto& change : candidate.changes)
                std::cout << settings_change_line(change.first,change.second) + '\n';
            std::cout << std::flush;
            active = std::move(candidate.config);
            device_text = std::move(candidate.text);
            settings.complete(*request,&active,restart_summary(affected));
        } catch (const std::exception& error) {
            settings.complete(*request,nullptr,error.what());
        }
    };

    try {
        start_workers();
        std::cout << "Mapping pose: " << (config.lio.enabled ? LioEngine::backend_name() : "fixed mount") << '\n';
        if (config.lio.enabled)
            std::cout << "LIO time: host monotonic clock; IMU topic: lidar/imu (one active IMU).\n";
        if (use_control) {
            const auto& bind = active.settings_bind_address.empty() ? active.room_map_websocket.bind_address :
                active.settings_bind_address;
            try {
                settings_server.emplace(spawn_settings_server(stop,bind,active.settings_port,*token,settings));
            } catch (const std::exception& error) {
                std::cerr << "Settings WebSocket unavailable: " << error.what() << '\n';
            }
        }
        std::cout << "Capture is running continuously. Press Ctrl+C to stop.\n";
        platform::wait_for_shutdown_request(stop,apply_settings);
    } catch (...) {
        stop.request_stop();
        bus.close();
        if (settings_server) settings_server->join();
        throw;
    }

    bus.close();
    if (settings_server) settings_server->join();
    workers.join_all();
    const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - capture_started);
    std::cout << "Capture complete: " << packets << " packet(s), " << points << " processed point(s), "
              << elapsed.count() << " seconds\n";
    std::cout << "Ring-buffer drops: decoder=" << dropped_decode << ", raw-logger=" << dropped_raw
              << ", preprocessing=" << dropped_clean << ", pcd-logger=" << dropped_pcd << '\n';
    if (final_grafana)
        std::cout << "Grafana Live: published=" << final_grafana->published_measurements
                  << ", imu-messages=" << final_grafana->received_imu_messages
                  << ", failed-attempts=" << final_grafana->failed_publish_attempts
                  << ", input-drops=" << final_grafana->dropped_input_messages << '\n';
    if (final_map)
        std::cout << "Room map: " << final_map->map_points << " points, input-drops="
                  << final_map->dropped_input_messages << '\n';
    if (final_foreground) {
        const auto& report = *final_foreground;
        std::cout << "Foreground scan filter: " << report.isolated_points << " isolated point(s) removed, "
                  << report.filter_unresolved_points << " unresolved, " << report.filter_distance_checks << " neighbor checks\n";
        std::cout << "Foreground: " << report.processed_frames << " sampled query frame(s), "
                  << report.foreground_frames << " nonempty frame(s), " << report.foreground_points
                  << " points, background=" << report.background_points << ", unresolved=" << report.unknown_points
                  << ", peak cache=" << report.peak_cache_points << ", query failures=" << report.query_failures
                  << ", input drops=" << report.dropped_input_messages << '\n';
    }
}
} // namespace

int main() {
    try {
        // All user settings are loaded from DeviceConfig.txt.
        const auto config_path = vista::default_device_config_file();
        const auto created = vista::create_missing_startup_config_files(
            config_path.parent_path());
        if (created.device_config_created) {
            std::cout << "Created default device configuration: "
                      << config_path.u8string() << '\n';
        }
        if (created.grafana_secret_created) {
            std::cout << "Created GrafanaSecret.txt beside the executable with an "
                         "empty GrafanaToken. Fill it in to enable Grafana authentication.\n";
        }
        if (created.mapping_control_secret_created) {
            std::cout << "Created MappingControlSecret.txt beside the executable. "
                         "Fill in MappingControlToken to enable remote controls.\n";
        }
        if (created.device_config_created) {
            // Do not issue hardware mode/restart commands with unreviewed defaults.
            std::cout << "Review DeviceConfig.txt (especially the COM port/IP), "
                         "then restart this application. No workers were started.\n";
            return EXIT_SUCCESS;
        }
        auto device_text = vista::read_device_config_text(config_path);
        auto active = vista::parse_device_config_text(device_text);
        auto config = active;
        vista::configure_session_output_paths(config,vista::application_data_root(),std::chrono::system_clock::now());
        std::cout << "Device configuration: " << config_path.u8string()
                  << " (selected " << vista::devices::to_string(config.lidar_type)
                  << ")\n";
        std::cout << "Platform: " << vista::platform::platform_name() << '\n';

        if (config.lidar_type == vista::devices::LidarType::realsense_l515) {
            std::cout << "LiDAR configuration: realsense-l515 over USB, depth="
                      << config.depth_width << 'x' << config.depth_height << '@'
                      << config.depth_fps << " FPS, device="
                      << (config.usb_serial ? *config.usb_serial
                                            : "first matching L515")
                      << '\n';
        } else if (config.lidar_type == vista::devices::LidarType::unitree_l2) {
            std::cout << "LiDAR configuration: unitree-l2, mode="
                      << vista::devices::to_string(
                             config.unitree_connection_mode)
                      << ", serial=" << config.unitree_serial_port << '@'
                      << config.unitree_baud_rate << ", UDP sensor="
                      << config.sensor_ip << ':'
                      << config.sensor_port.value_or(6101) << ", local="
                      << config.unitree_local_ip << ':'
                      << config.unitree_local_port.value_or(6201) << '\n';
        } else {
            std::cout << "LiDAR configuration: "
                      << vista::devices::to_string(config.lidar_type) << " at "
                      << config.sensor_ip;
            if (config.sensor_port) {
                std::cout << ':' << *config.sensor_port;
            } else {
                std::cout << ", using driver default port";
            }
            std::cout << '\n';
        }

        std::cout << "Raw output: "
                  << (config.raw_path ? config.raw_path->string() : "disabled")
                  << '\n';
        std::cout << "Point-cloud output: "
                  << (config.pcd_path ? config.pcd_path->string() : "disabled")
                  << '\n';
        std::cout << "LiDAR reconnect interval: "
                  << static_cast<double>(config.lidar_reconnect_interval.count()) /
                         1000.0
                  << " second(s)\n";
        if (config.mounting) {
            const auto& mount = *config.mounting;
            std::cout << "Initial/fixed LiDAR world pose: position=(" << mount.mount_x_m << ", "
                      << mount.mount_y_m << ", " << mount.mount_z_m
                      << ") m, RPY=(" << mount.mount_roll_deg << ", "
                      << mount.mount_pitch_deg << ", "
                      << mount.mount_yaw_deg << ") deg"
                      << '\n';
        } else {
            std::cout << "Initial/fixed LiDAR world pose: origin\n";
        }
        std::cout << "Grafana Live: ";
        if (config.grafana.enabled) {
            std::cout << "http://" << config.grafana.host << ':'
                      << config.grafana.port << "/api/live/push/"
                      << config.grafana.name_space << '\n';
        } else {
            std::cout << "disabled\n";
        }
        std::cout << "Point-cloud WebSocket: ";
        if (config.pointcloud_websocket.enabled) {
            std::cout << "ws://" << config.pointcloud_websocket.bind_address
                      << ':' << config.pointcloud_websocket.port
                      << " (maximum "
                      << config.pointcloud_websocket.maximum_points
                      << " points/frame, every "
                      << config.pointcloud_websocket.publish_interval.count()
                      << " ms)\n";
        } else {
            std::cout << "disabled\n";
        }

        if (config.room_map.enabled) {
            std::cout << "Room map: " << config.room_map.voxel_size_m << " m voxels, RAM cache "
                      << config.room_map.cache_max_voxels << " voxels (total map is disk-backed), freeze after "
                      << config.room_map.freeze_after.count() / 1000 << " active second(s) (0 = continuous)\n";
            std::cout << "Room map snapshot: " << config.room_map.file.string() << '\n';
            std::cout << "Room map mode: " << (config.room_map.load_existing ? "load read-only reference" : "build with free-space ray checks") << '\n';
        }
        if (config.grafana.enabled)
            std::cout << "Room-map WebSocket: ws://" << config.room_map_websocket.bind_address
                      << ':' << config.room_map_websocket.port << '\n';
        if(config.grafana.enabled && config.gaussian_map.enabled)
            std::cout<<"Gaussian-map WebSocket: ws://"<<config.gaussian_map_websocket.bind_address
                <<':'<<config.gaussian_map_websocket.port<<" (geometric intensity surfaces)\n";
        if(config.foreground.enabled)
            std::cout<<"Foreground: live world points outside confirmed RoomMap; not a motion label\n";
        if(config.foreground_websocket.enabled)
            std::cout<<"Foreground WebSocket: ws://"<<config.foreground_websocket.bind_address<<':'<<config.foreground_websocket.port<<'\n';
        std::optional<std::string> mapping_token;
        try {
            const auto* environment_token = std::getenv("VISTA_MAPPING_CONTROL_TOKEN");
            mapping_token = vista::load_mapping_control_token(config_path.parent_path(),
                environment_token ? environment_token : "");
        } catch (const std::exception& error) {
            std::cerr << "Remote controls disabled: " << error.what() << '\n';
        }
        run_capture(std::move(active),std::move(device_text),mapping_token);
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
