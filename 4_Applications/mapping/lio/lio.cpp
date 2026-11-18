#include "4_Applications/mapping/lio/lio_adapter.hpp"
#include "4_Applications/mapping/lio/imu_sample_history.hpp"
#include "4_Applications/mapping/lio/measurement_epoch.hpp"
#include "3_Devices/lidars/lidar.hpp"
#include "models/topics.hpp"
#include "1_Platform/timing/measurement_clock.hpp"
#include "4_Applications/mapping/mapping_pose_control.hpp"
#include "4_Applications/settings/runtime_settings.hpp"
#include <iostream>
#include "4_Applications/processing/pointcloud/pointcloud_processing.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

namespace vista::application {
platform::WorkerHandle spawn_lio_worker(platform::MessageBus& bus,
    platform::ThreadConfig thread,platform::StopToken stop,LioConfig config,LioCompletion on_complete,
    std::shared_ptr<MappingPoseControl> mapping_control) {
    validate_lio_config(config);
    platform::WorkerTopicInputs inputs(thread.name);
    auto cloud=inputs.subscribe<devices::LidarPointCloudMessage>(bus,models::topics::pointcloud_decoded);
    auto imu=inputs.subscribe<models::ImuMessage>(bus,models::topics::lidar_imu);
    auto world=bus.publisher<devices::LidarPointCloudMessage>(models::topics::pointcloud_world);
    auto telemetry=bus.publisher<models::LocalizationStatus>(models::topics::localization_status);
    return platform::spawn_worker(std::move(thread),[stop,config,inputs=std::move(inputs),cloud=std::move(cloud),
        imu=std::move(imu),world=std::move(world),telemetry=std::move(telemetry),on_complete=std::move(on_complete),
        mapping_control=std::move(mapping_control)]() mutable {
        try {
            MappingPose active;
            if (mapping_control) {
                active = mapping_control->status().active;
                const auto& mount = active.mount;
                config.initial_position_m = {mount.mount_x_m, mount.mount_y_m, mount.mount_z_m};
                config.initial_rpy_deg = {mount.mount_roll_deg, mount.mount_pitch_deg, mount.mount_yaw_deg};
            }
            std::optional<LioEngine> engine;
            if (!mapping_control || active.lio_enabled) engine.emplace(config);
            LioSynchronizer sync(config.maximum_imu_gap_s);
            LioReport report;models::LocalizationStatus latest;
            ImuSampleHistory imu_history;
            MeasurementEpochGate epochs;
            std::uint64_t stale_epoch_messages{}, clock_epoch_recoveries{};
            std::string recovery_reason;std::uint64_t recovery_count{};
            LioRecoveryTimer recovery;
            std::uint64_t reference_host{};bool have_reference{},clock_fault{},cloud_closed{};
            std::optional<std::uint64_t> last_imu_source_time, last_imu_host_time, last_cloud_source_time;
            std::string imu_id, lidar_id;
            std::string fault_reason;
            auto fail=[&](const std::string& reason) { if(!clock_fault) fault_reason=reason;clock_fault=true; };
            std::optional<double> previous_end;
            std::optional<std::chrono::steady_clock::time_point> synchronization_wait_since;
            std::deque<models::ImuMessage> early_imu;
            std::uint64_t early_imu_evictions{};
            struct Pending { std::shared_ptr<const devices::LidarPointCloudMessage> message;double begin,end;std::chrono::steady_clock::time_point arrival; };
            std::deque<Pending> pending;
            auto heartbeat=std::chrono::steady_clock::now();auto last_valid=heartbeat;
            std::optional<models::LocalizationState> published_state;bool published_valid{};
            auto reset_temporal=[&](const std::string& reason) {
                recovery_reason=reason;++recovery_count;
                latest=engine->restart_initialization();
                sync.clear();previous_end.reset();pending.clear();early_imu.clear();
                imu_history.clear();synchronization_wait_since.reset();
                last_imu_source_time.reset();last_imu_host_time.reset();last_cloud_source_time.reset();
                reference_host=0;have_reference=false;
                clock_fault=false;fault_reason.clear();
                heartbeat=std::chrono::steady_clock::now();
                recovery.update(models::LocalizationState::initializing,heartbeat);
            };
            const auto set_fixed_pose = [&] {
                latest = {};
                latest.state = models::LocalizationState::fixed_mount;
                latest.reason = "Fixed Mount pose; Point-LIO disabled";
                latest.pose_valid = true;
                const auto& mount = active.mount;
                latest.position_m = {mount.mount_x_m, mount.mount_y_m, mount.mount_z_m};
                constexpr double half_rad = 3.14159265358979323846 / 360.0;
                const auto sr = std::sin(mount.mount_roll_deg * half_rad);
                const auto cr = std::cos(mount.mount_roll_deg * half_rad);
                const auto sp = std::sin(mount.mount_pitch_deg * half_rad);
                const auto cp = std::cos(mount.mount_pitch_deg * half_rad);
                const auto sy = std::sin(mount.mount_yaw_deg * half_rad);
                const auto cy = std::cos(mount.mount_yaw_deg * half_rad);
                latest.orientation_xyzw = {sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
                                           cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy};
            };
            if (!engine) set_fixed_pose();

            std::uint64_t reset_cutoff_ns{};
            const auto apply_mapping_pose = [&](const MappingPose& requested) {
                const auto old = active;
                active = requested;
                const auto& mount = active.mount;
                config.initial_position_m = {mount.mount_x_m, mount.mount_y_m, mount.mount_z_m};
                config.initial_rpy_deg = {mount.mount_roll_deg, mount.mount_pitch_deg, mount.mount_yaw_deg};
                engine.reset();
                if (active.lio_enabled) engine.emplace(config);
                sync.clear();
                previous_end.reset();
                pending.clear();
                early_imu.clear();
                imu_history.clear();
                epochs = MeasurementEpochGate{};
                recovery = LioRecoveryTimer{};
                synchronization_wait_since.reset();
                last_imu_source_time.reset();
                last_imu_host_time.reset();
                last_cloud_source_time.reset();
                imu_id.clear();
                lidar_id.clear();
                reference_host = 0;
                have_reference = clock_fault = false;
                fault_reason.clear();
                recovery_reason.clear();
                last_valid = heartbeat = std::chrono::steady_clock::now();
                reset_cutoff_ns = platform::monotonic_timestamp_ns();
                if (engine) {
                    latest = engine->restart_initialization();
                    latest.reason = "Mount/LIO settings applied; initializing Point-LIO from Mount";
                } else set_fixed_pose();
                published_state.reset();
                const auto log = [](const char* key, float before, float after) {
                    if (before != after)
                        std::cout << settings_change_line(key,{numeric_setting_value(before),numeric_setting_value(after)}) + '\n';
                };
                log("MountX",old.mount.mount_x_m,active.mount.mount_x_m);
                log("MountY",old.mount.mount_y_m,active.mount.mount_y_m);
                log("MountZ",old.mount.mount_z_m,active.mount.mount_z_m);
                log("MountRollDeg",old.mount.mount_roll_deg,active.mount.mount_roll_deg);
                log("MountPitchDeg",old.mount.mount_pitch_deg,active.mount.mount_pitch_deg);
                log("MountYawDeg",old.mount.mount_yaw_deg,active.mount.mount_yaw_deg);
                if (old.lio_enabled != active.lio_enabled)
                    std::cout << settings_change_line("LioEnabled",{old.lio_enabled ? "1" : "0",active.lio_enabled ? "1" : "0"}) + '\n';
                std::cout << std::flush;
                mapping_control->acknowledge(active.generation);
            };

            auto publish_world = [&](const devices::LidarPointCloudMessage& message, models::PointCloudFrame frame) {
                frame.mapping_generation = active.generation;
                world.publish(devices::LidarPointCloudMessage(message.lidar_id, message.sequence,
                    message.sensor_timestamp_ns, message.received_timestamp_ns, std::move(frame),
                    message.received_monotonic_ns, message.measurement_timestamp_ns,
                    message.measurement_clock_epoch, message.measurement_epoch_started_ns));
                ++report.published;
                last_valid = std::chrono::steady_clock::now();
            };
            auto admit_epoch=[&](std::size_t stream,std::uint64_t epoch,std::uint64_t started,std::uint64_t received) {
                const auto result=epochs.observe(stream,epoch,started,received);
                if(result==MeasurementEpochGate::Result::stale){++stale_epoch_messages;return false;}
                if(result==MeasurementEpochGate::Result::invalid){fail("invalid measurement clock epoch metadata");return false;}
                if(result==MeasurementEpochGate::Result::changed){
                    ++clock_epoch_recoveries;
                    reset_temporal("sensor clock epoch changed; synchronized time and LIO reset; retained map held until matched");
                }
                return true;
            };
            auto accept_imu=[&](models::ImuMessage message) {
                // One active producer, no configured source selector. Refuse to
                // mix two physical IMUs or silently switch an epoch mid-session.
                if(message.source_id.empty() || !message.measurement_timestamp_ns || message.received_monotonic_ns==0)
                    {fail("IMU missing normalized host measurement time");return;}
                if(!imu_id.empty() && message.source_id!=imu_id)
                    {fail("multiple IMU sources or source changed");return;}
                if(!admit_epoch(0,message.measurement_clock_epoch,message.measurement_epoch_started_ns,message.received_monotonic_ns))return;
                if(clock_fault)return;
                imu_id=message.source_id;
                if(!have_reference){early_imu.push_back(std::move(message));while(early_imu.size()>4096){early_imu.pop_front();++early_imu_evictions;}return;}
                // A short, confirmed retransmission does not represent elapsed
                // sensor time. Discard it BEFORE strict normalized-time checks.
                // Unknown reordered samples, conflicting payloads, old epochs,
                // missing clocks and changed producers remain explicit faults.
                if(imu_history.contains_duplicate(message.payload)) {
                    ++report.duplicate_imu_samples;return;
                }
                if(last_imu_host_time && *message.measurement_timestamp_ns<=*last_imu_host_time)
                    {fail("IMU normalized host time is not monotonic");return;}
                if(message.payload.timestamp_ns && last_imu_source_time && message.payload.timestamp_ns<=*last_imu_source_time)
                    {fail("IMU sensor timestamp reset/replay loop");return;}
                imu_id=message.source_id;
                last_imu_host_time=message.measurement_timestamp_ns;
                if(message.payload.timestamp_ns)last_imu_source_time=message.payload.timestamp_ns;
                const auto t=static_cast<double>((static_cast<long double>(*message.measurement_timestamp_ns)-reference_host)*1e-9L);
                const auto& p=message.payload;
                if(!sync.add({t,{p.linear_acceleration_x_m_s2,p.linear_acceleration_y_m_s2,p.linear_acceleration_z_m_s2},
                                {p.angular_velocity_x_rad_s,p.angular_velocity_y_rad_s,p.angular_velocity_z_rad_s}})) fail("invalid IMU values or time ordering");
                else imu_history.remember(p);
            };
            auto drain_imu=[&]() {
                std::shared_ptr<const models::ImuMessage> message;
                // Re-check the actual queue, not just the earlier WaitSet
                // snapshot. Bound work so a busy producer cannot starve scans.
                for(std::size_t i=0;i<4096 && !stop.is_stop_requested();++i) {
                    if(imu.try_receive(message)!=platform::ReceiveStatus::message) break;
                    if (engine && message->received_monotonic_ns >= reset_cutoff_ns) accept_imu(*message);
                }
            };
            auto drop_diagnostics=[&]() {
                return "; imu_topic_drops="+std::to_string(imu.dropped_messages())+
                    "; cloud_topic_drops="+std::to_string(cloud.dropped_messages())+
                    "; pending_scan_drops="+std::to_string(report.dropped)+
                    "; early_imu_evictions="+std::to_string(early_imu_evictions);
            };
            while(!stop.is_stop_requested() && !cloud_closed) {
                if (mapping_control) {
                    if (auto requested = mapping_control->pending()) apply_mapping_pose(*requested);
                }
                const auto ready=inputs.wait_for(std::chrono::milliseconds(20));
                drain_imu();
                if(cloud.is_ready(ready)) {
                    std::shared_ptr<const devices::LidarPointCloudMessage> m;
                    for (std::size_t i = 0; i < cloud.capacity() && !stop.is_stop_requested(); ++i) {
                        const auto received=cloud.try_receive(m);
                        if(received==platform::ReceiveStatus::closed){cloud_closed=true;break;}
                        if(received!=platform::ReceiveStatus::message) break;
                        if (m->payload.points.empty() || m->received_monotonic_ns < reset_cutoff_ns) continue;
                        ++report.received;
                        if (!engine) {
                            auto frame = m->payload;
                            apply_fixed_mount(frame, active.mount);
                            publish_world(*m, std::move(frame));
                            continue;
                        }
                        if(m->lidar_id.empty() || (!lidar_id.empty() && lidar_id!=m->lidar_id)) {fail("LiDAR source changed or sensor timestamp reset/replay loop");continue;}
                        if(!admit_epoch(1,m->measurement_clock_epoch,m->measurement_epoch_started_ns,m->received_monotonic_ns))continue;
                        if(clock_fault)continue;
                        auto minimum=std::numeric_limits<std::uint64_t>::max();std::uint64_t maximum{};
                        for(const auto& p:m->payload.points){minimum=std::min(minimum,p.timestamp_ns);maximum=std::max(maximum,p.timestamp_ns);}
                        if(minimum==0 || maximum<minimum || maximum-minimum>500'000'000ULL){++report.rejected;latest.reason="invalid per-point timestamps/scan duration";latest.state=models::LocalizationState::degraded;latest.pose_valid=false;continue;}
                        const auto duration_ns=maximum-minimum;
                        if(m->lidar_id.empty() || m->received_monotonic_ns==0 || !m->sensor_timestamp_ns ||
                           (!lidar_id.empty() && lidar_id!=m->lidar_id) ||
                           (last_cloud_source_time && minimum<=*last_cloud_source_time)) {fail("LiDAR source changed or sensor timestamp reset/replay loop");continue;}
                        const auto packet_reference=*m->sensor_timestamp_ns;
                        const auto reference_distance=packet_reference>minimum?packet_reference-minimum:minimum-packet_reference;
                        if(reference_distance>500'000'000ULL){fail("LiDAR packet/point timestamps disagree");continue;}
                        const auto host_begin=platform::shift_measurement_time(m->measurement_timestamp_ns,minimum,*m->sensor_timestamp_ns);
                        if(!host_begin){fail("LiDAR missing normalized host measurement time");continue;}
                        lidar_id=m->lidar_id;last_cloud_source_time=minimum;
                        if(!have_reference){reference_host=*host_begin;have_reference=true;for(auto& sample:early_imu)accept_imu(std::move(sample));early_imu.clear();}
                        const auto start=static_cast<double>((static_cast<long double>(*host_begin)-reference_host)*1e-9L);
                        if(pending.size()==4){pending.pop_front();++report.dropped;}
                        pending.push_back({m,start,start+static_cast<double>(duration_ns)*1e-9,std::chrono::steady_clock::now()});
                    }
                }
                // Keep continuous IMU but prefer recent scans under overload.
                // One expensive solve per iteration gives ingestion a chance
                // to catch up instead of solving four obsolete scans in a row.
                if(pending.size()>1) {
                    report.dropped+=pending.size()-1;
                    auto newest=std::move(pending.back());pending.clear();pending.push_back(std::move(newest));
                }
                while(!clock_fault && !pending.empty() && !stop.is_stop_requested()) {
                    // Engine work can take time (especially under a debugger).
                    // Ingest new IMU between every scan and before declaring a
                    // timeout; do not process four scans against a stale buffer.
                    drain_imu();
                    // A new IMU epoch can clear this deque while draining.
                    if(clock_fault || pending.empty()) break;
                    auto& p=pending.front();
                    const auto horizon=engine->propagation_time_s();
                    const auto history_begin=horizon?horizon:previous_end;
                    const auto required_begin=history_begin?std::min(*history_begin,p.begin):p.begin;
                    const auto batch=sync.take(required_begin,p.end);
                    if(!batch) {
                        const auto now=std::chrono::steady_clock::now();
                        if(!synchronization_wait_since)synchronization_wait_since=p.arrival;
                        // Selecting a newer scan must not restart this deadline
                        // forever when the IMU has genuinely stopped.
                        if(now-*synchronization_wait_since<config.synchronization_timeout) break;
                        ++report.rejected;latest.pose_valid=false;
                        if(previous_end) {
                            // An uncovered propagation interval cannot safely be
                            // skipped or reseeded in a persistent world map.
                            fail("IMU coverage lost after LIO start: "+sync.describe_coverage(required_begin,p.end)+drop_diagnostics());
                            break;
                        }
                        latest.state=models::LocalizationState::waiting_imu;
                        latest.reason="IMU synchronization waiting: "+sync.describe_coverage(required_begin,p.end)+drop_diagnostics();pending.pop_front();continue;
                    }
                    synchronization_wait_since.reset();
                    const auto& frame=p.message->payload;
                    const auto point_reference=std::min_element(frame.points.begin(),frame.points.end(),[](const auto& a,const auto& b){return a.timestamp_ns<b.timestamp_ns;})->timestamp_ns;
                    const LioScanView scan{frame,p.begin,p.end,*batch,point_reference};
                    auto output=engine->process(scan);latest=std::move(output.status);previous_end=p.end;
                    if(const auto integrated=engine->propagation_time_s())sync.retain_from(*integrated);
                    if (output.world_cloud) publish_world(*p.message, std::move(*output.world_cloud));
                    else ++report.rejected;
                    pending.pop_front();
                    break;
                }
                if(clock_fault) {
                    // Never restart the world frame silently: that would corrupt
                    // the persistent room map built with the old origin.
                    pending.clear();latest.state=models::LocalizationState::lost;latest.pose_valid=false;
                    latest.reason=fault_reason+"; automatic initialization after 5s LOST";
                }
                const auto now=std::chrono::steady_clock::now();
                if(recovery.update(latest.state,now)) {
                    // Keep the world anchor/local map in the engine. Drop only
                    // stale temporal state; never guess a new world origin.
                    reset_temporal(latest.reason);
                }
                if(engine && latest.pose_valid && now-last_valid>config.synchronization_timeout){latest.pose_valid=false;latest.state=models::LocalizationState::degraded;latest.reason="no fresh localized scan (input/processing delay); map held";}
                // Fault/recovery transitions are delivered immediately; the 1 s
                // heartbeat is monitoring cadence, not permission to hide LOST.
                if(now>=heartbeat || !published_state || *published_state!=latest.state || published_valid!=latest.pose_valid) {
                    latest.timestamp_ns=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
                    latest.dropped_messages=report.dropped+cloud.dropped_messages()+imu.dropped_messages();
                    latest.rejected_scans=report.rejected;
                    auto diagnostic=latest;
                    diagnostic.mapping_generation = active.generation;
                    diagnostic.reason+="; duplicate_imu_samples="+std::to_string(report.duplicate_imu_samples)+
                        "; reinitializations="+std::to_string(recovery_count)+
                        "; clock_epoch_recoveries="+std::to_string(clock_epoch_recoveries)+
                        "; stale_epoch_messages="+std::to_string(stale_epoch_messages);
                    if(!latest.pose_valid && !recovery_reason.empty())
                        diagnostic.reason+="; previous_loss="+recovery_reason;
                    telemetry.publish(std::move(diagnostic));published_state=latest.state;published_valid=latest.pose_valid;
                    heartbeat=now+std::chrono::seconds(1);
                }
            }
            report.dropped+=cloud.dropped_messages()+imu.dropped_messages();
            on_complete(report,{});
        }catch(const std::exception& error){models::LocalizationStatus status;status.state=models::LocalizationState::lost;status.reason=error.what();telemetry.publish(status);on_complete(std::nullopt,error.what());}
    });
}
} // namespace vista::application
