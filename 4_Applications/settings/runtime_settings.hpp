#pragma once

#include "config.hpp"

#include <map>
#include <mutex>
#include <string_view>

namespace vista::application {

using SettingValues = std::map<std::string, std::string>;
using SettingChanges = std::map<std::string, std::pair<std::string, std::string>>;

struct SettingsRequest {
    std::uint64_t client{}, revision{};
    std::string group;
    SettingValues values;
};

struct SettingsCandidate {
    AppConfig config;
    std::string text;
    SettingChanges changes;
};

SettingValues runtime_setting_values(const AppConfig& config);
SettingsCandidate validate_settings_request(const std::string& active_text,
    const AppConfig& active_config, const SettingsRequest& request,
    const vista::fs::path& data_root);
std::string numeric_setting_value(float number);
std::string settings_change_line(const std::string& key, const std::pair<std::string,std::string>& change);

class RuntimeSettings {
public:
    explicit RuntimeSettings(const AppConfig& initial);

    void submit(SettingsRequest request);
    std::optional<SettingsRequest> take();
    void complete(const SettingsRequest& request, const AppConfig* updated, std::string message);

    struct Outcome { std::uint64_t client{}, number{}; std::string group, message; bool ok{}; };
    struct Snapshot {
        std::uint64_t revision{};
        bool busy{};
        SettingValues values;
        std::optional<Outcome> outcome;
    };
    Snapshot snapshot() const;

private:
    mutable std::mutex mutex_;
    Snapshot state_;
    std::optional<SettingsRequest> request_;
    std::uint64_t outcome_number_{};
};

platform::WorkerHandle spawn_settings_server(platform::StopToken stop, std::string bind_address,
    std::uint16_t port, std::string token, RuntimeSettings& settings);

} // namespace vista::application
