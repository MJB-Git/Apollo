/**
 * @file src/abr_controller.h
 * @brief Server-Side Network Adaptive Bitrate (ABR) Controller with Cubic Inflection and Delay-Gradient Pre-Braking.
 */
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <utility>

#include "logging.h"
#include "platform/common.h"

namespace abr {

  using namespace std::literals;

  enum class state_e : int {
    STEADY = 0,
    PRE_CONGESTION_BRAKE = 1,
    CONGESTED = 2,
    PROBING = 3
  };

  inline const char *state_to_string(state_e s) {
    switch (s) {
      case state_e::STEADY: return "STEADY";
      case state_e::PRE_CONGESTION_BRAKE: return "PRE_BRAKE";
      case state_e::CONGESTED: return "CONGESTED";
      case state_e::PROBING: return "PROBING";
      default: return "UNKNOWN";
    }
  }

  /**
   * @brief Helper to detect if an IP address belongs to RFC1918 private subnets, loopback, or local link.
   */
  inline bool is_private_or_local_ip(const std::string &ip_str) {
    if (ip_str.empty()) return false;
    if (ip_str == "127.0.0.1" || ip_str == "::1" || ip_str == "localhost") return true;

    // 10.0.0.0/8
    if (ip_str.rfind("10.", 0) == 0) return true;
    // 192.168.0.0/16
    if (ip_str.rfind("192.168.", 0) == 0) return true;
    // 169.254.0.0/16
    if (ip_str.rfind("169.254.", 0) == 0) return true;
    // 172.16.0.0 - 172.31.255.255
    if (ip_str.rfind("172.", 0) == 0) {
      auto second_dot = ip_str.find('.', 4);
      if (second_dot != std::string::npos) {
        try {
          int second_octet = std::stoi(ip_str.substr(4, second_dot - 4));
          if (second_octet >= 16 && second_octet <= 31) return true;
        } catch (...) {}
      }
    }

    // IPv6 link-local or ULA
    if (ip_str.rfind("fe80:", 0) == 0 || ip_str.rfind("fc", 0) == 0 || ip_str.rfind("fd", 0) == 0) {
      return true;
    }

    return false;
  }

  class abr_controller_t {
  public:
    struct config_t {
      bool enabled = true;
      uint32_t wan_max_kbps = 42000;          // Capped for 50 Mbps NBN FTTP uplink (~42 Mbps payload)
      uint32_t min_kbps = 3000;               // Floor bitrate (3 Mbps)
      uint32_t queue_delay_threshold_ms = 12; // Pre-drop trigger threshold
      uint32_t jitter_threshold_ms = 15;      // Jitter trigger threshold
      float pre_congestion_trim = 0.90f;      // -10% gentle pre-brake
      float loss_brake = 0.70f;               // -30% multiplicative cut on packet loss
      float fast_recovery_ramp = 0.12f;       // +12%/s below 0.85 * W_max
      float cruise_ramp = 0.015f;             // +1.5%/s plateau near W_max
      float probe_ramp = 0.05f;               // +5%/s exploration
      float idr_vbv_clamp_ratio = 1.30f;      // VBV single-frame clamp multiplier for IDR recovery
    };

    abr_controller_t() = default;

    void init(const config_t &cfg, uint32_t client_requested_bitrate_kbps, const std::string &client_ip) {
      std::lock_guard<std::mutex> lock(_mutex);
      _config = cfg;
      _client_ip = client_ip;
      _is_wan = !is_private_or_local_ip(client_ip);

      if (_is_wan) {
        _ceiling_kbps = std::min(client_requested_bitrate_kbps, _config.wan_max_kbps);
        // Start WAN slightly conservative to establish stable baseline immediately
        _current_target_kbps = std::min(_ceiling_kbps, 32000u);
      } else {
        _ceiling_kbps = client_requested_bitrate_kbps;
        _current_target_kbps = _ceiling_kbps;
      }

      _w_max_kbps = (float)_ceiling_kbps;
      _state = state_e::STEADY;

      auto now = std::chrono::steady_clock::now();
      _last_loss_time = now - 60s;
      _last_brake_time = now - 60s;
      _last_tick_time = now;
      _last_log_time = now;
      _consecutive_loss_free_seconds = 0.0f;
      _min_rtt_history.clear();

      BOOST_LOG(info) << "[ABR] Initialized: Client IP [" << client_ip << "] ("
                      << (_is_wan ? "WAN 5G/Remote" : "LAN Wi-Fi/Local")
                      << ") | Ceiling: " << (_ceiling_kbps / 1000.0)
                      << " Mbps | Initial Target: " << (_current_target_kbps / 1000.0) << " Mbps";
    }

    bool is_enabled() const {
      return _config.enabled;
    }

    bool is_wan() const {
      return _is_wan;
    }

    uint32_t get_target_bitrate_kbps() const {
      return _current_target_kbps;
    }

    float get_idr_vbv_clamp_ratio() const {
      return _config.idr_vbv_clamp_ratio;
    }

    /**
     * @brief Handle Moonlight IDX_LOSS_STATS packet (loss count and interval ms).
     */
    bool on_packet_loss_stats(int32_t loss_count, int32_t interval_ms, uint32_t &out_new_bitrate) {
      if (!_config.enabled) return false;

      std::lock_guard<std::mutex> lock(_mutex);
      auto now = std::chrono::steady_clock::now();

      if (loss_count > 0) {
        _w_max_kbps = (float)_current_target_kbps;
        uint32_t new_target = std::max(_config.min_kbps, (uint32_t)(_current_target_kbps * _config.loss_brake));

        _state = state_e::CONGESTED;
        _last_loss_time = now;
        _consecutive_loss_free_seconds = 0.0f;
        _last_loss_pct = (interval_ms > 0) ? ((float)loss_count * 100.0f / (interval_ms * 0.06f + 1.0f)) : 5.0f;

        if (new_target != _current_target_kbps) {
          _current_target_kbps = new_target;
          out_new_bitrate = new_target;
          log_telemetry(now, true);
          return true;
        }
      } else {
        _last_loss_pct = 0.0f;
      }
      return false;
    }

    /**
     * @brief Periodic 200ms tick evaluating RTT delay gradient, jitter, and cubic ramp.
     */
    bool tick(uint32_t current_rtt, uint32_t rtt_var, uint32_t &out_new_bitrate) {
      if (!_config.enabled) return false;

      std::lock_guard<std::mutex> lock(_mutex);
      auto now = std::chrono::steady_clock::now();
      float dt = std::chrono::duration<float>(now - _last_tick_time).count();
      if (dt < 0.08f) {
        return false;
      }
      _last_tick_time = now;

      // Track minimum RTT in sliding 10s window
      if (current_rtt > 0) {
        _min_rtt_history.emplace_back(now, current_rtt);
      }
      while (!_min_rtt_history.empty() && (now - _min_rtt_history.front().first > 10s)) {
        _min_rtt_history.pop_front();
      }

      uint32_t min_rtt = current_rtt;
      for (const auto &entry : _min_rtt_history) {
        if (entry.second < min_rtt) {
          min_rtt = entry.second;
        }
      }

      uint32_t queue_delay = (current_rtt > min_rtt) ? (current_rtt - min_rtt) : 0;
      bool delay_rising = (queue_delay > _last_queue_delay);
      _last_queue_delay = queue_delay;
      _last_rtt = current_rtt;
      _last_rtt_var = rtt_var;

      bool bitrate_changed = false;
      auto ms_since_loss = std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_loss_time).count();
      auto ms_since_brake = std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_brake_time).count();

      // Pre-Drop Delay Braking: Queuing delay rising above threshold or jitter spike
      if (_state != state_e::CONGESTED && ms_since_brake > 600 && ms_since_loss > 1500) {
        if ((queue_delay >= _config.queue_delay_threshold_ms && delay_rising) ||
            (rtt_var >= _config.jitter_threshold_ms)) {
          _w_max_kbps = (float)_current_target_kbps;
          uint32_t new_target = std::max(_config.min_kbps, (uint32_t)(_current_target_kbps * _config.pre_congestion_trim));

          _state = state_e::PRE_CONGESTION_BRAKE;
          _last_brake_time = now;

          if (new_target != _current_target_kbps) {
            _current_target_kbps = new_target;
            bitrate_changed = true;
          }
        }
      }

      // Cubic Inflection Recovery & Ramp
      if (!bitrate_changed && ms_since_loss > 2000 && ms_since_brake > 600) {
        _consecutive_loss_free_seconds += dt;

        // Exponential decay on W_max towards ceiling if link remains clear > 12s
        if (_consecutive_loss_free_seconds > 12.0f && _w_max_kbps < _ceiling_kbps) {
          float decay_alpha = 1.0f - std::exp(-dt / 30.0f);
          _w_max_kbps += (_ceiling_kbps - _w_max_kbps) * decay_alpha;
        }

        uint32_t new_target = _current_target_kbps;

        if (_current_target_kbps < 0.85f * _w_max_kbps) {
          // Fast ramp (+12%/s)
          float step = _current_target_kbps * (_config.fast_recovery_ramp * dt);
          step = std::max(200.0f, step);
          new_target = std::min((uint32_t)(0.85f * _w_max_kbps), (uint32_t)(_current_target_kbps + step));
          _state = state_e::STEADY;
        } else if (_current_target_kbps <= 1.05f * _w_max_kbps) {
          // Cruise plateau (+1.5%/s)
          float step = _current_target_kbps * (_config.cruise_ramp * dt);
          step = std::max(50.0f, step);
          new_target = std::min((uint32_t)(1.05f * _w_max_kbps), (uint32_t)(_current_target_kbps + step));
          new_target = std::min(_ceiling_kbps, new_target);
          _state = state_e::STEADY;
        } else if (_consecutive_loss_free_seconds > 12.0f && _current_target_kbps < _ceiling_kbps) {
          // Probing mode (+5%/s)
          float step = _current_target_kbps * (_config.probe_ramp * dt);
          step = std::max(100.0f, step);
          new_target = std::min(_ceiling_kbps, (uint32_t)(_current_target_kbps + step));
          _state = state_e::PROBING;
        }

        if (new_target != _current_target_kbps) {
          _current_target_kbps = new_target;
          bitrate_changed = true;
        }
      }

      log_telemetry(now, bitrate_changed);

      if (bitrate_changed) {
        out_new_bitrate = _current_target_kbps;
        return true;
      }
      return false;
    }

  private:
    void log_telemetry(const std::chrono::steady_clock::time_point &now, bool force) {
      if (force || (now - _last_log_time >= 2s)) {
        _last_log_time = now;
        BOOST_LOG(info) << "[ABR] State: " << state_to_string(_state)
                        << " | Target: " << (_current_target_kbps / 1000.0)
                        << " Mbps (Ceiling: " << (_ceiling_kbps / 1000.0)
                        << " Mbps, W_max: " << (_w_max_kbps / 1000.0)
                        << " Mbps) | RTT: " << _last_rtt
                        << "ms (+" << _last_rtt_var << "ms jitter: " << _last_queue_delay
                        << "ms) | Loss: " << _last_loss_pct << "%";
      }
    }

    std::mutex _mutex;
    config_t _config;
    std::string _client_ip;
    bool _is_wan = false;
    uint32_t _ceiling_kbps = 42000;
    uint32_t _current_target_kbps = 42000;
    float _w_max_kbps = 42000.0f;
    state_e _state = state_e::STEADY;

    std::chrono::steady_clock::time_point _last_loss_time;
    std::chrono::steady_clock::time_point _last_brake_time;
    std::chrono::steady_clock::time_point _last_tick_time;
    std::chrono::steady_clock::time_point _last_log_time;

    float _consecutive_loss_free_seconds = 0.0f;
    uint32_t _last_rtt = 0;
    uint32_t _last_rtt_var = 0;
    uint32_t _last_queue_delay = 0;
    float _last_loss_pct = 0.0f;

    std::deque<std::pair<std::chrono::steady_clock::time_point, uint32_t>> _min_rtt_history;
  };

}  // namespace abr
