/**
 * @file src/abr_controller.h
 * @brief Server-Side 5G Rail-Hardened Adaptive Bitrate (ABR v2) Controller.
 *        Features: 5G HARQ Filter, Handover HOLD, Delay-Gradient Pre-Brake,
 *        Guaranteed Latch Exits, Per-Episode Floor Guards, and Cubic Recovery.
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

#ifndef ABR_STANDALONE_TEST
#include "logging.h"
#include "platform/common.h"
#else
#include <iostream>
#define BOOST_LOG(x) std::cout << "\n[" #x "] "
#endif

namespace abr {

  using namespace std::literals;

  enum class state_e : int {
    STEADY = 0,
    HOLD = 1,                 // 5G Cell Handover / Radio stall pause
    PRE_CONGESTION_BRAKE = 2, // Active delay-gradient trim
    CONGESTED = 3,            // Real packet loss cut
    RECOVERY = 4,             // Fast multiplicative / additive ramp to W_max
    PROBING = 5               // Cautious headroom exploration above W_max
  };

  inline const char *state_to_string(state_e s) {
    switch (s) {
      case state_e::STEADY: return "STEADY";
      case state_e::HOLD: return "HOLD";
      case state_e::PRE_CONGESTION_BRAKE: return "PRE_BRAKE";
      case state_e::CONGESTED: return "CONGESTED";
      case state_e::RECOVERY: return "RECOVERY";
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
      uint32_t min_kbps = 6000;               // Floor bitrate (Raised to 6 Mbps for high-motion 120Hz gaming)
      uint32_t queue_delay_threshold_ms = 20; // Pre-drop trigger threshold (20ms queue growth)
      uint32_t jitter_threshold_ms = 18;      // Jitter trigger threshold (18ms)
      float pre_congestion_trim = 0.90f;      // -10% gentle pre-brake
      float loss_brake = 0.70f;               // -30% multiplicative cut on packet loss
      float fast_recovery_ramp = 0.15f;       // Fast ramp (+15%/s) below 0.85 * W_max
      float cruise_ramp = 0.02f;              // Plateau near W_max
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
        _current_target_kbps = std::min(_ceiling_kbps, 32000u);
      } else {
        _ceiling_kbps = client_requested_bitrate_kbps;
        _current_target_kbps = _ceiling_kbps;
      }

      _last_applied_bitrate_kbps = _current_target_kbps;
      _w_max_kbps = (float)_ceiling_kbps;
      _brake_floor_kbps = _config.min_kbps;
      _state = state_e::STEADY;

      auto now = std::chrono::steady_clock::now();
      _last_loss_time = now - 60s;
      _last_brake_time = now - 60s;
      _last_reconfigure_time = now;
      _last_tick_time = now;
      _last_log_time = now;
      _state_enter_time = now;
      _clear_since_time = now;
      _elevated_since_time = now;

      _consecutive_loss_free_seconds = 0.0f;
      _srtt = 0.0f;
      _prev_srtt = 0.0f;
      _rise_ticks = 0;
      _min_rtt_history.clear();

      BOOST_LOG(info) << "[ABR v2] Initialized: Client IP [" << client_ip << "] ("
                      << (_is_wan ? "WAN 5G/Remote" : "LAN Wi-Fi/Local")
                      << ") | Ceiling: " << (_ceiling_kbps / 1000.0)
                      << " Mbps | Floor: " << (_config.min_kbps / 1000.0)
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
        _brake_floor_kbps = std::max(_config.min_kbps, (uint32_t)(_w_max_kbps * 0.50f));
        uint32_t new_target = std::max(_brake_floor_kbps, (uint32_t)(_current_target_kbps * _config.loss_brake));

        enter_state(state_e::CONGESTED, now);
        _last_loss_time = now;
        _consecutive_loss_free_seconds = 0.0f;
        _last_loss_pct = (interval_ms > 0) ? ((float)loss_count * 100.0f / (interval_ms * 0.06f + 1.0f)) : 5.0f;

        _current_target_kbps = new_target;
        _last_applied_bitrate_kbps = _current_target_kbps;
        _last_reconfigure_time = now;
        out_new_bitrate = _current_target_kbps;
        log_telemetry(now, true);
        return true;
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

      // Smooth RTT using EWMA filter to ignore transient 5G PHY HARQ jitter
      float rtt_f = (float)current_rtt;
      _prev_srtt = (_srtt == 0.0f) ? rtt_f : _srtt;
      _srtt = (_srtt == 0.0f) ? rtt_f : (0.875f * _srtt + 0.125f * rtt_f);

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

      // Base-path shift detection: if RTT is elevated for >5s without rising queue, re-seed base RTT
      if (min_rtt > 0 && current_rtt > (uint32_t)(1.35f * min_rtt)) {
        auto ms_elevated = std::chrono::duration_cast<std::chrono::milliseconds>(now - _elevated_since_time).count();
        if (ms_elevated > 5000 && _state != state_e::HOLD) {
          _min_rtt_history.clear();
          _min_rtt_history.emplace_back(now, current_rtt);
          min_rtt = current_rtt;
          _elevated_since_time = now;
        }
      } else {
        _elevated_since_time = now;
      }

      uint32_t queue_delay = (current_rtt > min_rtt) ? (current_rtt - min_rtt) : 0;
      float grad = _srtt - _prev_srtt;

      // HARQ Jitter Guard: require sustained gradient over multiple ticks
      if (queue_delay >= _config.queue_delay_threshold_ms && grad > 0.5f) {
        _rise_ticks++;
      } else if (queue_delay < 15) {
        _rise_ticks = 0;
      }

      _last_queue_delay = queue_delay;
      _last_rtt = current_rtt;
      _last_rtt_var = rtt_var;

      bool is_handover = (current_rtt - min_rtt > 150 && grad > 25.0f) || (min_rtt > 0 && current_rtt > 4 * min_rtt);
      bool is_congested_gradient = (_rise_ticks >= 3);
      bool is_link_clear = (queue_delay < 15);

      auto ms_since_loss = std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_loss_time).count();
      auto ms_since_brake = std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_brake_time).count();
      auto ms_in_state = std::chrono::duration_cast<std::chrono::milliseconds>(now - _state_enter_time).count();

      // State Machine Transitions
      switch (_state) {
        case state_e::STEADY:
        case state_e::PROBING: {
          if (is_handover) {
            enter_state(state_e::HOLD, now);
          } else if (is_congested_gradient && ms_since_brake > 600 && ms_since_loss > 1500) {
            // Start pre-congestion brake episode with floor guard
            _w_max_kbps = std::max((float)_config.min_kbps, std::max(_w_max_kbps * 0.95f, (float)_current_target_kbps));
            _brake_floor_kbps = std::max(_config.min_kbps, (uint32_t)(0.50f * _w_max_kbps));
            uint32_t new_target = std::max(_brake_floor_kbps, (uint32_t)(_current_target_kbps * _config.pre_congestion_trim));

            enter_state(state_e::PRE_CONGESTION_BRAKE, now);
            _last_brake_time = now;
            _current_target_kbps = new_target;
          }
          break;
        }

        case state_e::HOLD: {
          // Cellular handover freeze: do NOT ratchet down bitrate into the floor
          if (current_rtt < (uint32_t)(1.5f * min_rtt) || is_link_clear) {
            enter_state(state_e::STEADY, now);
          } else if (ms_in_state > 3000) {
            if (is_congested_gradient) {
              _w_max_kbps = std::max((float)_config.min_kbps, (float)_current_target_kbps);
              _brake_floor_kbps = std::max(_config.min_kbps, (uint32_t)(0.50f * _w_max_kbps));
              _current_target_kbps = std::max(_brake_floor_kbps, (uint32_t)(_current_target_kbps * _config.pre_congestion_trim));
              _last_brake_time = now;
              enter_state(state_e::PRE_CONGESTION_BRAKE, now);
            } else {
              enter_state(state_e::RECOVERY, now);
            }
          }
          break;
        }

        case state_e::PRE_CONGESTION_BRAKE: {
          if (is_congested_gradient && ms_since_brake > 600) {
            uint32_t new_target = std::max(_brake_floor_kbps, (uint32_t)(_current_target_kbps * _config.pre_congestion_trim));
            _current_target_kbps = new_target;
            _last_brake_time = now;
          }
          // GUARANTEED EXIT FIX: Never latch forever!
          if (is_link_clear || ms_in_state > 2000) {
            enter_state(state_e::RECOVERY, now);
          }
          break;
        }

        case state_e::CONGESTED: {
          if (ms_since_loss > 2000) {
            enter_state(state_e::RECOVERY, now);
          }
          break;
        }

        case state_e::RECOVERY: {
          if (is_handover) {
            enter_state(state_e::HOLD, now);
          } else if (is_congested_gradient && ms_since_brake > 600) {
            _brake_floor_kbps = std::max(_config.min_kbps, (uint32_t)(0.50f * _w_max_kbps));
            _current_target_kbps = std::max(_brake_floor_kbps, (uint32_t)(_current_target_kbps * _config.pre_congestion_trim));
            _last_brake_time = now;
            _rise_ticks = 0;
            enter_state(state_e::PRE_CONGESTION_BRAKE, now);
          }
          break;
        }
      }

      // Cubic / Multiplicative Recovery Ramp (Active in RECOVERY, STEADY, PROBING)
      if (_state != state_e::PRE_CONGESTION_BRAKE && _state != state_e::CONGESTED && _state != state_e::HOLD &&
          ms_since_loss > 2000 && ms_since_brake > 500) {
        _consecutive_loss_free_seconds += dt;

        // Exponential decay on W_max towards ceiling if link remains clear > 8s
        if (_consecutive_loss_free_seconds > 8.0f && _w_max_kbps < _ceiling_kbps) {
          float decay_alpha = 1.0f - std::exp(-dt / 20.0f);
          _w_max_kbps += (_ceiling_kbps - _w_max_kbps) * decay_alpha;
        }

        uint32_t new_target = _current_target_kbps;

        if (_current_target_kbps < (uint32_t)(0.85f * _w_max_kbps)) {
          // Fast multiplicative ramp (+15%/s) below 0.85 * W_max
          float step = _current_target_kbps * (_config.fast_recovery_ramp * dt);
          step = std::max(300.0f, step);
          new_target = std::min((uint32_t)(0.85f * _w_max_kbps), (uint32_t)(_current_target_kbps + step));
        } else if (_current_target_kbps < (uint32_t)_w_max_kbps) {
          // Linear cruise ramp (+500 kbps/s) closing gap to W_max
          float step = 500.0f * dt;
          new_target = std::min((uint32_t)_w_max_kbps, (uint32_t)(_current_target_kbps + step));
          if (new_target >= (uint32_t)_w_max_kbps && _state == state_e::RECOVERY) {
            _state = state_e::STEADY;
          }
        } else if (_consecutive_loss_free_seconds > 8.0f && _current_target_kbps < _ceiling_kbps) {
          // Probing mode (+5%/s) above W_max towards ceiling
          float step = _current_target_kbps * (_config.probe_ramp * dt);
          step = std::max(150.0f, step);
          new_target = std::min(_ceiling_kbps, (uint32_t)(_current_target_kbps + step));
          _state = state_e::PROBING;
        } else {
          _state = state_e::STEADY;
        }

        _current_target_kbps = new_target;
      }

      // Hysteresis & Rate-Limiting Filter on nvEncReconfigureEncoder:
      // Prevents rapid driver stalls that cap frame rates to 20-30 FPS!
      auto ms_since_reconfig = std::chrono::duration_cast<std::chrono::milliseconds>(now - _last_reconfigure_time).count();
      bool should_reconfigure = false;
      if (_last_applied_bitrate_kbps > 0) {
        float ratio = (float)_current_target_kbps / (float)_last_applied_bitrate_kbps;
        if (ratio <= 0.92f && ms_since_reconfig >= 400) {
          // Stepping DOWN: At least 8% change and 400ms since last reconfigure
          should_reconfigure = true;
        } else if (ratio >= 1.08f && ms_since_reconfig >= 1000) {
          // Stepping UP: At least 8% change and 1000ms since last reconfigure
          should_reconfigure = true;
        }
      } else {
        should_reconfigure = true;
      }

      log_telemetry(now, should_reconfigure);

      if (should_reconfigure) {
        _last_applied_bitrate_kbps = _current_target_kbps;
        _last_reconfigure_time = now;
        out_new_bitrate = _current_target_kbps;
        return true;
      }

      return false;
    }

  private:
    void enter_state(state_e new_state, const std::chrono::steady_clock::time_point &now) {
      _state = new_state;
      _state_enter_time = now;
      _clear_since_time = now;
    }

    void log_telemetry(const std::chrono::steady_clock::time_point &now, bool force) {
      if (force || (now - _last_log_time >= 2s)) {
        _last_log_time = now;
        BOOST_LOG(info) << "[ABR v2] State: " << state_to_string(_state)
                        << " | Target: " << (_current_target_kbps / 1000.0)
                        << " Mbps (Ceiling: " << (_ceiling_kbps / 1000.0)
                        << " Mbps, W_max: " << (_w_max_kbps / 1000.0)
                        << " Mbps, Applied: " << (_last_applied_bitrate_kbps / 1000.0)
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
    uint32_t _last_applied_bitrate_kbps = 42000;
    uint32_t _brake_floor_kbps = 6000;
    float _w_max_kbps = 42000.0f;
    state_e _state = state_e::STEADY;

    std::chrono::steady_clock::time_point _last_loss_time;
    std::chrono::steady_clock::time_point _last_brake_time;
    std::chrono::steady_clock::time_point _last_reconfigure_time;
    std::chrono::steady_clock::time_point _last_tick_time;
    std::chrono::steady_clock::time_point _last_log_time;
    std::chrono::steady_clock::time_point _state_enter_time;
    std::chrono::steady_clock::time_point _clear_since_time;
    std::chrono::steady_clock::time_point _elevated_since_time;

    float _consecutive_loss_free_seconds = 0.0f;
    float _srtt = 0.0f;
    float _prev_srtt = 0.0f;
    int _rise_ticks = 0;

    uint32_t _last_rtt = 0;
    uint32_t _last_rtt_var = 0;
    uint32_t _last_queue_delay = 0;
    float _last_loss_pct = 0.0f;

    std::deque<std::pair<std::chrono::steady_clock::time_point, uint32_t>> _min_rtt_history;
  };

}  // namespace abr
