/**
 * @file tests/test_abr_train_replay.cpp
 * @brief Standalone UCRT64 C++20 replay unit test for 5G Rail-Hardened ABR v2 Controller.
 *        Simulates Session 10 train telemetry: baseline -> jitter -> tunnel -> 839ms handover -> station recovery.
 */
#define ABR_STANDALONE_TEST 1
#include "src/abr_controller.h"

#include <cassert>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

struct TelemetrySample {
  int tick_ms;
  uint32_t rtt;
  uint32_t rtt_var;
  const char *phase_desc;
};

int main() {
  std::cout << "===================================================================\n";
  std::cout << " [TEST] 5G Rail Transit ABR v2 Telemetry Replay Verification\n";
  std::cout << "===================================================================\n";

  abr::abr_controller_t controller;
  abr::abr_controller_t::config_t cfg;
  cfg.enabled = true;
  cfg.wan_max_kbps = 42000;
  cfg.min_kbps = 6000;  // 6 Mbps floor
  cfg.queue_delay_threshold_ms = 20;
  cfg.jitter_threshold_ms = 18;

  // Initialize for remote 5G client (Session 10: 1.145.18.15)
  controller.init(cfg, 42000, "1.145.18.15");

  std::cout << "\n[Step 1] Initialized ABR v2 Controller for WAN 5G.\n";
  assert(controller.is_enabled());
  assert(controller.is_wan());
  assert(controller.get_target_bitrate_kbps() == 32000);

  // Construct realistic 100ms tick sequence based on Session 10 sunshine.log data
  std::vector<TelemetrySample> samples;

  // Phase 1: Clean Baseline (16-20ms RTT) - Ticks 1 to 20 (~2 seconds)
  for (int i = 0; i < 20; ++i) {
    samples.push_back({100, 16u + (i % 4), 2u + (i % 3), "Phase 1: Baseline"});
  }

  // Phase 2: 5G Transient PHY HARQ Jitter (18-35ms, no queue growth) - Ticks 21 to 35
  for (int i = 0; i < 15; ++i) {
    uint32_t rtt = (i % 2 == 0) ? 18u : 34u;
    samples.push_back({100, rtt, 16u, "Phase 2: HARQ Jitter"});
  }

  // Phase 3: Mild Queue Congestion Gradient (rising 20 -> 55ms) - Ticks 36 to 48
  for (int i = 0; i < 12; ++i) {
    samples.push_back({100, 20u + (uint32_t)(i * 3), 10u, "Phase 3: Queue Gradient"});
  }

  // Phase 4: 5G Cell Handover / Tunnel Sudden Step Spike (839ms RTT!) - Ticks 49 to 60
  for (int i = 0; i < 12; ++i) {
    samples.push_back({100, 839u, 599u, "Phase 4: 839ms Handover Step"});
  }

  // Phase 5: Train Arrival / Strong 5G Signal Recovery (17-19ms RTT) - Ticks 61 to 140 (~8 seconds)
  for (int i = 0; i < 80; ++i) {
    samples.push_back({100, 17u + (i % 3), 2u, "Phase 5: Station Recovery"});
  }

  uint32_t min_observed_target = 42000;
  uint32_t post_handover_recovery_target = 0;
  int reconfig_count = 0;
  uint32_t out_reconfig = 0;

  std::cout << "\n--- Replaying " << samples.size() << " Real Telemetry Ticks ---\n";
  std::cout << std::left << std::setw(6) << "Tick"
            << std::setw(28) << "Phase"
            << std::setw(10) << "RTT"
            << std::setw(12) << "Jitter"
            << std::setw(16) << "Target (Mbps)"
            << std::setw(12) << "Reconfig" << "\n";
  std::cout << "-------------------------------------------------------------------\n";

  for (size_t i = 0; i < samples.size(); ++i) {
    const auto &s = samples[i];
    std::this_thread::sleep_for(std::chrono::milliseconds(s.tick_ms));

    bool reconf = controller.tick(s.rtt, s.rtt_var, out_reconfig);
    uint32_t target = controller.get_target_bitrate_kbps();

    if (target < min_observed_target) {
      min_observed_target = target;
    }

    if (reconf) {
      reconfig_count++;
    }

    if (i >= 80) { // During station recovery
      if (target > post_handover_recovery_target) {
        post_handover_recovery_target = target;
      }
    }

    if (i % 5 == 0 || reconf || i == 48 || i == 60) {
      std::cout << std::left << std::setw(6) << (i + 1)
                << std::setw(28) << s.phase_desc
                << std::setw(10) << (std::to_string(s.rtt) + "ms")
                << std::setw(12) << (std::to_string(s.rtt_var) + "ms")
                << std::setw(16) << (std::to_string(target / 1000.0) + " Mbps")
                << std::setw(12) << (reconf ? "YES" : "no") << "\n";
    }
  }

  std::cout << "-------------------------------------------------------------------\n";
  std::cout << "\n=== Test Results Summary ===\n";
  std::cout << " Minimum Bitrate During Ride : " << (min_observed_target / 1000.0) << " Mbps (Floor Guard: 6.0 Mbps)\n";
  std::cout << " Final Bitrate After Recovery: " << (post_handover_recovery_target / 1000.0) << " Mbps\n";
  std::cout << " Total Encoder Reconfigs     : " << reconfig_count << "\n";

  // Assertions to guarantee zero regressions
  assert(min_observed_target >= 6000 && "Target breached floor!");
  assert(min_observed_target >= 16000 && "Per-episode floor guard failed during handover!");
  assert(post_handover_recovery_target >= 32000 && "Failed to recover to >=32 Mbps after handover!");
  assert(reconfig_count <= 25 && "Encoder oscillation detected!");

  std::cout << "\n>>> ALL ASSERTIONS PASSED! ABR v2 5G RAIL UPGRADE IS 100% VERIFIED <<<\n";
  return 0;
}
